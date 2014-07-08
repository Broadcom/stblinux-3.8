/*
 * Generic Broadcom STB Level 2 Interrupt controller driver
 *
 * Copyright (C) 2013 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME	": " fmt

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/brcmstb-l2.h>
#include <linux/reboot.h>

#include <asm/mach/irq.h>

/* Register offsets in the L2 interrupt controller */
#define CPU_STATUS	0x00
#define CPU_SET		0x04
#define CPU_CLEAR	0x08
#define CPU_MASK_STATUS	0x0c
#define CPU_MASK_SET	0x10
#define CPU_MASK_CLEAR	0x14

/* L2 intc private data structure */
struct brcmstb_l2_intc_data {
	int parent_irq;
	void __iomem *base;
	struct irq_domain *domain;
	struct irq_chip_generic *gc;
	bool can_wake;
	u32 saved_mask; /* for suspend/resume */
	struct notifier_block reboot_notifier;
};

static void brcmstb_l2_intc_irq_handle(unsigned int irq, struct irq_desc *desc)
{
	struct brcmstb_l2_intc_data *b = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_get_chip(irq);
	u32 status;

	chained_irq_enter(chip, desc);

	status = __raw_readl(b->base + CPU_STATUS) &
		~(__raw_readl(b->base + CPU_MASK_STATUS));

	if (status == 0) {
		do_bad_IRQ(irq, desc);
		goto out;
	}

	do {
		irq = ffs(status) - 1;
		/* ack at our level */
		__raw_writel(1 << irq, b->base + CPU_CLEAR);
		status &= ~(1 << irq);
		generic_handle_irq(irq_find_mapping(b->domain, irq));
	} while (status);
out:
	chained_irq_exit(chip, desc);
}

static void brcmstb_l2_intc_suspend(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct brcmstb_l2_intc_data *b = gc->private;

	irq_gc_lock(gc);
	/* Save the current mask */
	b->saved_mask = __raw_readl(b->base + CPU_MASK_STATUS);

	if (b->can_wake) {
		/* Program the wakeup mask */
		__raw_writel(~gc->wake_active, b->base + CPU_MASK_SET);
		__raw_writel(gc->wake_active, b->base + CPU_MASK_CLEAR);
	}
	irq_gc_unlock(gc);
}

static void brcmstb_l2_intc_resume(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct brcmstb_l2_intc_data *b = gc->private;

	irq_gc_lock(gc);
	/* Clear unmasked non-wakeup interrupts */
	__raw_writel(~b->saved_mask & ~gc->wake_active, b->base + CPU_CLEAR);

	/* Restore the saved mask */
	__raw_writel(b->saved_mask, b->base + CPU_MASK_SET);
	__raw_writel(~b->saved_mask, b->base + CPU_MASK_CLEAR);
	irq_gc_unlock(gc);
}

static int brcmstb_l2_intc_reboot(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct brcmstb_l2_intc_data *b;
	b = container_of(nb, struct brcmstb_l2_intc_data, reboot_notifier);

	irq_gc_lock(b->gc);

	if (action == SYS_POWER_OFF) {
		if (!b->gc->wake_active)
			pr_err("WARNING: NO WAKEUP SOURCE CONFIGURED\n");
		/* Program the wakeup mask */
		__raw_writel(~b->gc->wake_active, b->base + CPU_MASK_SET);
		__raw_writel(b->gc->wake_active, b->base + CPU_MASK_CLEAR);
	}

	irq_gc_unlock(b->gc);

	return NOTIFY_DONE;
}

int __init brcmstb_l2_intc_of_init(struct device_node *np,
					struct device_node *parent)
{
	struct brcmstb_l2_intc_data *data;
	struct irq_chip_generic *gc;
	struct irq_chip_type *ct;
	int irq_base;
	int ret;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->base = of_iomap(np, 0);
	if (!data->base) {
		pr_err("failed to remap intc L2 registers\n");
		return -ENOMEM;
	}

	/* Disable all interrupts by default */
	__raw_writel(0xffffffff, data->base + CPU_MASK_SET);
	__raw_writel(0xffffffff, data->base + CPU_CLEAR);

	data->parent_irq = irq_of_parse_and_map(np, 0);
	if (data->parent_irq < 0) {
		pr_err("failed to find parent interrupt\n");
		return data->parent_irq;
	}

	/* Allocate a new irq_base for our generic irq chip, otherwise
	 * this will conflict with the existing ones (e.g: L1 INTC)
	 */
	irq_base = irq_alloc_descs(-1, 0, 32, numa_node_id());
	if (irq_base < 0) {
		pr_err("failed to allocate an irq_base\n");
		goto out_unmap;
	}

	/* Allocate a legacy IRQ domain */
	data->domain = irq_domain_add_legacy(np, 32, irq_base, 0,
			&irq_domain_simple_ops, NULL);
	if (!data->domain) {
		ret = -ENOMEM;
		goto out_free_desc;
	}

	/* Allocate a single Generic IRQ chip for this node */
	data->gc = irq_alloc_generic_chip(np->full_name, 1, irq_base,
			data->base, handle_edge_irq);
	if (!data->gc) {
		ret = -ENOMEM;
		goto out_free_domain;
	}

	/* Set the IRQ chaining logic */
	irq_set_handler_data(data->parent_irq, data);
	irq_set_chained_handler(data->parent_irq, brcmstb_l2_intc_irq_handle);

	gc = data->gc;
	gc->private = data;
	ct = gc->chip_types;

	ct->chip.irq_ack = irq_gc_ack_set_bit;
	ct->regs.ack = CPU_CLEAR;

	ct->chip.irq_mask = irq_gc_mask_disable_reg;
	ct->regs.disable = CPU_MASK_SET;

	ct->chip.irq_unmask = irq_gc_unmask_enable_reg;
	ct->regs.enable = CPU_MASK_CLEAR;

	ct->chip.irq_suspend = brcmstb_l2_intc_suspend;
	ct->chip.irq_resume = brcmstb_l2_intc_resume;

	if (of_property_read_bool(np, "brcm,irq-can-wake")) {
		data->can_wake = true;
		/* This IRQ chip can wake the system, set all child interrupts
		 * in wake_enabled mask
		 */
		gc->wake_enabled = 0xffffffff;
		ct->chip.irq_set_wake = irq_gc_set_wake;

		/* Run reboot notifier last */
		data->reboot_notifier.priority = -1;
		data->reboot_notifier.notifier_call = brcmstb_l2_intc_reboot;
		register_reboot_notifier(&data->reboot_notifier);
	}

	irq_setup_generic_chip(gc, IRQ_MSK(32), 0, 0, 0);

	pr_info("registered L2 intc (mem: 0x%p, parent irq: %d)\n",
			data->base, data->parent_irq);

	return 0;

out_free_domain:
	irq_domain_remove(data->domain);
out_free_desc:
	irq_free_descs(irq_base, 32);
out_unmap:
	iounmap(data->base);
	kfree(data);
	return ret;
}
