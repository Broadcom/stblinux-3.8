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
	struct irq_chip chip;
	spinlock_t lock;
	bool can_wake;
	u32 wakeup_enabled; /* vector of enabled wakeup interrupts */
	u32 saved_mask; /* for suspend/resume */
};

static void brcmstb_l2_intc_mask(struct irq_data *d)
{
	struct brcmstb_l2_intc_data *b = irq_data_get_irq_chip_data(d);

	__raw_writel(1 << d->hwirq, b->base + CPU_MASK_SET);
}

static void brcmstb_l2_intc_unmask(struct irq_data *d)
{
	struct brcmstb_l2_intc_data *b = irq_data_get_irq_chip_data(d);

	__raw_writel(1 << d->hwirq, b->base + CPU_MASK_CLEAR);
}

static void brcmstb_l2_intc_ack(struct irq_data *d)
{
	struct brcmstb_l2_intc_data *b = irq_data_get_irq_chip_data(d);

	__raw_writel(1 << d->hwirq, b->base + CPU_CLEAR);
}

static void brcmstb_l2_intc_irq_handle(unsigned int irq, struct irq_desc *desc)
{
	struct brcmstb_l2_intc_data *b = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_get_chip(irq);
	u32 status;

	chained_irq_enter(chip, desc);

	spin_lock(&b->lock);
	status = __raw_readl(b->base + CPU_STATUS) &
		~(__raw_readl(b->base + CPU_MASK_STATUS));
	spin_unlock(&b->lock);

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

static int brcmstb_l2_intc_map(struct irq_domain *d, unsigned int irq,
				irq_hw_number_t hwirq)
{
	struct brcmstb_l2_intc_data *data = d->host_data;

	irq_set_chip_data(irq, data);
	irq_set_chip_and_handler(irq, &data->chip, handle_level_irq);
	set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);

	return 0;
}

static struct irq_domain_ops brcmstb_l2_intc_ops = {
	.map = brcmstb_l2_intc_map,
	.xlate = irq_domain_xlate_onecell,
};

static void brcmstb_l2_intc_suspend(struct irq_data *d)
{
	struct brcmstb_l2_intc_data *b = irq_data_get_irq_chip_data(d);

	spin_lock(&b->lock);
	/* Save the current mask */
	b->saved_mask = __raw_readl(b->base + CPU_MASK_STATUS);

	if (b->can_wake) {
		/* Program the wakeup mask */
		__raw_writel(~b->wakeup_enabled, b->base + CPU_MASK_SET);
		__raw_writel(b->wakeup_enabled, b->base + CPU_MASK_CLEAR);
	}
	spin_unlock(&b->lock);
}

static void brcmstb_l2_intc_resume(struct irq_data *d)
{
	struct brcmstb_l2_intc_data *b = irq_data_get_irq_chip_data(d);

	spin_lock(&b->lock);
	/* Clear unmasked non-wakeup interrupts */
	__raw_writel(~b->saved_mask & ~b->wakeup_enabled, b->base + CPU_CLEAR);

	/* Restore the saved mask */
	__raw_writel(b->saved_mask, b->base + CPU_MASK_SET);
	__raw_writel(~b->saved_mask, b->base + CPU_MASK_CLEAR);
	spin_unlock(&b->lock);
}

static int brcmstb_l2_intc_set_wake(struct irq_data *d, unsigned int state)
{
	struct brcmstb_l2_intc_data *b = irq_data_get_irq_chip_data(d);
	u32 mask = 1 << d->hwirq;

	spin_lock(&b->lock);
	if (state)
		b->wakeup_enabled |= mask;
	else
		b->wakeup_enabled &= ~mask;
	spin_unlock(&b->lock);

	return 0;
}

int __init brcmstb_l2_intc_of_init(struct device_node *np,
					struct device_node *parent)
{
	struct brcmstb_l2_intc_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	spin_lock_init(&data->lock);

	data->base = of_iomap(np, 0);
	if (!data->base) {
		pr_err("failed to remap intc L2 registers\n");
		return -ENOMEM;
	}

	data->parent_irq = irq_of_parse_and_map(np, 0);
	if (data->parent_irq < 0) {
		pr_err("failed to find parent interrupt\n");
		return data->parent_irq;
	}

	data->chip.name = np->full_name;
	data->chip.irq_ack = brcmstb_l2_intc_ack;
	data->chip.irq_mask = brcmstb_l2_intc_mask;
	data->chip.irq_unmask = brcmstb_l2_intc_unmask;
	data->chip.irq_suspend = brcmstb_l2_intc_suspend;
	data->chip.irq_resume = brcmstb_l2_intc_resume;
	if (of_property_read_bool(np, "brcm,irq-can-wake")) {
		data->can_wake = true;
		data->chip.irq_set_wake = brcmstb_l2_intc_set_wake;
	}

	irq_set_handler_data(data->parent_irq, data);
	irq_set_chained_handler(data->parent_irq, brcmstb_l2_intc_irq_handle);

	/* Disable all interrupts by default */
	__raw_writel(0xffffffff, data->base + CPU_MASK_SET);
	__raw_writel(0xffffffff, data->base + CPU_CLEAR);

	/* Add a linear domain of 32 interrupts, we could probably parse some
	 * sort of mask
	 */
	data->domain = irq_domain_add_linear(np, 32,
				&brcmstb_l2_intc_ops, data);
	if (!data->domain)
		return -ENODEV;

	pr_info("registered L2 intc (mem: 0x%p, parent irq: %d)\n",
			data->base, data->parent_irq);

	return 0;
}
