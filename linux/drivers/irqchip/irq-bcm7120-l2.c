/*
 * Broadcom BCM7120 style Level 2 interrupt controller driver
 *
 * Copyright (C) 2014 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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

/* Register offset in the L2 interrupt controller */
#define IRQEN		0x00
#define IRQSTAT		0x04

struct bcm7120_l2_intc_data {
	void __iomem *base;
	struct irq_chip_generic *gc;
	struct irq_domain *domain;
	u32 irq_fwd_mask;
	u32 irq_map_mask;
	bool can_wake;
	u32 saved_mask;
};

static void bcm7120_l2_intc_irq_handle(unsigned int irq, struct irq_desc *desc)
{
	struct bcm7120_l2_intc_data *b = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 status;

	chained_irq_enter(chip, desc);

	irq_gc_lock(b->gc);
	status = __raw_readl(b->base + IRQSTAT);
	irq_gc_unlock(b->gc);

	if (status == 0) {
		do_bad_IRQ(irq, desc);
		goto out;
	}

	do {
		irq = ffs(status) - 1;
		status &= ~(1 << irq);
		generic_handle_irq(irq_find_mapping(b->domain, irq));
	} while (status);

out:
	chained_irq_exit(chip, desc);
}

static void bcm7120_l2_intc_suspend(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct bcm7120_l2_intc_data *b = gc->private;
	u32 reg;

	irq_gc_lock(gc);
	/* Save the current mask and the interrupt forward mask */
	b->saved_mask = __raw_readl(b->base) | b->irq_fwd_mask;
	if (b->can_wake) {
		reg = b->saved_mask | gc->wake_active;
		__raw_writel(reg, b->base);
	}
	irq_gc_unlock(gc);
}

static void bcm7120_l2_intc_resume(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct bcm7120_l2_intc_data *b = gc->private;

	/* Restore the saved mask */
	irq_gc_lock(gc);
	__raw_writel(b->saved_mask, b->base);
	irq_gc_unlock(gc);
}

static int bcm7120_l2_intc_map(struct irq_domain *d, unsigned int irq,
				irq_hw_number_t hwirq)
{
	struct bcm7120_l2_intc_data *p = d->host_data;

	if (!(p->irq_map_mask & (1 << hwirq)))
		return -EOPNOTSUPP;

	irq_set_chip_data(irq, p->gc);
	irq_set_chip_and_handler(irq, &p->gc->chip_types->chip,
					handle_level_irq);
	set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);

	return 0;
}

static struct irq_domain_ops bcm7120_l2_intc_ops = {
	.map = bcm7120_l2_intc_map,
	.xlate = irq_domain_xlate_onecell,
};

static int bcm7120_l2_intc_init_one(struct device_node *dn,
					struct bcm7120_l2_intc_data *data,
					int irq, const __be32 *map_mask)
{
	int parent_irq;
	int i;

	parent_irq = irq_of_parse_and_map(dn, irq);
	if (parent_irq < 0) {
		pr_err("failed to map interrupt %d\n", irq);
		return parent_irq;
	}

	data->irq_map_mask |= be32_to_cpup(map_mask + irq);

	irq_set_handler_data(parent_irq, data);
	irq_set_chained_handler(parent_irq, bcm7120_l2_intc_irq_handle);

	for (i = 0; i < 32; i++) {
		if (data->irq_map_mask & (1 << i))
			irq_create_mapping(data->domain, i);
	}

	return 0;
}

int __init bcm7120_l2_intc_of_init(struct device_node *dn,
					struct device_node *parent)
{
	struct bcm7120_l2_intc_data *data;
	struct irq_chip_type *ct;
	const __be32 *map_mask;
	int num_parent_irqs;
	int ret = 0, irq_base, len, irq;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->base = of_iomap(dn, 0);
	if (!data->base) {
		pr_err("failed to remap intc L2 registers\n");
		ret = -ENOMEM;
		goto out_free;
	}

	if (of_property_read_u32(dn, "brcm,int-fwd-mask", &data->irq_fwd_mask))
		data->irq_fwd_mask = 0;

	/* Enable all interrupt specified in the interrupt forward mask and have
	 * the other disabled
	 */
	__raw_writel(data->irq_fwd_mask, data->base + IRQEN);

	num_parent_irqs = of_irq_count(dn);
	if (num_parent_irqs <= 0) {
		pr_err("invalid number of parent interrupts\n");
		ret = -ENOMEM;
		goto out_unmap;
	}

	map_mask = of_get_property(dn, "brcm,int-map-mask", &len);
	if (!map_mask || (len != (sizeof(*map_mask) * num_parent_irqs))) {
		pr_err("invalid brcm,int-map-mask property\n");
		ret = -EINVAL;
		goto out_unmap;
	}

	irq_base = irq_alloc_descs(-1, 0, 32, numa_node_id());
	if (irq_base < 0) {
		pr_err("failed to allocate an irq_base\n");
		goto out_unmap;
	}

	data->domain = irq_domain_add_simple(dn, 32, irq_base,
					&bcm7120_l2_intc_ops, data);
	if (!data->domain) {
		ret = -ENOMEM;
		goto out_free_desc;
	}

	data->gc = irq_alloc_generic_chip(dn->full_name, 1, irq_base,
				data->base, handle_level_irq);
	if (!data->gc) {
		ret = -ENOMEM;
		goto out_free_domain;
	}

	data->gc->private = data;
	ct = data->gc->chip_types;

	ct->regs.mask = IRQEN;
	ct->chip.irq_mask = irq_gc_mask_clr_bit;
	ct->chip.irq_unmask = irq_gc_mask_set_bit;
	ct->chip.irq_ack = irq_gc_noop;
	ct->chip.irq_suspend = bcm7120_l2_intc_suspend;
	ct->chip.irq_resume = bcm7120_l2_intc_resume;

	if (of_property_read_bool(dn, "brcm,irq-can-wake")) {
		data->can_wake = true;
		/* This IRQ chip can wake the system, set all child interrupts
		 * in wake_enabled mask but the forward enable mask since those
		 * are not requestable interrupt bits.
		 */
		data->gc->wake_enabled = 0xffffffff & ~data->irq_fwd_mask;
		ct->chip.irq_set_wake = irq_gc_set_wake;
	}

	irq_setup_generic_chip(data->gc, IRQ_MSK(32), IRQ_GC_INIT_MASK_CACHE,
				0, 0);

	for (irq = 0; irq < num_parent_irqs; irq++) {
		ret = bcm7120_l2_intc_init_one(dn, data, irq, map_mask);
		if (ret)
			goto out_unreg_gc;
	}

	pr_info("registered BCM7120 L2 intc (mem: 0x%p, parent IRQ(s): %d)\n",
			data->base, num_parent_irqs);

	return 0;

out_unreg_gc:
	irq_remove_generic_chip(data->gc, IRQ_MSK(32), 0, 0);
out_free_domain:
	irq_domain_remove(data->domain);
out_free_desc:
	irq_free_descs(irq_base, 32);
out_unmap:
	iounmap(data->base);
out_free:
	kfree(data);
	return ret;
}
