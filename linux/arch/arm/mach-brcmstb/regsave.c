/*
 * Support for Broadcom STB reg save for power management
 *
 * Copyright (C) 2014 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/syscore_ops.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/slab.h>


struct brcmstb_rm_group {
	struct regmap *rm;
	int count;
	void *mem;
};
static struct brcmstb_rm_group *rm_groups;
static int num_regmap_groups;


static int reg_save(void)
{
	int i, ret, total;
	u32 *mem;

	if (0 == num_regmap_groups)
		return 0;

	for (i = 0, total = 0; i < num_regmap_groups; i++)
		total += rm_groups[i].count;
	mem = kmalloc(total * sizeof(u32), GFP_KERNEL);
	if (!mem)
		return -ENOMEM;

	for (i = 0, total = 0; i < num_regmap_groups; i++) {
		struct brcmstb_rm_group *p = &rm_groups[i];
		p->mem = &mem[total];
		ret = regmap_bulk_read(p->rm, 0, p->mem, p->count);
		if (ret)
			return ret;
		total += p->count;
	}
	return 0;
}


static void reg_restore(void)
{
	int i, ret, total;

	if (0 == num_regmap_groups)
		return;

	for (i = 0, total = 0; i < num_regmap_groups; i++) {
		struct brcmstb_rm_group *p = &rm_groups[i];
		ret = regmap_bulk_write(p->rm, 0, p->mem, p->count);
		if (ret)
			pr_err("failed to restore reg group\n");
	}
	kfree(rm_groups[0].mem);
}


static struct syscore_ops regsave_pm_ops = {
	.suspend        = reg_save,
	.resume         = reg_restore,
};


int brcmstb_regsave_init(void)
{
	struct regmap *rm;
	struct resource res;
	struct device_node *dn, *pp;
	int ret = 0, len, num_phandles, i;

	dn = of_find_node_by_name(NULL, "s3");
	if (!dn)
		/* FIXME: return -EINVAL when all bolts have 's3' node */
		goto fail;

	if (!of_get_property(dn, "syscon-refs", &len))
		/* FIXME: return -EINVAL when all bolts have 'syscon-refs' */
		goto fail;

	num_phandles = len / 4;
	rm_groups = kzalloc(num_phandles * sizeof(struct brcmstb_rm_group),
			    GFP_KERNEL);
	if (rm_groups == NULL) {
		ret = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < num_phandles; i++) {
		pp = of_parse_phandle(dn, "syscon-refs", i);
		if (pp) {
			rm = syscon_node_to_regmap(pp);
			if (rm == NULL) {
				ret = -EIO;
				goto fail;
			}
			WARN_ON(4 != regmap_get_val_bytes(rm));
			ret = of_address_to_resource(pp, 0, &res);
			if (ret)
				goto fail;
			rm_groups[num_regmap_groups].rm = rm;
			rm_groups[num_regmap_groups].count
				= resource_size(&res) >> 2;
			num_regmap_groups++;
		}
	};
	of_node_put(dn);
	register_syscore_ops(&regsave_pm_ops);
	return 0;
fail:
	of_node_put(dn);
	kfree(rm_groups);
	return ret;
}
