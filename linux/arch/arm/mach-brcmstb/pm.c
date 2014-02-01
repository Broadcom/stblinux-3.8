/*
 * ARM-specific support for Broadcom STB S2 power management
 *
 * Copyright © 2014 Broadcom Corporation
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

#define pr_fmt(fmt) "brcmstb-pm: " fmt

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <asm/fncpy.h>
#include <linux/brcmstb/brcmstb.h>

extern const unsigned int _brcmstb_pm_s2_start, _brcmstb_pm_s2_end;
extern asmlinkage int brcmstb_pm_do_s2(void __iomem *aon_ctrl_base,
		void __iomem *ddr_phy_pll_status);

static int (*brcmstb_pm_do_s2_sram)(void __iomem *aon_ctrl_base,
		void __iomem *ddr_phy_pll_status);
static struct brcmstb_pm_control *pm_ctrl;

#define BRCMSTB_DDR_PHY_PLL_STATUS	0x04

static int brcmstb_init_sram(struct device_node *dn)
{
	void __iomem *sram;
	struct resource res;
	int ret;
	unsigned int size = (uintptr_t)&_brcmstb_pm_s2_end -
			    (uintptr_t)&_brcmstb_pm_s2_start;

	ret = of_address_to_resource(dn, 0, &res);
	if (ret)
		return ret;

	if (resource_size(&res) < size) {
		pr_err("standby code will not fit in SRAM\n");
		return -ENOMEM;
	}

	/* Cached, executable remapping of SRAM */
	sram = __arm_ioremap_exec(res.start, size, true);
	if (!sram)
		return -ENOMEM;

	brcmstb_pm_do_s2_sram = fncpy(sram, &brcmstb_pm_do_s2, size);
	if (!brcmstb_pm_do_s2_sram)
		return -EINVAL;

	return 0;
}

int brcmstb_arch_pm_do_s2(void)
{
	return brcmstb_pm_do_s2_sram(pm_ctrl->aon_ctrl_base,
			pm_ctrl->ddr_phy_0_base + BRCMSTB_DDR_PHY_PLL_STATUS);
}

/*
 * Latch into the BRCM SRAM compatible property here to be more specific than
 * the standard "mmio-sram"
 *
 * FIXME: As of Linux 3.10, upstream's drivers/misc/sram.c should support
 * allocating SRAM via genalloc
 */
static struct of_device_id sram_dt_ids[] = {
	{ .compatible = "brcm,boot-sram" },
	{}
};

int brcmstb_arch_pm_init(struct brcmstb_pm_control *ctrl)
{
	struct device_node *dn;
	int ret;

	pm_ctrl = ctrl;

	dn = of_find_matching_node(NULL, sram_dt_ids);
	if (!dn) {
		pr_err("SRAM not found\n");
		return -EINVAL;
	}

	ret = brcmstb_init_sram(dn);
	if (ret)
		pr_err("error setting up SRAM for PM\n");

	return ret;
}
