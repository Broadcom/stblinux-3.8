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
#include <linux/init.h>
#include <linux/types.h>
#include <linux/suspend.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/compiler.h>
#include <linux/pm.h>
#include <asm/fncpy.h>

#include "common.h"

#define AON_CTRL_RESET_CTRL		0x00
#define AON_CTRL_PM_CTRL		0x04
#define AON_CTRL_PM_CPU_WAIT_COUNT	0x10
#define AON_CTRL_PM_INITIATE		0x88
#define AON_CTRL_HOST_MISC_CMDS		0x8c
#define AON_CTRL_SYSTEM_DATA_RAM_OFS	0x200

/* PM_CTRL bitfield */
enum {
	PM_FAST_PWRDOWN	= 1 << 6,
	PM_WARM_BOOT	= 1 << 5,
	PM_DEEP_STANDBY	= 1 << 4,
	PM_CPU_PWR	= 1 << 3,
	PM_USE_CPU_RDY	= 1 << 2,
	PM_PLL_PWRDOWN	= 1 << 1,
	PM_PWR_DOWN	= 1 << 0,
};

#define PM_COLD_CONFIG	(PM_DEEP_STANDBY | PM_PLL_PWRDOWN | PM_USE_CPU_RDY)

#define BRCMSTB_DDR_PHY_PLL_STATUS	0x04

struct brcmstb_pm_control {
	void __iomem *aon_ctrl_base;
	void __iomem *ddr_phy_0_base;
};

static struct brcmstb_pm_control ctrl;
static suspend_state_t suspend_state;

extern const unsigned long brcmstb_pm_do_s2_sz;
extern asmlinkage int brcmstb_pm_do_s2(void __iomem *aon_ctrl_base,
		void __iomem *ddr_phy_pll_status);

static int (*brcmstb_pm_do_s2_sram)(void __iomem *aon_ctrl_base,
		void __iomem *ddr_phy_pll_status);

static inline void __iomem * aon_ram_addr(int idx)
{
	return ctrl.aon_ctrl_base + AON_CTRL_SYSTEM_DATA_RAM_OFS + (idx << 2);
}

static int brcmstb_init_sram(struct device_node *dn)
{
	void __iomem *sram;
	struct resource res;
	int ret;
	unsigned int size = ALIGN(brcmstb_pm_do_s2_sz, FNCPY_ALIGN);

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

static void brcmstb_pm_handshake(void)
{
	void __iomem *base = ctrl.aon_ctrl_base;
	u32 tmp;

	/* BSP power handshake, v1 */
	tmp = __raw_readl(base + AON_CTRL_HOST_MISC_CMDS);
	tmp &= ~1UL;
	__raw_writel(tmp, base + AON_CTRL_HOST_MISC_CMDS);
	(void)__raw_readl(base + AON_CTRL_HOST_MISC_CMDS);

	tmp = __raw_readl(base + AON_CTRL_PM_INITIATE);
	tmp &= ~1UL;
	__raw_writel(tmp, base + AON_CTRL_PM_INITIATE);
	(void)__raw_readl(base + AON_CTRL_PM_INITIATE);

	tmp |= 1UL;
	__raw_writel(tmp, base + AON_CTRL_PM_INITIATE);
	(void)__raw_readl(base + AON_CTRL_PM_INITIATE);

	mdelay(10);
}

/* Support S3 cold boot out of "poweroff" */
static void brcmstb_pm_poweroff(void)
{
	void __iomem *base = ctrl.aon_ctrl_base;

	brcmstb_pm_handshake();

	/* Clear magic S3 warm-boot value */
	__raw_writel(0, aon_ram_addr(0));
	(void)__raw_readl(aon_ram_addr(0));

	/* pm_start_pwrdn transition 0->1 */
	__raw_writel(PM_COLD_CONFIG, base + AON_CTRL_PM_CTRL);
	(void)__raw_readl(base + AON_CTRL_PM_CTRL);

	__raw_writel(PM_COLD_CONFIG | PM_PWR_DOWN, base + AON_CTRL_PM_CTRL);
	(void)__raw_readl(base + AON_CTRL_PM_CTRL);

	wfi();
}

static int brcmstb_pm_standby(bool deep_standby)
{
	int ret;

	/* FIXME: no HW support yet */
	WARN_ON(deep_standby);

	brcmstb_pm_handshake();

	ret = brcmstb_pm_do_s2_sram(ctrl.aon_ctrl_base,
			ctrl.ddr_phy_0_base + BRCMSTB_DDR_PHY_PLL_STATUS);
	if (ret)
		pr_err("%s: standby failed\n", __func__);

	return ret;
}

static int brcmstb_pm_begin(suspend_state_t state)
{
	suspend_state = state;
	return 0;
}

static int brcmstb_pm_prepare(void)
{
	/* TODO: initialize memory hashing (for S3) here? */
	return 0;
}

static int brcmstb_pm_enter(suspend_state_t unused)
{
	int ret = -EINVAL;

	switch (suspend_state) {
	case PM_SUSPEND_STANDBY:
		ret = brcmstb_pm_standby(false);
		break;
	case PM_SUSPEND_MEM:
		ret = brcmstb_pm_standby(true);
		break;
	}

	return ret;
}

static void brcmstb_pm_finish(void)
{
	/* TODO: perform cleanup for S3? */
}

static void brcmstb_pm_end(void)
{
	suspend_state = PM_SUSPEND_ON;
}

static int brcmstb_pm_valid(suspend_state_t state)
{
	/* FIXME: Only support S2 standby */
	return state == PM_SUSPEND_STANDBY;
}

static const struct platform_suspend_ops brcmstb_pm_ops = {
	.begin		= brcmstb_pm_begin,
	.end		= brcmstb_pm_end,
	.prepare	= brcmstb_pm_prepare,
	.enter		= brcmstb_pm_enter,
	.finish		= brcmstb_pm_finish,
	.valid		= brcmstb_pm_valid,
};

static struct of_device_id aon_ctrl_dt_ids[] = {
	{ .compatible = "brcm,brcmstb-aon-ctrl" },
	{}
};

static struct of_device_id ddr_phy_dt_ids[] = {
	{ .compatible = "brcm,brcmstb-ddr-phy-v225.1" },
	/* TODO: add warm boot support */
	{ .compatible = "brcm,brcmstb-ddr-phy-v240.1" },
	{}
};

static void __iomem * brcmstb_ioremap_match(struct of_device_id *matches)
{
	struct device_node *dn;
	struct resource res, *res2;
	int ret;
	void __iomem *base;

	dn = of_find_matching_node(NULL, matches);
	if (!dn)
		return ERR_PTR(-EINVAL);

	ret = of_address_to_resource(dn, 0, &res);
	if (ret < 0)
		return ERR_PTR(ret);

	res2 = request_mem_region(res.start, resource_size(&res), dn->full_name);
	if (!res2)
		return ERR_PTR(-EIO);

	base = ioremap(res2->start, resource_size(res2));
	if (!base)
		return ERR_PTR(-ENOMEM);

	return base;
}

int brcmstb_pm_init(void)
{
	struct device_node *dn;
	void __iomem *base;
	int ret;

	base = brcmstb_ioremap_match(aon_ctrl_dt_ids);
	if (IS_ERR_OR_NULL(base)) {
		pr_err("error mapping AON_CTRL\n");
		return PTR_ERR(base);
	}
	ctrl.aon_ctrl_base = base;

	base = brcmstb_ioremap_match(ddr_phy_dt_ids);
	if (IS_ERR_OR_NULL(base)) {
		pr_err("error mapping DDR PHY\n");
		return PTR_ERR(base);
	}
	/* Only need DDR PHY 0 for now? */
	ctrl.ddr_phy_0_base = base;

	dn = of_find_matching_node(NULL, sram_dt_ids);
	if (!dn) {
		pr_err("SRAM not found\n");
		return -EINVAL;
	}

	ret = brcmstb_init_sram(dn);
	if (ret) {
		pr_err("error setting up SRAM for PM\n");
		return ret;
	}

	pm_power_off = brcmstb_pm_poweroff;
	suspend_set_ops(&brcmstb_pm_ops);
	return 0;
}
