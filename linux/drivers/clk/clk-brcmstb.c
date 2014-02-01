/*
 * Copyright (C) 2009 Broadcom Corporation
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
#define pr_fmt(fmt) "clk-brcmstb: " fmt

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/brcmstb/brcmstb.h>

struct bcm_clk {
	struct clk_hw hw;
	char name[16];
	char parent_name[16];
	u32 flags;
	struct clk_ops ops;
	void __iomem	*clk_ctrl;
	void __iomem	*clk_cfg;
};

#define to_brcmstb_clk(p) container_of(p, struct bcm_clk, hw)

static int
brcmstb_clk_pll_enable(struct clk_hw *hwclk)
{
	return 0;
}

static void
brcmstb_clk_pll_disable(struct clk_hw *hwclk)
{
}

static int
brcmstb_clk_set_rate(struct clk_hw *hwclk, unsigned long rate,
		     unsigned long parent_rate)
{
	return 0;
}

static long
brcmstb_clk_round_rate(struct clk_hw *hwclk, unsigned long rate,
		    unsigned long *prate)
{
	return 0;
}

static unsigned long
brcmstb_clk_recalc_rate(struct clk_hw *hwclk, unsigned long rate)

{
	return 0;
}

static struct clk * __init
brcmstb_clk_register(struct device *dev, struct bcm_clk *brcmstb_clk,
		     struct clk_ops *ops, const char *name,
		     const char *parent_name, u32 flags)
{
	struct clk *clk;
	struct clk_init_data init;

	init.name = name;
	init.ops = ops;
	init.flags = flags;
	init.parent_names = &parent_name;
	init.num_parents = (parent_name ? 1 : 0);
	brcmstb_clk->hw.init = &init;

	clk = clk_register(dev, &brcmstb_clk->hw);

	if (WARN_ON(IS_ERR(clk)))
		goto err_clk_register;

	return clk;

err_clk_register:
	return ERR_PTR(-EINVAL);
}

static void
brcmstb_clk_init(struct bcm_clk *brcmstb_clk_table, int lim)
{
	struct clk *clk;
	struct bcm_clk *brcmstb_clk;
	int clk_idx, ret;
	char *parent_name;

	for (clk_idx = 0; clk_idx < lim; clk_idx++) {
		brcmstb_clk = &brcmstb_clk_table[clk_idx];

		if (!brcmstb_clk->ops.enable)
			brcmstb_clk->ops.enable =
				&brcmstb_clk_pll_enable;

		if (!brcmstb_clk->ops.disable)
			brcmstb_clk->ops.disable =
				&brcmstb_clk_pll_disable;

		if (!brcmstb_clk->ops.set_rate)
			brcmstb_clk->ops.set_rate =
				&brcmstb_clk_set_rate;

		if (!brcmstb_clk->ops.round_rate)
			brcmstb_clk->ops.round_rate =
				&brcmstb_clk_round_rate;

		if (!brcmstb_clk->ops.recalc_rate)
			brcmstb_clk->ops.recalc_rate =
				&brcmstb_clk_recalc_rate;

		parent_name = brcmstb_clk_table[clk_idx].parent_name;
		clk = brcmstb_clk_register(NULL, brcmstb_clk,
					   &brcmstb_clk->ops,
					   brcmstb_clk->name,
					   parent_name,
					   (parent_name == NULL ?
					    CLK_IS_ROOT | CLK_IGNORE_UNUSED :
					    CLK_IGNORE_UNUSED));

		if (IS_ERR(clk))
			pr_err("%s clk_register failed\n",
				brcmstb_clk->name);

		ret = clk_register_clkdev(clk, brcmstb_clk->name, NULL);

		if (ret)
			pr_err("%s clk device registration failed\n",
			       brcmstb_clk->name);
	}
}

/*
 * MOCA clock
 */
enum brcmstb_moca_clk {
	BRCM_CLK_MOCA,
	BRCM_CLK_MOCA_CPU,
	BRCM_CLK_MOCA_PHY
};

static int bmoca_clk_prepare(struct clk_hw *hw);

static struct bcm_clk brcmstb_moca_clk_table[] = {
	[BRCM_CLK_MOCA] = {
		.name = "moca",
		.ops.prepare = bmoca_clk_prepare,
	},
	[BRCM_CLK_MOCA_CPU] = {
		.name		= "moca-cpu",
		.parent_name    = "moca",
	},
	[BRCM_CLK_MOCA_PHY] = {
		.name = "moca-phy",
		.parent_name = "moca",
	},
};

static int
bmoca_clk_prepare(struct clk_hw *hw)
{
#if (defined(CONFIG_BCM7445A0) || defined(CONFIG_BCM7445B0))
	/* Configure MoCA PLL to 3600MHz */
	BDEV_WR_F_RB(CLKGEN_PLL_MOCA_PLL_RESET, RESETA, 1);
	BDEV_WR_F_RB(CLKGEN_PLL_MOCA_PLL_DIV, PDIV, 3);
	BDEV_WR_F_RB(CLKGEN_PLL_MOCA_PLL_DIV, NDIV_INT, 200);
	BDEV_WR_F_RB(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL_CH_0, MDIV_CH0, 9);
	BDEV_WR_F_RB(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL_CH_1, MDIV_CH1, 12);
	BDEV_WR_F_RB(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL_CH_2, MDIV_CH2, 36);
	BDEV_WR_F_RB(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL_CH_3, MDIV_CH3, 9);
	BDEV_WR_F_RB(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL_CH_4, MDIV_CH4, 8);
	BDEV_WR_F_RB(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL_CH_5, MDIV_CH5, 36);
	BDEV_WR_F_RB(CLKGEN_PLL_MOCA_PLL_RESET, RESETA, 0);
#endif
	return 0;
}

static void __init bmoca_clk_init(void)
{
	/*
	 * MoCA clk init
	 */
	brcmstb_clk_init(brcmstb_moca_clk_table,
			 ARRAY_SIZE(brcmstb_moca_clk_table));
}

static DEFINE_SPINLOCK(lock);

static int cpu_clk_div_pos __initdata;
static int cpu_clk_div_width __initdata;

static int __init parse_cpu_clk_div_dimensions(struct device_node *np)
{
	struct property *prop;
	const __be32 *p = NULL;
	int len;
	int elem_cnt;
	const char *propname = "div-shift-width";

	prop = of_find_property(np, propname, &len);
	if (!prop) {
		pr_err("%s property undefined\n", propname);
		return -EINVAL;
	}

	elem_cnt = len / sizeof(u32);

	if (elem_cnt != 2) {
		pr_err("%s should have only 2 elements\n", propname);
		return -EINVAL;
	}

	p = of_prop_next_u32(prop, p, &cpu_clk_div_pos);
	of_prop_next_u32(prop, p, &cpu_clk_div_width);

	return 0;
}

static struct clk_div_table *cpu_clk_div_table;

static int __init parse_cpu_clk_div_table(struct device_node *np)
{
	struct property *prop;
	const __be32 *p = NULL;
	struct clk_div_table *cur_tbl_ptr;
	int len;
	int elem_cnt;
	int i;
	const char *propname = "div-table";

	prop = of_find_property(np, propname, &len);
	if (!prop) {
		pr_err("%s property undefined\n", propname);
		return -EINVAL;
	}

	elem_cnt = len / sizeof(u32);

	if (elem_cnt < 2) {
		pr_err("%s should have at least 2 elements\n", propname);
		return -EINVAL;
	}

	if ((elem_cnt % 2) != 0) {
		pr_err("%s should have even number of elements\n", propname);
		return -EINVAL;
	}

	/* need room for last sentinel entry */
	len += 2 * sizeof(u32);

	cpu_clk_div_table = kmalloc(len, GFP_KERNEL);
	if (!cpu_clk_div_table)
		return -ENOMEM;

	cur_tbl_ptr = cpu_clk_div_table;

	for (i = 0; i < elem_cnt; i += 2) {
		p = of_prop_next_u32(prop, p, &cur_tbl_ptr->val);
		p = of_prop_next_u32(prop, p, &cur_tbl_ptr->div);

		cur_tbl_ptr++;
	}

	/* last entry should be zeroed out */
	cur_tbl_ptr->val = 0;
	cur_tbl_ptr->div = 0;

	return 0;
}

static void __init cpu_clk_div_setup(struct device_node *np)
{
	struct clk *clk;
	void __iomem *reg;
	struct platform_device *pdev = NULL;
	int rc;

	pdev = of_find_device_by_node(np);
	if (!pdev) {
		pr_err("no platform_device for cpu clk divider\n");
		return;
	}

	reg = of_iomap(np, 0);
	if (!reg) {
		pr_err("unable to iomap cpu clk divider register!\n");
		return;
	}

	rc = parse_cpu_clk_div_dimensions(np);
	if (rc)
		goto err;

	rc = parse_cpu_clk_div_table(np);
	if (rc)
		goto err;

	clk = clk_register_divider_table(&pdev->dev, "cpu-clk-div",
					 of_clk_get_parent_name(np, 0), 0, reg,
					 cpu_clk_div_pos, cpu_clk_div_width,
					 0, cpu_clk_div_table, &lock);
	if (IS_ERR(clk))
		goto err;

	rc = of_clk_add_provider(np, of_clk_src_simple_get, clk);
	if (rc) {
		pr_err("error adding clock provider (%d)\n", rc);
		goto err;
	}

	return;

err:
	kfree(cpu_clk_div_table);
	cpu_clk_div_table = NULL;

	if (reg)
		iounmap(reg);
}

static const __initconst struct of_device_id brcmstb_clk_match[] = {
	{ .compatible = "fixed-clock", .data = of_fixed_clk_setup, },
	{ .compatible = "brcm,brcmstb-cpu-clk-div", .data = cpu_clk_div_setup, },
	{}
};

void __init brcmstb_clocks_init(void)
{
	/* DT-based clock config */
	of_clk_init(brcmstb_clk_match);

	/* Static clock config */
	bmoca_clk_init();
}
