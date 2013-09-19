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

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/clk-private.h>
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
			pr_err("clk-brcmstb: %s clk_register failed\n",
				brcmstb_clk->name);

		ret = clk_register_clkdev(clk, brcmstb_clk->name, NULL);

		if (ret)
			pr_err("clk-brcmstb: %s clk device registration failed\n",
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

void __init
__brcmstb_clk_init(void)
{
	/*
	 * MoCA clk init
	 */
	brcmstb_clk_init(brcmstb_moca_clk_table,
			 ARRAY_SIZE(brcmstb_moca_clk_table));
}
