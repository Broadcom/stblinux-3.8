/*
 *  sata_brcmstb_phy.c - Broadcom SATA3 AHCI Controller PHY Driver
 *
 *  Copyright (C) 2009 - 2013 Broadcom Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#define pr_fmt(fmt) "brcm-sata3-phy: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/libata.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/ahci_platform.h>
#include <linux/compiler.h>
#include <scsi/scsi_host.h>
#include <linux/brcmstb/brcmstb.h>

#include "sata_brcmstb.h"
#include "ahci.h"

static void sata_mdio_wr_28nm(void __iomem *addr, u32 port, u32 bank, u32 ofs,
			      u32 msk, u32 value)
{
	u32 tmp;
	void __iomem *base = addr + (port * SATA_MDIO_REG_SPACE_SIZE);

	writel(bank, base + SATA_MDIO_BANK_OFFSET);
	tmp = readl(base + SATA_MDIO_REG_OFFSET(ofs));
	tmp = (tmp & msk) | value;
	writel(tmp, base + SATA_MDIO_REG_OFFSET(ofs));
}

static void sata_mdio_wr_legacy(void __iomem *addr, u32 port, u32 bank, u32 ofs,
				u32 msk, u32 value)
{
	u32 tmp;
	u32 bank_port = bank + (port * SATA_MDIO_REG_LEGACY_BANK_OFS);

	writel(bank_port, addr + SATA_MDIO_BANK_OFFSET);
	tmp = readl(addr + SATA_MDIO_REG_OFFSET(ofs));
	tmp = (tmp & msk) | value;
	writel(tmp, addr + SATA_MDIO_REG_OFFSET(ofs));
}

/* These defaults were characterized by H/W group */
#define FMIN_VAL_DEFAULT 0x3df
#define FMAX_VAL_DEFAULT 0x3df
#define FMAX_VAL_SSC 0x83

static void cfg_ssc_28nm(void __iomem *base, int port, int ssc_en)
{
	u32 tmp;

	/* override the TX spread spectrum setting */
	tmp = TXPMD_CONTROL1_TX_SSC_EN_FRC_VAL | TXPMD_CONTROL1_TX_SSC_EN_FRC;
	sata_mdio_wr_28nm(base, port, TXPMD_REG_BANK, TXPMD_CONTROL1, ~tmp,
		tmp);

	/* set fixed min freq */
	sata_mdio_wr_28nm(base, port, TXPMD_REG_BANK,
		TXPMD_TX_FREQ_CTRL_CONTROL2,
		~TXPMD_TX_FREQ_CTRL_CONTROL2_FMIN_MASK,
		FMIN_VAL_DEFAULT);

	/* set fixed max freq depending on SSC config */
	if (ssc_en) {
		pr_info("Enabling SSC on port %d\n", port);
		tmp = FMAX_VAL_SSC;
	} else
		tmp = FMAX_VAL_DEFAULT;

	sata_mdio_wr_28nm(base, port, TXPMD_REG_BANK,
		TXPMD_TX_FREQ_CTRL_CONTROL3,
		~TXPMD_TX_FREQ_CTRL_CONTROL3_FMAX_MASK, tmp);
}

static void cfg_ssc_legacy(void __iomem *base, int port, int ssc_en)
{
	u32 tmp;

	/* override the TX spread spectrum setting */
	tmp = TXPMD_CONTROL1_TX_SSC_EN_FRC_VAL | TXPMD_CONTROL1_TX_SSC_EN_FRC;
	sata_mdio_wr_legacy(base, port, TXPMD_REG_BANK_LEGACY, TXPMD_CONTROL1,
		~tmp, tmp);

	/* set fixed min freq */
	sata_mdio_wr_legacy(base, port, TXPMD_REG_BANK_LEGACY,
		TXPMD_TX_FREQ_CTRL_CONTROL2,
		~TXPMD_TX_FREQ_CTRL_CONTROL2_FMIN_MASK,
		FMIN_VAL_DEFAULT);

	/* set fixed max freq depending on SSC config */
	if (ssc_en) {
		pr_info("Enabling SSC on port %d\n", port);
		tmp = FMAX_VAL_SSC;
	} else
		tmp = FMAX_VAL_DEFAULT;

	sata_mdio_wr_legacy(base, port, TXPMD_REG_BANK_LEGACY,
		TXPMD_TX_FREQ_CTRL_CONTROL3,
		~TXPMD_TX_FREQ_CTRL_CONTROL3_FMAX_MASK, tmp);
}

static struct sata_phy_cfg_ops cfg_op_tbl[SATA_PHY_MDIO_END] = {
	[SATA_PHY_MDIO_LEGACY] = {
		.cfg_ssc = cfg_ssc_legacy,
	},
	[SATA_PHY_MDIO_28NM] = {
		.cfg_ssc = cfg_ssc_28nm,
	},
};

static struct sata_phy_cfg_ops *cfg_op;

int brcm_sata3_phy_spd_get(const struct sata_brcm_pdata *pdata, int port)
{
	int val = (pdata->phy_force_spd[port / SPD_SETTING_PER_U32]
		   >> SPD_SETTING_SHIFT(port));

	return val & SPD_SETTING_MASK;
}

void brcm_sata3_phy_spd_set(struct sata_brcm_pdata *pdata, int port, int val)
{
	int tmp = pdata->phy_force_spd[port / SPD_SETTING_PER_U32];

	pr_debug("Forcing port %d to gen %d speed\n", port, val);

	tmp &= ~(SPD_SETTING_MASK << SPD_SETTING_SHIFT(port));
	tmp |= (val & SPD_SETTING_MASK) << SPD_SETTING_SHIFT(port);
	pdata->phy_force_spd[port / SPD_SETTING_WIDTH] = tmp;
}

static void brcm_sata3_phy_enable(const struct sata_brcm_pdata *pdata, int port)
{
	/* yfzhang@broadcom.com has stated that the core will only have (2)
	 * ports. Further, the RDB currently lacks documentation for these
	 * registers. So just keep a map of which port corresponds to these
	 * magic registers.
	 */
	const u32 port_to_phy_ctrl_ofs[MAX_PHY_CTRL_PORTS] = {
		SATA_TOP_CTRL_PHY_CTRL_OFS + (0 * SATA_TOP_CTRL_PHY_CTRL_LEN),
		SATA_TOP_CTRL_PHY_CTRL_OFS + (1 * SATA_TOP_CTRL_PHY_CTRL_LEN),
	};
	void __iomem *top_ctrl;
	u32 reg;

	if (pdata->quirks & SATA_BRCM_QK_ALT_RST) {
#if (defined(CONFIG_BRCMSTB) && \
defined(BCHP_SUN_TOP_CTRL_GENERAL_CTRL_0_sata_phy_disable_MASK))
		/*
		 * This version of the chip placed the reset bit in a non-SATA
		 * IP register.
		 */
		BDEV_WR_F(SUN_TOP_CTRL_GENERAL_CTRL_0, sata_phy_disable, 0);
#else
		pr_err("Unable to handle quirk SATA_BRCM_QK_ALT_RST\n");
#endif
	}

	top_ctrl = ioremap(pdata->top_ctrl_base_addr, SATA_TOP_CTRL_REG_LENGTH);
	if (!top_ctrl) {
		pr_err("failed to ioremap SATA top ctrl regs\n");
		return;
	}

	if (port < MAX_PHY_CTRL_PORTS) {
		void __iomem *p;

		/* clear PHY_DEFAULT_POWER_STATE */
		p = top_ctrl + port_to_phy_ctrl_ofs[port] +
			SATA_TOP_CTRL_PHY_CTRL_1;
		reg = readl(p);
		reg &= ~SATA_TOP_CTRL_2_PHY_DEFAULT_POWER_STATE;
		writel(reg, p);

		/* toggle PHY_GLOBAL_RST */
		p = top_ctrl + port_to_phy_ctrl_ofs[port] +
			SATA_TOP_CTRL_PHY_CTRL_2;
		reg = readl(p);
		reg |= SATA_TOP_CTRL_1_PHY_GLOBAL_RESET;
		writel(reg, p);
		reg &= ~SATA_TOP_CTRL_1_PHY_GLOBAL_RESET;
		writel(reg, p);
	}

	iounmap(top_ctrl);
}

void brcm_sata3_phy_init(const struct sata_brcm_pdata *pdata, int port)
{
	const u32 phy_base = pdata->phy_base_addr;
	const int ssc_enable = pdata->phy_enable_ssc_mask & (1 << port);
	void __iomem *base;

	base = ioremap(phy_base, SATA_MDIO_REG_LENGTH);
	if (!base) {
		pr_err("%s: Failed to ioremap PHY registers!\n", __func__);
		goto err;
	}

	if (pdata->phy_generation == 0x2800) {
		cfg_op = &cfg_op_tbl[SATA_PHY_MDIO_28NM];

		if (pdata->quirks & SATA_BRCM_QK_INIT_PHY) {
			/* The 28nm SATA PHY has incorrect PLL settings upon
			 * chip reset.
			 * The workaround suggested by the H/W team requires
			 * initialization of the PLL registers in order to force
			 * calibration.
			 *
			 * For more information, refer to: HWxxxx
			 */
			const u32 PLL_SM_CTRL_0_DFLT = 0x3089;

			sata_mdio_wr_28nm(base, port, PLL_REG_BANK_0,
				PLL_REG_BANK_0_PLLCONTROL_0,
				0x00000000, PLL_SM_CTRL_0_DFLT);
			sata_mdio_wr_28nm(base, port, PLL_REG_BANK_0,
				PLL_REG_BANK_0_PLLCONTROL_0,
				0xfffffffe, 0x0);
		}

		brcm_sata3_phy_enable(pdata, port);
	} else
		cfg_op = &cfg_op_tbl[SATA_PHY_MDIO_LEGACY];

	if (cfg_op->cfg_ssc)
		cfg_op->cfg_ssc(base, port, ssc_enable);

	iounmap(base);

err:
	return;
}

