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

static void __maybe_unused sata_mdio_wr(void __iomem *addr, u32 port,
					u32 bank, u32 ofs, u32 msk, u32 value)
{
	u32 tmp;
	void __iomem *base = addr + ((port) * SATA_MDIO_REG_SPACE_SIZE);

	writel(bank, base + SATA_MDIO_BANK_OFFSET);
	tmp = readl(base + SATA_MDIO_REG_OFFSET(ofs));
	tmp = (tmp & msk) | value;
	writel(tmp, base + SATA_MDIO_REG_OFFSET(ofs));
}

static void sata_mdio_wr_legacy(void __iomem *addr, u32 bank, u32 ofs, u32 msk,
				u32 value)
{
	u32 tmp;
	void __iomem *base = addr + (0x8F * 4);

	writel(bank, base);
	/* Read, mask, enable */
	tmp = readl(ofs * 4 + base);
	tmp = (tmp & msk) | value;
	/* Write */
	writel(tmp, ofs * 4 + base);
}

static void brcm_sata3_phy_enable(const struct sata_brcm_pdata *pdata, int port)
{
	/* yfzhang@broadcom.com has stated that the core will only have (2)
	 * ports. Further, the RDB currently lacks documentation for these
	 * registers. So just keep a map of which port corresponds to these
	 * magic registers.
	 */
	const u32 port_to_phy_ctrl_ofs[MAX_PHY_CTRL_PORTS] = {
		SATA_TOP_CTRL_PHY_CTRL_2,
		SATA_TOP_CTRL_PHY_CTRL_4
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
		reg = readl(top_ctrl + port_to_phy_ctrl_ofs[port]);
		reg |= SATA_TOP_CTRL_PHY_GLOBAL_RESET;
		writel(reg, top_ctrl + port_to_phy_ctrl_ofs[port]);
		reg &= ~SATA_TOP_CTRL_PHY_GLOBAL_RESET;
		writel(reg, top_ctrl + port_to_phy_ctrl_ofs[port]);
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

			sata_mdio_wr(base, port, PLL_REG_BANK_0, 0x81,
				     0x00000000, PLL_SM_CTRL_0_DFLT);
			sata_mdio_wr(base, port, PLL_REG_BANK_0, 0x81,
				     0xFFFFFFFE, 0x0);
		}

		brcm_sata3_phy_enable(pdata, port);

		/* FIXME: Need SSC setup routine for new PHY */
		brcm_sata3_phy_spd_get(pdata, port);
	} else {
		/* TXPMD_control1 - enable SSC force */
		sata_mdio_wr_legacy(base, TXPMD_0_REG_BANK(port),
				    TXPMD_CONTROL1, 0xFFFFFFFC, 0x00000003);

		/* TXPMD_tx_freq_ctrl_control2 - set fixed min freq */
		sata_mdio_wr_legacy(base, TXPMD_0_REG_BANK(port),
				    TXPMD_TX_FREQ_CTRL_CONTROL2,
				    0xFFFFFC00, 0x000003DF);

		/*
		 * TXPMD_tx_freq_ctrl_control3 - set fixed max freq
		 *  If ssc_enable == 0, center frequencies
		 *  Otherwise, spread spectrum frequencies
		 */
		if (ssc_enable) {
			pr_info("SATA3: enabling SSC on port %d\n", port);
			sata_mdio_wr_legacy(base, TXPMD_0_REG_BANK(port),
					    TXPMD_TX_FREQ_CTRL_CONTROL3,
					    0xFFFFFC00, 0x00000083);
		} else {
			sata_mdio_wr_legacy(base, TXPMD_0_REG_BANK(port),
					    TXPMD_TX_FREQ_CTRL_CONTROL3,
					    0xFFFFFC00, 0x000003DF);
		}
	}

	iounmap(base);

err:
	return;
}

