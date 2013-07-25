/*
 *
 * Copyright (c) 2002-2005 Broadcom Corporation
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
 *
 *
 *File Name  : bcmmii.c
 *
 *Description: Broadcom PHY/GPHY/Ethernet Switch Configuration
 *Revision:	09/25/2008, L.Sun created.
*/

#include "bcmgenet.h"
#include "bcmgenet_map.h"
#include "bcmmii.h"

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/bitops.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/brcmstb/brcmstb.h>

/* read a value from the MII */
static int bcmgenet_mii_read(struct net_device *dev, int phy_id, int location)
{
	int ret;
	struct bcmgenet_priv *priv = netdev_priv(dev);
	u32 reg;

	if (phy_id == BRCM_PHY_ID_NONE) {
		switch (location) {
		case MII_BMCR:
			return (priv->phy_speed == SPEED_1000) ?
				BMCR_FULLDPLX | BMCR_SPEED1000 :
				BMCR_FULLDPLX | BMCR_SPEED100;
		case MII_BMSR:
			return BMSR_LSTATUS;
		default:
			return 0;
		}
	}

	mutex_lock(&priv->mdio_mutex);

	bcmgenet_umac_writel(priv, (MDIO_RD | (phy_id << MDIO_PMD_SHIFT) |
			(location << MDIO_REG_SHIFT)), UMAC_MDIO_CMD);
	/* Start MDIO transaction*/
	reg = bcmgenet_umac_readl(priv, UMAC_MDIO_CMD);
	reg |= MDIO_START_BUSY;
	bcmgenet_umac_writel(priv, reg, UMAC_MDIO_CMD);
	wait_event_timeout(priv->wq,
			!(bcmgenet_umac_readl(priv, UMAC_MDIO_CMD)
				& MDIO_START_BUSY),
			HZ/100);
	ret = bcmgenet_umac_readl(priv, UMAC_MDIO_CMD);
	mutex_unlock(&priv->mdio_mutex);

	/* Don't check error codes from switches, as some of them are
	 * known to return MDIO_READ_FAIL on good transactions
	 */
	if (!priv->sw_type && (ret & MDIO_READ_FAIL)) {
		netif_dbg(priv, hw, dev, "MDIO read failure\n");
		ret = 0;
	}
	return ret & 0xffff;
}

/* write a value to the MII */
static void bcmgenet_mii_write(struct net_device *dev, int phy_id,
			int location, int val)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	u32 reg;

	if (phy_id == BRCM_PHY_ID_NONE)
		return;
	mutex_lock(&priv->mdio_mutex);
	bcmgenet_umac_writel(priv, (MDIO_WR | (phy_id << MDIO_PMD_SHIFT) |
			(location << MDIO_REG_SHIFT) | (0xffff & val)),
			UMAC_MDIO_CMD);
	reg = bcmgenet_umac_readl(priv, UMAC_MDIO_CMD);
	reg |= MDIO_START_BUSY;
	bcmgenet_umac_writel(priv, reg, UMAC_MDIO_CMD);
	wait_event_timeout(priv->wq,
			!(bcmgenet_umac_readl(priv, UMAC_MDIO_CMD) &
				MDIO_START_BUSY),
			HZ/100);
	mutex_unlock(&priv->mdio_mutex);
}

/* mii register read/modify/write helper function */
static int bcmgenet_mii_set_clr_bits(struct net_device *dev, int location,
		     int set_mask, int clr_mask)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	int phy_id = priv->phy_addr;
	int v;

	v = bcmgenet_mii_read(dev, phy_id, location);
	v &= ~clr_mask;
	v |= set_mask;
	bcmgenet_mii_write(dev, phy_id, location, v);
	return v;
}

/* setup netdev link state when PHY link status change and
 * update UMAC and RGMII block when link up
 */
void bcmgenet_mii_setup(struct net_device *dev)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	struct ethtool_cmd ecmd;
	int cur_link, prev_link;
	unsigned int val, cmd_bits;
	u32 reg;

	if (priv->phy_type == BRCM_PHY_TYPE_MOCA)
		return;

	cur_link = mii_link_ok(&priv->mii);
	prev_link = netif_carrier_ok(priv->dev);
	if (cur_link && !prev_link) {
		mii_ethtool_gset(&priv->mii, &ecmd);
		/* program UMAC and RGMII block based on established link
		 * speed, pause, and duplex.
		 * the speed set in umac->cmd tell RGMII block which clock
		 * 25MHz(100Mbps)/125MHz(1Gbps) to use for transmit.
		 * receive clock is provided by PHY.
		 */
		reg = bcmgenet_ext_readl(priv, EXT_RGMII_OOB_CTRL);
		reg &= ~OOB_DISABLE;
		reg |= RGMII_LINK;
		bcmgenet_ext_writel(priv, reg, EXT_RGMII_OOB_CTRL);

		/* speed */
		if (ecmd.speed == SPEED_1000)
			cmd_bits = UMAC_SPEED_1000;
		else if (ecmd.speed == SPEED_100)
			cmd_bits = UMAC_SPEED_100;
		else
			cmd_bits = UMAC_SPEED_10;
		cmd_bits <<= CMD_SPEED_SHIFT;

		/* duplex */
		if (ecmd.duplex != DUPLEX_FULL)
			cmd_bits |= CMD_HD_EN;

		/* pause capability */
		if (priv->phy_type == BRCM_PHY_TYPE_INT ||
		    priv->phy_type == BRCM_PHY_TYPE_EXT_MII ||
		    priv->phy_type == BRCM_PHY_TYPE_EXT_RVMII) {
			val = bcmgenet_mii_read(
				dev, priv->phy_addr, MII_LPA);
			if (!(val & LPA_PAUSE_CAP)) {
				cmd_bits |= CMD_RX_PAUSE_IGNORE;
				cmd_bits |= CMD_TX_PAUSE_IGNORE;
			}
		} else if (priv->ext_phy) { /* RGMII only */
			val = bcmgenet_mii_read(dev,
				priv->phy_addr, MII_BRCM_AUX_STAT_SUM);
			if (!(val & MII_BRCM_AUX_GPHY_RX_PAUSE))
				cmd_bits |= CMD_RX_PAUSE_IGNORE;
			if (!(val & MII_BRCM_AUX_GPHY_TX_PAUSE))
				cmd_bits |= CMD_TX_PAUSE_IGNORE;
		}

		reg = bcmgenet_umac_readl(priv, UMAC_CMD);
		reg &= ~((CMD_SPEED_MASK << CMD_SPEED_SHIFT) |
			       CMD_HD_EN |
			       CMD_RX_PAUSE_IGNORE | CMD_TX_PAUSE_IGNORE);
		reg |= cmd_bits;
		bcmgenet_umac_writel(priv, reg, UMAC_CMD);

		netif_carrier_on(priv->dev);
		netif_info(priv, link, dev,
			"link up, %d Mbps, %s duplex\n", ecmd.speed,
			ecmd.duplex == DUPLEX_FULL ? "full" : "half");
	} else if (!cur_link && prev_link) {
		netif_carrier_off(priv->dev);
		netif_info(priv, link, dev, "link down\n");
	}
}

void bcmgenet_ephy_workaround(struct net_device *dev)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	int phy_id = priv->phy_addr;

	if (priv->phy_type != BRCM_PHY_TYPE_INT)
		return;

#ifdef CONFIG_BCM7445A0
	/* increases ADC latency by 24ns */
	bcmgenet_mii_write(dev, phy_id, 0x17, 0x0038);
	bcmgenet_mii_write(dev, phy_id, 0x15, 0xAB95);
	/* increases internal 1V LDO voltage by 5% */
	bcmgenet_mii_write(dev, phy_id, 0x17, 0x2038);
	bcmgenet_mii_write(dev, phy_id, 0x15, 0xBB22);
	/* reduce RX low pass filter corner frequency */
	bcmgenet_mii_write(dev, phy_id, 0x17, 0x6038);
	bcmgenet_mii_write(dev, phy_id, 0x15, 0xFFC5);
	/* reduce RX high pass filter corner frequency */
	bcmgenet_mii_write(dev, phy_id, 0x17, 0x003a);
	bcmgenet_mii_write(dev, phy_id, 0x15, 0x2002);
	return;
#endif

	/* workarounds are only needed for 100Mbps PHYs */
	if (priv->phy_speed == SPEED_1000)
		return;

	/* workarounds are only needed for some 40nm chips, exclude
	 * GENET v1 or 60/65nm chips
	 */
	if (GENET_IS_V1(priv))
		return;

#if defined(CONFIG_BCM7342) || defined(CONFIG_BCM7468) || \
	defined(CONFIG_BCM7340) || defined(CONFIG_BCM7420)
	return;
#endif

	/* set shadow mode 2 */
	bcmgenet_mii_set_clr_bits(dev, 0x1f, 0x0004, 0x0004);

	/* Workaround for SWLINUX-2281: explicitly reset IDDQ_CLKBIAS
	 * in the Shadow 2 regset, due to power sequencing issues.
	 */
	/* set iddq_clkbias */
	bcmgenet_mii_write(dev, phy_id, 0x14, 0x0F00);
	udelay(10);
	/* reset iddq_clkbias */
	bcmgenet_mii_write(dev, phy_id, 0x14, 0x0C00);

	/* Workaround for SWLINUX-2056: fix timing issue between the ephy
	 * digital and the ephy analog blocks.  This clock inversion will
	 * inherently fix any setup and hold issue.
	 */
	bcmgenet_mii_write(dev, phy_id, 0x13, 0x7555);

	/* reset shadow mode 2 */
	bcmgenet_mii_set_clr_bits(dev, 0x1f, 0x0004, 0);
}

void bcmgenet_ephy_workaround_iddq(struct net_device *dev)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);

	/* Workaround for putting EPHY in iddq mode. */
	priv->mii.mdio_write(dev, priv->phy_addr, 0x1f, 0x008b);
	priv->mii.mdio_write(dev, priv->phy_addr, 0x10, 0x01c0);
	priv->mii.mdio_write(dev, priv->phy_addr, 0x14, 0x7000);
	priv->mii.mdio_write(dev, priv->phy_addr, 0x1f, 0x000f);
	priv->mii.mdio_write(dev, priv->phy_addr, 0x10, 0x20d0);
	priv->mii.mdio_write(dev, priv->phy_addr, 0x1f, 0x000b);
}

void bcmgenet_mii_reset(struct net_device *dev)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);

	bcmgenet_mii_write(dev, priv->phy_addr, MII_BMCR, BMCR_RESET);
	udelay(1);
	/* enable 64 clock MDIO */
	bcmgenet_mii_write(dev, priv->phy_addr, 0x1d, 0x1000);
	bcmgenet_mii_read(dev, priv->phy_addr, 0x1d);
	bcmgenet_ephy_workaround(dev);
}

int bcmgenet_mii_init(struct net_device *dev)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	u32 id_mode_dis = 0;
	char *phy_name;
	u32 reg;
	u32 port_ctrl;

	priv->mii.phy_id = priv->phy_addr;
	priv->mii.phy_id_mask = 0x1f;
	priv->mii.reg_num_mask = 0x1f;
	priv->mii.dev = dev;
	priv->mii.mdio_read = bcmgenet_mii_read;
	priv->mii.mdio_write = bcmgenet_mii_write;
	priv->mii.supports_gmii = 0;

	switch (priv->phy_type) {

	case BRCM_PHY_TYPE_INT:
		phy_name = "internal PHY";
		if (priv->phy_speed == SPEED_1000) {
			priv->mii.supports_gmii = 1;
			port_ctrl = PORT_MODE_INT_GPHY;
		} else
			port_ctrl = PORT_MODE_INT_EPHY;
		bcmgenet_sys_writel(priv, port_ctrl, SYS_PORT_CTRL);
		/* enable APD */
		reg = bcmgenet_ext_readl(priv, EXT_EXT_PWR_MGMT);
		reg |= EXT_PWR_DN_EN_LD;
		bcmgenet_ext_writel(priv, reg, EXT_EXT_PWR_MGMT);
		bcmgenet_mii_reset(dev);
		break;
	case BRCM_PHY_TYPE_EXT_MII:
		phy_name = "external MII";
		bcmgenet_sys_writel(priv,
				PORT_MODE_EXT_EPHY, SYS_PORT_CTRL);
		break;
	case BRCM_PHY_TYPE_EXT_RVMII:
		phy_name = "external RvMII";
		if (priv->phy_speed == SPEED_100)
			port_ctrl = PORT_MODE_EXT_RVMII_25;
		else
			port_ctrl = PORT_MODE_EXT_RVMII_50;
		bcmgenet_sys_writel(priv, port_ctrl, SYS_PORT_CTRL);
		break;
	case BRCM_PHY_TYPE_EXT_RGMII_NO_ID:
		/*
		 * RGMII_NO_ID: TXC transitions at the same time as TXD
		 *              (requires PCB or receiver-side delay)
		 * RGMII:       Add 2ns delay on TXC (90 degree shift)
		 *
		 * ID is implicitly disabled for 100Mbps (RG)MII operation.
		 */
		id_mode_dis = BIT(16);
		/* fall through */
	case BRCM_PHY_TYPE_EXT_RGMII:
		phy_name = "external RGMII";
		reg = bcmgenet_ext_readl(priv, EXT_RGMII_OOB_CTRL);
		reg |= RGMII_MODE_EN | id_mode_dis;
		bcmgenet_ext_writel(priv, reg, EXT_RGMII_OOB_CTRL);
		bcmgenet_sys_writel(priv,
				PORT_MODE_EXT_GPHY, SYS_PORT_CTRL);
		/*
		 * setup mii based on configure speed and RGMII txclk is set in
		 * umac->cmd, mii_setup() after link established.
		 */
		if (priv->phy_speed == SPEED_1000) {
			priv->mii.supports_gmii = 1;
		} else if (priv->phy_speed == SPEED_100) {
			/* disable 1000BASE-T full, half-duplex capability */
			bcmgenet_mii_set_clr_bits(dev, MII_CTRL1000, 0,
				(ADVERTISE_1000FULL|ADVERTISE_1000HALF));
			/* restart autoneg */
			bcmgenet_mii_set_clr_bits(dev, MII_BMCR, BMCR_ANRESTART,
				BMCR_ANRESTART);
		}
		break;
	case BRCM_PHY_TYPE_MOCA:
		phy_name = "MoCA";
		/* setup speed in umac->cmd for RGMII txclk set to 125MHz */
		reg = bcmgenet_umac_readl(priv, UMAC_CMD);
		reg |= (UMAC_SPEED_1000 << CMD_SPEED_SHIFT);
		bcmgenet_umac_writel(priv, reg, UMAC_CMD);
		priv->mii.force_media = 1;
		bcmgenet_sys_writel(priv,
			PORT_MODE_INT_GPHY | LED_ACT_SOURCE_MAC, SYS_PORT_CTRL);
		break;
	default:
		netdev_err(dev, "unknown phy_type: %d\n", priv->phy_type);
		return -1;
	}

	netdev_info(dev, "configuring instance for %s\n", phy_name);

	return 0;
}
