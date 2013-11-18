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
#include <linux/phy.h>
#include <linux/phy_fixed.h>
#include <linux/brcmphy.h>

/* read a value from the MII */
static int bcmgenet_mii_read(struct mii_bus *bus, int phy_id, int location)
{
	int ret;
	struct net_device *dev = bus->priv;
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
static int bcmgenet_mii_write(struct mii_bus *bus, int phy_id,
			int location, u16 val)
{
	struct net_device *dev = bus->priv;
	struct bcmgenet_priv *priv = netdev_priv(dev);
	u32 reg;

	if (phy_id == BRCM_PHY_ID_NONE)
		return 0;
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

	return 0;
}

/* mii register read/modify/write helper function */
static int bcmgenet_mii_set_clr_bits(struct net_device *dev, int location,
		     int set_mask, int clr_mask)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	int phy_id = priv->phy_addr;
	int v;

	v = bcmgenet_mii_read(priv->mii_bus, phy_id, location);
	v &= ~clr_mask;
	v |= set_mask;
	bcmgenet_mii_write(priv->mii_bus, phy_id, location, v);
	return v;
}

/* setup netdev link state when PHY link status change and
 * update UMAC and RGMII block when link up
 */
static void bcmgenet_mii_setup(struct net_device *dev)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	struct phy_device *phydev = priv->phydev;
	unsigned int val, cmd_bits = 0;
	u32 reg;
	unsigned int status_changed = 0;

	if (priv->old_link != phydev->link) {
		status_changed = 1;
		priv->old_link = phydev->link;
	}

	if (phydev->link) {
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
		if (phydev->speed == SPEED_1000)
			cmd_bits = UMAC_SPEED_1000;
		else if (phydev->speed == SPEED_100)
			cmd_bits = UMAC_SPEED_100;
		else
			cmd_bits = UMAC_SPEED_10;
		cmd_bits <<= CMD_SPEED_SHIFT;

		if (priv->old_duplex != phydev->duplex) {
			status_changed = 1;
			priv->old_duplex = phydev->duplex;
		}

		/* duplex */
		if (phydev->duplex != DUPLEX_FULL)
			cmd_bits |= CMD_HD_EN;

		if (priv->old_pause != phydev->pause) {
			status_changed = 1;
			priv->old_pause = phydev->pause;
		}

		/* pause capability */
		if ((phy_is_internal(priv->phydev) ||
			priv->phydev->interface == PHY_INTERFACE_MODE_MII ||
			priv->phydev->interface == PHY_INTERFACE_MODE_REVMII)
			&& !phydev->pause) {
			cmd_bits |= CMD_RX_PAUSE_IGNORE;
			cmd_bits |= CMD_TX_PAUSE_IGNORE;
		} else if (priv->ext_phy) { /* RGMII only */
			val = bcmgenet_mii_read(priv->mii_bus,
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
	}

	if (status_changed)
		phy_print_status(phydev);
}

void bcmgenet_mii_reset(struct net_device *dev)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);

	bcmgenet_mii_write(priv->mii_bus, priv->phy_addr, MII_BMCR, BMCR_RESET);
	udelay(1);
	/* enable 64 clock MDIO */
	bcmgenet_mii_write(priv->mii_bus, priv->phy_addr, 0x1d, 0x1000);
	bcmgenet_mii_read(priv->mii_bus, priv->phy_addr, 0x1d);

	/* call the PHY driver specific init routine */
	if (priv->phydev && priv->phydev->drv)
		priv->phydev->drv->config_init(priv->phydev);
}

/* 7366a0 EXT GPHY block comes with the CFG_IDDQ_BIAS and CFG_EXT_PWR_DOWN
 * bits set to 1 at reset, they need to be cleared. A reset must also be
 * issued. An initial reset value of 1500 micro seconds was not enough, 2000
 * micro seconds always works. The post-reset delay of 20 micro seconds could
 * be eliminated but better be safe than sorry.
 */
static void bcmgenet_ephy_power_up(struct net_device *dev)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	u32 reg = 0;

	/* EXT_GPHY_CTRL is only valid for GENETv4 and onward */
	if (!GENET_IS_V4(priv))
		return;

	reg = bcmgenet_ext_readl(priv, EXT_GPHY_CTRL);
	reg &= ~(EXT_CFG_IDDQ_BIAS | EXT_CFG_PWR_DOWN);
	reg |= EXT_GPHY_RESET;
	bcmgenet_ext_writel(priv, reg, EXT_GPHY_CTRL);
	mdelay(2);

	reg &= ~EXT_GPHY_RESET;
	bcmgenet_ext_writel(priv, reg, EXT_GPHY_CTRL);
	udelay(20);
}

static int bcmgenet_mii_probe(struct net_device *dev)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	struct phy_device *phydev;
	char phy_id[MII_BUS_ID_SIZE + 3];
	const char *fixed_bus = NULL;
	int phy_addr = priv->phy_addr;
	unsigned int phy_flags;

	if (priv->phydev) {
		pr_info("PHY already attached\n");
		return 0;
	}

	/* Bind to fixed-0 for MOCA and switches */
	if (priv->phy_type == BRCM_PHY_TYPE_MOCA)
		fixed_bus = "fixed-0";

	/* Connect either on the real MII bus or on the fixed one */
	snprintf(phy_id, sizeof(phy_id), PHY_ID_FMT,
		fixed_bus ? fixed_bus : priv->mii_bus->id,
		phy_addr);

	phy_flags = PHY_BRCM_100MBPS_WAR;

	/* workarounds are only needed for 100Mbps PHYs */
	if (priv->phy_speed == SPEED_1000)
		phy_flags = 0;

	/* workarounds are only needed for some 40nm chips, exclude
	 * GENET v1 or 60/65nm chips
	 */
	if (GENET_IS_V1(priv))
		phy_flags = 0;

#if defined(CONFIG_BCM7342) || defined(CONFIG_BCM7468) || \
	defined(CONFIG_BCM7340) || defined(CONFIG_BCM7420)
	phy_flags = 0;
#endif

	phydev = phy_connect(dev, phy_id, bcmgenet_mii_setup, phy_flags,
			priv->phy_interface);
	if (IS_ERR(phydev)) {
		pr_err("could not attach to PHY at %d\n", phy_addr);
		return PTR_ERR(phydev);
	}

	phydev->supported &= priv->phy_supported;
	phydev->advertising = phydev->supported;

	pr_info("attached PHY at address %d [%s]\n",
			phydev->addr, phydev->drv->name);

	priv->old_link = -1;
	priv->old_duplex = -1;
	priv->old_pause = -1;
	priv->phydev = phydev;

	return 0;
}

static int bcmgenet_mii_alloc(struct bcmgenet_priv *priv)
{
	struct mii_bus *bus;
	int ret = 0;

	if (priv->mii_bus)
		return 0;

	priv->mii_bus = mdiobus_alloc();
	if (!priv->mii_bus) {
		pr_err("failed to allocate\n");
		return -ENOMEM;
	}

	bus = priv->mii_bus;
	bus->priv = priv->dev;
	bus->name = "bcmgenet MII bus";
	bus->parent = &priv->pdev->dev;
	bus->read = bcmgenet_mii_read;
	bus->write = bcmgenet_mii_write;
	bus->phy_mask = ~(1 << priv->phy_addr);
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-%d",
			priv->pdev->name, priv->pdev->id);

	bus->irq = kzalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
	if (!bus->irq) {
		ret = -ENOMEM;
		goto out_mdio_free;
	}

	/* This is the correct thing to do, but libphy needs fixing
	 * with respect to ignoring interrupts
	 */
	if (priv->phy_type == BRCM_PHY_TYPE_INT)
		bus->irq[priv->phy_addr] = PHY_IGNORE_INTERRUPT;
	else
		bus->irq[priv->phy_addr] = PHY_POLL;

	ret = mdiobus_register(bus);
	if (ret)
		goto out_mdio_irq_free;

	/* default to FastEthernet PHY features */
	priv->phy_supported = PHY_BASIC_FEATURES;

	return 0;

out_mdio_irq_free:
	kfree(priv->mii_bus->irq);
out_mdio_free:
	mdiobus_free(priv->mii_bus);
	return ret;
}

static void bcmgenet_mii_free(struct bcmgenet_priv *priv)
{
	mdiobus_unregister(priv->mii_bus);
	kfree(priv->mii_bus->irq);
	mdiobus_free(priv->mii_bus);
}

int bcmgenet_mii_init(struct net_device *dev)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	u32 id_mode_dis = 0;
	char *phy_name;
	u32 reg;
	u32 port_ctrl;
	int ret;

	ret = bcmgenet_mii_alloc(priv);
	if (ret)
		return ret;

	switch (priv->phy_type) {

	case BRCM_PHY_TYPE_INT:
		phy_name = "internal PHY";
		if (priv->phy_speed == SPEED_1000) {
			priv->phy_interface = PHY_INTERFACE_MODE_GMII;
			priv->phy_supported = PHY_GBIT_FEATURES;
			port_ctrl = PORT_MODE_INT_GPHY;
		} else {
			priv->phy_interface = PHY_INTERFACE_MODE_MII;
			port_ctrl = PORT_MODE_INT_EPHY;
		}
		bcmgenet_sys_writel(priv, port_ctrl, SYS_PORT_CTRL);
		/* Power up EPHY */
		bcmgenet_ephy_power_up(dev);
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
		priv->phy_interface = PHY_INTERFACE_MODE_MII;
		break;
	case BRCM_PHY_TYPE_EXT_RVMII:
		phy_name = "external RvMII";
		if (priv->phy_speed == SPEED_100)
			port_ctrl = PORT_MODE_EXT_RVMII_25;
		else
			port_ctrl = PORT_MODE_EXT_RVMII_50;
		bcmgenet_sys_writel(priv, port_ctrl, SYS_PORT_CTRL);
		priv->phy_interface = PHY_INTERFACE_MODE_REVMII;
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
		priv->phy_interface = PHY_INTERFACE_MODE_RGMII;
		if (priv->phy_speed == SPEED_1000) {
			priv->phy_supported = PHY_GBIT_FEATURES;
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
		bcmgenet_sys_writel(priv,
			PORT_MODE_INT_GPHY | LED_ACT_SOURCE_MAC, SYS_PORT_CTRL);
		priv->phy_interface = PHY_INTERFACE_MODE_RGMII;
		priv->phy_supported = PHY_GBIT_FEATURES;
		priv->phy_addr = 0;
		break;
	default:
		netdev_err(dev, "unknown phy_type: %d\n", priv->phy_type);
		return -1;
	}

	netdev_info(dev, "configuring instance for %s\n", phy_name);

	ret = bcmgenet_mii_probe(dev);
	if (ret)
		bcmgenet_mii_free(priv);

	return ret;
}

void bcmgenet_mii_exit(struct net_device *dev)
{
	bcmgenet_mii_free(netdev_priv(dev));
}
