/*
 * Broadcom BCM7xxx internal transceivers support.
 *
 * Copyright (C) 2013, Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/phy.h>
#include <linux/delay.h>
#include <linux/mdio.h>
#include <linux/brcmphy.h>
#include <linux/clk.h>

struct bcm7xxx_phy_priv {
	struct clk	*clk;
};

#if defined(CONFIG_BCM7445A0) || defined(CONFIG_BCM7445B0)
static int bcm7445a0b0_config_init(struct phy_device *phydev)
{
	int ret;
	const struct bcm7445_regs {
		int reg;
		u16 value;
	} bcm7445_regs_cfg[] = {
		/* increases ADC latency by 24ns */
		{ 0x17, 0x0038 },
		{ 0x15, 0xAB95 },
		/* increases internal 1V LDO voltage by 5% */
		{ 0x17, 0x2038 },
		{ 0x15, 0xBB22 },
		/* reduce RX low pass filter corner frequency */
		{ 0x17, 0x6038 },
		{ 0x15, 0xFFC5 },
		/* reduce RX high pass filter corner frequency */
		{ 0x17, 0x003a },
		{ 0x15, 0x2002 },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(bcm7445_regs_cfg); i++) {
		ret = phy_write(phydev,
				bcm7445_regs_cfg[i].reg,
				bcm7445_regs_cfg[i].value);
		if (ret)
			return ret;
	}

	return 0;
}
#else
static inline int bcm7445a0b0_config_init(struct phy_device *phydev)
{
	return 0;
}
#endif /* CONFIG_BCM7445A0 || CONFIG_BCM7445B0 */

static int bcm7xxx_apd_enable(struct phy_device *phydev)
{
	int val;

	/* Enable powering down of the DLL during auto-power down */
	val = bcm54xx_shadow_read(phydev, BCM54XX_SHD_SCR3);
	if (val < 0)
		return val;

	val |= BCM54XX_SHD_SCR3_DLLAPD_DIS;
	bcm54xx_shadow_write(phydev, BCM54XX_SHD_SCR3, val);

	/* Enable auto-power down */
	val = bcm54xx_shadow_read(phydev, BCM54XX_SHD_APD);
	if (val < 0)
		return val;

	val |= BCM54XX_SHD_APD_EN;
	return bcm54xx_shadow_write(phydev, BCM54XX_SHD_APD, val);
}

static int bcm7xxx_eee_enable(struct phy_device *phydev)
{
	int val;

	val = phy_read_mmd_indirect(phydev->bus, CL45VEN_EEE_CONTROL,
				    MDIO_MMD_AN, phydev->addr);
	if (val < 0)
		return val;

	/* Enable general EEE feature at the PHY level */
	val |= LPI_FEATURE_EN | LPI_FEATURE_EN_DIG1000X;

	phy_write_mmd_indirect(phydev->bus, CL45VEN_EEE_CONTROL,
			       MDIO_MMD_AN, phydev->addr, val);

	/* Advertise supported modes */
	val = phy_read_mmd_indirect(phydev->bus, MDIO_AN_EEE_ADV,
				    MDIO_MMD_AN, phydev->addr);

	val |= (MDIO_AN_EEE_ADV_100TX | MDIO_AN_EEE_ADV_1000T);
	phy_write_mmd_indirect(phydev->bus, MDIO_AN_EEE_ADV,
			       MDIO_MMD_AN, phydev->addr, val);

	return 0;
}

#if defined(CONFIG_BCM7439A0) || defined(CONFIG_BCM7366A0) || \
	defined(CONFIG_BCM7445C0)

static void phy_write_exp(struct phy_device *phydev,
					u16 reg, u16 value)
{
	phy_write(phydev, 0x17, 0xf00 | reg);
	phy_write(phydev, 0x15, value);
}

static void phy_write_misc(struct phy_device *phydev,
					u16 reg, u16 chl, u16 value)
{
	int tmp;

	phy_write(phydev, 0x18, 0x7);

	tmp = phy_read(phydev, 0x18);
	tmp |= 0x800;
	phy_write(phydev, 0x18, tmp);

	tmp = (chl * 0x2000) | reg;
	phy_write(phydev, 0x17, tmp);

	phy_write(phydev, 0x15, value);
}

static int bcm7xxx_28nm_afe_config_init(struct phy_device *phydev)
{
	int ret;

	/* Increase VCO range to prevent unlocking problem of PLL at low
	 * temp
	 */
	phy_write_misc(phydev, 0x0032, 0x0001, 0x0048);

	/* Change Ki to 011 */
	phy_write_misc(phydev, 0x0032, 0x0002, 0x021b);

	/* Disable loading of TVCO buffer to bandgap, set bandgap trim
	 * to 111
	 */
	phy_write_misc(phydev, 0x0033, 0x0000, 0x0e20);

	/* Adjust bias current trim by -3 */
	phy_write_misc(phydev, 0x000a, 0x0000, 0x690b);

	/* Switch to CORE_BASE1E */
	phy_write(phydev, 0x1e, 0xd);

	/* Reset R_CAL/RC_CAL Engine */
	phy_write_exp(phydev, 0x00b0, 0x0010);

	/* Disable Reset R_CAL/RC_CAL Engine */
	phy_write_exp(phydev, 0x00b0, 0x0000);
	/* write AFE_RXCONFIG_0 */
	phy_write_misc(phydev, 0x38, 0x0000, 0xeb19);

	/* write AFE_RXCONFIG_1 */
	phy_write_misc(phydev, 0x38, 0x0001, 0x9a3f);

	/* write AFE_RX_LP_COUNTER */
	phy_write_misc(phydev, 0x38, 0x0003, 0x7fc0);

	/* write AFE_HPF_TRIM_OTHERS */
	phy_write_misc(phydev, 0x3A, 0x0000, 0x000b);

	/* write AFTE_TX_CONFIG */
	phy_write_misc(phydev, 0x39, 0x0000, 0x0800);

	ret = bcm7xxx_eee_enable(phydev);
	if (ret)
		return ret;

	return bcm7xxx_apd_enable(phydev);
}
#else
static inline int bcm7xxx_28nm_afe_config_init(struct phy_device *phydev)
{
	int ret;

	ret = bcm7xxx_eee_enable(phydev);
	if (ret)
		return ret;

	return bcm7xxx_apd_enable(phydev);
}
#endif /* CONFIG_BCM7366A0 || CONFIG_BCM7439A0 || CONFIG_BCM7445C0 */

static int bcm7xxx_28nm_config_init(struct phy_device *phydev)
{
	int ret;

	ret = bcm7445a0b0_config_init(phydev);
	if (ret)
		return ret;

	return bcm7xxx_28nm_afe_config_init(phydev);
}

static int bcm7xxx_28nm_resume(struct phy_device *phydev)
{
	int ret;

	/* Re-apply workarounds coming out of suspend/resume */
	ret = bcm7xxx_28nm_config_init(phydev);
	if (ret)
		return ret;

	/* Fixup the MII_ADVERTISE register which does not contain any half
	 * duplex modes after reset/resume
	 */
	return genphy_config_aneg(phydev);
}

static int phy_set_clr_bits(struct phy_device *dev, int location,
					int set_mask, int clr_mask)
{
	int v, ret;

	v = phy_read(dev, location);
	if (v < 0)
		return v;

	v &= ~clr_mask;
	v |= set_mask;

	ret = phy_write(dev, location, v);
	if (ret < 0)
		return ret;

	return v;
}

static int bcm7xxx_config_init(struct phy_device *phydev)
{
	/* Enable 64 clock MDIO */
	phy_write(phydev, 0x1d, 0x1000);
	phy_read(phydev, 0x1d);

	/* Workaround only required for 100Mbits/sec */
	if (!(phydev->dev_flags & PHY_BRCM_100MBPS_WAR))
		return 0;

	/* set shadow mode 2 */
	phy_set_clr_bits(phydev, 0x1f, 0x0004, 0x0004);

	/* Workaround for SWLINUX-2281: explicitly reset IDDQ_CLKBIAS
	 * in the Shadow 2 regset, due to power sequencing issues.
	 */
	/* set iddq_clkbias */
	phy_write(phydev, 0x14, 0x0F00);
	udelay(10);
	/* reset iddq_clkbias */
	phy_write(phydev, 0x14, 0x0C00);

	/* Workaround for SWLINUX-2056: fix timing issue between the ephy
	 * digital and the ephy analog blocks.  This clock inversion will
	 * inherently fix any setup and hold issue.
	 */
	phy_write(phydev, 0x13, 0x7555);

	/* reset shadow mode 2 */
	phy_set_clr_bits(phydev, 0x1f, 0x0004, 0);

	return 0;
}

/* Workaround for putting the PHY in IDDQ mode, required
 * for all BCM7XXX 40nm 65nm PHYs
 */
static int bcm7xxx_suspend(struct phy_device *phydev)
{
	int ret;
	const struct bcm7xxx_regs {
		int reg;
		u16 value;
	} bcm7xxx_suspend_cfg[] = {
		{ 0x1f, 0x008b },
		{ 0x10, 0x01c0 },
		{ 0x14, 0x7000 },
		{ 0x1f, 0x000f },
		{ 0x10, 0x20d0 },
		{ 0x1f, 0x000b },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(bcm7xxx_suspend_cfg); i++) {
		ret = phy_write(phydev,
				bcm7xxx_suspend_cfg[i].reg,
				bcm7xxx_suspend_cfg[i].value);
		if (ret)
			return ret;
	}

	return 0;
}

static int bcm7xxx_dummy_config_init(struct phy_device *phydev)
{
	return 0;
}

static int bcm7xxx_28nm_probe(struct phy_device *phydev)
{
	struct bcm7xxx_phy_priv *priv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	phydev->priv = priv;

	priv->clk = clk_get(&phydev->dev, "sw_gphy");
	if (IS_ERR(priv->clk)) {
		dev_err(&phydev->dev, "failed to request GPHY clock\n");
		priv->clk = NULL;
	}

	clk_prepare_enable(priv->clk);

	return 0;
}

static void bcm7xxx_28nm_remove(struct phy_device *phydev)
{
	struct bcm7xxx_phy_priv *priv = phydev->priv;

	clk_disable_unprepare(priv->clk);
	clk_put(priv->clk);
	kfree(priv);
	phydev->priv = NULL;
}

#define BCM7XXX_28NM_GPHY(_oui, _name) 					\
{									\
	.phy_id		= (_oui),					\
	.phy_id_mask	= 0xfffffff0,					\
	.name		= _name,					\
	.features	= PHY_GBIT_FEATURES |				\
			  SUPPORTED_Pause | SUPPORTED_Asym_Pause,	\
	.flags		= PHY_IS_INTERNAL,				\
	.config_init	= bcm7xxx_28nm_config_init,			\
	.config_aneg	= genphy_config_aneg,				\
	.probe		= bcm7xxx_28nm_probe,				\
	.read_status	= genphy_read_status,				\
	.remove		= bcm7xxx_28nm_remove,				\
	.resume		= bcm7xxx_28nm_resume,				\
	.driver		= { .owner = THIS_MODULE },			\
}

static struct phy_driver bcm7xxx_driver[] = {
	BCM7XXX_28NM_GPHY(PHY_ID_BCM7250, "Broadcom BCM7250"),
	BCM7XXX_28NM_GPHY(PHY_ID_BCM7364, "Broadcom BCM7364"),
	BCM7XXX_28NM_GPHY(PHY_ID_BCM7366, "Broadcom BCM7366"),
	BCM7XXX_28NM_GPHY(PHY_ID_BCM74371, "Broadcom BCM74371"),
	BCM7XXX_28NM_GPHY(PHY_ID_BCM7439, "Broadcom BCM7439"),
	BCM7XXX_28NM_GPHY(PHY_ID_BCM7445, "Broadcom BCM7445"),
	BCM7XXX_28NM_GPHY(PHY_ID_BCM7455, "Broadcom BCM7455"),
{
	.phy_id		= PHY_BCM_OUI_4,
	.phy_id_mask	= 0xffff0000,
	.name		= "Broadcom BCM7XXX 40nm",
	.features	= PHY_GBIT_FEATURES |
			  SUPPORTED_Pause | SUPPORTED_Asym_Pause,
	.flags		= PHY_IS_INTERNAL,
	.config_init	= bcm7xxx_config_init,
	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.suspend	= bcm7xxx_suspend,
	.resume		= bcm7xxx_config_init,
	.driver		= { .owner = THIS_MODULE },
}, {
	.phy_id		= PHY_BCM_OUI_5,
	.phy_id_mask	= 0xffffff00,
	.name		= "Broadcom BCM7XXX 65nm",
	.features	= PHY_BASIC_FEATURES |
			  SUPPORTED_Pause | SUPPORTED_Asym_Pause,
	.flags		= PHY_IS_INTERNAL,
	.config_init	= bcm7xxx_dummy_config_init,
	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.suspend	= bcm7xxx_suspend,
	.resume		= bcm7xxx_config_init,
	.driver		= { .owner = THIS_MODULE },
} };

static struct mdio_device_id __maybe_unused bcm7xxx_tbl[] = {
	{ PHY_ID_BCM7250, 0xfffffff0, },
	{ PHY_ID_BCM7364, 0xfffffff0, },
	{ PHY_ID_BCM7366, 0xfffffff0, },
	{ PHY_ID_BCM74371, 0xfffffff0, },
	{ PHY_ID_BCM7439, 0xfffffff0, },
	{ PHY_ID_BCM7445, 0xfffffff0, },
	{ PHY_ID_BCM7455, 0xfffffff0, },
	{ PHY_BCM_OUI_4, 0xffff0000 },
	{ PHY_BCM_OUI_5, 0xffffff00 },
	{ }
};

static int __init bcm7xxx_phy_init(void)
{
	return phy_drivers_register(bcm7xxx_driver,
			ARRAY_SIZE(bcm7xxx_driver));
}

static void __exit bcm7xxx_phy_exit(void)
{
	phy_drivers_unregister(bcm7xxx_driver,
			ARRAY_SIZE(bcm7xxx_driver));
}

module_init(bcm7xxx_phy_init);
module_exit(bcm7xxx_phy_exit);

MODULE_DEVICE_TABLE(mdio, bcm7xxx_tbl);

MODULE_DESCRIPTION("Broadcom BCM7xxx internal PHY driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
