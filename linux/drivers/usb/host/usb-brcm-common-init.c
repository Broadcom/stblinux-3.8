/*
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
 *
 */

/*
 * This module is used by both the bootloader and Linux and
 * contains USB initialization for power up and S3 resume.
 */

#if defined(_BOLT_)
#include "lib_types.h"
#include "common.h"
#include "bchp_common.h"
#include "bchp_usb_ctrl.h"
#include "timer.h"
#define msleep bolt_msleep
#define udelay bolt_usleep
#if defined(BCHP_USB1_CTRL_REG_START)
#include "bchp_usb1_ctrl.h"
#endif
#else
#include <linux/delay.h>
#include <linux/brcmstb/brcmstb.h>
#endif

#include "usb-brcm-common-init.h"

#if defined(BCHP_USB_CTRL_REG_START)
#define USB_CTRL_REG(base, reg)	(base + BCHP_USB_CTRL_##reg - \
		BCHP_USB_CTRL_SETUP)
#define USB_CTRL_MASK(reg, field) (BCHP_USB_CTRL_##reg##_##field##_MASK)
#define USB_CTRL_SHIFT(reg, field) (BCHP_USB_CTRL_##reg##_##field##_SHIFT)
#elif defined(BCHP_USB1_CTRL_REG_START)
#define USB_CTRL_REG(base, reg)	(base + BCHP_USB1_CTRL_##reg - \
		BCHP_USB1_CTRL_SETUP)
#define USB_CTRL_MASK(reg, field) (BCHP_USB1_CTRL_##reg##_##field##_MASK)
#define USB_CTRL_SHIFT(reg, field) (BCHP_USB1_CTRL_##reg##_##field##_SHIFT)
#endif

#define USB_CTRL_SET(base, reg, mask) do {		\
		DEV_SET(USB_CTRL_REG(base, reg),	\
			USB_CTRL_MASK(reg, mask));	\
	} while (0)

#define USB_CTRL_UNSET(base, reg, mask) do {		\
		DEV_UNSET(USB_CTRL_REG(base, reg),	\
			USB_CTRL_MASK(reg, mask));	\
	} while (0)

#define MDIO_USB2	0
#define MDIO_USB3	(1 << 31)

#define USB_CTRL_SETUP_CONDITIONAL_BITS (	\
		USB_CTRL_MASK(SETUP, BABO) |	\
		USB_CTRL_MASK(SETUP, FNHW) |	\
		USB_CTRL_MASK(SETUP, FNBO) |	\
		USB_CTRL_MASK(SETUP, WABO) |	\
		USB_CTRL_MASK(SETUP, IOC)  |	\
		USB_CTRL_MASK(SETUP, IPP))

#ifdef __LITTLE_ENDIAN
#define ENDIAN_SETTINGS ( \
		USB_CTRL_MASK(SETUP, BABO) |	\
		USB_CTRL_MASK(SETUP, FNHW))
#else
#define ENDIAN_SETTINGS ( \
		USB_CTRL_MASK(SETUP, FNHW) |	 \
		USB_CTRL_MASK(SETUP, FNBO) |	 \
		USB_CTRL_MASK(SETUP, WABO))
#endif

static uint32_t usb_mdio_read(uintptr_t ctrl_base, uint32_t reg, int mode)
{
	uint32_t data;

	data = (reg << 16) | mode;
	DEV_WR(USB_CTRL_REG(ctrl_base, MDIO), data);
	data |= (1 << 24);
	DEV_WR(USB_CTRL_REG(ctrl_base, MDIO), data);
	data &= ~(1 << 24);
	udelay(10);
	DEV_WR(USB_CTRL_REG(ctrl_base, MDIO), data);
	udelay(10);

	return DEV_RD(USB_CTRL_REG(ctrl_base, MDIO2)) & 0xffff;
}

static void usb_mdio_write(uintptr_t ctrl_base, uint32_t reg,
			uint32_t val, int mode)
{
	uint32_t data;

	data = (reg << 16) | val | mode;
	DEV_WR(USB_CTRL_REG(ctrl_base, MDIO), data);
	data |= (1 << 25);
	DEV_WR(USB_CTRL_REG(ctrl_base, MDIO), data);
	data &= ~(1 << 25);
	udelay(10);
	DEV_WR(USB_CTRL_REG(ctrl_base, MDIO), data);
}


static void usb_phy_ldo_fix(uintptr_t usbctrl)
{
	USB_CTRL_UNSET(usbctrl, PLL_CTL, PLL_RESETB);
	DEV_WR(USB_CTRL_REG(usbctrl, UTMI_CTL_1), 0);
	DEV_WR(USB_CTRL_REG(usbctrl, PLL_LDO_CTL),
		USB_CTRL_MASK(PLL_LDO_CTL, AFE_CORERDY_VDDC));
	USB_CTRL_SET(usbctrl, PLL_CTL, PLL_IDDQ_PWRDN);
	msleep(10);
	USB_CTRL_UNSET(usbctrl, PLL_CTL, PLL_IDDQ_PWRDN);
	USB_CTRL_SET(usbctrl, PLL_LDO_CTL, AFE_BG_PWRDWNB);
	USB_CTRL_SET(usbctrl, PLL_LDO_CTL, AFE_LDO_PWRDWNB);
	msleep(1);
	DEV_WR(USB_CTRL_REG(usbctrl, UTMI_CTL_1),
		USB_CTRL_MASK(UTMI_CTL_1, UTMI_SOFT_RESETB) |
		USB_CTRL_MASK(UTMI_CTL_1, UTMI_SOFT_RESETB_P1));
	USB_CTRL_SET(usbctrl, PLL_CTL, PLL_RESETB);
}

static void usb2_eye_fix(uintptr_t ctrl_base)
{
	/* Increase USB 2.0 TX level to meet spec requirement */
	usb_mdio_write(ctrl_base, 0x1f, 0x80a0, MDIO_USB2);
	usb_mdio_write(ctrl_base, 0x0a, 0xc6a0, MDIO_USB2);
}


static void usb3_pll_fix(uintptr_t ctrl_base)
{
	/* Set correct window for PLL lock detect */
	usb_mdio_write(ctrl_base, 0x1f, 0x8000, MDIO_USB3);
	usb_mdio_write(ctrl_base, 0x07, 0x1503, MDIO_USB3);
}


static void usb3_ssc_enable(uintptr_t ctrl_base)
{
	uint32_t val;

	/* Enable USB 3.0 TX spread spectrum */
	usb_mdio_write(ctrl_base, 0x1f, 0x8040, MDIO_USB3);
	val = usb_mdio_read(ctrl_base, 0x01, MDIO_USB3) | 3;
	usb_mdio_write(ctrl_base, 0x01, val, MDIO_USB3);
}


void brcm_usb_common_ctrl_xhci_soft_reset(uintptr_t ctrl, int on_off)
{
#if defined(BCHP_USB_CTRL_USB_PM_xhc_soft_resetb_MASK) || \
	defined(BCHP_USB1_CTRL_USB_PM_xhc_soft_resetb_MASK)
	/* Assert reset */
	if (on_off) {
		DEV_UNSET(USB_CTRL_REG(ctrl, USB_PM),
			USB_CTRL_MASK(USB_PM, xhc_soft_resetb));
	}
	/* De-assert reset */
	else {
		DEV_SET(USB_CTRL_REG(ctrl, USB_PM),
			USB_CTRL_MASK(USB_PM, xhc_soft_resetb));
	}
#else
	/* Assert reset */
	if (on_off) {
		DEV_UNSET(USB_CTRL_REG(ctrl, USB30_CTL1),
			USB_CTRL_MASK(USB30_CTL1, xhc_soft_resetb));
	}
	/* De-assert reset */
	else {
		DEV_SET(USB_CTRL_REG(ctrl, USB30_CTL1),
			USB_CTRL_MASK(USB30_CTL1, xhc_soft_resetb));
	}
#endif
}


void brcm_usb_common_ctrl_init(uintptr_t ctrl, int ioc, int ipp, int xhci)
{
	uint32_t reg;

#if defined(CONFIG_BCM7145A0)
	if (xhci) {
		/* Updating USB 3.0 PHY registers */
		/* fix PLL lock */
		usb_mdio_write(ctrl, 0x1f, 0x8000, MDIO_USB3);
		usb_mdio_write(ctrl, 0x0d, 0x0020, MDIO_USB3);
		usb_mdio_write(ctrl, 0x0e, 0x000e, MDIO_USB3);
		usb_mdio_write(ctrl, 0x12, 0x19f8, MDIO_USB3);
		usb_mdio_write(ctrl, 0x01, 0x0000, MDIO_USB3);
		usb_mdio_write(ctrl, 0x01, 0x9000, MDIO_USB3);
		usb_mdio_write(ctrl, 0x1f, 0x8060, MDIO_USB3);
		/* Tx demphasis override */
		usb_mdio_write(ctrl, 0x0c, 0x8840, MDIO_USB3);
		 /* max margin error count */
		usb_mdio_write(ctrl, 0x05, 0xff08, MDIO_USB3);
		/* deglitch params 6,3 */
		usb_mdio_write(ctrl, 0x03, 0x6302, MDIO_USB3);
	}
#endif
#if defined(CONFIG_BCM7445C0) || defined(CONFIG_BCM7439A0) || \
	defined(CONFIG_BCM7366)
	/*
	 * The PHY3_SOFT_RESETB bits default to the wrong state.
	 */
	DEV_SET(USB_CTRL_REG(ctrl, USB30_PCTL), 0x20002);
#endif

#if defined(BCHP_USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK) || \
	defined(BCHP_USB1_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK)
	/* Take USB out of power down */
	DEV_UNSET(USB_CTRL_REG(ctrl, PLL_CTL),
		USB_CTRL_MASK(PLL_CTL, PLL_IDDQ_PWRDN));
	/* 1 millisecond - for USB clocks to settle down */
	msleep(1);
#else
	/* Take USB out of power down */
	DEV_UNSET(USB_CTRL_REG(ctrl, USB_PM),
		USB_CTRL_MASK(USB_PM, USB_PWRDN));
	/* 1 millisecond - for USB clocks to settle down */
	msleep(1);
#endif

#if defined(CONFIG_BCM7445D0)
	DEV_UNSET(USB_CTRL_REG(ctrl, UTMI_CTL_1),
		USB_CTRL_MASK(UTMI_CTL_1, POWER_UP_FSM_EN_P1) |
		USB_CTRL_MASK(UTMI_CTL_1, POWER_UP_FSM_EN));
#endif

#if defined(BCHP_USB_CTRL_USB30_CTL1_usb3_ipp_MASK) || \
	defined(BCHP_USB1_CTRL_USB30_CTL1_usb3_ipp_MASK)
	/* Starting with the 7445d0, there are no longer separate 3.0
	 * versions of IOC and IPP.
	 */
	DEV_SET(USB_CTRL_REG(ctrl, USB30_CTL1),
		USB_CTRL_MASK(USB30_CTL1, usb3_ipp) |
		USB_CTRL_MASK(USB30_CTL1, usb3_ioc));
#endif

	DEV_SET(USB_CTRL_REG(ctrl, USB30_CTL1),
		USB_CTRL_MASK(USB30_CTL1, phy3_pll_seq_start));
	DEV_SET(USB_CTRL_REG(ctrl, PLL_CTL),
		USB_CTRL_MASK(PLL_CTL, PLL_SUSPEND_EN));

	usb2_eye_fix(ctrl);
	usb_phy_ldo_fix(ctrl);
	if (xhci) {
		usb3_pll_fix(ctrl);
		usb3_ssc_enable(ctrl);
	}

	/* Setup the endian bits */
	reg = DEV_RD(USB_CTRL_REG(ctrl, SETUP));
	reg &= ~USB_CTRL_SETUP_CONDITIONAL_BITS;
	reg |= ENDIAN_SETTINGS;

#if defined(CONFIG_BCM7364A0)
	/* Suppress overcurrent indication from USB30 ports */
	reg |= BCHP_USB_CTRL_SETUP_OC3_DISABLE_MASK;
#endif

	/*
	 * Make sure the the second and third memory controller
	 * interfaces are enabled.
	 */
	reg |= (USB_CTRL_MASK(SETUP, scb1_en) |
		USB_CTRL_MASK(SETUP, scb2_en));

	/* Override the default OC and PP polarity */
	if (ioc)
		reg |= USB_CTRL_MASK(SETUP, IOC);
	if (ipp)
		reg |= USB_CTRL_MASK(SETUP, IPP);
	DEV_WR(USB_CTRL_REG(ctrl, SETUP), reg);

	/* override lame bridge defaults */
	reg = DEV_RD(USB_CTRL_REG(ctrl, OBRIDGE));
	reg &= ~USB_CTRL_MASK(OBRIDGE, OBR_SEQ_EN);
	DEV_WR(USB_CTRL_REG(ctrl, OBRIDGE), reg);
	reg = DEV_RD(USB_CTRL_REG(ctrl, EBRIDGE));
	reg &= ~USB_CTRL_MASK(EBRIDGE, EBR_SEQ_EN);
	reg &= ~USB_CTRL_MASK(EBRIDGE, EBR_SCB_SIZE);
	reg |= (0x08 << USB_CTRL_SHIFT(EBRIDGE, EBR_SCB_SIZE));
	DEV_WR(USB_CTRL_REG(ctrl, EBRIDGE), reg);
}

