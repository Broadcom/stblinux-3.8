/*
 *  sata_brcmstb.h - Broadcom SATA3 AHCI Controller Driver
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

#ifndef __SATA_BRCMSTB_H__
#define __SATA_BRCMSTB_H__

#ifdef __BIG_ENDIAN
#define DATA_ENDIAN			 2 /* AHCI->DDR inbound accesses */
#define MMIO_ENDIAN			 2 /* CPU->AHCI outbound accesses */
#else
#define DATA_ENDIAN			 0
#define MMIO_ENDIAN			 0
#endif

#define MAX_PORTS			32

#define SATA_MDIO_BANK_OFFSET		0x23C
#define SATA_MDIO_REG_OFFSET(ofs)	((ofs) * 4)
#define SATA_MDIO_REG_SPACE_SIZE	0x1000
#define SATA_MDIO_REG_LENGTH		0x1F00
#define TXPMD_0_REG_BANK(port)		(0x1A0 + ((port) * 0x10))

#define SPD_SETTING_WIDTH		4
#define SPD_SETTING_PER_U32		(32 / SPD_SETTING_WIDTH)
#define SPD_SETTING_MASK		0xF
#define SPD_SETTING_SHIFT(port)		\
	(((port) % SPD_SETTING_PER_U32) * SPD_SETTING_WIDTH)

#define SATA_TOP_CTRL_REG_LENGTH	0x24
#define SATA_TOP_CTRL_BUS_CTRL		0x4

enum sata_mdio_phy_legacy_regs {
	TXPMD_CONTROL1 = 0x81,
	TXPMD_TX_FREQ_CTRL_CONTROL1 = 0x82,
	TXPMD_TX_FREQ_CTRL_CONTROL2 = 0x83,
	TXPMD_TX_FREQ_CTRL_CONTROL3 = 0x84,
};

enum sata_mdio_phy_regs {
	PLL_REG_BANK_0 = 0x50,
};

/**
 * struct pdev_map - Doubly-linked list used to associate a struct device
 *                     its associated platform devices.
 * @node: Forward/reverse links
 * @key: A pointer to a device struct
 * @brcm_pdev: The Broadcom SATA AHCI platform_device associated with the key
 * @ahci_pdev: The AHCI platform_device associated with the key
 */
struct pdev_map {
	struct list_head node;
	struct device *key;
	struct platform_device *brcm_pdev;
	struct platform_device *ahci_pdev;
};

/*
 * struct sata_brcm_pdata - Platform data for the Broadcom SATA AHCI driver
 *
 * These fields are defined in the driver's DT binding documentation.
 */
struct sata_brcm_pdata {
	struct platform_device *ahci_pdev;
	u32 phy_generation;
	u32 phy_base_addr;
	u32 phy_enable_ssc_mask;
	u32 phy_force_spd[MAX_PORTS / SPD_SETTING_PER_U32];
	u32 top_ctrl_base_addr;
};

#endif /* __SATA_BRCMSTB_H__ */
