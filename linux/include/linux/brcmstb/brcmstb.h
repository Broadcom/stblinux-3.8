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

#ifndef _ASM_BRCMSTB_BRCMSTB_H
#define _ASM_BRCMSTB_BRCMSTB_H

#if !defined(__ASSEMBLY__)

#include <linux/types.h>
#include <linux/smp.h>
#include <linux/device.h>

#if defined(CONFIG_MIPS)
#include <linux/brcmstb/brcmapi.h>
#include <asm/addrspace.h>
#include <asm/mipsregs.h>
#include <asm/setup.h>
#include <irq.h>
#include <spaces.h>
#endif

#endif /* !defined(__ASSEMBLY__) */

#if defined(CONFIG_MIPS)

#include <asm/bmips.h>
#define BVIRTADDR(x)		KSEG1ADDR(BPHYSADDR(x))

#else

#define BRCMSTB_PERIPH_VIRT	0xfc000000
#define BRCMSTB_PERIPH_PHYS	0xf0000000
#define BRCMSTB_PERIPH_LENGTH	0x02000000

/*
 * NOTE: for cable combo chips like 7145, this could wind up looking like:
 *
 * x = BCHP_UARTA_REG_START = 0x2040_6c00 (offset)
 * BCHP_PHYSICAL_OFFSET = 0xd000_0000 (base for UBUS registers)
 * BPHYSADDR = BCHP_PHYSICAL_OFFSET + x = 0xf040_6c00
 * BVIRTADDR = BRCMSTB_PERIPH_VIRT + (x & 0x0fffffff) = 0xfa406c00
 */
#define BVIRTADDR(x)		(BRCMSTB_PERIPH_VIRT + ((x) & 0x0fffffff))

#define BRCMRG_PERIPH_VIRT	0xe0000000
#define BRCMRG_PERIPH_PHYS	0xd0000000
#define BRCMRG_PERIPH_LENGTH	0x10000000

#endif

/***********************************************************************
 * BCHP header lists
 *
 * NOTE: This section is autogenerated.  Do not edit by hand.
 ***********************************************************************/

#if defined(CONFIG_BCM7145A0)
#include <linux/brcmstb/7145a0/bchp_aon_ctrl.h>
#include <linux/brcmstb/7145a0/bchp_aon_pin_ctrl.h>
#include <linux/brcmstb/7145a0/bchp_aon_pm_l2.h>
#include <linux/brcmstb/7145a0/bchp_bspi.h>
#include <linux/brcmstb/7145a0/bchp_bspi_raf.h>
#include <linux/brcmstb/7145a0/bchp_clkgen.h>
#include <linux/brcmstb/7145a0/bchp_common.h>
#include <linux/brcmstb/7145a0/bchp_ddr34_phy_byte_lane_0_0.h>
#include <linux/brcmstb/7145a0/bchp_ddr34_phy_byte_lane_0_1.h>
#include <linux/brcmstb/7145a0/bchp_ddr34_phy_byte_lane_1_0.h>
#include <linux/brcmstb/7145a0/bchp_ddr34_phy_byte_lane_1_1.h>
#include <linux/brcmstb/7145a0/bchp_ddr34_phy_byte_lane_2_0.h>
#include <linux/brcmstb/7145a0/bchp_ddr34_phy_byte_lane_2_1.h>
#include <linux/brcmstb/7145a0/bchp_ddr34_phy_byte_lane_3_0.h>
#include <linux/brcmstb/7145a0/bchp_ddr34_phy_byte_lane_3_1.h>
#include <linux/brcmstb/7145a0/bchp_ddr34_phy_control_regs_0.h>
#include <linux/brcmstb/7145a0/bchp_ddr34_phy_control_regs_1.h>
#include <linux/brcmstb/7145a0/bchp_ebi.h>
#include <linux/brcmstb/7145a0/bchp_fpm_ctrl_fpm.h>
#include <linux/brcmstb/7145a0/bchp_fpm_pool_fpm.h>
#include <linux/brcmstb/7145a0/bchp_gio.h>
#include <linux/brcmstb/7145a0/bchp_gio_aon.h>
#include <linux/brcmstb/7145a0/bchp_hif_continuation.h>
#include <linux/brcmstb/7145a0/bchp_hif_cpubiuctrl.h>
#include <linux/brcmstb/7145a0/bchp_hif_intr2.h>
#include <linux/brcmstb/7145a0/bchp_hif_mspi.h>
#include <linux/brcmstb/7145a0/bchp_hif_spi_intr2.h>
#include <linux/brcmstb/7145a0/bchp_hif_top_ctrl.h>
#include <linux/brcmstb/7145a0/bchp_irq0.h>
#include <linux/brcmstb/7145a0/bchp_irq1.h>
#include <linux/brcmstb/7145a0/bchp_memc_arb_1.h>
#include <linux/brcmstb/7145a0/bchp_memc_ddr_0.h>
#include <linux/brcmstb/7145a0/bchp_memc_ddr_1.h>
#include <linux/brcmstb/7145a0/bchp_memc_misc_1.h>
#include <linux/brcmstb/7145a0/bchp_moca_hostmisc.h>
#include <linux/brcmstb/7145a0/bchp_nand.h>
#include <linux/brcmstb/7145a0/bchp_pcie_0_dma.h>
#include <linux/brcmstb/7145a0/bchp_pcie_0_ext_cfg.h>
#include <linux/brcmstb/7145a0/bchp_pcie_0_intr2.h>
#include <linux/brcmstb/7145a0/bchp_pcie_0_misc.h>
#include <linux/brcmstb/7145a0/bchp_pcie_0_misc_perst.h>
#include <linux/brcmstb/7145a0/bchp_pcie_0_rc_cfg_pcie.h>
#include <linux/brcmstb/7145a0/bchp_pcie_0_rc_cfg_type1.h>
#include <linux/brcmstb/7145a0/bchp_pcie_0_rc_cfg_vendor.h>
#include <linux/brcmstb/7145a0/bchp_pcie_0_rgr1.h>
#include <linux/brcmstb/7145a0/bchp_shimphy_addr_cntl_0.h>
#include <linux/brcmstb/7145a0/bchp_shimphy_addr_cntl_1.h>
#include <linux/brcmstb/7145a0/bchp_sun_top_ctrl.h>
#include <linux/brcmstb/7145a0/bchp_switch_acb.h>
#include <linux/brcmstb/7145a0/bchp_switch_core.h>
#include <linux/brcmstb/7145a0/bchp_switch_fcb.h>
#include <linux/brcmstb/7145a0/bchp_switch_indir_rw.h>
#include <linux/brcmstb/7145a0/bchp_switch_intrl2_0.h>
#include <linux/brcmstb/7145a0/bchp_switch_intrl2_1.h>
#include <linux/brcmstb/7145a0/bchp_switch_mdio.h>
#include <linux/brcmstb/7145a0/bchp_switch_reg.h>
#include <linux/brcmstb/7145a0/bchp_wktmr.h>

#elif defined(CONFIG_BCM7366A0)
#include <linux/brcmstb/7366a0/bchp_aon_ctrl.h>
#include <linux/brcmstb/7366a0/bchp_aon_pin_ctrl.h>
#include <linux/brcmstb/7366a0/bchp_aon_pm_l2.h>
#include <linux/brcmstb/7366a0/bchp_bspi.h>
#include <linux/brcmstb/7366a0/bchp_bspi_raf.h>
#include <linux/brcmstb/7366a0/bchp_clkgen.h>
#include <linux/brcmstb/7366a0/bchp_common.h>
#include <linux/brcmstb/7366a0/bchp_ddr34_phy_byte_lane_0_0.h>
#include <linux/brcmstb/7366a0/bchp_ddr34_phy_byte_lane_1_0.h>
#include <linux/brcmstb/7366a0/bchp_ddr34_phy_byte_lane_2_0.h>
#include <linux/brcmstb/7366a0/bchp_ddr34_phy_byte_lane_3_0.h>
#include <linux/brcmstb/7366a0/bchp_ddr34_phy_control_regs_0.h>
#include <linux/brcmstb/7366a0/bchp_ebi.h>
#include <linux/brcmstb/7366a0/bchp_gio.h>
#include <linux/brcmstb/7366a0/bchp_gio_aon.h>
#include <linux/brcmstb/7366a0/bchp_hif_continuation.h>
#include <linux/brcmstb/7366a0/bchp_hif_cpubiuctrl.h>
#include <linux/brcmstb/7366a0/bchp_hif_intr2.h>
#include <linux/brcmstb/7366a0/bchp_hif_mspi.h>
#include <linux/brcmstb/7366a0/bchp_hif_spi_intr2.h>
#include <linux/brcmstb/7366a0/bchp_hif_top_ctrl.h>
#include <linux/brcmstb/7366a0/bchp_irq0.h>
#include <linux/brcmstb/7366a0/bchp_irq1.h>
#include <linux/brcmstb/7366a0/bchp_memc_ddr_0.h>
#include <linux/brcmstb/7366a0/bchp_moca_hostmisc.h>
#include <linux/brcmstb/7366a0/bchp_nand.h>
#include <linux/brcmstb/7366a0/bchp_pcie_0_dma.h>
#include <linux/brcmstb/7366a0/bchp_pcie_0_ext_cfg.h>
#include <linux/brcmstb/7366a0/bchp_pcie_0_intr2.h>
#include <linux/brcmstb/7366a0/bchp_pcie_0_misc.h>
#include <linux/brcmstb/7366a0/bchp_pcie_0_misc_perst.h>
#include <linux/brcmstb/7366a0/bchp_pcie_0_rc_cfg_pcie.h>
#include <linux/brcmstb/7366a0/bchp_pcie_0_rc_cfg_type1.h>
#include <linux/brcmstb/7366a0/bchp_pcie_0_rc_cfg_vendor.h>
#include <linux/brcmstb/7366a0/bchp_pcie_0_rgr1.h>
#include <linux/brcmstb/7366a0/bchp_shimphy_addr_cntl_0.h>
#include <linux/brcmstb/7366a0/bchp_sun_top_ctrl.h>
#include <linux/brcmstb/7366a0/bchp_wktmr.h>

#elif defined(CONFIG_BCM7439A0)
#include <linux/brcmstb/7439a0/bchp_aon_ctrl.h>
#include <linux/brcmstb/7439a0/bchp_aon_pin_ctrl.h>
#include <linux/brcmstb/7439a0/bchp_aon_pm_l2.h>
#include <linux/brcmstb/7439a0/bchp_bspi.h>
#include <linux/brcmstb/7439a0/bchp_bspi_raf.h>
#include <linux/brcmstb/7439a0/bchp_clkgen.h>
#include <linux/brcmstb/7439a0/bchp_common.h>
#include <linux/brcmstb/7439a0/bchp_ddr34_phy_byte_lane_0_0.h>
#include <linux/brcmstb/7439a0/bchp_ddr34_phy_byte_lane_1_0.h>
#include <linux/brcmstb/7439a0/bchp_ddr34_phy_byte_lane_2_0.h>
#include <linux/brcmstb/7439a0/bchp_ddr34_phy_byte_lane_3_0.h>
#include <linux/brcmstb/7439a0/bchp_ddr34_phy_control_regs_0.h>
#include <linux/brcmstb/7439a0/bchp_ebi.h>
#include <linux/brcmstb/7439a0/bchp_gio.h>
#include <linux/brcmstb/7439a0/bchp_gio_aon.h>
#include <linux/brcmstb/7439a0/bchp_hif_continuation.h>
#include <linux/brcmstb/7439a0/bchp_hif_cpubiuctrl.h>
#include <linux/brcmstb/7439a0/bchp_hif_intr2.h>
#include <linux/brcmstb/7439a0/bchp_hif_mspi.h>
#include <linux/brcmstb/7439a0/bchp_hif_spi_intr2.h>
#include <linux/brcmstb/7439a0/bchp_hif_top_ctrl.h>
#include <linux/brcmstb/7439a0/bchp_irq0.h>
#include <linux/brcmstb/7439a0/bchp_irq1.h>
#include <linux/brcmstb/7439a0/bchp_memc_ddr_0.h>
#include <linux/brcmstb/7439a0/bchp_moca_hostmisc.h>
#include <linux/brcmstb/7439a0/bchp_nand.h>
#include <linux/brcmstb/7439a0/bchp_pcie_0_dma.h>
#include <linux/brcmstb/7439a0/bchp_pcie_0_ext_cfg.h>
#include <linux/brcmstb/7439a0/bchp_pcie_0_intr2.h>
#include <linux/brcmstb/7439a0/bchp_pcie_0_misc.h>
#include <linux/brcmstb/7439a0/bchp_pcie_0_misc_perst.h>
#include <linux/brcmstb/7439a0/bchp_pcie_0_rc_cfg_pcie.h>
#include <linux/brcmstb/7439a0/bchp_pcie_0_rc_cfg_type1.h>
#include <linux/brcmstb/7439a0/bchp_pcie_0_rc_cfg_vendor.h>
#include <linux/brcmstb/7439a0/bchp_pcie_0_rgr1.h>
#include <linux/brcmstb/7439a0/bchp_shimphy_addr_cntl_0.h>
#include <linux/brcmstb/7439a0/bchp_sun_top_ctrl.h>
#include <linux/brcmstb/7439a0/bchp_wktmr.h>

#elif defined(CONFIG_BCM7445A0)
#include <linux/brcmstb/7445a0/bchp_aon_ctrl.h>
#include <linux/brcmstb/7445a0/bchp_aon_pin_ctrl.h>
#include <linux/brcmstb/7445a0/bchp_aon_pm_l2.h>
#include <linux/brcmstb/7445a0/bchp_bspi.h>
#include <linux/brcmstb/7445a0/bchp_bspi_raf.h>
#include <linux/brcmstb/7445a0/bchp_clkgen.h>
#include <linux/brcmstb/7445a0/bchp_common.h>
#include <linux/brcmstb/7445a0/bchp_ddr34_phy_byte_lane_0_0.h>
#include <linux/brcmstb/7445a0/bchp_ddr34_phy_byte_lane_0_1.h>
#include <linux/brcmstb/7445a0/bchp_ddr34_phy_byte_lane_0_2.h>
#include <linux/brcmstb/7445a0/bchp_ddr34_phy_byte_lane_1_0.h>
#include <linux/brcmstb/7445a0/bchp_ddr34_phy_byte_lane_1_1.h>
#include <linux/brcmstb/7445a0/bchp_ddr34_phy_byte_lane_1_2.h>
#include <linux/brcmstb/7445a0/bchp_ddr34_phy_byte_lane_2_0.h>
#include <linux/brcmstb/7445a0/bchp_ddr34_phy_byte_lane_2_1.h>
#include <linux/brcmstb/7445a0/bchp_ddr34_phy_byte_lane_2_2.h>
#include <linux/brcmstb/7445a0/bchp_ddr34_phy_byte_lane_3_0.h>
#include <linux/brcmstb/7445a0/bchp_ddr34_phy_byte_lane_3_1.h>
#include <linux/brcmstb/7445a0/bchp_ddr34_phy_byte_lane_3_2.h>
#include <linux/brcmstb/7445a0/bchp_ddr34_phy_control_regs_0.h>
#include <linux/brcmstb/7445a0/bchp_ddr34_phy_control_regs_1.h>
#include <linux/brcmstb/7445a0/bchp_ddr34_phy_control_regs_2.h>
#include <linux/brcmstb/7445a0/bchp_ebi.h>
#include <linux/brcmstb/7445a0/bchp_gio.h>
#include <linux/brcmstb/7445a0/bchp_gio_aon.h>
#include <linux/brcmstb/7445a0/bchp_hif_continuation.h>
#include <linux/brcmstb/7445a0/bchp_hif_cpubiuctrl.h>
#include <linux/brcmstb/7445a0/bchp_hif_intr2.h>
#include <linux/brcmstb/7445a0/bchp_hif_mspi.h>
#include <linux/brcmstb/7445a0/bchp_hif_spi_intr2.h>
#include <linux/brcmstb/7445a0/bchp_hif_top_ctrl.h>
#include <linux/brcmstb/7445a0/bchp_irq0.h>
#include <linux/brcmstb/7445a0/bchp_irq1.h>
#include <linux/brcmstb/7445a0/bchp_memc_arb_1.h>
#include <linux/brcmstb/7445a0/bchp_memc_ddr_0.h>
#include <linux/brcmstb/7445a0/bchp_memc_ddr_1.h>
#include <linux/brcmstb/7445a0/bchp_memc_ddr_2.h>
#include <linux/brcmstb/7445a0/bchp_memc_misc_1.h>
#include <linux/brcmstb/7445a0/bchp_moca_hostmisc.h>
#include <linux/brcmstb/7445a0/bchp_nand.h>
#include <linux/brcmstb/7445a0/bchp_pcie_0_dma.h>
#include <linux/brcmstb/7445a0/bchp_pcie_0_ext_cfg.h>
#include <linux/brcmstb/7445a0/bchp_pcie_0_intr2.h>
#include <linux/brcmstb/7445a0/bchp_pcie_0_misc.h>
#include <linux/brcmstb/7445a0/bchp_pcie_0_misc_perst.h>
#include <linux/brcmstb/7445a0/bchp_pcie_0_rc_cfg_pcie.h>
#include <linux/brcmstb/7445a0/bchp_pcie_0_rc_cfg_type1.h>
#include <linux/brcmstb/7445a0/bchp_pcie_0_rc_cfg_vendor.h>
#include <linux/brcmstb/7445a0/bchp_pcie_0_rgr1.h>
#include <linux/brcmstb/7445a0/bchp_shimphy_addr_cntl_0.h>
#include <linux/brcmstb/7445a0/bchp_shimphy_addr_cntl_1.h>
#include <linux/brcmstb/7445a0/bchp_shimphy_addr_cntl_2.h>
#include <linux/brcmstb/7445a0/bchp_sun_top_ctrl.h>
#include <linux/brcmstb/7445a0/bchp_wktmr.h>

#elif defined(CONFIG_BCM7445B0)
#include <linux/brcmstb/7445b0/bchp_aon_ctrl.h>
#include <linux/brcmstb/7445b0/bchp_aon_pin_ctrl.h>
#include <linux/brcmstb/7445b0/bchp_aon_pm_l2.h>
#include <linux/brcmstb/7445b0/bchp_bspi.h>
#include <linux/brcmstb/7445b0/bchp_bspi_raf.h>
#include <linux/brcmstb/7445b0/bchp_clkgen.h>
#include <linux/brcmstb/7445b0/bchp_common.h>
#include <linux/brcmstb/7445b0/bchp_ddr34_phy_byte_lane_0_0.h>
#include <linux/brcmstb/7445b0/bchp_ddr34_phy_byte_lane_0_1.h>
#include <linux/brcmstb/7445b0/bchp_ddr34_phy_byte_lane_0_2.h>
#include <linux/brcmstb/7445b0/bchp_ddr34_phy_byte_lane_1_0.h>
#include <linux/brcmstb/7445b0/bchp_ddr34_phy_byte_lane_1_1.h>
#include <linux/brcmstb/7445b0/bchp_ddr34_phy_byte_lane_1_2.h>
#include <linux/brcmstb/7445b0/bchp_ddr34_phy_byte_lane_2_0.h>
#include <linux/brcmstb/7445b0/bchp_ddr34_phy_byte_lane_2_1.h>
#include <linux/brcmstb/7445b0/bchp_ddr34_phy_byte_lane_2_2.h>
#include <linux/brcmstb/7445b0/bchp_ddr34_phy_byte_lane_3_0.h>
#include <linux/brcmstb/7445b0/bchp_ddr34_phy_byte_lane_3_1.h>
#include <linux/brcmstb/7445b0/bchp_ddr34_phy_byte_lane_3_2.h>
#include <linux/brcmstb/7445b0/bchp_ddr34_phy_control_regs_0.h>
#include <linux/brcmstb/7445b0/bchp_ddr34_phy_control_regs_1.h>
#include <linux/brcmstb/7445b0/bchp_ddr34_phy_control_regs_2.h>
#include <linux/brcmstb/7445b0/bchp_ebi.h>
#include <linux/brcmstb/7445b0/bchp_gio.h>
#include <linux/brcmstb/7445b0/bchp_gio_aon.h>
#include <linux/brcmstb/7445b0/bchp_hif_continuation.h>
#include <linux/brcmstb/7445b0/bchp_hif_cpubiuctrl.h>
#include <linux/brcmstb/7445b0/bchp_hif_intr2.h>
#include <linux/brcmstb/7445b0/bchp_hif_mspi.h>
#include <linux/brcmstb/7445b0/bchp_hif_spi_intr2.h>
#include <linux/brcmstb/7445b0/bchp_hif_top_ctrl.h>
#include <linux/brcmstb/7445b0/bchp_irq0.h>
#include <linux/brcmstb/7445b0/bchp_irq1.h>
#include <linux/brcmstb/7445b0/bchp_memc_arb_1.h>
#include <linux/brcmstb/7445b0/bchp_memc_ddr_0.h>
#include <linux/brcmstb/7445b0/bchp_memc_ddr_1.h>
#include <linux/brcmstb/7445b0/bchp_memc_ddr_2.h>
#include <linux/brcmstb/7445b0/bchp_memc_misc_1.h>
#include <linux/brcmstb/7445b0/bchp_moca_hostmisc.h>
#include <linux/brcmstb/7445b0/bchp_nand.h>
#include <linux/brcmstb/7445b0/bchp_pcie_0_dma.h>
#include <linux/brcmstb/7445b0/bchp_pcie_0_ext_cfg.h>
#include <linux/brcmstb/7445b0/bchp_pcie_0_intr2.h>
#include <linux/brcmstb/7445b0/bchp_pcie_0_misc.h>
#include <linux/brcmstb/7445b0/bchp_pcie_0_misc_perst.h>
#include <linux/brcmstb/7445b0/bchp_pcie_0_rc_cfg_pcie.h>
#include <linux/brcmstb/7445b0/bchp_pcie_0_rc_cfg_type1.h>
#include <linux/brcmstb/7445b0/bchp_pcie_0_rc_cfg_vendor.h>
#include <linux/brcmstb/7445b0/bchp_pcie_0_rgr1.h>
#include <linux/brcmstb/7445b0/bchp_shimphy_addr_cntl_0.h>
#include <linux/brcmstb/7445b0/bchp_shimphy_addr_cntl_1.h>
#include <linux/brcmstb/7445b0/bchp_shimphy_addr_cntl_2.h>
#include <linux/brcmstb/7445b0/bchp_sun_top_ctrl.h>
#include <linux/brcmstb/7445b0/bchp_wktmr.h>

#elif defined(CONFIG_BCM7445C0)
#include <linux/brcmstb/7445c0/bchp_aon_ctrl.h>
#include <linux/brcmstb/7445c0/bchp_aon_pin_ctrl.h>
#include <linux/brcmstb/7445c0/bchp_aon_pm_l2.h>
#include <linux/brcmstb/7445c0/bchp_bspi.h>
#include <linux/brcmstb/7445c0/bchp_bspi_raf.h>
#include <linux/brcmstb/7445c0/bchp_clkgen.h>
#include <linux/brcmstb/7445c0/bchp_common.h>
#include <linux/brcmstb/7445c0/bchp_ddr34_phy_byte_lane_0_0.h>
#include <linux/brcmstb/7445c0/bchp_ddr34_phy_byte_lane_0_1.h>
#include <linux/brcmstb/7445c0/bchp_ddr34_phy_byte_lane_0_2.h>
#include <linux/brcmstb/7445c0/bchp_ddr34_phy_byte_lane_1_0.h>
#include <linux/brcmstb/7445c0/bchp_ddr34_phy_byte_lane_1_1.h>
#include <linux/brcmstb/7445c0/bchp_ddr34_phy_byte_lane_1_2.h>
#include <linux/brcmstb/7445c0/bchp_ddr34_phy_byte_lane_2_0.h>
#include <linux/brcmstb/7445c0/bchp_ddr34_phy_byte_lane_2_1.h>
#include <linux/brcmstb/7445c0/bchp_ddr34_phy_byte_lane_2_2.h>
#include <linux/brcmstb/7445c0/bchp_ddr34_phy_byte_lane_3_0.h>
#include <linux/brcmstb/7445c0/bchp_ddr34_phy_byte_lane_3_1.h>
#include <linux/brcmstb/7445c0/bchp_ddr34_phy_byte_lane_3_2.h>
#include <linux/brcmstb/7445c0/bchp_ddr34_phy_control_regs_0.h>
#include <linux/brcmstb/7445c0/bchp_ddr34_phy_control_regs_1.h>
#include <linux/brcmstb/7445c0/bchp_ddr34_phy_control_regs_2.h>
#include <linux/brcmstb/7445c0/bchp_ebi.h>
#include <linux/brcmstb/7445c0/bchp_gio.h>
#include <linux/brcmstb/7445c0/bchp_gio_aon.h>
#include <linux/brcmstb/7445c0/bchp_hif_continuation.h>
#include <linux/brcmstb/7445c0/bchp_hif_cpubiuctrl.h>
#include <linux/brcmstb/7445c0/bchp_hif_intr2.h>
#include <linux/brcmstb/7445c0/bchp_hif_mspi.h>
#include <linux/brcmstb/7445c0/bchp_hif_spi_intr2.h>
#include <linux/brcmstb/7445c0/bchp_hif_top_ctrl.h>
#include <linux/brcmstb/7445c0/bchp_irq0.h>
#include <linux/brcmstb/7445c0/bchp_irq1.h>
#include <linux/brcmstb/7445c0/bchp_memc_arb_1.h>
#include <linux/brcmstb/7445c0/bchp_memc_ddr_0.h>
#include <linux/brcmstb/7445c0/bchp_memc_ddr_1.h>
#include <linux/brcmstb/7445c0/bchp_memc_ddr_2.h>
#include <linux/brcmstb/7445c0/bchp_memc_misc_1.h>
#include <linux/brcmstb/7445c0/bchp_moca_hostmisc.h>
#include <linux/brcmstb/7445c0/bchp_nand.h>
#include <linux/brcmstb/7445c0/bchp_pcie_0_dma.h>
#include <linux/brcmstb/7445c0/bchp_pcie_0_ext_cfg.h>
#include <linux/brcmstb/7445c0/bchp_pcie_0_intr2.h>
#include <linux/brcmstb/7445c0/bchp_pcie_0_misc.h>
#include <linux/brcmstb/7445c0/bchp_pcie_0_misc_perst.h>
#include <linux/brcmstb/7445c0/bchp_pcie_0_rc_cfg_pcie.h>
#include <linux/brcmstb/7445c0/bchp_pcie_0_rc_cfg_type1.h>
#include <linux/brcmstb/7445c0/bchp_pcie_0_rc_cfg_vendor.h>
#include <linux/brcmstb/7445c0/bchp_pcie_0_rgr1.h>
#include <linux/brcmstb/7445c0/bchp_shimphy_addr_cntl_0.h>
#include <linux/brcmstb/7445c0/bchp_shimphy_addr_cntl_1.h>
#include <linux/brcmstb/7445c0/bchp_shimphy_addr_cntl_2.h>
#include <linux/brcmstb/7445c0/bchp_sun_top_ctrl.h>
#include <linux/brcmstb/7445c0/bchp_wktmr.h>

#endif

#define BRCM_NMI_VEC		0x80000000
#define BRCM_WARM_RESTART_VEC	0x80000380

/* Kernel will program WKTMR to expire in 1 second */
#define BRCM_STANDBY_TEST		0x01
/* Wait 120s before entering standby, to allow registers to be read */
#define BRCM_STANDBY_DELAY		0x02
/* Show UART output at each step */
#define BRCM_STANDBY_VERBOSE		0x04
/* Don't enter standby - just delay for 5s then return */
#define BRCM_STANDBY_NO_SLEEP		0x08
/* Don't shut down MIPS PLL */
#define BRCM_STANDBY_MIPS_PLL_ON	0x10
/* Don't shut down DDR PLL */
#define BRCM_STANDBY_DDR_PLL_ON		0x20

#if defined(CONFIG_BRCM_HAS_AON)
#if defined(BCHP_AON_CTRL_SYSTEM_DATA_00)
#define AON_RAM_BASE		BCHP_AON_CTRL_SYSTEM_DATA_00
#else
#define AON_RAM_BASE		BCHP_AON_CTRL_SYSTEM_DATA_RAMi_ARRAY_BASE
#endif
#define AON_RAM(idx)		(AON_RAM_BASE + (idx << 2))
#endif

#define UPGTMR_FREQ		27000000

/***********************************************************************
 * Register access macros - sample usage:
 *
 * DEV_RD(0xb0404000)                       -> reads 0xb0404000
 * BDEV_RD(0x404000)                        -> reads 0xb0404000
 * BDEV_RD(BCHP_SUN_TOP_CTRL_PROD_REVISION) -> reads 0xb0404000
 *
 * _RB means read back after writing.
 ***********************************************************************/

#define BPHYSADDR(x)	((x) + BCHP_PHYSICAL_OFFSET)

#if !defined(__ASSEMBLY__)

#define DEV_RD(x) (*((volatile unsigned long *)(x)))
#define DEV_WR(x, y) do { *((volatile unsigned long *)(x)) = (y); } while (0)
#define DEV_UNSET(x, y) do { DEV_WR((x), DEV_RD(x) & ~(y)); } while (0)
#define DEV_SET(x, y) do { DEV_WR((x), DEV_RD(x) | (y)); } while (0)

#define DEV_WR_RB(x, y) do { DEV_WR((x), (y)); DEV_RD(x); } while (0)
#define DEV_SET_RB(x, y) do { DEV_SET((x), (y)); DEV_RD(x); } while (0)
#define DEV_UNSET_RB(x, y) do { DEV_UNSET((x), (y)); DEV_RD(x); } while (0)

#define BDEV_RD(x) (DEV_RD(BVIRTADDR(x)))
#define BDEV_WR(x, y) do { DEV_WR(BVIRTADDR(x), (y)); } while (0)
#define BDEV_UNSET(x, y) do { BDEV_WR((x), BDEV_RD(x) & ~(y)); } while (0)
#define BDEV_SET(x, y) do { BDEV_WR((x), BDEV_RD(x) | (y)); } while (0)

#define BDEV_SET_RB(x, y) do { BDEV_SET((x), (y)); BDEV_RD(x); } while (0)
#define BDEV_UNSET_RB(x, y) do { BDEV_UNSET((x), (y)); BDEV_RD(x); } while (0)
#define BDEV_WR_RB(x, y) do { BDEV_WR((x), (y)); BDEV_RD(x); } while (0)

#define BDEV_RD_F(reg, field) \
	((BDEV_RD(BCHP_##reg) & BCHP_##reg##_##field##_MASK) >> \
	 BCHP_##reg##_##field##_SHIFT)
#define BDEV_WR_F(reg, field, val) do { \
	BDEV_WR(BCHP_##reg, \
	(BDEV_RD(BCHP_##reg) & ~BCHP_##reg##_##field##_MASK) | \
	(((val) << BCHP_##reg##_##field##_SHIFT) & \
	 BCHP_##reg##_##field##_MASK)); \
	} while (0)
#define BDEV_WR_F_RB(reg, field, val) do { \
	BDEV_WR(BCHP_##reg, \
	(BDEV_RD(BCHP_##reg) & ~BCHP_##reg##_##field##_MASK) | \
	(((val) << BCHP_##reg##_##field##_SHIFT) & \
	 BCHP_##reg##_##field##_MASK)); \
	BDEV_RD(BCHP_##reg); \
	} while (0)

/***********************************************************************
 * Platform features (based on bond options or bootloader settings)
 ***********************************************************************/

extern int brcm_sata_enabled;
extern int brcm_pcie_enabled;
extern int brcm_docsis_platform;
extern int brcm_moca_enabled;
extern int brcm_usb_enabled;
extern int brcm_pm_enabled;

extern unsigned long brcm_mtd_rootfs_start;
extern unsigned long brcm_mtd_rootfs_len;
extern unsigned long brcm_mtd_kernel_start;
extern unsigned long brcm_mtd_kernel_len;
extern unsigned long brcm_mtd_ocap_start;
extern unsigned long brcm_mtd_ocap_len;
extern unsigned long brcm_mtd_flash_size_mb;

#ifdef CONFIG_BRCM_SLOW_TVM_CLOCK
#define BRCM_BASE_BAUD_TVM	(54000000 / 16)
#else
#define BRCM_BASE_BAUD_TVM	(216000000 / 16)
#endif

#define BRCM_BASE_BAUD_STB	(81000000 / 16)
#define BRCM_BASE_BAUD_PCU	(192000000 / 16)
extern unsigned long brcm_base_baud0;
extern unsigned long brcm_base_baud;

#define CFE_STRING_SIZE		64

extern char brcm_mtd_flash_type[CFE_STRING_SIZE];
extern char brcm_cfe_boardname[CFE_STRING_SIZE];

extern unsigned long brcm_moca_i2c_base;
extern unsigned long brcm_moca_rf_band;

#define BRCM_PCI_SLOTS		16

struct mtd_partition;

struct brcmnand_platform_data {
	int			chip_select;
	int			nr_parts;
	struct mtd_partition	*parts;
};

#define BRCM_FLASH_CS_NONE	-1

/***********************************************************************
 * HIF L2 IRQ controller - shared by EDU, SDIO
 ***********************************************************************/

#define HIF_ENABLE_IRQ(bit) do { \
	BDEV_WR_RB(BCHP_HIF_INTR2_CPU_MASK_CLEAR, \
		   BCHP_HIF_INTR2_CPU_MASK_CLEAR_##bit##_INTR_MASK); \
	} while (0)

#define HIF_DISABLE_IRQ(bit) do { \
	BDEV_WR_RB(BCHP_HIF_INTR2_CPU_MASK_SET, \
		   BCHP_HIF_INTR2_CPU_MASK_SET_##bit##_INTR_MASK); \
	} while (0)

#define HIF_TEST_IRQ(bit) \
	(((BDEV_RD(BCHP_HIF_INTR2_CPU_STATUS) & \
	   ~BDEV_RD(BCHP_HIF_INTR2_CPU_MASK_STATUS)) & \
	  BCHP_HIF_INTR2_CPU_STATUS_##bit##_INTR_MASK))

#define HIF_ACK_IRQ(bit) do { \
	BDEV_WR_RB(BCHP_HIF_INTR2_CPU_CLEAR, \
		   BCHP_HIF_INTR2_CPU_CLEAR_##bit##_INTR_MASK); \
	} while (0)

#define HIF_TRIGGER_IRQ(bit) do { \
	BDEV_WR_RB(BCHP_HIF_INTR2_CPU_SET, \
		   BCHP_HIF_INTR2_CPU_SET_##bit##_INTR_MASK); \
	} while (0)

/***********************************************************************
 * Internal (BSP/driver) APIs and definitions
 ***********************************************************************/

#ifdef BCHP_SUN_TOP_CTRL_CHIP_FAMILY_ID

/* 40nm chips */

#define BRCM_CHIP_ID()		({ \
	u32 reg = BDEV_RD(BCHP_SUN_TOP_CTRL_CHIP_FAMILY_ID); \
	(reg >> 28 ? reg >> 16 : reg >> 8); \
	})
#define BRCM_PROD_ID()		({ \
	u32 reg = BDEV_RD(BCHP_SUN_TOP_CTRL_PRODUCT_ID); \
	(reg >> 28 ? reg >> 16 : reg >> 8); \
	})
#define BRCM_CHIP_REV()		\
	((u32)BDEV_RD(BCHP_SUN_TOP_CTRL_CHIP_FAMILY_ID) & 0xff)

#else

/* 130nm, 65nm chips */

#define BRCM_CHIP_ID()		({ \
	u32 reg = BDEV_RD(BCHP_SUN_TOP_CTRL_PROD_REVISION); \
	(reg >> 28 ? reg >> 16 : reg >> 8); \
	})
#define BRCM_PROD_ID()		BRCM_CHIP_ID()
#define BRCM_CHIP_REV()		\
	((u32)BDEV_RD(BCHP_SUN_TOP_CTRL_PROD_REVISION) & 0xff)

#endif

#define __BMIPS_GET_CBR()              ((unsigned long)BMIPS_GET_CBR())

asmlinkage void brcm_upper_tlb_setup(void);
void board_pinmux_setup(void);
void board_get_ram_size(unsigned long *dram0_mb, unsigned long *dram1_mb);
int board_get_partition_map(struct mtd_partition **p);
void brcm_wraparound_check(void);

void ebi_restore_settings(void);

int bchip_strap_flash_type(void);
void brcmstb_cpu_setup(void);
void bchip_sata3_init(void);
void bchip_usb_init(void);
void bchip_moca_init(void);
void bchip_check_compat(void);
void bchip_set_features(void);
void bchip_early_setup(void);
void brcm_machine_restart(const char *command);
void brcm_machine_halt(void);
char *brcmstb_pcibios_setup(char *str);

void __cpuinit bmips_start_cpu_cores(void);

void cfe_die(char *fmt, ...);

ssize_t brcm_pm_show_cpu_div(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t brcm_pm_store_cpu_div(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t brcm_pm_show_cpu_pll(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t brcm_pm_store_cpu_pll(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);

ssize_t brcm_pm_show_usb_power(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t brcm_pm_store_usb_power(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t brcm_pm_show_sata_power(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t brcm_pm_store_sata_power(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t brcm_pm_show_ddr_timeout(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t brcm_pm_store_ddr_timeout(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t brcm_pm_show_standby_flags(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t brcm_pm_store_standby_flags(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t brcm_pm_show_standby_timeout(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t brcm_pm_store_standby_timeout(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t brcm_pm_show_memc1_power(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t brcm_pm_store_memc1_power(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t brcm_pm_show_halt_mode(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t brcm_pm_store_halt_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t brcm_pm_show_time_at_wakeup(struct device *dev,
	struct device_attribute *attr, char *buf);

void brcm_irq_standby_enter(int wake_irq);
void brcm_irq_standby_exit(void);

#ifdef BCHP_PM_L2_CPU_STATUS
#define TIMER_INTR_MASK		BCHP_PM_L2_CPU_STATUS_TIMER_INTR_MASK
#ifdef BCHP_PM_L2_CPU_STATUS_WOL_ENET_MASK
#define WOL_ENET_MASK		BCHP_PM_L2_CPU_STATUS_WOL_ENET_MASK
#elif defined(BCHP_PM_L2_CPU_STATUS_WOL_MPD_MASK)
#define WOL_ENET_MASK		(BCHP_PM_L2_CPU_STATUS_WOL_MPD_MASK | \
				 BCHP_PM_L2_CPU_STATUS_WOL_HFB_MASK)
#else
#define WOL_ENET_MASK		(0)
#endif
#ifdef BCHP_PM_L2_CPU_STATUS_WOL_MOCA_MASK
#define WOL_MOCA_MASK		BCHP_PM_L2_CPU_STATUS_WOL_MOCA_MASK
#else
#define WOL_MOCA_MASK		(0)
#endif
#else
#define TIMER_INTR_MASK		BCHP_AON_PM_L2_CPU_STATUS_TIMER_INTR_MASK
#define WOL_ENET_MASK		BCHP_AON_PM_L2_CPU_STATUS_WOL_ENET_MASK
#ifdef BCHP_AON_PM_L2_CPU_STATUS_WOL_MOCA_MASK
#define WOL_MOCA_MASK		BCHP_AON_PM_L2_CPU_STATUS_WOL_MOCA_MASK
#else
#ifdef CONFIG_BCM7231B0
#define WOL_MOCA_MASK		BCHP_AON_PM_L2_CPU_STATUS_WOL_ENET1_MASK
#else
#define WOL_MOCA_MASK		(0)
#endif
#endif
#endif

void brcm_pm_wakeup_source_enable(u32 mask, int enable);
int brcm_pm_wakeup_get_status(u32 mask);

asmlinkage int brcm_pm_standby_asm(int icache_linesz, unsigned long ebase,
	unsigned int vec_size, unsigned long flags);
int brcm_pm_s3_standby(int dcache_linesz, unsigned long options);
void brcm_pm_s3_cold_boot(void);

#define NUM_MEMC_CLIENTS		128
void brcm_pm_save_restore_rts(unsigned long reg_addr, u32 *data, int restore);

#define BRCM_MEM_DMA_SCRAM_NONE		0
#define BRCM_MEM_DMA_SCRAM_BLOCK	1
#define BRCM_MEM_DMA_SCRAM_MPEG		2
#define BRCM_MEM_DMA_SCRAM_DTV		3

extern int brcm_pm_hash_enabled;

struct brcm_mem_transfer;

struct brcm_mem_transfer {
	struct brcm_mem_transfer *next; /* chained transfers */
	void		*src;
	void		*dst;
	dma_addr_t	pa_src; /* remapped by or known to the caller */
	dma_addr_t	pa_dst; /* remapped by or known to the caller */
	u32		len;
	u8		key;
	u8		mode:2;
	u8		src_remapped:1;
	u8		dst_remapped:1;
	u8		src_dst_remapped:1;
};

int brcm_mem_dma_transfer(struct brcm_mem_transfer *xfer);
int brcm_mem_dma_simple_transfer(struct brcm_mem_transfer *xfer);

int brcm_pm_dram_encoder_prepare(struct brcm_mem_transfer *param);
int brcm_pm_dram_encoder_complete(struct brcm_mem_transfer *param);
void brcm_pm_dram_encoder_start(void);

struct brcm_dram_encoder_ops {
	int (*prepare)(struct brcm_mem_transfer *);
	void (*start)(void);
	int (*complete)(struct brcm_mem_transfer *);
};

void brcm_pm_set_dram_encoder(struct brcm_dram_encoder_ops *);

asmlinkage void brcm_pm_irq(void);
int brcm_pm_deep_sleep(void);

void brcm_pm_sata3(int enable);

extern unsigned long brcm_dram0_size_mb;
extern unsigned long brcm_dram1_size_mb;
extern unsigned long brcm_dram1_linux_mb;
extern unsigned long brcm_dram1_start;
extern unsigned long brcm_min_auth_region_size;

/* NMI / TP1 reset vector */
extern char brcm_reset_nmi_vec[];
extern char brcm_reset_nmi_vec_end[];

/* TP1 warm restart interrupt vector */
extern char brcm_tp1_int_vec[];
extern char brcm_tp1_int_vec_end[];

extern atomic_t brcm_unaligned_fp_count;
extern atomic_t brcm_rdhwr_count;		/* excludes rdhwr fastpath */

extern unsigned long brcm_cpu_khz;
extern unsigned long brcm_adj_cpu_khz;

#define WKTMR_FREQ		27000000
#define WKTMR_1US		(WKTMR_FREQ / 1000000)
#define WKTMR_1MS		(WKTMR_FREQ / 1000)

struct wktmr_time {
	u32			sec;
	u32			pre;
};

void wktmr_read(struct wktmr_time *t);
unsigned long wktmr_elapsed(struct wktmr_time *t);

#endif /* !defined(__ASSEMBLY__) */

#endif /* _ASM_BRCMSTB_BRCMSTB_H */