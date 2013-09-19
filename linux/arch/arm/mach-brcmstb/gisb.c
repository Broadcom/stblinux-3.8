/*
 * Copyright (C) 2013 Broadcom Corporation
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

#include <linux/init.h>
#include <linux/types.h>

#include <linux/brcmstb/brcmstb.h>

#include <asm/bug.h>
#include <asm/signal.h>

#define ARB_ERR_CAP_CLR			0x7e4
#define  ARB_ERR_CAP_CLEAR		(1 << 0)
#define ARB_ERR_CAP_HI_ADDR		0x7e8
#define ARB_ERR_CAP_ADDR		0x7ec
#define ARB_ERR_CAP_DATA		0x7f0
#define ARB_ERR_CAP_STATUS		0x7f4
#define  ARB_ERR_CAP_STATUS_TIMEOUT	(1 << 12)
#define  ARB_ERR_CAP_STATUS_TEA		(1 << 11)
#define  ARB_ERR_CAP_STATUS_BS_SHIFT	(1 << 2)
#define  ARB_ERR_CAP_STATUS_BS_MASK	0x3c
#define  ARB_ERR_CAP_STATUS_WRITE	(1 << 1)
#define  ARB_ERR_CAP_STATUS_VALID	(1 << 0)

static unsigned long gisb_arb_base = BCHP_SUN_GISB_ARB_REG_START;

static int brcmstb_bus_error_handler(unsigned long addr, unsigned int fsr,
				     struct pt_regs *regs)
{
	u32 cap_status;
	unsigned long arb_addr;

	cap_status = BDEV_RD(gisb_arb_base + ARB_ERR_CAP_STATUS);

	/* Invalid captured address, bail out */
	if (!(cap_status & ARB_ERR_CAP_STATUS_VALID))
		return 1;

	/* Read the address */
	arb_addr = BDEV_RD(gisb_arb_base + ARB_ERR_CAP_ADDR) & 0xffffffff;
#ifdef CONFIG_PHYS_ADDR_T_64BIT
	arb_addr |= (u64)BDEV_RD(gisb_arb_base + ARB_ERR_CAP_HI_ADDR) << 32;
#endif
	pr_crit("%s: bus error at 0x%lx [%c]\n",
		__func__, arb_addr,
		cap_status & ARB_ERR_CAP_STATUS_WRITE ? 'W' : 'R');

	/* clear the GISB error */
	BDEV_WR(ARB_ERR_CAP_CLEAR, gisb_arb_base + ARB_ERR_CAP_CLR);

	/*
	 * If it was an imprecise abort, then we need to correct the
	 * return address to be _after_ the instruction.
	*/
	if (fsr & (1 << 10))
		regs->ARM_pc += 4;

	return 0;
}

void __init brcmstb_hook_fault_code(void)
{
	hook_fault_code(22, brcmstb_bus_error_handler, SIGBUS, 0,
			"imprecise external abort");
}
