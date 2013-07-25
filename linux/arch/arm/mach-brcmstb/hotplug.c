/*
 * Broadcom STB CPU hotplug support for ARM
 *
 *  Copyright (C) 2013 Broadcom Corporation
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/printk.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/brcmstb/brcmstb.h>

#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>

#include "common.h"

#define PWR_ZONE_REG(x) BCHP_HIF_CPUBIUCTRL_CPU0_PWR_ZONE_CNTRL_REG_##x

static u32 pwr_zone_ctrl_get_base(unsigned int cpu)
{
	u32 base = BCHP_HIF_CPUBIUCTRL_CPU0_PWR_ZONE_CNTRL_REG;
	base += (cpu * 4);
	BUG_ON(base >= BCHP_HIF_CPUBIUCTRL_L2BIU_PWR_ZONE_CNTRL_REG);
	return base;
}

static u32 pwr_zone_ctrl_rd(unsigned int cpu)
{
	u32 base = pwr_zone_ctrl_get_base(cpu);
	return BDEV_RD(base);
}

static void pwr_zone_ctrl_wr(unsigned int cpu, u32 val)
{
	u32 base = pwr_zone_ctrl_get_base(cpu);
	BDEV_WR(base, val);
	dsb();
}

void brcmstb_cpu_boot(unsigned int cpu)
{
	unsigned long boot_vector;
	const int reg_ofs = cpu * 8;

	pr_info("SMP: Booting CPU%d...\n", cpu);

	/*
	* set the reset vector to point to the secondary_startup
	* routine
	*/
	boot_vector = virt_to_phys(brcmstb_secondary_startup);
	BDEV_WR(BCHP_HIF_CONTINUATION_STB_BOOT_HI_ADDR0 + reg_ofs, 0);
	BDEV_WR(BCHP_HIF_CONTINUATION_STB_BOOT_ADDR0 + reg_ofs, boot_vector);

	smp_wmb();
	flush_cache_all();

	/* unhalt the cpu */
	BDEV_UNSET(BCHP_HIF_CPUBIUCTRL_CPU_RESET_CONFIG_REG, BIT(cpu));
}

void brcmstb_cpu_power_on(unsigned int cpu)
{
	/*
	 * The secondary cores power was cut, so we must go through
	 * power-on initialization.
	 */
	u32 tmp;

	pr_info("SMP: Powering up CPU%d...\n", cpu);

	/* Request zone power up */
	pwr_zone_ctrl_wr(cpu, PWR_ZONE_REG(ZONE_PWR_UP_REQ_MASK));

	/* Wait for the power up FSM to complete */
	do {
		tmp = pwr_zone_ctrl_rd(cpu);
	} while (!(tmp & PWR_ZONE_REG(ZONE_PWR_ON_STATE_MASK)));
}

int brcmstb_cpu_get_power_state(unsigned int cpu)
{
	int tmp = pwr_zone_ctrl_rd(cpu);
	return (tmp & PWR_ZONE_REG(ZONE_RESET_STATE_MASK)) ? 0 : 1;
}

void __ref brcmstb_cpu_die(unsigned int cpu)
{
	/* Derived from misc_bpcm_arm.c */

	/* Clear SCTLR.C bit */
	__asm__(
		"mrc	p15, 0, r0, c1, c0, 0\n"
		"bic	r0, r0, #(1 << 2)\n"
		"mcr	p15, 0, r0, c1, c0, 0\n"
		: /* no output */
		: /* no input */
		: "r0"	/* clobber r0 */
	);

	/*
	 * Instruction barrier to ensure cache is really disabled before
	 * cleaning/invalidating the caches
	 */
	isb();

	flush_cache_all();

	/* Invalidate all instruction caches to PoU (ICIALLU) */
	/* Data sync. barrier to ensure caches have emptied out */
	__asm__("mcr	p15, 0, r0, c7, c5, 0\n" : : : "r0");
	dsb();

	/*
	 * Clear ACTLR.SMP bit to prevent broadcast TLB messages from reaching
	 * this core
	 */
	__asm__(
		"mrc	p15, 0, r0, c1, c0, 1\n"
		"bic	r0, r0, #(1 << 6)\n"
		"mcr	p15, 0, r0, c1, c0, 1\n"
		: /* no output */
		: /* no input */
		: "r0"	/* clobber r0 */
	);

	/* Disable all IRQs for this CPU */
	arch_local_irq_disable();

	/*
	 * Final full barrier to ensure everything before this instruction has
	 * quiesced.
	 */
	isb();
	dsb();

	/* Sit and wait to die */
	wfi();

	/* We should never get here... */
	nop();
	panic("Spurious interrupt on CPU %d received!\n", cpu);
}

static void busy_wait(int i)
{
	while (--i != 0)
		nop();
}

int brcmstb_cpu_kill(unsigned int cpu)
{
	u32 tmp;

	pr_info("SMP: Powering down CPU%d...\n", cpu);

	/* Program zone reset */
	pwr_zone_ctrl_wr(cpu, PWR_ZONE_REG(ZONE_RESET_STATE_MASK) |
			      PWR_ZONE_REG(ZONE_BLK_RST_ASSERT_MASK) |
			      PWR_ZONE_REG(ZONE_PWR_DN_REQ_MASK)) ;

	/* Verify zone reset */
	tmp = pwr_zone_ctrl_rd(cpu);
	if (!(tmp & PWR_ZONE_REG(ZONE_RESET_STATE_MASK)))
		pr_err("%s: Zone reset bit for CPU %d not asserted!\n",
			__func__, cpu);

	/* Wait for power down */
	do {
		tmp = pwr_zone_ctrl_rd(cpu);
	} while (!(tmp & PWR_ZONE_REG(ZONE_PWR_OFF_STATE_MASK)));

	/* Magic delay from misc_bpcm_arm.c */
	busy_wait(10000);

	/* Assert reset on the CPU */
	BDEV_SET(BCHP_HIF_CPUBIUCTRL_CPU_RESET_CONFIG_REG, BIT(cpu));

	return 1;
}

