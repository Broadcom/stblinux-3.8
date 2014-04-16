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
#define BIOPTR(x) __typesafe_io(BVIRTADDR(x))

enum {
	ZONE_MAN_CLKEN_MASK		= BIT(0),
	ZONE_MAN_RESET_CNTL_MASK	= BIT(1),
	ZONE_MAN_MEM_PWR_MASK		= BIT(4),
	ZONE_RESERVED_1_MASK		= BIT(5),
	ZONE_MAN_ISO_CNTL_MASK		= BIT(6),
	ZONE_MANUAL_CONTROL_MASK	= BIT(7),
	ZONE_PWR_DN_REQ_MASK		= BIT(9),
	ZONE_PWR_UP_REQ_MASK		= BIT(10),
	ZONE_BLK_RST_ASSERT_MASK	= BIT(12),
	ZONE_PWR_OFF_STATE_MASK		= BIT(25),
	ZONE_PWR_ON_STATE_MASK		= BIT(26),
	ZONE_DPG_PWR_STATE_MASK		= BIT(28),
	ZONE_MEM_PWR_STATE_MASK		= BIT(29),
	ZONE_RESET_STATE_MASK		= BIT(31),
};

#if (defined(CONFIG_BCM7445C0) || defined(CONFIG_BCM7445D0) || \
	defined(CONFIG_BCM7439A0) || defined(CONFIG_BCM7366A0))
/* HW7445-1290, HW7439-463, HW7366-422: 2nd'ary CPU cores may fail to boot
 * ---
 *
 * There is a design flaw with the BPCM logic, which requires a manual
 * software sequencing during CPU power-on/power-off.
 */
#define USE_MANUAL_MODE 1
#else
#define USE_MANUAL_MODE 0
#endif

DEFINE_PER_CPU(int, per_cpu_sw_state);

static void __iomem *pwr_ctrl_base(unsigned int cpu)
{
	void __iomem *base =
		BIOPTR(BCHP_HIF_CPUBIUCTRL_CPU0_PWR_ZONE_CNTRL_REG);
	base += (cpu * 4);
	BUG_ON(base >= BIOPTR(BCHP_HIF_CPUBIUCTRL_L2BIU_PWR_ZONE_CNTRL_REG));
	return base;
}

static u32 pwr_ctrl_rd(unsigned int cpu)
{
	void __iomem *base = pwr_ctrl_base(cpu);
	return readl(base);
}

static void pwr_ctrl_wr(unsigned int cpu, u32 val)
{
	void __iomem *base = pwr_ctrl_base(cpu);
	writel(val, base);
}

static void pwr_ctrl_set(unsigned int cpu, u32 val, u32 mask)
{
	void __iomem *base = pwr_ctrl_base(cpu);
	writel((readl(base) & mask) | val, base);
}

static void pwr_ctrl_clr(unsigned int cpu, u32 val, u32 mask)
{
	void __iomem *base = pwr_ctrl_base(cpu);
	writel((readl(base) & mask) & ~val, base);
}

#define POLL_TMOUT_MS 500
static int pwr_ctrl_wait_tmout(unsigned int cpu, u32 set, u32 mask)
{
	const unsigned long timeo = jiffies + msecs_to_jiffies(POLL_TMOUT_MS);
	u32 tmp;

	do {
		tmp = pwr_ctrl_rd(cpu) & mask;
		if (!set == !tmp)
			return 0;
	} while (time_before(jiffies, timeo));

	tmp = pwr_ctrl_rd(cpu) & mask;
	if (!set == !tmp)
		return 0;

	return -ETIMEDOUT;
}

void brcmstb_cpu_boot(unsigned int cpu)
{
	unsigned long boot_vector;
	const int reg_ofs = cpu * 8;
	u32 tmp;

	pr_info("SMP: Booting CPU%d...\n", cpu);

	/*
	* set the reset vector to point to the secondary_startup
	* routine
	*/
	boot_vector = virt_to_phys(brcmstb_secondary_startup);
	writel_relaxed(0,
		BIOPTR(BCHP_HIF_CONTINUATION_STB_BOOT_HI_ADDR0 + reg_ofs));
	writel_relaxed(boot_vector,
		BIOPTR(BCHP_HIF_CONTINUATION_STB_BOOT_ADDR0 + reg_ofs));

	smp_wmb();
	flush_cache_all();

	/* unhalt the cpu */
	tmp = readl_relaxed(BIOPTR(BCHP_HIF_CPUBIUCTRL_CPU_RESET_CONFIG_REG))
		& ~BIT(cpu);
	writel_relaxed(tmp, BIOPTR(BCHP_HIF_CPUBIUCTRL_CPU_RESET_CONFIG_REG));
}

void brcmstb_cpu_power_on(unsigned int cpu)
{
	/*
	 * The secondary cores power was cut, so we must go through
	 * power-on initialization.
	 */
	pr_info("SMP: Powering up CPU%d...\n", cpu);

	if (USE_MANUAL_MODE) {
		pwr_ctrl_set(cpu, ZONE_MAN_ISO_CNTL_MASK, 0xffffff00);
		pwr_ctrl_set(cpu, ZONE_MANUAL_CONTROL_MASK, -1);
		pwr_ctrl_set(cpu, ZONE_RESERVED_1_MASK, -1);

		pwr_ctrl_set(cpu, ZONE_MAN_MEM_PWR_MASK, -1);

		if (pwr_ctrl_wait_tmout(cpu, 1, ZONE_MEM_PWR_STATE_MASK))
			panic("ZONE_MEM_PWR_STATE_MASK set timeout");

		pwr_ctrl_set(cpu, ZONE_MAN_CLKEN_MASK, -1);

		if (pwr_ctrl_wait_tmout(cpu, 1, ZONE_DPG_PWR_STATE_MASK))
			panic("ZONE_DPG_PWR_STATE_MASK set timeout");

		pwr_ctrl_clr(cpu, ZONE_MAN_ISO_CNTL_MASK, -1);
		pwr_ctrl_set(cpu, ZONE_MAN_RESET_CNTL_MASK, -1);
	} else {
		/* Request zone power up */
		pwr_ctrl_wr(cpu, ZONE_PWR_UP_REQ_MASK);

		/* Wait for the power up FSM to complete */
		if (pwr_ctrl_wait_tmout(cpu, 1, ZONE_PWR_ON_STATE_MASK))
			panic("ZONE_PWR_ON_STATE_MASK set timeout");
	}

	per_cpu(per_cpu_sw_state, cpu) = 1;
}

int brcmstb_cpu_get_power_state(unsigned int cpu)
{
	int tmp = pwr_ctrl_rd(cpu);
	return (tmp & ZONE_RESET_STATE_MASK) ? 0 : 1;
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

	per_cpu(per_cpu_sw_state, cpu) = 0;

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

int brcmstb_cpu_kill(unsigned int cpu)
{
	u32 tmp;

#if (defined(CONFIG_BCM7445B0) || USE_MANUAL_MODE)
	/* HW7445-1175: TI2C master reset non-functional if CPU0 is powered
	 * off.
	 *
	 * Do not allow powering off CPU0, the core CPU hotplug code already
	 * refuses to power off this CPU because it is the boot CPU, but
	 * we also need to make sure it does not get powered off by a
	 * fatal exception in interrupt handler for instance, otherwise
	 * TI2C master reset will not bring us to reset as the CPU remains
	 * powered off.
	 *
	 * Although this was fixed on 7445C0 and newer, operating the BPCM
	 * via the manual mode nullifies the fix. So, we have to prevent
	 * CPU0 power-down for all chips that use the manual mode.
	 */
	if (cpu == 0) {
		pr_warn("SMP: refusing to power off CPU0\n");
		return 1;
	}
#endif

	while (per_cpu(per_cpu_sw_state, cpu))
		;

	pr_info("SMP: Powering down CPU%d...\n", cpu);

	if (USE_MANUAL_MODE) {
		pwr_ctrl_set(cpu, ZONE_MANUAL_CONTROL_MASK, -1);
		pwr_ctrl_clr(cpu, ZONE_MAN_RESET_CNTL_MASK, -1);
		pwr_ctrl_clr(cpu, ZONE_MAN_CLKEN_MASK, -1);
		pwr_ctrl_set(cpu, ZONE_MAN_ISO_CNTL_MASK, -1);
		pwr_ctrl_clr(cpu, ZONE_MAN_MEM_PWR_MASK, -1);

		if (pwr_ctrl_wait_tmout(cpu, 0, ZONE_MEM_PWR_STATE_MASK))
			panic("ZONE_MEM_PWR_STATE_MASK clear timeout");

		pwr_ctrl_clr(cpu, ZONE_RESERVED_1_MASK, -1);

		if (pwr_ctrl_wait_tmout(cpu, 0, ZONE_DPG_PWR_STATE_MASK))
			panic("ZONE_DPG_PWR_STATE_MASK clear timeout");
	} else {
		/* Program zone reset */
		pwr_ctrl_wr(cpu, ZONE_RESET_STATE_MASK |
				 ZONE_BLK_RST_ASSERT_MASK |
				 ZONE_PWR_DN_REQ_MASK) ;

		/* Verify zone reset */
		tmp = pwr_ctrl_rd(cpu);
		if (!(tmp & ZONE_RESET_STATE_MASK))
			pr_err("%s: Zone reset bit for CPU %d not asserted!\n",
				__func__, cpu);

		/* Wait for power down */
		if (pwr_ctrl_wait_tmout(cpu, 1, ZONE_PWR_OFF_STATE_MASK))
			panic("ZONE_PWR_OFF_STATE_MASK set timeout");
	}

	/* Settle-time from Broadcom-internal DVT reference code */
	udelay(7);

	/* Assert reset on the CPU */
	tmp = readl_relaxed(BIOPTR(BCHP_HIF_CPUBIUCTRL_CPU_RESET_CONFIG_REG))
		| BIT(cpu);
	writel_relaxed(tmp, BIOPTR(BCHP_HIF_CPUBIUCTRL_CPU_RESET_CONFIG_REG));

	return 1;
}

