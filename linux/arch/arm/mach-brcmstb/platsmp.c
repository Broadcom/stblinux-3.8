/*
 * Broadcom STB SMP support for ARM
 * Based on arch/arm/mach-tegra/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 *  Copyright (C) 2009 Palm
 *  All Rights Reserved
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

static DEFINE_SPINLOCK(boot_lock);

/***********************************************************************
 * SMP boot
 ***********************************************************************/

static void __cpuinit brcmstb_secondary_init(unsigned int cpu)
{
	/*
	 * if any interrupts are already enabled for the primary
	 * core (e.g. timer irq), then they will not have been enabled
	 * for us: do so
	 */
	gic_secondary_init(0);

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

static int __cpuinit brcmstb_boot_secondary(unsigned int cpu,
					    struct task_struct *idle)
{
	unsigned long boot_vector;

	pr_info("SMP: Booting CPU%d...\n", cpu);

	/*
	 * set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/* set the reset vector to point to the secondary_startup routine */
	boot_vector = virt_to_phys(brcmstb_secondary_startup);
	BDEV_WR(BCHP_HIF_CONTINUATION_STB_BOOT_ADDR0 + cpu * 8, boot_vector);

	smp_wmb();

	flush_cache_all();

	/* unhalt the cpu */
	BDEV_UNSET(BCHP_HIF_CPUBIUCTRL_CPU_RESET_CONFIG_REG, BIT(cpu));

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return 0;
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
static void __init brcmstb_smp_init_cpus(void)
{
	/* FIXME: ncores needs to come from DT */
	unsigned int i, ncores = 4;

	if (ncores > nr_cpu_ids) {
		pr_warn("SMP: %u cores greater than maximum (%u), clipping\n",
			ncores, nr_cpu_ids);
		ncores = nr_cpu_ids;
	}

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);

	set_smp_cross_call(gic_raise_softirq);
}

static void __init brcmstb_smp_prepare_cpus(unsigned int max_cpus)
{
}

#ifdef CONFIG_HOTPLUG_CPU
static void brcmstb_cpu_die(unsigned int cpu)
{
	while (1)
		;
}
#endif

struct smp_operations brcmstb_smp_ops __initdata = {
	.smp_init_cpus		= brcmstb_smp_init_cpus,
	.smp_prepare_cpus	= brcmstb_smp_prepare_cpus,
	.smp_secondary_init	= brcmstb_secondary_init,
	.smp_boot_secondary	= brcmstb_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= brcmstb_cpu_die,
#endif
};
