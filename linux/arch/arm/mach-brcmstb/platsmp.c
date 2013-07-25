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
	/*
	 * set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/* Bring up power to the core if necessary */
	if (brcmstb_cpu_get_power_state(cpu) == 0)
		brcmstb_cpu_power_on(cpu);

	brcmstb_cpu_boot(cpu);

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return 0;
}

static void __init brcmstb_smp_init_cpus(void)
{
	if (num_possible_cpus() > 1)
		set_smp_cross_call(gic_raise_softirq);
	else
		setup_max_cpus = 0;
}

static void __init brcmstb_smp_prepare_cpus(unsigned int max_cpus)
{
}

struct smp_operations brcmstb_smp_ops __initdata = {
	.smp_init_cpus		= brcmstb_smp_init_cpus,
	.smp_prepare_cpus	= brcmstb_smp_prepare_cpus,
	.smp_secondary_init	= brcmstb_secondary_init,
	.smp_boot_secondary	= brcmstb_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_kill		= brcmstb_cpu_kill,
	.cpu_die		= brcmstb_cpu_die,
#endif
};
