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

#ifndef __BRCMSTB_COMMON_H__
#define __BRCMSTB_COMMON_H__

#include <linux/smp.h>

#ifdef CONFIG_SMP
extern void brcmstb_secondary_startup(void);
extern void brcmstb_cpu_boot(unsigned int cpu);
extern void brcmstb_cpu_power_on(unsigned int cpu);
extern int brcmstb_cpu_get_power_state(unsigned int cpu);
extern struct smp_operations brcmstb_smp_ops;
#endif

#ifdef CONFIG_HOTPLUG_CPU
extern void brcmstb_cpu_die(unsigned int cpu);
extern int brcmstb_cpu_kill(unsigned int cpu);
#endif

extern void brcmstb_hook_fault_code(void);

#endif /* __BRCMSTB_COMMON_H__ */
