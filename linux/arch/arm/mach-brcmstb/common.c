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

#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <asm/arch_timer.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/hardware/gic.h>

#include <linux/brcmstb/brcmstb.h>

#include "common.h"

static struct map_desc io_map[] __initdata = {
	{
	.virtual = (unsigned long)BRCMSTB_PERIPH_VIRT,
	.pfn     = __phys_to_pfn(BRCMSTB_PERIPH_PHYS),
	.length  = BRCMSTB_PERIPH_LENGTH,
	.type    = MT_DEVICE,
	},
};

static const char *brcmstb_match[] __initdata = {
	"brcm,brcmstb",
	"brcm,bcm7445a0",
	NULL
};

static const struct of_device_id brcmstb_dt_irq_match[] __initconst = {
	{ .compatible = "arm,cortex-a15-gic", .data = gic_of_init },
	{ }
};

static void __init brcmstb_map_io(void)
{
	iotable_init(io_map, ARRAY_SIZE(io_map));
}

static void __init brcmstb_machine_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static void brcmstb_restart(char mode, const char *cmd)
{
	BDEV_WR_F_RB(SUN_TOP_CTRL_RESET_SOURCE_ENABLE,
		sw_master_reset_enable, 1);
	BDEV_WR_F_RB(SUN_TOP_CTRL_SW_MASTER_RESET, chip_master_reset, 1);
	while (1)
		;
}

static void __init timer_init(void)
{
	arch_timer_of_register();
}

static struct sys_timer __initdata brcmstb_timer = {
	.init = timer_init,
};

void __init brcmstb_dt_init_irq(void)
{
	BDEV_WR(BCHP_IRQ0_IRQEN, BCHP_IRQ0_IRQEN_uarta_irqen_MASK
		| BCHP_IRQ0_IRQEN_uartb_irqen_MASK
		| BCHP_IRQ0_IRQEN_uartc_irqen_MASK
		);
	of_irq_init(brcmstb_dt_irq_match);
}

DT_MACHINE_START(BRCMSTB, "Broadcom STB (Flattened Device Tree)")
	.map_io		= brcmstb_map_io,
	.init_irq	= brcmstb_dt_init_irq,
	.handle_irq	= gic_handle_irq,
	.timer		= &brcmstb_timer,
	.init_machine	= brcmstb_machine_init,
	.dt_compat	= brcmstb_match,
	.restart	= brcmstb_restart,
	.smp		= &brcmstb_smp_ops,
MACHINE_END
