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
#include <linux/console.h>
#include <linux/clk/clk-brcmstb.h>

#include <asm/arch_timer.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/hardware/gic.h>

#include <linux/brcmstb/brcmstb.h>
#include <linux/phy.h>
#include <linux/phy_fixed.h>

#include "common.h"

/***********************************************************************
 * STB CPU (main application processor)
 ***********************************************************************/

static struct map_desc brcmstb_io_map[] __initdata = {
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
	"brcm,bcm7445b0",
	"brcm,bcm7145a0",
	NULL
};

static const struct of_device_id brcmstb_dt_irq_match[] __initconst = {
	{ .compatible = "arm,cortex-a15-gic", .data = gic_of_init },
	{ }
};

static void __init brcmstb_map_io(void)
{
	iotable_init(brcmstb_io_map, ARRAY_SIZE(brcmstb_io_map));
}

/* Fixed PHY for MoCA, need to be registered before the fixed MDIO bus is
 * probed
 */
static struct fixed_phy_status fixed_phy_status = {
	.link	= 1,
	.speed	= SPEED_1000,
	.duplex	= DUPLEX_FULL,
};

static void __init brcmstb_machine_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);


	brcmstb_hook_fault_code();

	fixed_phy_add(PHY_POLL, 0, &fixed_phy_status);
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
	arch_timer_sched_clock_init();
	__brcmstb_clk_init();
}

static void __init brcmstb_init_early(void)
{
	add_preferred_console("ttyS", 0, "115200");
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

void wktmr_read(struct wktmr_time *t)
{
	uint32_t tmp;

	do {
		t->sec = BDEV_RD(BCHP_WKTMR_COUNTER);
		tmp = BDEV_RD(BCHP_WKTMR_PRESCALER_VAL);
	} while (tmp >= WKTMR_FREQ);

	t->pre = WKTMR_FREQ - tmp;
}

unsigned long wktmr_elapsed(struct wktmr_time *t)
{
	struct wktmr_time now;

	wktmr_read(&now);
	now.sec -= t->sec;
	if (now.pre > t->pre) {
		now.pre -= t->pre;
	} else {
		now.pre = WKTMR_FREQ + now.pre - t->pre;
		now.sec--;
	}
	return (now.sec * WKTMR_FREQ) + now.pre;
}

DT_MACHINE_START(BRCMSTB, "Broadcom STB (Flattened Device Tree)")
	.map_io		= brcmstb_map_io,
	.init_irq	= brcmstb_dt_init_irq,
	.handle_irq	= gic_handle_irq,
	.timer		= &brcmstb_timer,
	.init_machine	= brcmstb_machine_init,
	.dt_compat	= brcmstb_match,
	.restart	= brcmstb_restart,
	.smp		= smp_ops(brcmstb_smp_ops),
	.init_early	= brcmstb_init_early,
MACHINE_END

/***********************************************************************
 * RG coprocessor kernel - for cable combo chips only
 ***********************************************************************/

static struct map_desc brcmrg_io_map[] __initdata = {
	{
	.virtual = (unsigned long)BRCMRG_PERIPH_VIRT,
	.pfn     = __phys_to_pfn(BRCMRG_PERIPH_PHYS),
	.length  = BRCMRG_PERIPH_LENGTH,
	.type    = MT_DEVICE,
	},
};

static const char *brcmrg_match[] __initdata = {
	"brcm,bcm7145a0-rg",
	NULL
};

static void __init brcmrg_map_io(void)
{
	iotable_init(brcmrg_io_map, ARRAY_SIZE(brcmrg_io_map));
}

void __init brcmrg_dt_init_irq(void)
{
	of_irq_init(brcmstb_dt_irq_match);
}

DT_MACHINE_START(BRCMRG, "Broadcom RG CPU (Flattened Device Tree)")
	.map_io		= brcmrg_map_io,
	.init_irq	= brcmrg_dt_init_irq,
	.handle_irq	= gic_handle_irq,
	.timer		= &brcmstb_timer,
	.init_machine	= brcmstb_machine_init,
	.dt_compat	= brcmrg_match,
	.init_early	= brcmstb_init_early,
MACHINE_END
