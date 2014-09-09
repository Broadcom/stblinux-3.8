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
#include <linux/irqchip/brcmstb-l2.h>
#include <linux/irqchip/bcm7120-l2.h>
#include <linux/dma-contiguous.h>
#include <linux/export.h>
#include <asm/hardware/gic.h>

#include <asm/arch_timer.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/hardware/gic.h>
#include <asm/setup.h>

#include <linux/brcmstb/brcmstb.h>
#include <linux/brcmstb/cma_driver.h>
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
	{ .compatible = "brcm,l2-intc", .data = brcmstb_l2_intc_of_init },
	{ .compatible = "brcm,bcm7120-l2-intc", .data = bcm7120_l2_intc_of_init },
	{ }
};

static void __init brcmstb_map_io(void)
{
	iotable_init(brcmstb_io_map, ARRAY_SIZE(brcmstb_io_map));
}

#ifdef CONFIG_FIXED_PHY
static int of_add_one_fixed_phy(struct device_node *np)
{
	struct fixed_phy_status status = {};
	u32 *fixed_link;
	int ret;

	fixed_link  = (u32 *)of_get_property(np, "fixed-link", NULL);
	if (!fixed_link)
		return 1;

	status.link = 1;
	status.duplex = be32_to_cpu(fixed_link[1]);
	status.speed = be32_to_cpu(fixed_link[2]);
	status.pause = be32_to_cpu(fixed_link[3]);
	status.asym_pause = be32_to_cpu(fixed_link[4]);

	ret = fixed_phy_add(PHY_POLL, be32_to_cpu(fixed_link[0]), &status);
	if (ret)
		of_node_put(np);

	return ret;
}

static int __init of_add_fixed_phys(void)
{
	struct device_node *np, *child, *port;
	u32 phy_type;
	struct fixed_phy_status status = {};
	unsigned int i = 0;

	for_each_compatible_node(np, NULL, "brcm,bcm7445-switch-v4.0") {
		for_each_child_of_node(np, child) {
			for_each_child_of_node(child, port) {
				if (of_add_one_fixed_phy(port))
					continue;
			}
		}
	}

	/* SYSTEMPORT Ethernet MAC also uses the 'fixed-link' property */
	for_each_compatible_node(np, NULL, "brcm,systemport-v1.00")
		of_add_one_fixed_phy(np);

	/* For compatibility with the old DT binding, we just match
	 * against our specific Ethernet driver compatible property
	 * and we just parse the speed settings for the fixed-PHY
	 */
	for_each_compatible_node(np, NULL, "brcm,genet-v4") {
		status.link = 1;
		status.duplex = DUPLEX_FULL;
		status.pause = 0;
		status.asym_pause = 0;

		/* Look for the old binding, identified by the 'phy-type'
		 * property existence
		 */
		if (!of_property_read_u32(np, "phy-type", &phy_type)) {
			/* Do not register a fixed PHY for internal PHYs */
			if (phy_type == BRCM_PHY_TYPE_INT)
				continue;

			if (!of_property_read_u32(np, "phy-speed",
						&status.speed)) {
				/* Convention with the old (inflexible) binding
				 * is 0 -> MoCA, 1 -> anything else
				 */
				if (phy_type == BRCM_PHY_TYPE_MOCA)
					i = 0;
				else
					i = 1;
				fixed_phy_add(PHY_POLL, i, &status);
			}
		} else {
			/* Or try the new, standard 'fixed-link' binding */
			of_add_one_fixed_phy(np);
		}
	}

	return 0;
}
#else
static inline void of_add_fixed_phys(void)
{
}
#endif /* CONFIG_FIXED_PHY */

#ifdef CONFIG_BRCMSTB_USE_MEGA_BARRIER
#ifdef CONFIG_CMA
static phys_addr_t so_mem_paddr[NR_BANKS];
static void __iomem *so_mem_vaddr[NR_BANKS];

static struct cma_dev *cma_dev_get_by_addr(phys_addr_t start, phys_addr_t end)
{
	int i = 0;
	for (i = 0; i < CMA_DEV_MAX; i++) {
		struct cma_dev *cma_dev = cma_dev_get_cma_dev(i);
		if (!cma_dev)
			continue;

		if (cma_dev->range.base >= start &&
		    (cma_dev->range.base + cma_dev->range.size) <= end)
			return cma_dev;
	}

	return NULL;
}

static int brcmstb_mega_barrier_init(void)
{
	int bank_nr;

	pr_info("brcmstb: setting up mega-barrier workaround\n");

	for_each_bank(bank_nr, &meminfo) {
		struct page *page;
		struct cma_dev *cma_dev;
		const struct membank *bank = &meminfo.bank[bank_nr];
		const int len = PAGE_SIZE;

		cma_dev = cma_dev_get_by_addr(bank_phys_start(bank),
						bank_phys_end(bank));
		if (!cma_dev) {
			pr_warn("no cma dev for addr range (%x,%x) exists\n",
					bank_phys_start(bank),
					bank_phys_end(bank));
			continue;
		}

		page = dma_alloc_from_contiguous(cma_dev->dev,
							len >> PAGE_SHIFT, 0);
		if (!page) {
			pr_err("failed to alloc page for dummy store on bank %d\n",
				bank_nr);
			continue;
		}

		so_mem_paddr[bank_nr] = page_to_phys(page);
		so_mem_vaddr[bank_nr] = cma_dev_kva_map(page, len >> PAGE_SHIFT,
					pgprot_noncached(pgprot_kernel));
	}

	return 0;
}
late_initcall(brcmstb_mega_barrier_init);

/*
 * The suggested workaround requires a dummy store to memory mapped as
 * STRONGLY ORDERED on each MEMC, followed by a data sync barrier.
 *
 * This function should be called following all cache flush operations.
 */
void brcmstb_mega_barrier(void)
{
	int bank_nr;

	__asm__("dsb");

	for (bank_nr = 0; bank_nr < NR_BANKS; bank_nr++) {
		if (so_mem_vaddr[bank_nr])
			writel_relaxed(0, so_mem_vaddr[bank_nr]);
	}

	__asm__("dsb");
}
EXPORT_SYMBOL(brcmstb_mega_barrier);
#else
#warning "The mega-barrier workaround requires CMA!"
void brcmstb_mega_barrier(void)
{
	panic("mega-barrier workaround requires CMA, but CMA was not enabled");
}
#endif /* CONFIG_CMA */
#else /* !CONFIG_BRCMSTB_USE_MEGA_BARRIER */
void brcmstb_mega_barrier(void)
{
	; /* this is an empty stub left in as a courtesy for the refsw folks */
}
EXPORT_SYMBOL(brcmstb_mega_barrier);
#endif /* CONFIG_BRCMSTB_USE_MEGA_BARRIER */

static void __init brcmstb_machine_init(void)
{
	int ret;

	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	of_add_fixed_phys();
	brcmstb_hook_fault_code();
	brcmstb_clocks_init();
	ret = brcmstb_pm_init();
	if (ret)
		pr_warn("PM: initialization failed with code %d\n", ret);
	cma_register();
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
}

static void __init brcmstb_init_early(void)
{
	add_preferred_console("ttyS", 0, "115200");
}

static struct sys_timer __initdata brcmstb_timer = {
	.init = timer_init,
};

void brcmstb_irq0_init(void)
{
	BDEV_WR(BCHP_IRQ0_IRQEN, BCHP_IRQ0_IRQEN_uarta_irqen_MASK
		| BCHP_IRQ0_IRQEN_uartb_irqen_MASK
		| BCHP_IRQ0_IRQEN_uartc_irqen_MASK
		);
}

void __init brcmstb_dt_init_irq(void)
{
	/* Force lazily-disabled IRQs to be masked before suspend */
	gic_arch_extn.flags |= IRQCHIP_MASK_ON_SUSPEND;

	brcmstb_irq0_init();
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

static void __init brcmstb_reserve(void)
{
	cma_reserve();
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
	.reserve	= brcmstb_reserve,
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
