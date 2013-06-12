/*
 * Copyright (C) 2009 - 2013 Broadcom Corporation
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
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/printk.h>
#include <linux/syscore_ops.h>
#include <linux/brcmstb/brcmstb.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/irqdomain.h>


/* Use assigned bus numbers so "ops" can tell the controllers apart */
#define BRCM_BUSNO_PCIE			0x0
#define BRCM_NUM_PCI_CONTROLLERS	0x1
#define BRCM_NUM_PCI_OUT_WINS		0x4

static int brcm_pci_read_config(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *data);
static int brcm_pci_write_config(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 data);
static inline int get_busno_pcie(void)  { return BRCM_BUSNO_PCIE;  }

static struct pci_ops brcm_pci_ops = {
	.read = brcm_pci_read_config,
	.write = brcm_pci_write_config,
};

static int brcm_setup_pcie_bridge(int nr, struct pci_sys_data *sys);
struct pci_bus __init *brcm_pci_sys_scan_bus(int nr, struct pci_sys_data *sys);
static int __init brcm_map_irq(const struct pci_dev *dev, u8 slot, u8 pin);

static __initdata struct hw_pci brcm_pcie_hw = {
	.nr_controllers	= BRCM_NUM_PCI_CONTROLLERS,
	.setup		= brcm_setup_pcie_bridge,
	.scan		= brcm_pci_sys_scan_bus,
	.map_irq	= brcm_map_irq,
};

/* Internal Bus Controller Information.*/
struct brcm_pci_bus {
	char			*name;
	unsigned int		hw_busnum;
	unsigned int		busnum_shift;
	unsigned int		slot_shift;
	unsigned int		func_shift;
	int			memory_hole;
	unsigned long		idx_reg;
	unsigned long		data_reg;
	struct clk		*clk;
	struct device_node	*node;
};

static struct brcm_pci_bus brcm_buses[] = {
	[BRCM_BUSNO_PCIE] = { "PCIe", 1, 20, 15, 12, 0 },
};
static struct resource pcie_iomem_res[BRCM_NUM_PCI_OUT_WINS];
static int brcm_num_pci_out_wins;



/***********************************************************************
 * PCIe Bridge setup
 ***********************************************************************/
#if defined(__BIG_ENDIAN)
#define	DATA_ENDIAN		2	/* PCI->DDR inbound accesses */
#define MMIO_ENDIAN		2	/* CPU->PCI outbound accesses */
#else
#define	DATA_ENDIAN		0
#define MMIO_ENDIAN		0
#endif


static void set_pcie_outbound_win(unsigned win, u64 start, u64 len)
{
	BDEV_WR((BCHP_PCIE_0_MISC_CPU_2_PCIE_MEM_WIN0_LO+(win*8)),
		(u32)(start) + MMIO_ENDIAN);
	BDEV_WR((BCHP_PCIE_0_MISC_CPU_2_PCIE_MEM_WIN0_HI+(win*8)),
		(u32)(start >> 32));
	BDEV_WR((BCHP_PCIE_0_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT+(win*4)),
		((((u32)start) >> 20) << 4)
		| (((((u32)start) + ((u32)len) - 1) >> 20) << 20));
}


static struct wktmr_time pcie_reset_started;

#define PCIE_LINK_UP()							\
	(((BDEV_RD(BCHP_PCIE_0_MISC_PCIE_STATUS) & 0x30) == 0x30) ? 1 : 0)


static int brcm_pcie_setup_core(int nr, struct pci_sys_data *sys)
{
	const u32 *ranges;
	u64 pci_addr, cpu_addr, size;
	int pna, rlen, np, i;
	struct device_node *node = brcm_buses[BRCM_BUSNO_PCIE].node;

	/*
	 * Starts PCIe link negotiation immediately at kernel boot time.  The
	 * RC is supposed to give the endpoint device 100ms to settle down
	 * before attempting configuration accesses.  So we let the link
	 * negotiation happen in the background instead of busy-waiting.
	 */
	struct wktmr_time tmp;

	ranges = of_get_property(node, "ranges", &rlen);
	if (ranges == NULL) {
		pr_err("PCIe: no ranges property in dev tree.\n");
		return 1;
	}

	/* reset the bridge and the endpoint device */
	BDEV_WR_F_RB(PCIE_0_RGR1_SW_INIT_1, PCIE_BRIDGE_SW_INIT, 1);
	BDEV_WR_F_RB(PCIE_0_RGR1_SW_INIT_1, PCIE_SW_PERST, 1);

	/* delay 100us */
	wktmr_read(&tmp);
	while (wktmr_elapsed(&tmp) < (100 * WKTMR_1US))
		;

	/* take the bridge out of reset */
	BDEV_WR_F_RB(PCIE_0_RGR1_SW_INIT_1, PCIE_BRIDGE_SW_INIT, 0);

	/* enable SCB_MAX_BURST_SIZE | CSR_READ_UR_MODE | SCB_ACCESS_EN */
	BDEV_WR(BCHP_PCIE_0_MISC_MISC_CTRL, 0x81e03000);

	/* set up CPU->PCIE memory windows (max of four) */
	pna = of_n_addr_cells(node);
	np = pna + 5;
	for (i = 0; i < BRCM_NUM_PCI_OUT_WINS; i++) {
		rlen -= np * 4;
		if  (rlen < 0) {
			/* Disable window */
			set_pcie_outbound_win(i, 0, 0);
			continue;
		}
		pci_addr = of_read_number(ranges + 1, 2);
		cpu_addr = of_translate_address(node, ranges + 3);
		size = of_read_number(ranges + pna + 3, 2);
		ranges += np;
		set_pcie_outbound_win(i, cpu_addr, size);
		pcie_iomem_res[i].name = "External PCIe MEM";
		pcie_iomem_res[i].flags	= IORESOURCE_MEM;
		pcie_iomem_res[i].start = cpu_addr;
		pcie_iomem_res[i].end = cpu_addr + size - 1;

		/* Request memory region resources. */
		if (request_resource(&iomem_resource, &pcie_iomem_res[i]))
			panic("PCIe: request PCIe Prefetch Memory resource failed\n");
		pci_add_resource_offset(&sys->resources, &pcie_iomem_res[i],
					sys->mem_offset);
		brcm_num_pci_out_wins++;
	}

	if (brcm_num_pci_out_wins == 0) {
		pr_err("PCIe: no out windows allocated.\n");
		return 1;
	}

	/* set up 4GB PCIE->SCB memory window on BAR2 */
	BDEV_WR(BCHP_PCIE_0_MISC_RC_BAR2_CONFIG_LO, 0x00000011);
	BDEV_WR(BCHP_PCIE_0_MISC_RC_BAR2_CONFIG_HI, 0x00000000);
	BDEV_WR_F(PCIE_0_MISC_MISC_CTRL, SCB0_SIZE, 0x10); /* 2 Gb */
	BDEV_WR_F(PCIE_0_MISC_MISC_CTRL, SCB1_SIZE, 0x0f); /* 1 Gb */

	/* disable the PCIE->GISB memory window */
	BDEV_WR(BCHP_PCIE_0_MISC_RC_BAR1_CONFIG_LO, 0x00000000);

	/* disable the PCIE->SCB memory window */
	BDEV_WR(BCHP_PCIE_0_MISC_RC_BAR3_CONFIG_LO, 0x00000000);

	/* disable MSI (for now...) */
	BDEV_WR(BCHP_PCIE_0_MISC_MSI_BAR_CONFIG_LO, 0);

	/* set up L2 interrupt masks */
	BDEV_WR_RB(BCHP_PCIE_0_INTR2_CPU_CLEAR, 0);
	BDEV_WR_RB(BCHP_PCIE_0_INTR2_CPU_MASK_CLEAR, 0);
	BDEV_WR_RB(BCHP_PCIE_0_INTR2_CPU_MASK_SET, 0xffffffff);

	/* take the EP device out of reset */
	BDEV_WR_F_RB(PCIE_0_RGR1_SW_INIT_1, PCIE_SW_PERST, 0);

	/* record the current time */
	wktmr_read(&pcie_reset_started);
	return 0;
}


static int brcm_setup_pcie_bridge(int nr, struct pci_sys_data *sys)
{
	u32 pcie_out_win_start, pcie_out_win_end;
	struct clk *clk;
	int i;

	/* Program PCIE Core Controller Registers.*/
	if (brcm_pcie_setup_core(nr, sys))
		goto FAIL;

	/* Give the RC/EP time to wake up, before trying to configure RC */
	while (wktmr_elapsed(&pcie_reset_started) < (100 * WKTMR_1MS))
		;

	if (!PCIE_LINK_UP()) {
		pr_info("PCIe: link down\n");
		goto FAIL;
	}
	pr_info("PCIe link up, %sGbps x%lu\n",
		BDEV_RD_F(PCIE_0_RC_CFG_PCIE_LINK_STATUS_CONTROL,
			  NEG_LINK_SPEED) == 0x2 ? "5.0" : "2.5",
		BDEV_RD_F(PCIE_0_RC_CFG_PCIE_LINK_STATUS_CONTROL,
			  NEG_LINK_WIDTH));

	if (nr != get_busno_pcie()) {
		pr_err("PCIe: brcm_pcie_setup bailing early, nr = %d\n", nr);
		goto FAIL;
	}

	/* Enable MEM_SPACE and BUS_MASTER for RC */
	BDEV_WR(BCHP_PCIE_0_RC_CFG_TYPE1_STATUS_COMMAND, 0x6);

	/* Set base/limit for outbound transactions.  Assume that
	 * the out windows are contiguous.
	 */
	pcie_out_win_start = pcie_iomem_res[0].start;
	pcie_out_win_end = pcie_out_win_start - 1;
	for (i = 0; i < brcm_num_pci_out_wins; i++)
		pcie_out_win_end += pcie_iomem_res[i].end
			- pcie_iomem_res[i].start + 1;

	BDEV_WR(BCHP_PCIE_0_RC_CFG_TYPE1_RC_MEM_BASE_LIMIT,
		((pcie_out_win_end & 0xfff00000)
		 | ((pcie_out_win_start>>16)&0x0000fff0)));

	/* Disable the prefetch range */
	BDEV_WR(BCHP_PCIE_0_RC_CFG_TYPE1_RC_PREF_BASE_LIMIT, 0x0000fff0);

	/* Set pri/sec bus numbers */
	BDEV_WR(BCHP_PCIE_0_RC_CFG_TYPE1_PRI_SEC_BUS_NO, 0x00010100);

	/* Enable configuration request retry (see pci_scan_device()) */
	BDEV_WR_F(PCIE_0_RC_CFG_PCIE_ROOT_CAP_CONTROL, RC_CRS_EN, 1);

	/* PCIE->SCB endian mode for BAR */
	BDEV_WR_F_RB(PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1,
		     ENDIAN_MODE_BAR2, DATA_ENDIAN);

	return 1;
FAIL:
	clk = brcm_buses[BRCM_BUSNO_PCIE].clk;
	if (clk) {
		clk_disable(clk);
		clk_put(clk);
	}
	return 0;

}


struct pci_bus __init *brcm_pci_sys_scan_bus(int nr, struct pci_sys_data *sys)
{
	struct pci_bus *pbus;
	pr_info("PCIe: Scanning Root Bus 0.\n");

	pbus = pci_scan_root_bus(NULL, sys->busnr, &brcm_pci_ops, sys,
				 &sys->resources);
	return pbus;
}


#if defined(CONFIG_PM)
/*
 * syscore device to handle PCIe bus suspend and resume
 */
static inline void pcie_enable(int enable)
{
	struct clk *clk;

	clk = brcm_buses[BRCM_BUSNO_PCIE].clk;
	if (clk)
		enable ? clk_enable(clk) : clk_disable(clk);
}


static int pcie_suspend(void)
{
	pcie_enable(0);
	return 0;
}


static void pcie_resume(void)
{
	pcie_enable(1);
}


static struct syscore_ops pcie_pm_ops = {
	.suspend        = pcie_suspend,
	.resume         = pcie_resume,
};
#endif



/***********************************************************************
 * PCI controller registration
 ***********************************************************************/
static int __init brcm_pci_init(void)
{
	brcm_buses[BRCM_BUSNO_PCIE].clk = clk_get(NULL, "pcie");
	if (IS_ERR(brcm_buses[BRCM_BUSNO_PCIE].clk))
		brcm_buses[BRCM_BUSNO_PCIE].clk = NULL;
	brcm_buses[BRCM_BUSNO_PCIE].idx_reg =
		BVIRTADDR(BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX);
	brcm_buses[BRCM_BUSNO_PCIE].data_reg =
		BVIRTADDR(BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_DATA);

#if defined(CONFIG_PM)
	register_syscore_ops(&pcie_pm_ops);
#endif
	pci_common_init(&brcm_pcie_hw);
	return PCIBIOS_SUCCESSFUL;
}




/***********************************************************************
 * Read/write PCI configuration registers
 ***********************************************************************/
#define CFG_INDEX(bus, devfn, reg)					\
	(((PCI_SLOT(devfn) & 0x1f) << (brcm_buses[bus->number].slot_shift)) | \
	 ((PCI_FUNC(devfn) & 0x07) << (brcm_buses[bus->number].func_shift)) | \
	 (brcm_buses[bus->number].hw_busnum <<				\
	  brcm_buses[bus->number].busnum_shift) |			\
	 (reg))

static int devfn_ok(struct pci_bus *bus, unsigned int devfn)
{
	/* PCIe: check for link down or invalid slot number */
	if (bus->number == BRCM_BUSNO_PCIE &&
	    (!PCIE_LINK_UP() || PCI_SLOT(devfn) != 0))
		return 0;

	return 1;	/* OK */
}


static int brcm_pci_write_config(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 data)
{
	u32 val = 0, mask, shift;

	if (!devfn_ok(bus, devfn))
		return PCIBIOS_FUNC_NOT_SUPPORTED;

	BUG_ON(((where & 3) + size) > 4);

	if (size < 4) {
		/* partial word - read, modify, write */
		DEV_WR_RB(brcm_buses[bus->number].idx_reg,
			  CFG_INDEX(bus, devfn, where & ~3));
		val = DEV_RD(brcm_buses[bus->number].data_reg);
	}

	shift = (where & 3) << 3;
	mask = (0xffffffff >> ((4 - size) << 3)) << shift;

	DEV_WR_RB(brcm_buses[bus->number].idx_reg,
		  CFG_INDEX(bus, devfn, where & ~3));
	val = (val & ~mask) | ((data << shift) & mask);
	DEV_WR_RB(brcm_buses[bus->number].data_reg, val);

	return PCIBIOS_SUCCESSFUL;
}


static int brcm_pci_read_config(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *data)
{
	u32 val, mask, shift;

	if (!devfn_ok(bus, devfn))
		return PCIBIOS_FUNC_NOT_SUPPORTED;

	BUG_ON(((where & 3) + size) > 4);

	DEV_WR_RB(brcm_buses[bus->number].idx_reg,
		  CFG_INDEX(bus, devfn, where & ~3));
	val = DEV_RD(brcm_buses[bus->number].data_reg);

	shift = (where & 3) << 3;
	mask = (0xffffffff >> ((4 - size) << 3)) << shift;

	*data = (val & mask) >> shift;
	return PCIBIOS_SUCCESSFUL;
}




/***********************************************************************
 * PCI slot to IRQ mappings (aka "fixup")
 ***********************************************************************/
static int pcie_irq[] = { 0, 0, 0, 0, };  /* four entries per slot */

static int __init brcm_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	if (dev->bus->number == BRCM_BUSNO_PCIE) {
		if ((pin - 1) > 3)
			return 0;
		return pcie_irq[pin - 1];
	}

	return 0;
}


/* Do platform specific device initialization at pci_enable_device() time */
int pcibios_plat_dev_init(struct pci_dev *dev)
{
	return PCIBIOS_SUCCESSFUL;
}




/***********************************************************************
 * Per-device initialization
 ***********************************************************************/
static void __attribute__((__section__("pci_fixup_early")))
brcm_pcibios_fixup(struct pci_dev *dev)
{
	int slot = PCI_SLOT(dev->devfn);

	pr_info("found device %04x:%04x on %s bus, slot %d (irq %d)\n",
		dev->vendor, dev->device, brcm_buses[dev->bus->number].name,
		slot, brcm_map_irq(dev, slot, 1));

	/* zero out the BARs and let Linux assign an address */
	pci_write_config_dword(dev, PCI_COMMAND, 0);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_0, 0);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_1, 0);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_2, 0);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_3, 0);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_4, 0);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_5, 0);
	pci_write_config_dword(dev, PCI_INTERRUPT_LINE, 0);
}
DECLARE_PCI_FIXUP_EARLY(PCI_ANY_ID, PCI_ANY_ID, brcm_pcibios_fixup);




/***********************************************************************
 * DMA address remapping
 ***********************************************************************/
static dma_addr_t brcm_phys_to_pci(struct device *dev, unsigned long phys)
{
	/* for now (32bit addrs) */
	return phys;
}

unsigned long plat_dma_addr_to_phys(struct device *dev, dma_addr_t dma_addr)
{
	/* for now (32bit addrs) */
	return dma_addr;
}

dma_addr_t plat_map_dma_mem(struct device *dev, void *addr, size_t size)
{
	return brcm_phys_to_pci(dev, virt_to_phys(addr));
}

dma_addr_t plat_map_dma_mem_page(struct device *dev, struct page *page)
{
	return brcm_phys_to_pci(dev, page_to_phys(page));
}




/***********************************************************************
 * PCI Platform Driver
 ***********************************************************************/
static int __init brcm_pci_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	const u32 *imap_prop;
	int len, i, irq_offset;

	imap_prop = of_get_property(node, "interrupt-map", &len);
	irq_offset = irq_of_parse_and_map(node, 0);
	if (!imap_prop || !irq_offset)
		return -EINVAL;

	for (i = 0; i < 4 && i*4 < len; i++)
		pcie_irq[i] = irq_offset
			+ of_read_number(imap_prop + (i * 7 + 5), 1);
	brcm_buses[BRCM_BUSNO_PCIE].node = node;
	brcm_pci_init();
	return 0;
}


static const struct of_device_id brcm_pci_match[] = {
	{ .compatible = "brcm,pci-plat-dev" },
	{},
};
MODULE_DEVICE_TABLE(of, brcm_pci_match);


static struct platform_driver brcm_pci_driver = {
	.probe = brcm_pci_probe,
	.driver = {
		.name = "brcm-pci",
		.owner = THIS_MODULE,
		.of_match_table = brcm_pci_match,
	},
};


int __init brcm_pcibios_init(void)
{
	int ret;
	ret = platform_driver_register(&brcm_pci_driver);
	if (ret)
		pr_info("brcm-pci: Error registering platform driver!");
	return ret;
}


arch_initcall(brcm_pcibios_init);
