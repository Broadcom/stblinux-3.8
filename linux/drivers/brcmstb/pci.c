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
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/irqdomain.h>
#include <linux/jiffies.h>

/* Broadcom PCIE Offsets */
#define PCIE_RC_CFG_TYPE1_STATUS_COMMAND		0x0004
#define PCIE_RC_CFG_TYPE1_PRI_SEC_BUS_NO		0x0018
#define PCIE_RC_CFG_TYPE1_RC_MEM_BASE_LIMIT		0x0020
#define PCIE_RC_CFG_TYPE1_RC_PREF_BASE_LIMIT		0x0024
#define PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL		0x00bc
#define PCIE_RC_CFG_PCIE_ROOT_CAP_CONTROL		0x00c8
#define PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1		0x0188
#define PCIE_MISC_MISC_CTRL				0x4008
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO		0x400c
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI		0x4010
#define PCIE_MISC_RC_BAR1_CONFIG_LO			0x402c
#define PCIE_MISC_RC_BAR1_CONFIG_HI			0x4030
#define PCIE_MISC_RC_BAR2_CONFIG_LO			0x4034
#define PCIE_MISC_RC_BAR2_CONFIG_HI			0x4038
#define PCIE_MISC_RC_BAR3_CONFIG_LO			0x403c
#define PCIE_MISC_RC_BAR3_CONFIG_HI			0x4040
#define PCIE_MISC_MSI_BAR_CONFIG_LO			0x4044
#define PCIE_MISC_MSI_BAR_CONFIG_HI			0x4048
#define PCIE_MISC_PCIE_STATUS				0x4068
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT	0x4070
#define PCIE_INTR2_CPU_CLEAR				0x4308
#define PCIE_INTR2_CPU_MASK_SET				0x4310
#define PCIE_INTR2_CPU_MASK_CLEAR			0x4314
#define PCIE_EXT_CFG_PCIE_EXT_CFG_INDEX			0x9000
#define PCIE_EXT_CFG_PCIE_EXT_CFG_DATA			0x9004
#define PCIE_RGR1_SW_INIT_1				0x9210

#define BRCM_MAX_PCI_CONTROLLERS	0x2
#define BRCM_NUM_PCI_OUT_WINS		0x4

#define PCI_BUSNUM_SHIFT		20
#define PCI_SLOT_SHIFT			15
#define PCI_FUNC_SHIFT			12

#define IDX_ADDR(base)		((base) + PCIE_EXT_CFG_PCIE_EXT_CFG_INDEX)
#define DATA_ADDR(base)		((base) + PCIE_EXT_CFG_PCIE_EXT_CFG_DATA)

static int brcm_pci_read_config(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *data);
static int brcm_pci_write_config(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 data);

static struct pci_ops brcm_pci_ops = {
	.read = brcm_pci_read_config,
	.write = brcm_pci_write_config,
};

static int brcm_setup_pcie_bridge(int nr, struct pci_sys_data *sys);
struct pci_bus __init *brcm_pci_sys_scan_bus(int nr, struct pci_sys_data *sys);
static int __init brcm_map_irq(const struct pci_dev *dev, u8 slot, u8 pin);

static __initdata struct hw_pci brcm_pcie_hw = {
	.nr_controllers	= 0,
	.setup		= brcm_setup_pcie_bridge,
	.scan		= brcm_pci_sys_scan_bus,
	.map_irq	= brcm_map_irq,
};

struct brcm_window {
	u64 pci_addr;
	u64 size;
	u64 cpu_addr;
	u32 info;
	struct resource pcie_iomem_res;
};


/* Internal Bus Controller Information.*/
static struct brcm_pci_bus {
	void __iomem		*base;
	char			name[8];
	int			busnr;
	unsigned int		hw_busnum;
	struct clk		*clk;
	struct device_node	*dn;
	unsigned long		pcie_wake_up_time_jiffies;
	int			pcie_irq[4];
	int			num_out_wins;
	struct brcm_window	out_wins[BRCM_NUM_PCI_OUT_WINS];
} brcm_buses[BRCM_MAX_PCI_CONTROLLERS];

static int brcm_num_pci_controllers;


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


static int busnr_to_nr(int busnr)
{
	int i;
	for (i = 0; i < brcm_num_pci_controllers; i++) {
		if (brcm_buses[i].busnr == busnr)
			return i;
	}
	return -EINVAL;
}

static void wr_fld(void __iomem *p, u32 mask, int shift, u32 val)
{
	u32 reg = __raw_readl(p);
	reg = (reg & ~mask) | (val << shift);
	__raw_writel(reg, p);
}


static void wr_fld_rb(void __iomem *p, u32 mask, int shift, u32 val)
{
	wr_fld(p, mask, shift, val);
	(void) __raw_readl(p);
}


static void set_pcie_outbound_win(void __iomem *base, unsigned win, u64 start,
				  u64 len)
{
	u32 tmp;

	__raw_writel((u32)(start) + MMIO_ENDIAN,
		     base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO+(win*8));
	__raw_writel((u32)(start >> 32),
		     base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI+(win*8));
	tmp = ((((u32)start) >> 20) << 4)
		| (((((u32)start) + ((u32)len) - 1) >> 20) << 20);
	__raw_writel(tmp,
		     base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT+(win*4));
}


static int is_pcie_link_up(int nr)
{
	void __iomem *base = brcm_buses[nr].base;
	u32 val = __raw_readl(base + PCIE_MISC_PCIE_STATUS);
	return  ((val & 0x30) == 0x30) ? 1 : 0;
}


static void brcm_pcie_setup_early(int nr)
{
	struct brcm_pci_bus *bus = &brcm_buses[nr];
	void __iomem *base = bus->base;
	int i;

	/*
	 * Starts PCIe link negotiation immediately at kernel boot time.  The
	 * RC is supposed to give the endpoint device 100ms to settle down
	 * before attempting configuration accesses.  So we let the link
	 * negotiation happen in the background instead of busy-waiting.
	 */

	/* reset the bridge and the endpoint device */
	/* field: PCIE_BRIDGE_SW_INIT = 1 */
	wr_fld_rb(base + PCIE_RGR1_SW_INIT_1, 0x00000002, 1, 1);
	/* field: PCIE_SW_PERST = 1 */
	wr_fld_rb(base + PCIE_RGR1_SW_INIT_1, 0x00000001, 0, 1);

	/* delay 100us */
	udelay(100);

	/* take the bridge out of reset */
	/* field: PCIE_BRIDGE_SW_INIT = 0 */
	wr_fld_rb(base + PCIE_RGR1_SW_INIT_1, 0x00000002, 1, 0);

	/* enable SCB_MAX_BURST_SIZE | CSR_READ_UR_MODE | SCB_ACCESS_EN */
	__raw_writel(0x81e03000, base + PCIE_MISC_MISC_CTRL);

	for (i = 0; i < bus->num_out_wins; i++) {
		struct brcm_window *w = &bus->out_wins[i];
		set_pcie_outbound_win(base, i, w->cpu_addr, w->size);
	}

	/* set up 4GB PCIE->SCB memory window on BAR2 */
	__raw_writel(0x00000011, base + PCIE_MISC_RC_BAR2_CONFIG_LO);
	__raw_writel(0x00000000, base + PCIE_MISC_RC_BAR2_CONFIG_HI);
	/* field: SCB0_SIZE = 2 Gb */
	wr_fld(base + PCIE_MISC_MISC_CTRL, 0xf8000000, 27, 0x10);
	/* field: SCB1_SIZE = 1 Gb */
	wr_fld(base + PCIE_MISC_MISC_CTRL, 0x07c00000, 22, 0x0f);

	/* disable the PCIE->GISB memory window */
	__raw_writel(0x00000000, base + PCIE_MISC_RC_BAR1_CONFIG_LO);

	/* disable the PCIE->SCB memory window */
	__raw_writel(0x00000000, base + PCIE_MISC_RC_BAR3_CONFIG_LO);

	/* disable MSI (for now...) */
	__raw_writel(0x00000000, base + PCIE_MISC_MSI_BAR_CONFIG_LO);

	/* set up L2 interrupt masks */
	__raw_writel(0x00000000, base + PCIE_INTR2_CPU_CLEAR);
	(void) __raw_readl(base + PCIE_INTR2_CPU_CLEAR);
	__raw_writel(0x00000000, base + PCIE_INTR2_CPU_MASK_CLEAR);
	(void) __raw_readl(base + PCIE_INTR2_CPU_MASK_CLEAR);
	__raw_writel(0xffffffff, base + PCIE_INTR2_CPU_MASK_SET);
	(void) __raw_readl(base + PCIE_INTR2_CPU_MASK_SET);

	/* take the EP device out of reset */
	/* field: PCIE_SW_PERST = 0 */
	wr_fld_rb(base + PCIE_RGR1_SW_INIT_1, 0x00000001, 0, 0);

	/* record the current time, add 100ms */
	brcm_buses[nr].pcie_wake_up_time_jiffies = jiffies
		+ msecs_to_jiffies(100);
}


static int brcm_setup_pcie_bridge(int nr, struct pci_sys_data *sys)
{
	struct brcm_pci_bus *bus = &brcm_buses[nr];
	void __iomem *base = bus->base;
	u32 pcie_out_win_start, pcie_out_win_end;
	struct clk *clk;
	unsigned status;
	int i;

	/* Give the RC/EP time to wake up, before trying to configure RC */
	while (!is_pcie_link_up(nr)
	       && time_before_eq(jiffies,
				 brcm_buses[nr].pcie_wake_up_time_jiffies))
		;

	if (!is_pcie_link_up(nr)) {
		pr_info("PCIe: link down\n");
		goto FAIL;
	}

	for (i = 0; i < bus->num_out_wins; i++) {
		struct brcm_window *w = &bus->out_wins[i];
		pci_add_resource_offset(&sys->resources, &w->pcie_iomem_res,
					sys->mem_offset);
	}

	status = __raw_readl(base + PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL);
	pr_info("PCIe link up, %sGbps x%u\n",
		((status & 0x000f0000) >> 16) == 0x2 ? "5.0" : "2.5",
		(status & 0x03f00000) >> 20);

	/* Enable MEM_SPACE and BUS_MASTER for RC */
	__raw_writel(0x6, base + PCIE_RC_CFG_TYPE1_STATUS_COMMAND);

	/* Set base/limit for outbound transactions.  Assume that
	 * the out windows are contiguous.
	 */
	pcie_out_win_start = bus->out_wins[0].pcie_iomem_res.start;
	pcie_out_win_end = pcie_out_win_start - 1;
	for (i = 0; i < bus->num_out_wins; i++)
		pcie_out_win_end += bus->out_wins[i].pcie_iomem_res.end
			- bus->out_wins[i].pcie_iomem_res.start + 1;

	__raw_writel(((pcie_out_win_end & 0xfff00000)
		      | ((pcie_out_win_start>>16)&0x0000fff0)),
		     base + PCIE_RC_CFG_TYPE1_RC_MEM_BASE_LIMIT);

	/* Disable the prefetch range */
	__raw_writel(0x0000fff0, base + PCIE_RC_CFG_TYPE1_RC_PREF_BASE_LIMIT);

	/* Set pri/sec bus numbers */
	__raw_writel(0x00010100, base + PCIE_RC_CFG_TYPE1_PRI_SEC_BUS_NO);

	/* Enable configuration request retry (see pci_scan_device()) */
	/* field RC_CRS_EN = 1 */
	wr_fld(base + PCIE_RC_CFG_PCIE_ROOT_CAP_CONTROL, 0x00000010, 4, 1);

	/* PCIE->SCB endian mode for BAR */
	/* field ENDIAN_MODE_BAR2 = DATA_ENDIAN */
	wr_fld_rb(base + PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1, 0x0000000c, 2,
		  DATA_ENDIAN);

	return 1;
FAIL:
	clk = brcm_buses[nr].clk;
	if (clk) {
		clk_disable(clk);
		clk_put(clk);
	}
	return 0;

}


struct pci_bus __init *brcm_pci_sys_scan_bus(int nr, struct pci_sys_data *sys)
{
	struct pci_bus *pbus;
	pr_info("PCIe: Scanning Root Bus %d, busnr %d\n", nr, sys->busnr);
	brcm_buses[nr].busnr = sys->busnr;
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
	int i;

	for (i = 0; i < brcm_num_pci_controllers; i++) {
		clk = brcm_buses[i].clk;
		if (clk)
			enable ? clk_enable(clk) : clk_disable(clk);
	}
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
 * Read/write PCI configuration registers
 ***********************************************************************/
static int cfg_index(const struct pci_bus *bus, int devfn, int reg)
{
	int nr = busnr_to_nr(bus->number);
	WARN_ON(nr < 0);
	return ((PCI_SLOT(devfn) & 0x1f) << PCI_SLOT_SHIFT)
		| ((PCI_FUNC(devfn) & 0x07) << PCI_FUNC_SHIFT)
		| (brcm_buses[nr].hw_busnum << PCI_BUSNUM_SHIFT)
		| (reg);
}

static int devfn_ok(struct pci_bus *bus, unsigned int devfn)
{
	/* PCIe: check for link down or invalid slot number */
	int nr = busnr_to_nr(bus->number);
	if (nr >= 0 && (!is_pcie_link_up(nr) || PCI_SLOT(devfn) != 0))
		return 0;

	return 1;	/* OK */
}


static int brcm_pci_write_config(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 data)
{
	u32 val = 0, mask, shift;
	int nr = busnr_to_nr(bus->number);
	void __iomem *base = brcm_buses[nr].base;

	if (!devfn_ok(bus, devfn))
		return PCIBIOS_FUNC_NOT_SUPPORTED;

	BUG_ON(((where & 3) + size) > 4);

	if (size < 4) {
		/* partial word - read, modify, write */
		__raw_writel(cfg_index(bus, devfn, where & ~3), IDX_ADDR(base));
		__raw_readl(IDX_ADDR(base));
		val = __raw_readl(DATA_ADDR(base));
	}

	shift = (where & 3) << 3;
	mask = (0xffffffff >> ((4 - size) << 3)) << shift;
	__raw_writel(cfg_index(bus, devfn, where & ~3), IDX_ADDR(base));
	__raw_readl(IDX_ADDR(base));

	val = (val & ~mask) | ((data << shift) & mask);
	__raw_writel(val, DATA_ADDR(base));
	__raw_readl(DATA_ADDR(base));

	return PCIBIOS_SUCCESSFUL;
}


static int brcm_pci_read_config(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *data)
{
	u32 val, mask, shift;
	int nr = busnr_to_nr(bus->number);
	void __iomem *base = brcm_buses[nr].base;

	if (!devfn_ok(bus, devfn))
		return PCIBIOS_FUNC_NOT_SUPPORTED;

	BUG_ON(((where & 3) + size) > 4);

	__raw_writel(cfg_index(bus, devfn, where & ~3), IDX_ADDR(base));
	__raw_readl(IDX_ADDR(base));
	val = __raw_readl(DATA_ADDR(base));

	shift = (where & 3) << 3;
	mask = (0xffffffff >> ((4 - size) << 3)) << shift;

	*data = (val & mask) >> shift;
	return PCIBIOS_SUCCESSFUL;
}




/***********************************************************************
 * PCI slot to IRQ mappings (aka "fixup")
 ***********************************************************************/
static int __init brcm_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	int nr = busnr_to_nr(dev->bus->number);
	if (nr >= 0) {
		if ((pin - 1) > 3)
			return 0;
		return brcm_buses[nr].pcie_irq[pin - 1];
	}
	return 0;
}


/***********************************************************************
 * Per-device initialization
 ***********************************************************************/
static void __attribute__((__section__("pci_fixup_early")))
brcm_pcibios_fixup(struct pci_dev *dev)
{
	int slot = PCI_SLOT(dev->devfn);
	int nr = busnr_to_nr(dev->bus->number);

	pr_info("found device %04x:%04x on %s bus, slot %d (irq %d)\n",
		dev->vendor, dev->device, brcm_buses[nr].name,
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
 * PCI Platform Driver
 ***********************************************************************/
static int __init brcm_pci_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node;
	const u32 *imap_prop;
	int len, i, irq_offset, rlen, pna, np;
	struct brcm_pci_bus *bus = &brcm_buses[brcm_num_pci_controllers];
	struct resource *r;
	const u32 *ranges;
	void __iomem *base;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r)
		return -EINVAL;

	base = devm_request_and_ioremap(&pdev->dev, r);
	if (!base)
		return -ENOMEM;

	imap_prop = of_get_property(dn, "interrupt-map", &len);
	if (imap_prop == NULL) {
		dev_err(&pdev->dev, "missing interrupt-map\n");
		return -EINVAL;
	}

	irq_offset = irq_of_parse_and_map(dn, 0);
	for (i = 0; i < 4 && i*4 < len; i++)
		bus->pcie_irq[i] = irq_offset
			+ of_read_number(imap_prop + (i * 7 + 5), 1);

	snprintf(bus->name,
		 sizeof(bus->name)-1, "PCIe%d", brcm_num_pci_controllers);
	bus->hw_busnum = 1;
	bus->clk = of_clk_get_by_name(dn, "pcie");
	if (IS_ERR(bus->clk))
		bus->clk = NULL;
	bus->dn = dn;
	bus->base = base;

	ranges = of_get_property(dn, "ranges", &rlen);
	if (ranges == NULL) {
		pr_err("PCIe: no ranges property in dev tree.\n");
		return -EINVAL;
	}
	/* set up CPU->PCIE memory windows (max of four) */
	pna = of_n_addr_cells(dn);
	np = pna + 5;

	bus->num_out_wins = rlen / (np * 4);

	for (i = 0; i < bus->num_out_wins; i++) {
		struct brcm_window *w = &bus->out_wins[i];
		w->info = (u32) of_read_ulong(ranges + 0, 1);
		w->pci_addr = of_read_number(ranges + 1, 2);
		w->cpu_addr = of_translate_address(dn, ranges + 3);
		w->size = of_read_number(ranges + pna + 3, 2);
		ranges += np;

		w->pcie_iomem_res.name	= "External PCIe MEM";
		w->pcie_iomem_res.flags	= IORESOURCE_MEM;
		w->pcie_iomem_res.start	= w->cpu_addr;
		w->pcie_iomem_res.end	= w->cpu_addr + w->size - 1;

		/* Request memory region resources. */
		if (request_resource(&iomem_resource, &w->pcie_iomem_res)) {
			dev_err(&pdev->dev,
			"PCIe: request PCIe Memory resource failed\n");
			return -EIO;
		}
	}

	/* Program PCIE Core Controller Registers.*/
	brcm_pcie_setup_early(brcm_num_pci_controllers);

	brcm_num_pci_controllers++;
	return 0;
}


static const struct of_device_id brcm_pci_match[] = {
	{ .compatible = "brcm,pci-plat-dev" },
	{},
};
MODULE_DEVICE_TABLE(of, brcm_pci_match);


static struct platform_driver __refdata brcm_pci_driver = {
	.probe = brcm_pci_probe,
	.driver = {
		.name = "brcm-pci",
		.owner = THIS_MODULE,
		.of_match_table = brcm_pci_match,
	},
};


int __init brcm_pcibios_init(void)
{
	int i, ret;

	for (i = 0; i < BRCM_MAX_PCI_CONTROLLERS; i++)
		brcm_buses[i].busnr = -1;

	ret = platform_driver_probe(&brcm_pci_driver, brcm_pci_probe);
	if (!ret && brcm_num_pci_controllers > 0) {
		brcm_pcie_hw.nr_controllers = brcm_num_pci_controllers;
#if defined(CONFIG_PM)
		register_syscore_ops(&pcie_pm_ops);
#endif

		pci_common_init(&brcm_pcie_hw);
	}
	return ret;
}


arch_initcall(brcm_pcibios_init);
