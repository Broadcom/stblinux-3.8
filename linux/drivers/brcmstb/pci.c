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

/* Broadcom PCIE Offsets */
#define PCIE_RC_CFG_PCIE_LINK_CAPABILITY		0x00b8
#define PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL		0x00bc
#define PCIE_RC_CFG_PCIE_ROOT_CAP_CONTROL		0x00c8
#define PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL_2		0x00dc
#define PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1		0x0188
#define PCIE_RC_CFG_PRIV1_ID_VAL3			0x043c
#define PCIE_RC_DL_MDIO_ADDR				0x1100
#define PCIE_RC_DL_MDIO_WR_DATA				0x1104
#define PCIE_RC_DL_MDIO_RD_DATA				0x1108
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
#define PCIE_MISC_HARD_PCIE_HARD_DEBUG			0x4204
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

static struct hw_pci brcm_pcie_hw __initdata = {
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
	bool			suspended;
	int			busnr_start;
	int			busnr_end;
	struct clk		*clk;
	struct device_node	*dn;
	int			pcie_irq[4];
	int			num_out_wins;
	bool			ssc;
	int			gen;
	struct brcm_window	out_wins[BRCM_NUM_PCI_OUT_WINS];
	struct pci_sys_data	*sys;
	struct device		*dev;
} brcm_buses[BRCM_MAX_PCI_CONTROLLERS];

static int brcm_num_pci_controllers;
static int next_busnr;
static int num_memc;
static void turn_off(void __iomem *base);


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


/* negative return value indicates error */
static int mdio_read(void __iomem *base, u8 phyad, u8 regad)
{
	u32 data = ((phyad & 0xf) << 16)
		| (regad & 0x1f)
		| 0x100000;

	__raw_writel(data, base + PCIE_RC_DL_MDIO_ADDR);
	__raw_readl(base + PCIE_RC_DL_MDIO_ADDR);

	data = __raw_readl(base + PCIE_RC_DL_MDIO_RD_DATA);
	if (!(data & 0x80000000)) {
		mdelay(1);
		data = __raw_readl(base + PCIE_RC_DL_MDIO_RD_DATA);
	}
	return (data & 0x80000000) ? (data & 0xffff) : -EIO;
}


/* negative return value indicates error */
static int mdio_write(void __iomem *base, u8 phyad, u8 regad, u16 wrdata)
{
	u32 data = ((phyad & 0xf) << 16) | (regad & 0x1f);

	__raw_writel(data, base + PCIE_RC_DL_MDIO_ADDR);
	__raw_readl(base + PCIE_RC_DL_MDIO_ADDR);

	__raw_writel(0x80000000 | wrdata, base + PCIE_RC_DL_MDIO_WR_DATA);
	data = __raw_readl(base + PCIE_RC_DL_MDIO_WR_DATA);
	if (!(data & 0x80000000)) {
		mdelay(1);
		data = __raw_readl(base + PCIE_RC_DL_MDIO_WR_DATA);
	}
	return (data & 0x80000000) ? 0 : -EIO;
}


static int busnr_to_nr(int busnr)
{
	int i;

	for (i = 0; i < brcm_num_pci_controllers; i++) {
		struct brcm_pci_bus *bus = &brcm_buses[i];
		if (bus->busnr_start >= 0
		    && bus->busnr_start <= busnr
		    && busnr <= bus->busnr_end)
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


/* configures device for ssc mode; negative return value indicates error */
static int set_ssc(void __iomem *base)
{
	int tmp;
	u16 wrdata;

	tmp = mdio_write(base, 0, 0x1f, 0x1100);
	if (tmp < 0)
		return tmp;

	tmp = mdio_read(base, 0, 2);
	if (tmp < 0)
		return tmp;

	wrdata = ((u16)tmp & 0x3fff) | 0xc000;
	tmp = mdio_write(base, 0, 2, wrdata);
	if (tmp < 0)
		return tmp;

	mdelay(1);
	tmp = mdio_read(base, 0, 1);
	if (tmp < 0)
		return tmp;

	return 0;
}


/* returns 0 if in ssc mode, 1 if not, <0 on error */
static int is_ssc(void __iomem *base)
{
	int tmp = mdio_write(base, 0, 0x1f, 0x1100);
	if (tmp < 0)
		return tmp;
	tmp = mdio_read(base, 0, 1);
	if (tmp < 0)
		return tmp;
	return (tmp & 0xc00) == 0xc00 ? 0 : 1;
}


/* limits operation to a specific generation (1, 2, or 3) */
static void set_gen(void __iomem *base, int gen)
{
	wr_fld(base + PCIE_RC_CFG_PCIE_LINK_CAPABILITY, 0xf, 0, gen);
	wr_fld(base + PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL_2, 0xf, 0, gen);
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

	/* field: SCB0_SIZE = 1 Gb */
	wr_fld(base + PCIE_MISC_MISC_CTRL, 0xf8000000, 27, 0x0f);
	/* field: SCB1_SIZE = 1 Gb */
	if (num_memc > 1)
		wr_fld(base + PCIE_MISC_MISC_CTRL, 0x07c00000, 22, 0x0f);
	/* field: SCB2_SIZE = 1 Gb */
	if (num_memc > 2)
		wr_fld(base + PCIE_MISC_MISC_CTRL, 0x0000001f, 0, 0x0f);

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

	if (bus->ssc)
		if (set_ssc(base))
			dev_err(bus->dev, "error while configuring ssc mode\n");
	if (bus->gen)
		set_gen(base, bus->gen);

	/* take the EP device out of reset */
	/* field: PCIE_SW_PERST = 0 */
	wr_fld_rb(base + PCIE_RGR1_SW_INIT_1, 0x00000001, 0, 0);
}


static int brcm_setup_pcie_bridge(int nr, struct pci_sys_data *sys)
{
	struct brcm_pci_bus *bus = &brcm_buses[nr];
	void __iomem *base = bus->base;
	const int limit = bus->suspended ? 1000 : 100;
	struct clk *clk;
	unsigned status;
	static const char *link_speed[4] = { "???", "2.5", "5.0", "8.0" };
	int i, j;

	bus->sys = sys;
	if (!bus->suspended) {
		bus->busnr_start = sys->busnr;
		bus->busnr_end = 0xff;
	}

	/* Give the RC/EP time to wake up, before trying to configure RC.
	 * Intermittently check status for link-up, up to a total of 100ms
	 * when we don't know if the device is there, and up to 1000ms if
	 * we do know the device is there. */
	for (i = 1, j = 0; j < limit && !is_pcie_link_up(nr); j += i, i = i*2)
		mdelay(i + j > limit ? limit - j : i);

	if (!is_pcie_link_up(nr)) {
		dev_info(bus->dev, "link down\n");
		goto FAIL;
	}

	/* For config space accesses on the RC, show the right class for
	 * a PCI-PCI bridge */
	wr_fld_rb(base + PCIE_RC_CFG_PRIV1_ID_VAL3, 0x00ffffff, 0, 0x060400);

	if (!bus->suspended)
		for (i = 0; i < bus->num_out_wins; i++)
			pci_add_resource_offset(&sys->resources,
					&bus->out_wins[i].pcie_iomem_res,
					sys->mem_offset);

	status = __raw_readl(base + PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL);
	dev_info(bus->dev, "link up, %s Gbps x%u\n",
		 link_speed[((status & 0x000f0000) >> 16) & 0x3],
		 (status & 0x03f00000) >> 20);

	if (bus->ssc && is_ssc(base) != 0)
		dev_err(bus->dev, "failed to enter ssc mode\n");

	/* Enable configuration request retry (see pci_scan_device()) */
	/* field RC_CRS_EN = 1 */
	wr_fld(base + PCIE_RC_CFG_PCIE_ROOT_CAP_CONTROL, 0x00000010, 4, 1);

	/* PCIE->SCB endian mode for BAR */
	/* field ENDIAN_MODE_BAR2 = DATA_ENDIAN */
	wr_fld_rb(base + PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1, 0x0000000c, 2,
		  DATA_ENDIAN);

	return 1;
FAIL:
#if defined(CONFIG_PM)
	turn_off(base);
#endif
	clk = brcm_buses[nr].clk;
	if (bus->suspended)
		clk_disable(clk);
	else {
		clk_disable_unprepare(clk);
		clk_put(clk);
		bus->busnr_start = -1;
		bus->busnr_end = -1;
	}
	return 0;

}


struct pci_bus __init *brcm_pci_sys_scan_bus(int nr, struct pci_sys_data *sys)
{
	struct pci_bus *pbus;
	struct brcm_pci_bus *bus = &brcm_buses[nr];

	dev_info(bus->dev, "scanning root bus %d, busnr %d\n", nr, sys->busnr);
	pbus = pci_scan_root_bus(bus->dev, sys->busnr, &brcm_pci_ops, sys,
				 &sys->resources);
	bus->busnr_end = pbus->busn_res.end;
	next_busnr = bus->busnr_end + 1;
	return pbus;
}


#if defined(CONFIG_PM)
/*
 * syscore device to handle PCIe bus suspend and resume
 */

static void turn_off(void __iomem *base)
{
	/* Reset endpoint device */
	wr_fld_rb(base + PCIE_RGR1_SW_INIT_1, 0x00000001, 0, 1);
	/* SERDES_IDDQ = 1 */
	wr_fld_rb(base + PCIE_MISC_HARD_PCIE_HARD_DEBUG, 0x08000000,
		  27, 1);
	/* Shutdown PCIe bridge */
	wr_fld_rb(base + PCIE_RGR1_SW_INIT_1, 0x00000002, 1, 1);
}


static int pcie_suspend(void)
{
	int i;

	for (i = 0; i < brcm_num_pci_controllers; i++) {
		struct brcm_pci_bus *bus = &brcm_buses[i];
		void __iomem *base = bus->base;

		if (bus->busnr_start < 0)
			continue;

		turn_off(base);
		clk_disable(bus->clk);
		bus->suspended = true;
	}
	return 0;
}


static void pcie_resume(void)
{
	int i;

	for (i = 0; i < brcm_num_pci_controllers; i++) {
		struct brcm_pci_bus *bus = &brcm_buses[i];
		void __iomem *base = bus->base;

		if (bus->busnr_start < 0)
			continue;

		clk_enable(bus->clk);

		/* Take bridge out of reset so we can access the SERDES reg */
		wr_fld_rb(base + PCIE_RGR1_SW_INIT_1, 0x00000002, 1, 0);

		/* SERDES_IDDQ = 0 */
		wr_fld_rb(base + PCIE_MISC_HARD_PCIE_HARD_DEBUG, 0x08000000,
			  27, 0);
		/* wait for serdes to be stable */
		udelay(100);

		brcm_pcie_setup_early(i);
	}

	for (i = 0; i < brcm_num_pci_controllers; i++) {
		struct brcm_pci_bus *bus = &brcm_buses[i];

		if (bus->busnr_start < 0)
			continue;

		brcm_setup_pcie_bridge(i, bus->sys);
		bus->suspended = false;
	}
}

static struct syscore_ops pcie_pm_ops = {
	.suspend        = pcie_suspend,
	.resume         = pcie_resume,
};
#endif


/***********************************************************************
 * Read/write PCI configuration registers
 ***********************************************************************/
static int cfg_index(int busnr, int devfn, int reg)
{
	return ((PCI_SLOT(devfn) & 0x1f) << PCI_SLOT_SHIFT)
		| ((PCI_FUNC(devfn) & 0x07) << PCI_FUNC_SHIFT)
		| (busnr << PCI_BUSNUM_SHIFT)
		| (reg & ~3);
}

static u32 read_config(void __iomem *base, int cfg_idx)
{
	__raw_writel(cfg_idx, IDX_ADDR(base));
	__raw_readl(IDX_ADDR(base));
	return __raw_readl(DATA_ADDR(base));
}

static void write_config(void __iomem *base, int cfg_idx, u32 val)
{
	__raw_writel(cfg_idx, IDX_ADDR(base));
	__raw_readl(IDX_ADDR(base));
	__raw_writel(val, DATA_ADDR(base));
	__raw_readl(DATA_ADDR(base));
}


static int brcm_pci_write_config(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 data)
{
	u32 val = 0, mask, shift;
	int nr = busnr_to_nr(bus->number);
	void __iomem *base;
	bool rc_access;
	int idx;

	if (nr < 0 || !is_pcie_link_up(nr))
		return PCIBIOS_DEVICE_NOT_FOUND;

	base = brcm_buses[nr].base;
	rc_access = bus->number == brcm_buses[nr].busnr_start;
	idx = cfg_index(bus->number, devfn, where);
	BUG_ON(((where & 3) + size) > 4);

	if (rc_access && PCI_SLOT(devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (size < 4) {
		/* partial word - read, modify, write */
		if (rc_access)
			val = __raw_readl(base + (where & ~3));
		else
			val = read_config(base, idx);
	}

	shift = (where & 3) << 3;
	mask = (0xffffffff >> ((4 - size) << 3)) << shift;
	val = (val & ~mask) | ((data << shift) & mask);

	if (rc_access) {
		__raw_writel(val, base + (where & ~3));
		__raw_readl(base + (where & ~3));
	} else {
		write_config(base, idx, val);
	}
	return PCIBIOS_SUCCESSFUL;
}


static int brcm_pci_read_config(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *data)
{
	u32 val, mask, shift;
	int nr = busnr_to_nr(bus->number);
	void __iomem *base;
	bool rc_access;
	int idx;

	if (nr < 0 || !is_pcie_link_up(nr))
		return PCIBIOS_DEVICE_NOT_FOUND;

	base = brcm_buses[nr].base;
	rc_access = bus->number == brcm_buses[nr].busnr_start;
	idx = cfg_index(bus->number, devfn, where);
	BUG_ON(((where & 3) + size) > 4);

	if (rc_access && PCI_SLOT(devfn)) {
		*data = 0xffffffff;
		return PCIBIOS_FUNC_NOT_SUPPORTED;
	}

	if (rc_access)
		val = __raw_readl(base + (where & ~3));
	else
		val = read_config(base, idx);

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
	struct brcm_pci_bus *bus = &brcm_buses[nr];

	dev_info(bus->dev,
		 "found device %04x:%04x on bus %d (%s), slot %d (irq %d)\n",
		 dev->vendor, dev->device, dev->bus->number, bus->name,
		 slot, brcm_map_irq(dev, slot, 1));
}
DECLARE_PCI_FIXUP_EARLY(PCI_ANY_ID, PCI_ANY_ID, brcm_pcibios_fixup);


/***********************************************************************
 * PCI Platform Driver
 ***********************************************************************/
static int __init brcm_pci_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node, *mdn;
	const u32 *imap_prop;
	int len, i, irq_offset, rlen, pna, np, ret;
	struct brcm_pci_bus *bus = &brcm_buses[brcm_num_pci_controllers];
	struct resource *r;
	const u32 *ranges;
	void __iomem *base;
	u32 tmp;

	/* 'num_memc' will be set only by the first controller, and all
	 * other controllers will use the value set by the first. */
	if (num_memc == 0)
		for_each_compatible_node(mdn, NULL, "brcm,brcmstb-memc")
			if (of_device_is_available(mdn))
				num_memc++;

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
	bus->suspended = false;
	bus->clk = of_clk_get_by_name(dn, "sw_pcie");
	if (IS_ERR(bus->clk)) {
		dev_err(&pdev->dev, "could not get clock\n");
		bus->clk = NULL;
	}
	ret = clk_prepare_enable(bus->clk);
	if (ret) {
		dev_err(&pdev->dev, "could not enable clock\n");
		return ret;
	}
	bus->dn = dn;
	bus->base = base;
	bus->dev = &pdev->dev;
	bus->gen = 0;

	ret = of_property_read_u32(dn, "brcm,gen", &tmp);
	if (ret == 0) {
		if (tmp > 0 && tmp < 3)
			bus->gen = (int) tmp;
		else
			dev_warn(bus->dev, "bad DT value for prop 'brcm,gen");
	} else if (ret != -EINVAL) {
		dev_warn(bus->dev, "error reading DT prop 'brcm,gen");
	}

	bus->ssc = of_property_read_bool(dn, "brcm,ssc");

	ranges = of_get_property(dn, "ranges", &rlen);
	if (ranges == NULL) {
		dev_err(bus->dev, "no ranges property in dev tree.\n");
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
				"request PCIe memory resource failed\n");
			return -EIO;
		}
	}

	/*
	 * Starts PCIe link negotiation immediately at kernel boot time.  The
	 * RC is supposed to give the endpoint device 100ms to settle down
	 * before attempting configuration accesses.  So we let the link
	 * negotiation happen in the background instead of busy-waiting.
	 */
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

	for (i = 0; i < BRCM_MAX_PCI_CONTROLLERS; i++) {
		brcm_buses[i].busnr_start = -1;
		brcm_buses[i].busnr_end = -1;
	}

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
