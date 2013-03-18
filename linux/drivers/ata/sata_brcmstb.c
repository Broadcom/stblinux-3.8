/*
 *  sata_brcmstb.c - Broadcom SATA3 AHCI Controller Driver
 *
 *  Copyright (C) 2009 - 2013 Broadcom Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/libata.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/ahci_platform.h>
#include <scsi/scsi_host.h>

#include "sata_brcmstb.h"
#include "ahci.h"

static DEFINE_SPINLOCK(child_pdevs_lock);

static struct pdev_map child_pdevs = {
	.node = LIST_HEAD_INIT(child_pdevs.node),
	.key = NULL,
	.brcm_pdev = NULL,
	.ahci_pdev = NULL
};

static int pdev_map(struct device *ahci_dev,
		    struct platform_device *ahci_pdev,
		    struct platform_device *brcm_pdev)
{
	int status = 0;
	struct pdev_map *head = &child_pdevs;
	struct pdev_map *curr = kmalloc(sizeof(struct pdev_map),
					  GFP_KERNEL);

	if (!curr) {
		pr_err("Cannot allocate map node!\n");
		status = -ENOMEM;
		goto err;
	}

	curr->key = ahci_dev;
	curr->brcm_pdev = brcm_pdev;
	curr->ahci_pdev = ahci_pdev;

	spin_lock(&child_pdevs_lock);

	list_add_tail(&curr->node, &head->node);

	spin_unlock(&child_pdevs_lock);

err:
	return status;
}

static struct pdev_map *__pdev_lookup(struct pdev_map *head,
				       struct device *dev)
{
	struct list_head *pos;
	struct pdev_map *result = NULL;

	list_for_each(pos, &head->node) {
		struct pdev_map *curr = (struct pdev_map *)pos;
		if (curr->key == dev) {
			result = curr;
			break;
		}
	}

	return result;
}

static int pdev_unmap(struct device *dev)
{
	int status = -EFAULT;
	struct pdev_map *entry;

	spin_lock(&child_pdevs_lock);

	entry = __pdev_lookup(&child_pdevs, dev);
	if (entry) {
		list_del(&entry->node);
		kfree(entry);
		status = 0;
	}

	spin_unlock(&child_pdevs_lock);

	if (status)
		pr_err("Cannot unmap entry!\n");

	return status;
}

static int pdev_lookup(struct device *dev,
		       struct platform_device **ahci_pdev,
		       struct platform_device **brcm_pdev)
{
	int status = -EFAULT;
	struct pdev_map *entry;

	spin_lock(&child_pdevs_lock);

	entry = __pdev_lookup(&child_pdevs, dev);
	if (entry) {
		if (brcm_pdev)
			*brcm_pdev = entry->brcm_pdev;
		if (ahci_pdev)
			*ahci_pdev = entry->ahci_pdev;
		status = 0;
	}

	spin_unlock(&child_pdevs_lock);

	if (status)
		pr_err("Cannot locate map entry!\n");

	return status;
}

static int spd_setting_get(const struct sata_brcm_pdata *pdata, int port)
{
	int val = (pdata->phy_force_spd[port / SPD_SETTING_PER_U32]
		   >> SPD_SETTING_SHIFT(port));

	return val & SPD_SETTING_MASK;
}

static void spd_setting_set(struct sata_brcm_pdata *pdata, int port, int val)
{
	int tmp = pdata->phy_force_spd[port / SPD_SETTING_PER_U32];

	pr_debug("Forcing port %d to gen %d speed\n", port, val);

	tmp &= ~(SPD_SETTING_MASK << SPD_SETTING_SHIFT(port));
	tmp |= (val & SPD_SETTING_MASK) << SPD_SETTING_SHIFT(port);
	pdata->phy_force_spd[port / SPD_SETTING_WIDTH] = tmp;
}

static void sata_mdio_wr(void __iomem *addr, u32 port, u32 bank, u32 ofs,
			 u32 msk, u32 value)
{
	u32 tmp;
	void __iomem *base = addr + ((port) * SATA_MDIO_REG_SPACE_SIZE);

	writel(bank, base + SATA_MDIO_BANK_OFFSET);
	tmp = readl(base + SATA_MDIO_REG_OFFSET(ofs));
	tmp = (tmp & msk) | value;
	writel(tmp, base + SATA_MDIO_REG_OFFSET(ofs));
}

static void sata_mdio_wr_legacy(void __iomem *addr, u32 bank, u32 ofs, u32 msk,
				u32 value)
{
	u32 tmp;
	void __iomem *base = addr + (0x8F * 4);

	writel(bank, base);
	/* Read, mask, enable */
	tmp = readl(ofs * 4 + base);
	tmp = (tmp & msk) | value;
	/* Write */
	writel(tmp, ofs * 4 + base);
}

static void brcm_sata3_init_phy(const struct sata_brcm_pdata *pdata, int port)
{
	const u32 phy_base = pdata->phy_base_addr;
	const int ssc_enable = pdata->phy_enable_ssc_mask & (1 << port);
	void __iomem *base;

	base = ioremap(phy_base, SATA_MDIO_REG_LENGTH);
	if (!base) {
		pr_err("%s: Failed to ioremap PHY registers!\n", __func__);
		goto err;
	}

	if (pdata->phy_generation == 0x2800) {
#if defined(CONFIG_BCM7445A0)
		/* The 28nm SATA PHY has incorrect PLL settings upon
		 * chip reset.
		 * The workaround suggested by the H/W team requires
		 * initialization of the PLL registers in order to force
		 * calibration.
		 *
		 * For more information, refer to: HWxxxx
		 */
		const u32 PLL_SM_CTRL_0_DFLT = 0x3089;

		sata_mdio_wr(base, port, PLL_REG_BANK_0, 0x81, 0x00000000,
			     PLL_SM_CTRL_0_DFLT);
		sata_mdio_wr(base, port, PLL_REG_BANK_0, 0x81, 0xFFFFFFFE, 0x0);
#endif
		/* FIXME: Need SSC setup routine for new PHY */
		spd_setting_get(pdata, port);
	} else {
		/* TXPMD_control1 - enable SSC force */
		sata_mdio_wr_legacy(base, TXPMD_0_REG_BANK(port),
				    TXPMD_CONTROL1, 0xFFFFFFFC, 0x00000003);

		/* TXPMD_tx_freq_ctrl_control2 - set fixed min freq */
		sata_mdio_wr_legacy(base, TXPMD_0_REG_BANK(port),
				    TXPMD_TX_FREQ_CTRL_CONTROL2,
				    0xFFFFFC00, 0x000003DF);

		/*
		 * TXPMD_tx_freq_ctrl_control3 - set fixed max freq
		 *  If ssc_enable == 0, center frequencies
		 *  Otherwise, spread spectrum frequencies
		 */
		if (ssc_enable) {
			pr_info("SATA3: enabling SSC on port %d\n", port);
			sata_mdio_wr_legacy(base, TXPMD_0_REG_BANK(port),
					    TXPMD_TX_FREQ_CTRL_CONTROL3,
					    0xFFFFFC00, 0x00000083);
		} else {
			sata_mdio_wr_legacy(base, TXPMD_0_REG_BANK(port),
					    TXPMD_TX_FREQ_CTRL_CONTROL3,
					    0xFFFFFC00, 0x000003DF);
		}
	}

	iounmap(base);

err:
	return;
}

static int brcm_sata3_init_config_endian(struct sata_brcm_pdata *brcm_pdata)
{
	int status = 0;
	void __iomem *top_regs = ioremap(brcm_pdata->top_ctrl_base_addr,
			   SATA_TOP_CTRL_REG_LENGTH);

	if (!top_regs) {
		pr_err("%s: Failed to ioremap TOP registers!\n", __func__);
		status = -EFAULT;
		goto done;
	}

	writel((DATA_ENDIAN << 4) | (DATA_ENDIAN << 2) | (MMIO_ENDIAN << 0),
		top_regs + SATA_TOP_CTRL_BUS_CTRL);

	iounmap(top_regs);

done:
	return status;
}

static int brcm_sata3_init(struct device *dev, void __iomem *addr)
{
	int status = 0;
	int i;
	int ports;
	struct platform_device *brcm_pdev = NULL;
	struct platform_device *ahci_pdev = NULL;
	struct sata_brcm_pdata *brcm_pdata = NULL;

	status = pdev_lookup(dev, &ahci_pdev, &brcm_pdev);
	if (status) {
		pr_err("Cannot locate matching platform device!\n");
		goto done;
	}

	brcm_pdata = brcm_pdev->dev.platform_data;
	ports = fls(readl(addr + HOST_PORTS_IMPL));

	status = brcm_sata3_init_config_endian(brcm_pdata);
	if (status)
		goto done;

	for (i = 0; i < ports; i++)
		brcm_sata3_init_phy(brcm_pdata, i);

done:
	return status;
}

static int brcm_ahci_init(struct device *dev, void __iomem *addr)
{
	return brcm_sata3_init(dev, addr);
}

static void brcm_ahci_exit(struct device *dev)
{
}

static int brcm_ahci_suspend(struct device *dev)
{
	return 0;
}

static int brcm_ahci_resume(struct device *dev)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	void __iomem *addr = hpriv->mmio;

	return brcm_sata3_init(dev, addr);
}

static struct ahci_platform_data brcm_ahci_pdata = {
	.init = &brcm_ahci_init,
	.exit = &brcm_ahci_exit,
	.suspend = &brcm_ahci_suspend,
	.resume = &brcm_ahci_resume,
};

static const struct of_device_id ahci_of_match[] = {
	{.compatible = "brcm,sata3-ahci"},
	{},
};

MODULE_DEVICE_TABLE(of, ahci_of_match);

static int brcm_ahci_parse_dt_prop_u32(struct device_node *of_node,
				       const char *propname, u32 *dst)
{
	int status = 0;
	int tmp;

	if (!of_property_read_u32(of_node, propname, &tmp)) {
		pr_debug("%s = %xh\n", propname, tmp);
		*dst = tmp;
	} else {
		pr_err("Missing %s property!\n", propname);
		status = -EFAULT;
	}

	return status;
}

static int brcm_ahci_parse_dt_node(struct platform_device *pdev)
{
	int status = 0;
	struct sata_brcm_pdata *brcm_pdata = pdev->dev.platform_data;
	struct device_node *of_node = pdev->dev.of_node;
	struct property *prop;
	char *propname;

	/* MANDATORY */
	status = brcm_ahci_parse_dt_prop_u32(of_node, "phy-generation",
					     &brcm_pdata->phy_generation);
	if (status)
		goto err;

	/* MANDATORY */
	status = brcm_ahci_parse_dt_prop_u32(of_node, "phy-base-addr",
					     &brcm_pdata->phy_base_addr);
	if (status)
		goto err;

	/* MANDATORY */
	status = brcm_ahci_parse_dt_prop_u32(of_node, "top-ctrl-base-addr",
					     &brcm_pdata->top_ctrl_base_addr);
	if (status)
		goto err;

	/* OPTIONAL */
	status = brcm_ahci_parse_dt_prop_u32(of_node, "phy-enable-ssc-mask",
					     &brcm_pdata->phy_enable_ssc_mask);
	if (status)
		brcm_pdata->phy_enable_ssc_mask = 0;

	/* OPTIONAL */
	propname = "phy-force-spd";
	prop = of_find_property(of_node, propname, NULL);
	if (prop) {
		if ((prop->length % 8) == 0) {
			int num_entries = prop->length / sizeof(u32) / 2;
			const __be32 *ptr = prop->value;
			while (num_entries-- != 0) {
				const u32 port = be32_to_cpup(ptr++);
				const u32 val = be32_to_cpup(ptr++);
				spd_setting_set(brcm_pdata, port, val);
			}
		} else
			pr_err("%s property is malformed!\n", propname);
	}

err:
	return status;
}

static int brcm_ahci_probe(struct platform_device *pdev)
{
	int status;
	struct platform_device *ahci_pdev;
	static u64 brcm_ahci_dmamask = DMA_BIT_MASK(32);
	struct sata_brcm_pdata *brcm_pdata = NULL;

	brcm_pdata = kmalloc(sizeof(struct sata_brcm_pdata), GFP_KERNEL);
	if (brcm_pdata == NULL) {
		status = -ENOMEM;
		goto done;
	}

	pdev->dev.platform_data = brcm_pdata;

	status = brcm_ahci_parse_dt_node(pdev);
	if (status)
		goto err_free_brcm_pdata;

	ahci_pdev = platform_device_alloc("strict-ahci", 0);
	if (ahci_pdev == NULL) {
		pr_err("Cannot allocate AHCI platform device!\n");
		status = -ENOMEM;
		goto err_free_brcm_pdata;
	}

	status = pdev_map(&ahci_pdev->dev, ahci_pdev, pdev);
	if (status)
		goto err_free_ahci_pdev;

	status = platform_device_add_data(ahci_pdev, &brcm_ahci_pdata,
					  sizeof(brcm_ahci_pdata));
	if (status)
		goto err_free_ahci_pdev;

	status = platform_device_add_resources(ahci_pdev,
					       pdev->resource,
					       pdev->num_resources);
	if (status)
		goto err_free_ahci_pdev;

	ahci_pdev->dev.dma_mask = &brcm_ahci_dmamask;
	ahci_pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	status = platform_device_add(ahci_pdev);
	if (status)
		goto err_free_ahci_pdev;

	/* Keep reference to the "child" platform device */
	brcm_pdata->ahci_pdev = ahci_pdev;

	goto done;

err_free_ahci_pdev:
	pdev_unmap(&ahci_pdev->dev);
	platform_device_put(ahci_pdev);

err_free_brcm_pdata:
	kfree(brcm_pdata);

done:
	return status;
}

static int brcm_ahci_remove(struct platform_device *pdev)
{
	struct sata_brcm_pdata *brcm_pdata = pdev->dev.platform_data;
	struct platform_device *ahci_pdev = brcm_pdata->ahci_pdev;

	if (ahci_pdev) {
		pdev_unmap(&ahci_pdev->dev);
		platform_device_unregister(brcm_pdata->ahci_pdev);
	}

	kfree(brcm_pdata);
	pdev->dev.platform_data = NULL;

	return 0;
}

static struct platform_driver brcm_ahci_driver = {
	.probe = brcm_ahci_probe,
	.remove = brcm_ahci_remove,
	.driver = {
		   .name = "brcm-ahci",
		   .owner = THIS_MODULE,
		   .of_match_table = ahci_of_match,
		   },
};

module_platform_driver(brcm_ahci_driver);

MODULE_DESCRIPTION("Broadcom SATA3 AHCI Controller Driver");
MODULE_AUTHOR("Marc Carino <mcarino@broadcom.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sata-brcmstb");
