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
#include <linux/compiler.h>
#include <scsi/scsi_host.h>
#include <linux/string.h>
#include <linux/brcmstb/brcmstb.h>

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

static int brcm_sata3_init_config(void __iomem *ahci_regs,
				  struct platform_device *ahci_pdev,
				  struct sata_brcm_pdata *brcm_pdata)
{
	int status = 0;
	void __iomem *top_regs = NULL;

	top_regs = ioremap(brcm_pdata->top_ctrl_base_addr,
			   SATA_TOP_CTRL_REG_LENGTH);
	if (!top_regs) {
		status = -EFAULT;
		goto done;
	}

	/* Configure endianness */
	writel((DATA_ENDIAN << 4) | (DATA_ENDIAN << 2) | (MMIO_ENDIAN << 0),
		top_regs + SATA_TOP_CTRL_BUS_CTRL);

	if (brcm_pdata->quirks & SATA_BRCM_QK_NONCQ) {
		/* Temporarily allow writing to AHCI RO registers */
		u32 reg = readl(top_regs + SATA_TOP_CTRL_BUS_CTRL);
		reg |= SATA_TOP_CTRL_BUS_CTRL_OVERRIDE_HWINIT;
		writel(reg, top_regs + SATA_TOP_CTRL_BUS_CTRL);

		/* Clear out the NCQ bit so the AHCI driver will not issue
		 * FPDMA/NCQ commands.
		 */
		reg = readl(ahci_regs + HOST_CAP);
		reg &= ~HOST_CAP_NCQ;
		writel(reg, ahci_regs + HOST_CAP);

		/* Re-enable AHCI RO property */
		reg = readl(top_regs + SATA_TOP_CTRL_BUS_CTRL);
		reg &= ~SATA_TOP_CTRL_BUS_CTRL_OVERRIDE_HWINIT;
		writel(reg, top_regs + SATA_TOP_CTRL_BUS_CTRL);
	}

done:
	if (top_regs)
		iounmap(top_regs);

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

	status = brcm_sata3_init_config(addr, ahci_pdev, brcm_pdata);
	if (status)
		goto done;

	for (i = 0; i < ports; i++)
		brcm_sata3_phy_init(brcm_pdata, i);

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
				brcm_sata3_phy_spd_set(brcm_pdata, port, val);
			}
		} else
			pr_err("%s property is malformed!\n", propname);
	}

err:
	return status;
}

static void brcm_ahci_setup_quirks(struct platform_device *pdev)
{
#if defined(CONFIG_BRCMSTB)
	struct sata_brcm_pdata *brcm_pdata = pdev->dev.platform_data;
	const u32 chip_id = BDEV_RD(BCHP_SUN_TOP_CTRL_CHIP_FAMILY_ID);

	brcm_pdata->quirks = 0;

	switch (chip_id) {
	case 0x71450000: /* 7145a0 */
		brcm_pdata->quirks |= SATA_BRCM_QK_ALT_RST;
		brcm_pdata->quirks |= SATA_BRCM_QK_NONCQ;
		break;
	case 0x74390000: /* 7439a0 */
		brcm_pdata->quirks |= SATA_BRCM_QK_ALT_RST;
		break;
	case 0x74450000: /* 7445a0 */
		brcm_pdata->quirks |= SATA_BRCM_QK_INIT_PHY;
		brcm_pdata->quirks |= SATA_BRCM_QK_NONCQ;
		break;
	case 0x74450010: /* 7445b0 */
		brcm_pdata->quirks |= SATA_BRCM_QK_INIT_PHY;
		brcm_pdata->quirks |= SATA_BRCM_QK_NONCQ;
		break;
	default:
		break;
	}
#endif
}

static int setup_ahci_pdata(struct platform_device *pdev,
	struct ahci_platform_data *ahci_pd)
{
	int status = 0;

	memset(ahci_pd, 0, sizeof(*ahci_pd));
	ahci_pd->init = &brcm_ahci_init;
	ahci_pd->exit = &brcm_ahci_exit;
	ahci_pd->suspend = &brcm_ahci_suspend;
	ahci_pd->resume = &brcm_ahci_resume;

	return status;
}

static int brcm_ahci_probe(struct platform_device *pdev)
{
	int status;
	int mapped = 0;
	struct platform_device *ahci_pdev = NULL;
	struct sata_brcm_pdata brcm_pdata;
	struct ahci_platform_data ahci_pdata;
	static u64 brcm_ahci_dmamask = DMA_BIT_MASK(32);

	ahci_pdev = platform_device_alloc("strict-ahci", 0);
	if (ahci_pdev == NULL) {
		pr_err("Cannot allocate AHCI platform device!\n");
		status = -ENOMEM;
		goto err_cleanup;
	}

	memset(&brcm_pdata, 0, sizeof(struct sata_brcm_pdata));

	/*
	 * Configure the Broadcom AHCI wrapper
	 */

	/* Keep reference to the "child" platform device */
	brcm_pdata.ahci_pdev = ahci_pdev;

	status = platform_device_add_data(pdev, &brcm_pdata,
					  sizeof(struct sata_brcm_pdata));
	if (status)
		goto err_cleanup;

	status = brcm_ahci_parse_dt_node(pdev);
	if (status)
		goto err_cleanup;

	brcm_ahci_setup_quirks(pdev);

	/*
	 * Configure the platform AHCI device
	 */
	status = setup_ahci_pdata(pdev, &ahci_pdata);
	if (status)
		goto err_cleanup;

	status = platform_device_add_data(ahci_pdev, &ahci_pdata,
					  sizeof(struct ahci_platform_data));
	if (status)
		goto err_cleanup;

	/* Pass the register addresses over to the AHCI platform device */
	status = platform_device_add_resources(ahci_pdev,
					       pdev->resource,
					       pdev->num_resources);
	if (status)
		goto err_cleanup;

	ahci_pdev->dev.dma_mask = &brcm_ahci_dmamask;
	ahci_pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	status = pdev_map(&ahci_pdev->dev, ahci_pdev, pdev);
	if (status)
		goto err_cleanup;
	else
		mapped = 1;

	/* Ready to handoff configuration to platform AHCI driver */
	status = platform_device_add(ahci_pdev);
	if (status)
		goto err_cleanup;

	goto done;

err_cleanup:
	if (mapped)
		pdev_unmap(&ahci_pdev->dev);

	if (ahci_pdev != NULL)
		platform_device_put(ahci_pdev);

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
