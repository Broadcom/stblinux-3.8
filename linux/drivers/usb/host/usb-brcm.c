/*
 * Copyright (C) 2010 Broadcom Corporation
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

#include <linux/usb.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/clk.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>

#include <linux/usb/hcd.h>

#include "usb-brcm-common-init.h"

static struct clk *usb_clk;

/* FIXME */
#define clk_enable(...) do { } while (0)
#define clk_disable(...) do { } while (0)

struct brcm_usb_instance {
	void __iomem		*ctrl_regs;
	int			ioc;
	int			ipp;
	int			has_xhci;
};

/***********************************************************************
 * Library functions
 ***********************************************************************/

int brcm_usb_probe(struct platform_device *pdev, char *hcd_name,
	const struct hc_driver *hc_driver)
{
	struct resource *res = NULL;
	struct usb_hcd *hcd = NULL;
	int irq, ret, len;

	if (usb_disabled())
		return -ENODEV;

	if (!usb_clk)
		usb_clk = clk_get(NULL, "usb");
	clk_enable(usb_clk);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "platform_get_resource error.\n");
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "platform_get_irq error.\n");
		return -ENODEV;
	}

	/* initialize hcd */
	hcd = usb_create_hcd(hc_driver, &pdev->dev, (char *)hcd_name);
	if (!hcd) {
		dev_err(&pdev->dev, "Failed to create hcd\n");
		return -ENOMEM;
	}

	len = res->end - res->start + 1;
	hcd->regs = ioremap(res->start, len);
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = len;
	ret = usb_add_hcd(hcd, irq, IRQF_DISABLED);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to add hcd\n");
		iounmap(hcd->regs);
		usb_put_hcd(hcd);
		clk_disable(usb_clk);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL(brcm_usb_probe);

int brcm_usb_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	clk_disable(usb_clk);
	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	usb_put_hcd(hcd);

	return 0;
}
EXPORT_SYMBOL(brcm_usb_remove);

void brcm_usb_suspend(struct usb_hcd *hcd)
{
	/* Since all HCs share clock source, once we enable USB clock, all
	   controllers are capable to generate interrupts if enabled. Since some
	   controllers at this time are still marked as non-accessible, this
	   leads to spurious interrupts.
	   To avoid this, disable controller interrupts.
	*/
	disable_irq(hcd->irq);
	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	clk_disable(usb_clk);
}
EXPORT_SYMBOL(brcm_usb_suspend);

void brcm_usb_resume(struct usb_hcd *hcd)
{
	clk_enable(usb_clk);
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	enable_irq(hcd->irq);
}
EXPORT_SYMBOL(brcm_usb_resume);

#ifdef CONFIG_OF

/***********************************************************************
 * DT support for USB instances
 ***********************************************************************/

static int brcm_usb_instance_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node;
	struct resource ctrl_res;
	const u32 *prop;
	struct brcm_usb_instance *priv;
	struct device_node *node;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	dev_set_drvdata(&pdev->dev, priv);

	if (of_address_to_resource(dn, 0, &ctrl_res)) {
		dev_err(&pdev->dev, "can't get USB_CTRL base address\n");
		return -EINVAL;
	}

	priv->ctrl_regs = devm_request_and_ioremap(&pdev->dev, &ctrl_res);
	if (!priv->ctrl_regs) {
		dev_err(&pdev->dev, "can't map register space\n");
		return -EINVAL;
	}

	prop = of_get_property(dn, "ipp", NULL);
	if (prop)
		priv->ipp = be32_to_cpup(prop);

	prop = of_get_property(dn, "ioc", NULL);
	if (prop)
		priv->ioc = be32_to_cpup(prop);

	node = of_find_compatible_node(dn, NULL, "xhci-platform");
	of_node_put(node);
	priv->has_xhci = node != NULL;
	brcm_usb_common_ctrl_xhci_soft_reset((uintptr_t)priv->ctrl_regs, false);
	brcm_usb_common_ctrl_init((uintptr_t)priv->ctrl_regs, priv->ioc,
				priv->ipp, priv->has_xhci);
	return of_platform_populate(dn, NULL, NULL, NULL);
}

#ifdef CONFIG_PM
static int brcm_usb_instance_resume(struct device *dev)
{
	struct brcm_usb_instance *priv = dev_get_drvdata(dev);
	brcm_usb_common_ctrl_xhci_soft_reset((uintptr_t)priv->ctrl_regs, false);
	brcm_usb_common_ctrl_init((uintptr_t)priv->ctrl_regs, priv->ioc,
				priv->ipp, priv->has_xhci);
	return 0;
}


static const struct dev_pm_ops brcm_usb_instance_pmops = {
	.resume_early	= brcm_usb_instance_resume,
};

#define BRCM_USB_INSTANCE_PMOPS (&brcm_usb_instance_pmops)

#else
#define BRCM_USB_INSTANCE_PMOPS NULL
#endif

static const struct of_device_id brcm_usb_instance_match[] = {
	{ .compatible = "brcm,usb-instance" },
	{},
};

static struct platform_driver brcm_usb_instance_driver = {
	.driver = {
		.name = "usb-brcm",
		.bus = &platform_bus_type,
		.of_match_table = of_match_ptr(brcm_usb_instance_match),
		.pm = BRCM_USB_INSTANCE_PMOPS,
	}
};

/*
 * We really don't want to try to undo of_platform_populate(), so it
 * is not possible to unbind/deregister this driver.
 */
static int __init brcm_usb_instance_init(void)
{
	return platform_driver_probe(&brcm_usb_instance_driver,
				     brcm_usb_instance_probe);
}
module_init(brcm_usb_instance_init);

#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("Broadcom USB common functions");
