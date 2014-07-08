/*
 * sdhci-brcmstb.c Support for SDHCI on Broadcom SoC's
 *
 * Copyright (C) 2013 Broadcom Corporation
 *
 * Author: Al Cooper <acooper@broadcom.com>
 * Based on sdhci-dove.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/brcmstb/brcmstb.h>

#include "sdhci-pltfm.h"

#if defined(CONFIG_BCM7445C0) || defined(CONFIG_BCM7439A0) || \
	defined(CONFIG_BCM7366A0)
/*
 * HW7445-1183
 * Setting the RESET_ALL or RESET_DATA bits will hang the SDIO
 * core so don't allow these bits to be set. This workaround
 * allows the driver to be used for development and testing
 * but will prevent recovery from normally recoverable errors
 * and should NOT be used in production systems.
 */
static void sdhci_brcmstb_writeb(struct sdhci_host *host, u8 val, int reg)
{
	if (reg == SDHCI_SOFTWARE_RESET)
		val &= ~(SDHCI_RESET_ALL | SDHCI_RESET_DATA);
	writeb(val, host->ioaddr + reg);
}

static struct sdhci_ops sdhci_brcmstb_ops = {
	.write_b	= sdhci_brcmstb_writeb,
};

static struct sdhci_pltfm_data sdhci_brcmstb_pdata = {
	.ops	= &sdhci_brcmstb_ops,
};

#else
static struct sdhci_pltfm_data sdhci_brcmstb_pdata = {
};
#endif

static int sdhci_brcmstb_clk_control(struct sdhci_host *host, int on_off)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	if (on_off)
		return clk_prepare_enable(pltfm_host->clk);
	clk_disable_unprepare(pltfm_host->clk);
	return 0;
}

#ifdef CONFIG_PM

static int sdhci_brcmstb_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	int res;

	res = sdhci_suspend_host(host);
	if (res)
		return res;
	sdhci_brcmstb_clk_control(host, 0);
	return 0;
}

static int sdhci_brcmstb_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);

	sdhci_brcmstb_clk_control(host, 1);
	return sdhci_resume_host(host);
}

const struct dev_pm_ops sdhci_brcmstb_pmops = {
	.suspend	= sdhci_brcmstb_suspend,
	.resume		= sdhci_brcmstb_resume,
};

#define SDHCI_BRCMSTB_PMOPS (&sdhci_brcmstb_pmops)

#else /* CONFIG_PM */

#define SDHCI_BRCMSTB_PMOPS NULL

#endif /* CONFIG_PM */

static int sdhci_brcmstb_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	int res;

	res = sdhci_pltfm_register(pdev, &sdhci_brcmstb_pdata);
	if (res)
		return res;
	host = platform_get_drvdata(pdev);
	pltfm_host = sdhci_priv(host);

	pltfm_host->clk = of_clk_get_by_name(dn, "sw_sdio");
	if (IS_ERR(pltfm_host->clk))
		pltfm_host->clk = NULL;

	res = sdhci_brcmstb_clk_control(host, 1);
	if (res) {
		sdhci_pltfm_unregister(pdev);
		return res;
	}
	return 0;
}

static int sdhci_brcmstb_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	int res;
	res = sdhci_pltfm_unregister(pdev);
	sdhci_brcmstb_clk_control(host, 0);
	return res;
}

static const struct of_device_id sdhci_brcmstb_of_match[] = {
	{ .compatible = "brcm,sdhci-brcmstb" },
	{},
};

static struct platform_driver sdhci_brcmstb_driver = {
	.driver		= {
		.name	= "sdhci-brcmstb",
		.owner	= THIS_MODULE,
		.pm	= SDHCI_BRCMSTB_PMOPS,
		.of_match_table = of_match_ptr(sdhci_brcmstb_of_match),
	},
	.probe		= sdhci_brcmstb_probe,
	.remove		= sdhci_brcmstb_remove,
};

module_platform_driver(sdhci_brcmstb_driver);

MODULE_DESCRIPTION("SDHCI driver for Broadcom");
MODULE_AUTHOR("Al Cooper <acooper@broadcom.com>");
MODULE_LICENSE("GPL v2");
