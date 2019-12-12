// SPDX-License-Identifier: GPL-2.0+
/*******************************************************************************
 *
 * CTU CAN FD IP Core
 * Copyright (C) 2015-2018
 *
 * Authors:
 *     Ondrej Ille <ondrej.ille@gmail.com>
 *     Martin Jerabek <martin.jerabek01@gmail.com>
 *     Jaroslav Beran <jara.beran@gmail.com>
 *
 * Project advisors:
 *     Jiri Novak <jnovak@fel.cvut.cz>
 *     Pavel Pisa <pisa@cmp.felk.cvut.cz>
 *
 * Department of Measurement         (http://meas.fel.cvut.cz/)
 * Faculty of Electrical Engineering (http://www.fel.cvut.cz)
 * Czech Technical University        (http://www.cvut.cz/)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 ******************************************************************************/

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "ctu_can_fd.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Martin Jerabek");
MODULE_DESCRIPTION("CTU CAN FD for platform");

#define DRV_NAME	"ctucanfd"

static void ctucan_platform_set_drvdata(struct device *dev,
										struct net_device *ndev)
{
	struct platform_device *pdev = container_of(dev, struct platform_device,
												dev);

	platform_set_drvdata(pdev, ndev);
}

/**
 * ctucan_platform_probe - Platform registration call
 * @pdev:	Handle to the platform device structure
 *
 * This function does all the memory allocation and registration for the CAN
 * device.
 *
 * Return: 0 on success and failure value on error
 */
static int ctucan_platform_probe(struct platform_device *pdev)
{
	struct resource *res; /* IO mem resources */
	struct device	*dev = &pdev->dev;
	void __iomem *addr;
	int ret;
	unsigned int ntxbufs;
	int irq;

	/* Get the virtual base address for the device */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(addr)) {
		dev_err(dev, "Cannot remap address.\n");
		ret = PTR_ERR(addr);
		goto err;
	}
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "Cannot find interrupt.\n");
		ret = irq;
		goto err;
	}

	#if 0
	/* tntxbufs should be used in future */
	ret = of_property_read_u32(pdev->dev.of_node, "tntxbufs", &ntxbufs);
	if (ret < 0)
		goto err;
	#endif

	ntxbufs = 4;

	ret = ctucan_probe_common(dev, addr, irq, ntxbufs, 0,
							  1, ctucan_platform_set_drvdata);

	if (ret < 0)
		platform_set_drvdata(pdev, NULL);

err:
	return ret;
}

/**
 * ctucan_platform_remove - Unregister the device after releasing the resources
 * @pdev:	Handle to the platform device structure
 *
 * This function frees all the resources allocated to the device.
 * Return: 0 always
 */
static int ctucan_platform_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct ctucan_priv *priv = netdev_priv(ndev);

	netdev_dbg(ndev, "ctucan_remove");

	unregister_candev(ndev);
	pm_runtime_disable(&pdev->dev);
	netif_napi_del(&priv->napi);
	free_candev(ndev);

	return 0;
}

static const struct dev_pm_ops ctucan_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ctucan_suspend, ctucan_resume)
};

/* Match table for OF platform binding */
static const struct of_device_id ctucan_of_match[] = {
	{ .compatible = "ctu,canfd-2", },
	{ .compatible = "ctu,ctucanfd", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, ctucan_of_match);

static struct platform_driver ctucanfd_driver = {
	.probe	= ctucan_platform_probe,
	.remove	= ctucan_platform_remove,
	.driver	= {
		.name = DRV_NAME,
		.pm = &ctucan_dev_pm_ops,
		.of_match_table	= ctucan_of_match,
	},
};

module_platform_driver(ctucanfd_driver);
