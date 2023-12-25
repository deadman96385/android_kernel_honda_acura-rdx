/*
 * Copyright (C) 2015 Cetitec, GmbH. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/**
 * This is the configuration code specific to TI Jacinto6
 * MLB support, using device tree (Open Firmware) for
 * hardware configuration.
 */

#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pm_runtime.h>

#include "mlb150.h"
#include "mlb150_ext.h"
#include "mlb150_int.h"
#include "j6.h"

const struct of_device_id mlb150_of_device_ids[] = {
	{.compatible = "smsc,mlb150"},
	{},
};

int mlb150_get_mlb_io(struct mlb_data *drvdata)
{
	void __iomem *base;
	const struct of_device_id *match;

	match = of_match_device(mlb150_of_device_ids, drvdata->dev);

	if (!match) {
		dev_err(drvdata->dev, "mlb device not found in the device tree\n");
		return -ENOENT;
	}

	drvdata->irq_mlb  = irq_of_parse_and_map(drvdata->dev->of_node, 0);
	drvdata->irq_ahb0 = irq_of_parse_and_map(drvdata->dev->of_node, 1);
	drvdata->irq_ahb1 = irq_of_parse_and_map(drvdata->dev->of_node, 2);
	base = of_iomap(drvdata->dev->of_node, 0);

	dev_dbg(drvdata->dev, "MLB DIM mapped at 0x%p, virq: mlb %u, ahb0 %u, ahb1 %u\n",
		base, drvdata->irq_mlb, drvdata->irq_ahb0, drvdata->irq_ahb1);

	if (unlikely(base == NULL)) {
		dev_err(drvdata->dev, "failed to do ioremap with mlb150 base\n");
		return -ENOMEM;
	}

	drvdata->membase = base;
#ifdef MLB_DIM_REGISTER_OFFSET
	if (MLB_DIM_REGISTER_OFFSET) {
		base += MLB_DIM_REGISTER_OFFSET;
		dev_info(drvdata->dev, "adjusted mlb register base: 0x%p\n", base);
	}
#endif
	drvdata->mlbregs = base;
	return 0;
}

int mlb150_init_mlb_io(struct mlb_data *drvdata)
{
	int ret;

	/* Enable the clock/module so that we can access the registers */
	pm_runtime_enable(drvdata->dev);
	pm_runtime_get_sync(drvdata->dev);

	ret = jacinto6_enableMLBmodule();
	if (ret) {
		dev_err(drvdata->dev, "failed enabling MLB module (%d)\n", ret);
		goto fail;
	}

	ret = jacinto6_6pinMLB();
	if (ret) {
		dev_err(drvdata->dev, "failed initializing MLB IO (%d)\n", ret);
		goto fail;
	}
	return 0;
fail:
	/* Disable the clock/module */
	pm_runtime_put_sync(drvdata->dev);
	pm_runtime_disable(drvdata->dev);

	return ret;
}

void mlb150_free_mlb_io(struct mlb_data *drvdata)
{
	if (drvdata->irq_ahb0) {
		free_irq(drvdata->irq_ahb0, drvdata);
		drvdata->irq_ahb0 = 0;
	}
	if (drvdata->irq_ahb1) {
		free_irq(drvdata->irq_ahb1, drvdata);
		drvdata->irq_ahb1 = 0;
	}
	if (drvdata->irq_mlb) {
		free_irq(drvdata->irq_mlb, drvdata);
		drvdata->irq_mlb = 0;
	}
	if (drvdata->membase) {
		iounmap(drvdata->membase);
		drvdata->membase = NULL;
		drvdata->mlbregs = NULL;
	}
	/* Disable the clock/module */
	pm_runtime_put_sync(drvdata->dev);
	pm_runtime_disable(drvdata->dev);
}

int __init mlb150_init_driver(struct platform_driver *drv, int (*probe)(struct platform_device *))
{
	return platform_driver_probe(drv, probe);
}

void __exit mlb150_exit_driver(struct platform_driver *drv)
{
	platform_driver_unregister(drv);
}

