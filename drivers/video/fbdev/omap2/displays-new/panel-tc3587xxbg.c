/*
 * TLC3587x DPI-to-DSI Driver
 *
 * Copyright (C) 2013 Texas Instruments
 * Author: Marcus Cooksey <mcooksey@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/of_device.h>

#include <video/omapdss.h>
#include <video/omap-panel-data.h>
#include <video/of_display_timing.h>

#define TC3587X_NAME		"tc3587xxbg"
/* LCD TC3587X_RGB to MIPI DSI Bridge registers */
#define TC3587X_SLAVE_ADDR_778      (0x0E)

/* Global(16-bit TC3587X_addressable) */
#define TC3587X_CHIPID		0x0000
#define TC3587X_SYSCTL		0x0002
#define TC3587X_CONFCTL		0x0004
#define TC3587X_VSDLY		0x0006
#define TC3587X_DATAFMT		0x0008
#define TC3587X_GPIOEN		0x000E
#define TC3587X_GPIODIR		0x0010
#define TC3587X_GPIOIN		0x0012
#define TC3587X_GPIOOUT		0x0014
#define TC3587X_PLLCTl0		0x0016
#define TC3587X_PLLCTl1		0x0018
#define TC3587X_CMDBYTE		0x0022
#define TC3587X_PP_MISC		0x0032
#define TC3587X_DSITX_DT	0x0050
#define TC3587X_FIFOSTATUS	0x00F8

/* TX TC3587X_PHY(32-bit addressable) */
#define TC3587X_CLW_DPHYCONTTX	0x0100
#define TC3587X_D0W_DPHYCONTTX	0x0104
#define TC3587X_D1W_DPHYCONTTX	0x0108
#define TC3587X_D2W_DPHYCONTTX	0x010C
#define TC3587X_D3W_DPHYCONTTX	0x0110
#define TC3587X_CLW_CNTRL	0x0140
#define TC3587X_D0W_CNTRL	0x0144
#define TC3587X_D1W_CNTRL	0x0148
#define TC3587X_D2W_CNTRL	0x014C
#define TC3587X_D3W_CNTRL	0x0150

/* TX TC3587X_PPI(32-bit addressable) */
/* DSITX Start Control Register */
#define TC3587X_STARTCNTRL	0x0204
/* DSITX Status Register */
#define TC3587X_DSITXSTATUS	0x0208
/* DSITX Line Initialization Control Register */
#define TC3587X_LINEINITCNT	0x0210
/* SYSLPTX Timing Generation Counter */
#define TC3587X_LPTXTIMECNT	0x0214
/* TCLK_ZERO and TCLK_PREPARE Counter */
#define TC3587X_TCLK_HEADERCNT	0x0218
/* TCLK_TRAIL Counter */
#define TC3587X_TCLK_TRAILCNT	0x021C
/* THS_ZERO and THS_PREPARE Counter */
#define TC3587X_THS_HEADERCNT	0x0220
/* TWAKEUP Counter */
#define TC3587X_TWAKEUP		0x0224
/* TCLK_POST Counter */
#define TC3587X_TCLK_POSTCNT	0x0228
/* THS_TRAIL Counter */
#define TC3587X_THS_TRAILCNT	0x022C
/* TX Voltage Regulator setup Wait Counter */
#define TC3587X_HSTXVREGCNT	0x0230
/* Voltage regulator enable for HSTX Data Lanes */
#define TC3587X_HSTXVREGEN	0x0234
/* TX Option Control */
#define TC3587X_TXOPTIONCNTRL	0x0238
/* BTA Control */
#define TC3587X_BTACNTRL1	0x023C

/*  TX TC3587X_CTRL(32-bit addressable) */
#define TC3587X_DSI_CONTROL		0x040C
#define TC3587X_DSI_STATUS		0x0410
#define TC3587X_DSI_INT			0x0414
#define TC3587X_DSI_INT_ENA		0x0418
#define TC3587X_DSICMD_RXFIFO		0x0430
#define TC3587X_DSI_ACKERR		0x0434
#define TC3587X_DSI_ACKERR_INTENA	0x0438
#define TC3587X_DSI_ACKERR_HALT		0x043C
#define TC3587X_DSI_RXERR		0x0440
#define TC3587X_DSI_RXERR_INTENA	0x0444
#define TC3587X_DSI_RXERR_HALT		0x0448
#define TC3587X_DSI_ERR			0x044C
#define TC3587X_DSI_ERR_INTENA		0x0450
#define TC3587X_DSI_ERR_HALT		0x0454
#define TC3587X_DSI_CONFW		0x0500
#define TC3587X_DSI_RESET		0x0504
#define TC3587X_DSI_INT_CLR		0x050C
#define TC3587X_DSI_START		0x0518

/* DSITX TC3587X_CTRL(16-bit addressable) */
#define TC3587X_DSICMD_TX		0x0600
#define TC3587X_DSICMD_TYPE		0x0602
#define TC3587X_DSICMD_WC		0x0604
#define TC3587X_DSICMD_WD0		0x0610
#define TC3587X_DSICMD_WD1		0x0612
#define TC3587X_DSICMD_WD2		0x0614
#define TC3587X_DSICMD_WD3		0x0616
#define TC3587X_DSI_EVENT		0x0620
#define TC3587X_DSI_VSW			0x0622
#define TC3587X_DSI_VBPR		0x0624
#define TC3587X_DSI_VACT		0x0626
#define TC3587X_DSI_HSW			0x0628
#define TC3587X_DSI_HBPR		0x062A
#define TC3587X_DSI_HACT		0x062C

/* Debug(16-bit TC3587X_addressable) */
#define TC3587X_VBUFCTRL		0x00E0
#define TC3587X_DBG_WIDTH		0x00E2
#define TC3587X_DBG_VBLANK		0x00E4
#define TC3587X_DBG_DATA		0x00E8


struct panel_drv_data {
	struct omap_dss_device dssdev;
	struct omap_dss_device *in;

	struct omap_video_timings videomode;

	struct gpio_desc *enable_gpio;
	struct regmap *regmap_16bit;
	struct regmap *regmap_32bit;
	struct device *dev;

	const struct tc_board_data *board_data;
};

struct tc_board_data {
	struct omap_video_timings timings;
	const unsigned int *init_seq;
	unsigned init_seq_len;
};

static const unsigned int tc_10_inch_init_seq[] = {
	/* Disable Parallel Input */
	TC3587X_CONFCTL, 0x0004, 0,
	/* Assert Reset */
	TC3587X_SYSCTL,  0x0001, 0,
	/* Release Reset, Exit Sleep */
	TC3587X_SYSCTL,  0x0000, 0,
	/* Program DSI Tx PLL */
	TC3587X_PLLCTl0, 0x5127, 0,
	/* FRS[11:10],LBWS[9:8], Clock Enable[4], ResetB[1], PLL En[0] */
	TC3587X_PLLCTl1, 0x0203, 0,
	/* PRD[15:12], FBD[8:0] */
	TC3587X_PLLCTl1, 0x0213, 0,

	/* DSI Input Control */

	/* V/Hsync Delay */
	TC3587X_VSDLY, 0x00D0, 0,
	/* DataFmt[7:4]=3, Loose Pk=0, Rsvrd=1, DSITx_En=1, Rsvrd=1 */
	TC3587X_DATAFMT, 0x0037, 0,
	/* DSTX DataID, RGB888 */
	TC3587X_DSITX_DT, 0x003E, 0,

	/* DSI Tx Phy */

	/* Disable DSI Clock Lane */
	TC3587X_CLW_CNTRL, 0x0000, 0,
	/* Disable DSI Clock Lane0 */
	TC3587X_D0W_CNTRL, 0x0000, 0,
	/* Disable DSI Data Lane1 */
	TC3587X_D1W_CNTRL, 0x0000, 0,
	/* Disable DSI Data Lane2 */
	TC3587X_D2W_CNTRL, 0x0000, 0,
	/* Disable DSI Data Lane3 */
	TC3587X_D3W_CNTRL, 0x0000, 0,

	/* Byte value format is set as big endian */
	/* Disable DSI Data lane3 */

	/* LP11 = 100 us for D-PHY Rx Init */
	TC3587X_LINEINITCNT,	0x0000882C, 1,
	TC3587X_LPTXTIMECNT,	0x00000500, 1,
	TC3587X_TCLK_HEADERCNT,	0x0000061F, 1,
	TC3587X_TCLK_TRAILCNT,	0x00000300, 1,
	TC3587X_THS_HEADERCNT,	0x00000606, 1,
	TC3587X_TWAKEUP,	0x0000884A, 1,
	TC3587X_TCLK_POSTCNT,	0x00000B00, 1,
	TC3587X_THS_TRAILCNT,	0x00000400, 1,
	TC3587X_HSTXVREGEN,	0x00001F00, 1,

	/* Enable Voltage Regulator for CSI */

	/* (4 Data + Clk) Lanes */
	/* [0] = 1, Continuous Clock */
	TC3587X_TXOPTIONCNTRL, 0x00000100, 1,
	/* Disable DSI Data Lane3 */
	TC3587X_BTACNTRL1,     0x05000500, 1,
	/* Start PPI */
	TC3587X_STARTCNTRL,    0x00000100, 1,

	/* DSI Tx Timing Control */

	/* Set Event */
	TC3587X_DSI_EVENT, 0x0001, 0,
	/* Set this register to: Vertical Sync */
	/* `Width + VBPR */
	TC3587X_DSI_VSW,   0x0012, 0,
	/* Set Vertical Active line */
	TC3587X_DSI_VACT,  0x04B0, 0,
	/* (HSW+HBPR)/PClkFreq*ByteClkFreq*#Lanes */
	TC3587X_DSI_HSW,   0x0090, 0,
	/* Set Horizontal Active line byte count */
	TC3587X_DSI_HACT,  0x1680, 0,

	/* Start DSI Tx */
	TC3587X_DSI_START, 0x00000100, 1,
	TC3587X_DSI_CONFW, 0x00A3A700, 1,
	TC3587X_DSI_CONFW, 0x00C30180, 1,

	/* DSI Panel Programming */
	/* Set Event Mode */
	TC3587X_CONFCTL, 0x0044, 0,
};

static const struct tc_board_data tc_10_inch_data = {
	.timings = {
		.x_res          = 1920,
		.y_res          = 1200,

		.pixelclock     = 154000000,

		.hfp            = 112,
		.hsw            = 16,
		.hbp            = 32,

		.vfp            = 17,
		.vsw            = 2,
		.vbp            = 16,

		.vsync_level    = OMAPDSS_SIG_ACTIVE_LOW,
		.hsync_level    = OMAPDSS_SIG_ACTIVE_LOW,
		.data_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
		.de_level       = OMAPDSS_SIG_ACTIVE_HIGH,
		.sync_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
	},
	.init_seq = tc_10_inch_init_seq,
	.init_seq_len = ARRAY_SIZE(tc_10_inch_init_seq),
};

static int tc_init(struct panel_drv_data *ddata)
{
	struct regmap *map16 = ddata->regmap_16bit;
	struct regmap *map32 = ddata->regmap_32bit;
	struct device *dev = ddata->dev;
	unsigned int i, len, value;
	unsigned int *buf;
	unsigned int *buf2;
	const unsigned int *seq;

	buf =  devm_kzalloc(dev, sizeof(unsigned int), GFP_KERNEL);
	buf2 = devm_kzalloc(dev, sizeof(unsigned int), GFP_KERNEL);

	len = ddata->board_data->init_seq_len;
	seq = ddata->board_data->init_seq;

	for (i = 0; i < len; i += 3) {
		if (seq[i + 2]  == 0) { /* 16-bit register access */
			regmap_write(map16, seq[i], seq[i + 1]);
			regmap_read(map16, seq[i], &value);
		} else {   /*  32 -bit regster access  */
			memcpy(buf, (seq + (i + 1)), 4);
			regmap_raw_write(map32, seq[i], buf, 4);
			regmap_raw_read(map32, seq[i], buf2, 4);
		}

		if (seq[i] == TC3587X_PLLCTl1)
			usleep_range(10000, 15000);
	}

	return 0;
}

static int tc_uninit(struct panel_drv_data *ddata)
{
	return 0;
}

#define to_panel_data(p) container_of(p, struct panel_drv_data, dssdev)

static int panel_dpi_connect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	if (omapdss_device_is_connected(dssdev))
		return 0;

	r = in->ops.dpi->connect(in, dssdev);
	if (r)
		return r;

	return 0;
}

static void panel_dpi_disconnect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (!omapdss_device_is_connected(dssdev))
		return;

	in->ops.dpi->disconnect(in, dssdev);
}

static int panel_dpi_enable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	if (!omapdss_device_is_connected(dssdev))
		return -ENODEV;

	if (omapdss_device_is_enabled(dssdev))
		return 0;

	in->ops.dpi->set_timings(in, &ddata->videomode);

	r = in->ops.dpi->enable(in);
	if (r)
		return r;

	tc_init(ddata);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void panel_dpi_disable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (!omapdss_device_is_enabled(dssdev))
		return;

	tc_uninit(ddata);

	in->ops.dpi->disable(in);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static void panel_dpi_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	ddata->videomode = *timings;
	dssdev->panel.timings = *timings;

	in->ops.dpi->set_timings(in, timings);
}

static void panel_dpi_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	*timings = ddata->videomode;
}

static int panel_dpi_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	return in->ops.dpi->check_timings(in, timings);
}

static struct omap_dss_driver panel_dpi_ops = {
	.connect	= panel_dpi_connect,
	.disconnect	= panel_dpi_disconnect,

	.enable		= panel_dpi_enable,
	.disable	= panel_dpi_disable,

	.set_timings	= panel_dpi_set_timings,
	.get_timings	= panel_dpi_get_timings,
	.check_timings	= panel_dpi_check_timings,

	.get_resolution	= omapdss_default_get_resolution,
};

static const struct of_device_id tc3587xxbg_of_match[] = {
	{
		.compatible = "omapdss,ti,tc3587xxbg",
		.data = &tc_10_inch_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, tc3587xxbg_of_match);

static int tc_probe_of(struct device *dev)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct device_node *np = dev->of_node;
	const struct of_device_id *of_dev_id;
	struct gpio_desc *gpio;

	ddata->dev = dev;
	gpio = devm_gpiod_get(dev, "enable");

	if (IS_ERR(gpio)) {
		if (PTR_ERR(gpio) != -ENOENT)
			return PTR_ERR(gpio);
		else
			gpio = NULL;
	} else {
		gpiod_direction_output(gpio, 1);
	}

	ddata->enable_gpio = gpio;

	ddata->in = omapdss_of_find_source_for_first_ep(np);
	if (IS_ERR(ddata->in)) {
		dev_err(dev, "failed to find video source\n");
		return PTR_ERR(ddata->in);
	}

	of_dev_id = of_match_device(tc3587xxbg_of_match, dev);
	if (!of_dev_id) {
		dev_err(dev, "Unable to match device\n");
		return -ENODEV;
	}

	ddata->board_data = of_dev_id->data;
	ddata->videomode = ddata->board_data->timings;

	return 0;
}

struct regmap_config tc3587xxbg_regmap_config_16bit = {
	.reg_bits = 16,
	.val_bits = 16,
};

struct regmap_config tc3587xxbg_regmap_config_32bit = {
	.reg_bits = 16,
	.val_bits = 32,
};

static int tc3587xxbg_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int r;
	struct regmap *regmap_16bit;
	struct regmap *regmap_32bit;
	struct panel_drv_data *ddata;
	struct device *dev = &client->dev;
	struct omap_dss_device *dssdev;

	dev_err(dev, "tc3587xxbg_i2c_probe\n");

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (ddata == NULL)
		return -ENOMEM;

	dev_set_drvdata(dev, ddata);

	regmap_32bit = devm_regmap_init_i2c(client,
				&tc3587xxbg_regmap_config_32bit);
	if (IS_ERR(regmap_32bit)) {
		r = PTR_ERR(regmap_32bit);
		dev_err(dev, "Failed to init regmap: %d\n", r);
		goto err_gpio;
	}

	regmap_16bit = devm_regmap_init_i2c(client,
				&tc3587xxbg_regmap_config_16bit);
	if (IS_ERR(regmap_16bit)) {
		r = PTR_ERR(regmap_16bit);
		dev_err(dev, "Failed to init regmap: %d\n", r);
		goto err_gpio;
	}


	ddata->regmap_16bit = regmap_16bit;
	ddata->regmap_32bit = regmap_32bit;

	r = tc_probe_of(dev);
	if (r)
		return r;

	usleep_range(10000, 15000);

	dssdev = &ddata->dssdev;
	dssdev->dev = dev;
	dssdev->driver = &panel_dpi_ops;
	dssdev->type = OMAP_DISPLAY_TYPE_DPI;
	dssdev->owner = THIS_MODULE;
	dssdev->panel.timings = ddata->videomode;

	r = omapdss_register_display(dssdev);
	if (r) {
		dev_err(dev, "Failed to register panel\n");
		goto err_reg;
	}

	dev_info(dev, "Successfully initialized %s\n", TC3587X_NAME);

	return 0;
err_reg:
err_gpio:
	omap_dss_put_device(ddata->in);
	return r;
}

static int tc3587xxbg_i2c_remove(struct i2c_client *client)
{
	struct panel_drv_data *ddata = dev_get_drvdata(&client->dev);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	struct omap_dss_device *in = ddata->in;

	if (ddata->enable_gpio)
		gpiod_set_value_cansleep(ddata->enable_gpio, 0);

	omapdss_unregister_display(dssdev);

	panel_dpi_disable(dssdev);
	panel_dpi_disconnect(dssdev);

	omap_dss_put_device(in);

	return 0;
}

static const struct i2c_device_id tc3587xxbg_id[] = {
	{ TC3587X_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tc3587xxbg_id);

static struct i2c_driver tc3587xxbg_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= TC3587X_NAME,
		.of_match_table = tc3587xxbg_of_match,
	},
	.id_table	= tc3587xxbg_id,
	.probe		= tc3587xxbg_i2c_probe,
	.remove		= tc3587xxbg_i2c_remove,
};

module_i2c_driver(tc3587xxbg_i2c_driver);

MODULE_AUTHOR("Marcus Cooksey  <mcooksey@ti.com>");
MODULE_DESCRIPTION("TC3587XXBG DPI-to-DSI Panel Driver");
MODULE_LICENSE("GPL");
