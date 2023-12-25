/*
 * TLC59108 BackLight Driver
 *
 * Copyright (C) 2015 Texas Instruments
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


#define TLCBL_NAME		"tlc59108bl"
#define TLCBL_I2C_ADDR		0x40

#define TLC59108BL_MODE1		0x00
#define TLC59108BL_PWM2		0x04
#define TLC59108BL_LEDOUT0	0x0c
#define TLC59108BL_LEDOUT1	0x0d

struct tlcbl_drv_data {
	struct gpio_desc *enable_gpio;
	struct regmap *regmap;
	struct device *dev;

	const struct tlcbl_board_data *board_data;
};

struct tlcbl_board_data {
	const unsigned int *init_seq;
	unsigned init_seq_len;
};

static const unsigned int tlc59108bl_init_seq[] = {
	/* Init the TLC chip */
	TLC59108BL_MODE1, 0x01,
	/* LDR0: ON, LDR1: OFF, LDR2: PWM, LDR3: OFF */
	TLC59108BL_LEDOUT0, 0x21,
	/* Set LED2 PWM to full */
	TLC59108BL_PWM2, 0xff,
	/* LDR4: OFF, LDR5: OFF, LDR6: OFF, LDR7: ON */
	TLC59108BL_LEDOUT1, 0x40,
};

static const struct tlcbl_board_data tlc59108bl_data = {
	.init_seq = tlc59108bl_init_seq,
	.init_seq_len = ARRAY_SIZE(tlc59108bl_init_seq),
};

static int tlcbl_init(struct tlcbl_drv_data *ddata)
{
	struct regmap *map = ddata->regmap;
	unsigned i, len;
	const unsigned int *seq;

	len = ddata->board_data->init_seq_len;
	seq = ddata->board_data->init_seq;


	for (i = 0; i < len; i += 2)
		regmap_write(map, seq[i], seq[i + 1]);

	return 0;
}

static int tlcbl_uninit(struct tlcbl_drv_data *ddata)
{
	struct regmap *map = ddata->regmap;

	/* clear TLC chip regs */
	regmap_write(map, TLC59108BL_PWM2, 0x0);
	regmap_write(map, TLC59108BL_LEDOUT0, 0x0);
	regmap_write(map, TLC59108BL_LEDOUT1, 0x0);

	regmap_write(map, TLC59108BL_MODE1, 0x0);

	return 0;
}


static const struct of_device_id tlc59108bl_of_match[] = {
	{
		.compatible = "ti,tlc59108-bl",
		.data = &tlc59108bl_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, tlc59108bl_of_match);

static int tlcbl_probe_of(struct device *dev)
{
	struct tlcbl_drv_data *ddata = dev_get_drvdata(dev);
	const struct of_device_id *of_dev_id;
	struct gpio_desc *gpio;

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

	of_dev_id = of_match_device(tlc59108bl_of_match, dev);
	if (!of_dev_id) {
		dev_err(dev, "Unable to match device\n");
		return -ENODEV;
	}

	ddata->board_data = of_dev_id->data;


	return 0;
}

struct regmap_config tlc59108bl_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int tlc59108bl_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int r;
	struct regmap *regmap;
	struct tlcbl_drv_data *ddata;
	struct device *dev = &client->dev;
	unsigned int val;


	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (ddata == NULL)
		return -ENOMEM;

	dev_set_drvdata(dev, ddata);

	r = tlcbl_probe_of(dev);
	if (r)
		return r;

	regmap = devm_regmap_init_i2c(client, &tlc59108bl_regmap_config);
	if (IS_ERR(regmap)) {
		r = PTR_ERR(regmap);
		dev_err(dev, "Failed to init regmap: %d\n", r);
		goto err_gpio;
	}

	ddata->regmap = regmap;
	ddata->dev = dev;

	usleep_range(10000, 15000);

	/* Try to read a TLC register to verify if i2c works */
	r = regmap_read(ddata->regmap, TLC59108BL_MODE1, &val);
	if (r < 0) {
		dev_err(dev, "Failed to set MODE1: %d\n", r);
		goto err_read;
	}

	tlcbl_init(ddata);

	dev_info(dev, "Successfully initialized %s\n", TLCBL_NAME);

	return 0;
err_read:
err_gpio:

	return r;
}

static int tlc59108bl_i2c_remove(struct i2c_client *client)
{
	struct tlcbl_drv_data *ddata = dev_get_drvdata(&client->dev);

	if (ddata->enable_gpio)
		gpiod_set_value_cansleep(ddata->enable_gpio, 0);

	tlcbl_uninit(ddata);

	return 0;
}

static const struct i2c_device_id tlc59108bl_id[] = {
	{ TLCBL_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tlc59108bl_id);

static struct i2c_driver tlc59108bl_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= TLCBL_NAME,
		.of_match_table = tlc59108bl_of_match,
	},
	.id_table	= tlc59108bl_id,
	.probe		= tlc59108bl_i2c_probe,
	.remove		= tlc59108bl_i2c_remove,
};

module_i2c_driver(tlc59108bl_i2c_driver);

MODULE_AUTHOR("Marcus Cooksey  <mcooksey@ti.com>");
MODULE_DESCRIPTION("TLC-59108 Backlight Controller");
MODULE_LICENSE("GPL");
