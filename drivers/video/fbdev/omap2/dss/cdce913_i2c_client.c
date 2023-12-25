/*********************************************************
  *
  *  Copyright (c) 2016 Honda R&D Americas, Inc.
  *  All rights reserved. You may not copy, distribute, publicly display,
  *  create derivative works from or otherwise use or modify this
  *  software without first obtaining a license from Honda R&D Americas, Inc.
  *
 *********************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#define CDCE913_OUTPUT_OFFSET   0x82
#define CDCE913_OUTPUT_ENABLE   0x9C
#define CDCE913_OUTPUT_DISABLIE 0x94

static struct i2c_client *client_913 = NULL;

static inline int i2c_write_le8(struct i2c_client *client, unsigned addr,
				unsigned data)
{
	int ret = i2c_smbus_write_byte_data(client, addr, data);
	if (ret)
		dev_dbg(&client->dev, "Failed to write 0x%02x to 0x%02x",
			data, addr);
	return ret;
}

static inline int i2c_read_le8(struct i2c_client *client, unsigned addr)
{
	int ret = i2c_smbus_read_byte_data(client, addr);
	if (ret < 0)
		dev_dbg(&client->dev, "Failed to read 0x%02x, Error = %d",
			addr, ret);
	return ret;
}

int start_cdce913(void)
{
	int ret;

	if (client_913 == NULL)
		return -ENODEV;

	ret  = i2c_write_le8(client_913, CDCE913_OUTPUT_OFFSET,
			CDCE913_OUTPUT_ENABLE);
	if (ret < 0)
		dev_dbg(&client_913->dev, "start_cdce913  failed!!!");

	return ret;
}

EXPORT_SYMBOL_GPL(start_cdce913);

int stop_cdce913(void)
{
	int ret;

	if (client_913 == NULL)
		return -ENODEV;

	ret  = i2c_write_le8(client_913, CDCE913_OUTPUT_OFFSET,
			CDCE913_OUTPUT_DISABLIE);
	if (ret < 0)
		dev_dbg(&client_913->dev, "stop_cdce913  failed!!!");

	return ret;
}

EXPORT_SYMBOL_GPL(stop_cdce913);

static int  cdce913_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;

	printk(KERN_INFO "support: +cdce913_i2c_probe()\n");

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
		I2C_FUNC_SMBUS_I2C_BLOCK)) {
		printk(KERN_ERR "%s: needed i2c functionality is not supported\n", __func__);
		return -ENODEV;
	}

	client_913 = client;

	printk(KERN_INFO "CDCE913 found by %s: probe exit\n", __func__);
	return 0;
}

static int cdce913_i2c_remove(struct i2c_client *client)
{
	client_913 = NULL;
	return 0;
}


static const struct i2c_device_id cdce913_i2c_id[] = {
	{ "ti,cdce913", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c,cdce913_i2c_id);

static const struct of_device_id cdce913_i2c_of_match[] = {
	{.compatible = "ti,cdce913",},
	{ }
};

MODULE_DEVICE_TABLE(of, cdce913_i2c_of_match);


static struct i2c_driver cdce913_i2c_driver = {
	.probe    = cdce913_i2c_probe,
	.remove   = cdce913_i2c_remove,
	.id_table = cdce913_i2c_id,
	.driver   = {
		.owner = THIS_MODULE,
		.name = "cdce913",
		.of_match_table = cdce913_i2c_of_match,
	},
};
module_i2c_driver(cdce913_i2c_driver);

MODULE_DESCRIPTION("cdce913 I2C client driver");
MODULE_LICENSE("GPL");
