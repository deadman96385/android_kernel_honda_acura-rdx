/*
 * (C) Copyright 2013 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 *	SMI130 Linux IIO Driver
 */

/*********************************************************
*
*  This file modified by Honda R&D Americas, Inc. on November 24, 2014
*
*  All modifications made by Honda R&D Americas, Inc.
*  are Copyright (c) 2014-2016 Honda R&D Americas, Inc.
*
*  Honda R&D Americas, Inc. hereby licenses those modifications
*  under the terms set forth in the file HONDA-NOTICE
*  located in the root of the directory /vendor/honda
*
*********************************************************/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <asm/unaligned.h>
#include "smi130_iio.h"

/* sensor specific */
#define SENSOR_NAME "smi130"

#define SENSOR_CHIP_ID_SMI (0x0f)

#define SMI_I2C_WRITE_DELAY_TIME 1

/* generic */
#define SMI_MAX_RETRY_I2C_XFER (100)
#define SMI_MAX_RETRY_WAKEUP (5)
#define SMI_MAX_RETRY_WAIT_DRDY (100)

#define SMI_DELAY_MIN (1)
#define SMI_DELAY_DEFAULT (200)

#define BYTES_PER_LINE (16)

#define SMI_SELF_TEST 0

#define BUFF_SIZE 256

static struct i2c_client *smi_client;

/*!
 * @brief this particular data structure is defined
 * for SMI full scale or range or sensor resolution.
 * @gyro_fs_value: the specific value from Register definition.
 * @gyro_fs_dps: sensor's full scale, the unit is degree pre second.
 * @gyro_fs_rslt: sensor's resolution.
 * example: 0(Reg val)----2000dps(full scale)----61000(resolution)
*/
static struct smi_fullscale_avl smi_fs_avl_array[] = {
	[0] = {
		.gyro_fs_value = SMI_FSR_2000DPS_VAL,
		.gyro_fs_dps = SMI_FS_AVL_2000DPS,
		.gyro_fs_rslt = 61000,
	},
	[1] = {
		.gyro_fs_value = SMI_FSR_1000DPS_VAL,
		.gyro_fs_dps = SMI_FS_AVL_1000DPS,
		.gyro_fs_rslt = 30500,
	},
	[2] = {
		.gyro_fs_value = SMI_FSR_500DPS_VAL,
		.gyro_fs_dps = SMI_FS_AVL_500DPS,
		.gyro_fs_rslt = 15300,
	},
	[3] = {
		.gyro_fs_value = SMI_FSR_250DPS_VAL,
		.gyro_fs_dps = SMI_FS_AVL_250DPS,
		.gyro_fs_rslt = 7600,
	},
	[4] = {
		.gyro_fs_value = SMI_FSR_125DPS_VAL,
		.gyro_fs_dps = SMI_FS_AVL_125DPS,
		.gyro_fs_rslt = 3800,
	},
};

static const struct smi_chip_config chip_config_smi = {
	.fsr = SMI_FSR_2000DPS_VAL,
	.filter_bw = SMI_FILTER_12HZ,
	.gyro_fifo_enable = false,
};

static const struct smi_hw smi_hw_info[NUM_DEVICE_PARTS] = {
	{
		.name = "smi130",
		.chip_id = SENSOR_CHIP_ID_SMI,
		.config = &chip_config_smi,
	},
};

#define SMI_SENSORS_12_BITS		12
#define SMI_SENSORS_16_BITS		16
#define SMI_TEMP_SCALE			5000
#define SMI_TEMP_OFFSET			12000
#define WATER_MARK_LEVEL			40

#define SMI_GYRO_CHANNELS_CONFIG(device_type, si, mod, \
							endian, bits, addr) \
	{ \
		.type = device_type, \
		.modified = 1, \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
		.scan_index = si, \
		.channel2 = mod, \
		.address = addr, \
		.scan_type = { \
			.sign = 's', \
			.realbits = bits, \
			.shift = 16 - bits, \
			.storagebits = 16, \
			.endianness = endian, \
		}, \
	}
#define SMI_DEV_ATTR_SAMP_FREQ() \
		IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO, \
			smi_read_frequency, \
			smi_write_frequency)

#define SMI_DEV_ATTR_R_W(name) \
	IIO_DEVICE_ATTR(name, S_IRUGO | S_IWUSR , \
		smi_show_##name, \
		smi_store_##name, 0);

#define SMI_DEV_ATTR_R(name) \
		IIO_DEVICE_ATTR(name, S_IRUGO, \
			smi_show_##name, NULL , 0);

#define SMI_DEV_ATTR_W(name) \
		IIO_DEVICE_ATTR(name, S_IWUSR, \
			NULL, smi_store_##name, 0);

#define SMI_BYTE_FOR_PER_AXIS_CHANNEL		2

/*iio chan spec for 12bit gyro sensor*/
static const struct iio_chan_spec smi_12bit_raw_channels[] = {
	{	.type = IIO_TEMP,
		.channel = IIO_NO_MOD,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
				| BIT(IIO_CHAN_INFO_OFFSET)
				| BIT(IIO_CHAN_INFO_SCALE),
		.address = SMI130_TEMP_ADDR,
		.scan_index = -1,
		.scan_type = { .sign = 'u', .realbits = 8,
			.storagebits = 8, .shift = 0, .endianness = IIO_LE },
	},

	SMI_GYRO_CHANNELS_CONFIG(IIO_ANGL_VEL, SMI_SCAN_GYRO_X,
	IIO_MOD_X, IIO_LE, SMI_SENSORS_12_BITS, SMI130_RATE_X_LSB_VALUEX__REG),

	SMI_GYRO_CHANNELS_CONFIG(IIO_ANGL_VEL, SMI_SCAN_GYRO_Y,
	IIO_MOD_Y, IIO_LE, SMI_SENSORS_12_BITS, SMI130_RATE_Y_LSB_VALUEY__REG),

	SMI_GYRO_CHANNELS_CONFIG(IIO_ANGL_VEL, SMI_SCAN_GYRO_Z,
	IIO_MOD_Z, IIO_LE, SMI_SENSORS_12_BITS, SMI130_RATE_Z_LSB_VALUEZ__REG),

	IIO_CHAN_SOFT_TIMESTAMP(SMI_SCAN_TIMESTAMP),

};

/*iio chan spec for 16bit gyro sensor*/
static const struct iio_chan_spec smi_16bit_raw_channels[] = {
	{	.type = IIO_TEMP,
		.channel = IIO_NO_MOD,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
				| BIT(IIO_CHAN_INFO_OFFSET)
				| BIT(IIO_CHAN_INFO_SCALE),
		.address = SMI130_TEMP_ADDR,
		.scan_index = -1,
		.scan_type = { .sign = 'u', .realbits = 8,
			.storagebits = 8, .shift = 0, .endianness = IIO_LE },
	},

	SMI_GYRO_CHANNELS_CONFIG(IIO_ANGL_VEL, SMI_SCAN_GYRO_X,
	IIO_MOD_X, IIO_LE, SMI_SENSORS_16_BITS, SMI130_RATE_X_LSB_VALUEX__REG),

	SMI_GYRO_CHANNELS_CONFIG(IIO_ANGL_VEL, SMI_SCAN_GYRO_Y,
	IIO_MOD_Y, IIO_LE, SMI_SENSORS_16_BITS, SMI130_RATE_Y_LSB_VALUEY__REG),

	SMI_GYRO_CHANNELS_CONFIG(IIO_ANGL_VEL, SMI_SCAN_GYRO_Z,
	IIO_MOD_Z, IIO_LE, SMI_SENSORS_16_BITS, SMI130_RATE_Z_LSB_VALUEZ__REG),

	IIO_CHAN_SOFT_TIMESTAMP(SMI_SCAN_TIMESTAMP),
};

/* smi i2c routine read */
static char smi_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
	//u8 reg_addr_tmp = reg_addr;
#if !defined SMI_USE_BASIC_I2C_FUNC
	s32 dummy;
	if (NULL == client)
		return -1;

	while (0 != len--) {
#ifdef SMI_SMBUS
		dummy = i2c_smbus_read_byte_data(client, reg_addr);
		if (dummy < 0) {
			dev_err(&client->dev, "i2c bus read error");
			return -1;
		}
		*data = (u8)(dummy & 0xff);
#else
		dummy = i2c_master_send(client, (char *)&reg_addr, 1);
		if (dummy < 0)
			return -1;

		dummy = i2c_master_recv(client, (char *)data, 1);
		if (dummy < 0)
			return -1;
#endif
		reg_addr++;
		data++;
	}
	//if (len = 1)
	//	dev_info(&client->dev, "Reading I2C dev:0x%02x add:0x%02x => 0x%02x \n", client->addr, reg_addr_tmp, *(--data));

	return 0;
#else
	int retry;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &reg_addr,
		},

		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};

	for (retry = 0; retry < SMI_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			mdelay(SMI_I2C_WRITE_DELAY_TIME);
	}

	//if (len = 1)
	//	dev_info(&client->dev, "Reading I2Cx dev:0x%02x add:0x%02x => 0x%02x \n", client->addr, reg_addr_tmp, *data);

	if (SMI_MAX_RETRY_I2C_XFER <= retry) {
		dev_err(&client->dev, "I2C xfer error");
		return -EIO;
	}

	return 0;
#endif
}

static void smi_i2c_delay(SMI130_S32 msec)
{
	mdelay(msec);
}

static void smi_dump_reg(struct i2c_client *client)
{
	int i;
	u8 dbg_buf[64];
	u8 dbg_buf_str[64 * 3 + 1] = "";

	for (i = 0; i < BYTES_PER_LINE; i++) {
		dbg_buf[i] = i;
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	dev_dbg(&client->dev, "%s\n", dbg_buf_str);

	smi_i2c_read(client, SMI130_CHIP_ID_ADDR, dbg_buf, 64);
	for (i = 0; i < 64; i++) {
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	dev_dbg(&client->dev, "%s\n", dbg_buf_str);
}


/* i2c operation for API */
static int smi_check_chip_id(struct i2c_client *client)
{
	int err = 0;
	u8 chip_id = 0;

	smi_i2c_read(client, SMI130_CHIP_ID_ADDR, &chip_id, 1);
	dev_info(&client->dev, "read chip id result: %#x", chip_id);

	if ((chip_id & 0xff) != SENSOR_CHIP_ID_SMI)
		err = -1;

	return err;
}


/*	i2c write routine*/
static char smi_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined SMI_USE_BASIC_I2C_FUNC
	s32 dummy;

#ifndef SMI_SMBUS
	u8 buffer[2];
#endif

	//dev_info(&client->dev, "Writing I2C dev:0x%02x add:0x%02x<=0x%02x \n", client->addr, reg_addr, *data);

	if (NULL == client)
		return -EPERM;

	while (0 != len--) {
#ifdef SMI_SMBUS
		dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
#else
		buffer[0] = reg_addr;
		buffer[1] = *data;
		dummy = i2c_master_send(client, (char *)buffer, 2);
#endif
		reg_addr++;
		data++;
		if (dummy < 0) {
			dev_err(&client->dev, "error writing i2c bus");
			return -EPERM;
		}

	}
	return 0;
#else
	u8 buffer[2];
	int retry;
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 2,
		 .buf = buffer,
		 },
	};

	while (0 != len--) {
		buffer[0] = reg_addr;
		buffer[1] = *data;
		for (retry = 0; retry < SMI_MAX_RETRY_I2C_XFER; retry++) {
			if (i2c_transfer(client->adapter, msg,
						ARRAY_SIZE(msg)) > 0) {
				break;
			} else {
				mdelay(SMI_I2C_WRITE_DELAY_TIME);
			}
		}
		if (SMI_MAX_RETRY_I2C_XFER <= retry) {
			dev_err(&client->dev, "I2C xfer error");
			return -EIO;
		}
		reg_addr++;
		data++;
	}

	return 0;
#endif
}

static char smi_i2c_read_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	char err;
	err = smi_i2c_read(smi_client, reg_addr, data, len);
	return err;
}

static char smi_i2c_write_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	char err;
	err = smi_i2c_write(smi_client, reg_addr, data, len);
	return err;
}


static int smi_read_axis_data(struct iio_dev *indio_dev, u8 reg_address,
		int *data)
{
	int ret;
	unsigned char axis_outdata[SMI_BYTE_FOR_PER_AXIS_CHANNEL];
	struct  smi_client_data *client_data = iio_priv(indio_dev);

	ret = smi_i2c_read(client_data->client, reg_address,
				axis_outdata, SMI_BYTE_FOR_PER_AXIS_CHANNEL);
	if (ret < 0)
		return ret;

	*data = (s16)get_unaligned_le16(axis_outdata);
	return 0;
}

static int smi_read_temp_data(struct iio_dev *indio_dev, u8 reg_address,
		int *data)
{
	int ret;
	signed char temp_outdata;
	struct  smi_client_data *client_data = iio_priv(indio_dev);

	ret = smi_i2c_read(client_data->client, reg_address, &temp_outdata, 1);
	if (ret < 0)
		return ret;
	*data = (temp_outdata) & 0xff;
	return 0;
}

static ssize_t smi_read_frequency(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	int ret;
	unsigned char bandwidth = 0;

	mutex_lock(&indio_dev->mlock);
	smi130_get_bw(&bandwidth);
	mutex_unlock(&indio_dev->mlock);

	ret = scnprintf(buf, BUFF_SIZE, "%d\n", bandwidth);
	return ret;
}

static ssize_t smi_write_frequency(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	int err;
	unsigned long bandwidth;
	err = kstrtoul(buf, 10, &bandwidth);
	if (err)
		return err;

	mutex_lock(&indio_dev->mlock);
	smi130_set_bw(bandwidth);
	mutex_unlock(&indio_dev->mlock);

	return count;
}

static ssize_t smi_show_bandwidth(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char bandwidth = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	mutex_lock(&indio_dev->mlock);
	smi130_get_bw(&bandwidth);
	mutex_unlock(&indio_dev->mlock);

	err = scnprintf(buf, BUFF_SIZE, "%d\n", bandwidth);
	return err;
}

static ssize_t smi_store_bandwidth(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long bandwidth;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	err = kstrtoul(buf, 10, &bandwidth);
	if (err)
		return err;

	mutex_lock(&indio_dev->mlock);
	smi130_set_bw(bandwidth);
	mutex_unlock(&indio_dev->mlock);

	return count;
}

static ssize_t smi_show_selftest(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char selftest;
	SMI130_U16 datax_check = 0;
	SMI130_U16 datay_check = 0;
	SMI130_U16 dataz_check = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct smi_client_data *client_data = iio_priv(indio_dev);

	//smi130_selftest(&selftest);
	smi130_selftest(&selftest);
	if (selftest)	{
		dev_err(&indio_dev->dev, "gyro driver self test can not pass!\n");
		return C_SMI130_FAILURE;
	}
	smi130_set_bw(C_SMI130_Three_U8X);/*set bandwidth 47Hz*/
	mutex_lock(&indio_dev->mlock);
	smi130_get_dataxyz(&client_data->value);
	datax_check = abs(1000 * client_data->value.datax / 16384);
	datay_check = abs(1000 * client_data->value.datay / 16384);
	dataz_check = abs(1000 * client_data->value.dataz / 16384);
	mutex_unlock(&indio_dev->mlock);
	if ((datax_check <= 5) && (datay_check <= 5) && (dataz_check <= 5))
		dev_notice(&indio_dev->dev, "Self test successfully!\n");
	else {
		dev_err(&indio_dev->dev, "Self test checking value failed!");
		dev_err(&indio_dev->dev, "x y z axis values:%d,%d,%d\n",
			datax_check, datay_check, dataz_check);
		selftest |= C_SMI130_FAILURE;
	}
	err = scnprintf(buf, BUFF_SIZE, "%d\n", selftest);
	return err;
}

static ssize_t smi_show_wdt(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char wdt = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	mutex_lock(&indio_dev->mlock);
	smi130_get_wdt(&wdt);
	mutex_unlock(&indio_dev->mlock);

	err = scnprintf(buf, BUFF_SIZE, "%d\n", wdt);
	return err;
}

static ssize_t smi_store_wdt(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long wdt;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	err = kstrtoul(buf, 10, &wdt);
	if (err)
		return err;

	mutex_lock(&indio_dev->mlock);
	smi130_set_wdt(wdt);
	mutex_unlock(&indio_dev->mlock);


	return count;
}

static int smi_set_fullscale(struct smi_client_data *client_data, int val)
{
	int i;
	if ((val > C_SMI130_Four_U8X) || (val < C_SMI130_Zero_U8X))
		return -EINVAL;
	for (i = 0; i < NUM_SMI_FSR_VAL; i++) {
		if (val == smi_fs_avl_array[i].gyro_fs_value) {
			smi130_set_range_reg(val);
			client_data->current_fullscale =
			(struct smi_fullscale_avl *)&smi_fs_avl_array[i];
			return 0;
		} else if (i == NUM_SMI_FSR_VAL)
			return -EINVAL;
	}
	return -EINVAL;
}

static int smi_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *ch, int *val,
							int *val2, long mask)
{
	int ret, result;
	struct smi_client_data *client_data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	{
		result = 0;
		ret = IIO_VAL_INT;
		mutex_lock(&indio_dev->mlock);
		switch (ch->type) {
		case IIO_ANGL_VEL:
			result = smi_read_axis_data(indio_dev,
							ch->address, val);
			*val = *val >> ch->scan_type.shift;
			break;
		case IIO_TEMP:
			result = smi_read_temp_data(indio_dev,
							ch->address, val);
			*val = *val >> ch->scan_type.shift;
			break;
		default:
			ret = -EINVAL;
			break;
		}
	mutex_unlock(&indio_dev->mlock);
	if (result < 0)
		return result;
	return ret;
	}

	case IIO_CHAN_INFO_SCALE:
	{
		switch (ch->type) {
		case IIO_ANGL_VEL:
			*val = 0;
			*val2 = client_data->current_fullscale->gyro_fs_rslt;
			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_TEMP:
			*val = 0;
			*val2 = SMI_TEMP_SCALE;
			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}
	}

	case IIO_CHAN_INFO_OFFSET:
	{
		switch (ch->type) {
		case IIO_TEMP:
			*val = SMI_TEMP_OFFSET;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	}

	default:
		return -EINVAL;
	}

}

static int smi_wirte_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *ch, int val,
							int val2, long mask)
{
	int ret;
	struct smi_client_data *client_data = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (ch->type) {
		case IIO_ANGL_VEL:
			ret = smi_set_fullscale(client_data, val);
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&indio_dev->mlock);
	return ret;
}

/* Sysfs entries */
static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("100 200 400 1000 2000");
static SMI_DEV_ATTR_SAMP_FREQ(); // RCG DENSO  SYSFS ATTRIBUTE: smi_read_frequency  smi_write_frequency
static SMI_DEV_ATTR_R_W(bandwidth); // RCG DENSO  SYSFS ATTRIBUTE: smi_show_bandwidth();  smi_store_bandwidth();
static SMI_DEV_ATTR_R(selftest); // RCG DENSO  SYSFS ATTRIBUTE: smi_show_selftest();
static SMI_DEV_ATTR_R_W(wdt); // RCG DENSO  SYSFS ATTRIBUTE: smi_show_wdt();  smi_store_wdt();

static struct attribute *smi_attributes[] = {
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_bandwidth.dev_attr.attr,
	&iio_dev_attr_selftest.dev_attr.attr,
	&iio_dev_attr_wdt.dev_attr.attr,
	NULL,
};

static const struct attribute_group smi_attribute_group = {
	.attrs = smi_attributes,
};

static const struct iio_info smi_iio_info = {
	.driver_module = THIS_MODULE,
	.attrs = &smi_attribute_group,
	.read_raw = &smi_read_raw,
	.write_raw = &smi_wirte_raw,
};




static int smi_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct iio_dev *indio_dev;
	struct smi_client_data *client_data = NULL;
	struct device_node *np = client->dev.of_node;
	int wdt_en = 0;

	dev_info(&client->dev, "function entrance");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error!");
		err = -EIO;
		goto exit_err_clean;
	}

	if (NULL == smi_client) {
		smi_client = client;
	} else {
		dev_err(&client->dev,
			"this driver does not support multiple clients");
		err = -EINVAL;
		goto exit_err_clean;
	}

	err = smi_check_chip_id(client);
	if (!err) {
		dev_notice(&client->dev,
			"Bosch Sensortec Device %s detected", SENSOR_NAME);
	} else {
		dev_err(&client->dev,
			"ERROR: Bosch Sensortec Device not found, chip id mismatch");
		err = -ENXIO;
		goto exit_err_clean;
	}

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*client_data));
	if (indio_dev == NULL) {
		dev_err(indio_dev->dev.parent,
				"IIO device alloc err %d\n", err);
		return -ENOMEM;
	}

	client_data = iio_priv(indio_dev);
	client_data->client = client;
	i2c_set_clientdata(client, indio_dev);
/* get gpio interrupt pin number from DTS
 * need to set in the dts file by using "gpio_int" key word.
 */
	err = of_property_read_u32(np, "gpio_int",
				&client_data->gpio_interrupt_pin);
	if (err) {
		dev_err(&client_data->client->dev,
			"Not found usable gpio interrupt pin!\n");
		client_data->gpio_interrupt_pin = 0;
	}

	wdt_en = of_property_read_bool(np, "wdt_en");

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = client->name;
	indio_dev->channels = smi_16bit_raw_channels;
	indio_dev->num_channels = ARRAY_SIZE(smi_16bit_raw_channels);
	indio_dev->info = &smi_iio_info;
	client_data->current_fullscale = (struct smi_fullscale_avl *)
							&smi_fs_avl_array[0];
	indio_dev->modes = INDIO_DIRECT_MODE;
	/* h/w init */
	client_data->device.bus_read = smi_i2c_read_wrapper;
	client_data->device.bus_write = smi_i2c_write_wrapper;
	client_data->device.delay_msec = smi_i2c_delay;


	smi130_init(&client_data->device);
	smi_dump_reg(client);
	client_data->enable = 0;

#ifdef SMI130_ENABLE_INT1
	/* maps interrupt to INT1 pin */
	smi130_set_int_od(SMI130_INT1, INT_PIN_PUSH_PULL);
	smi130_set_int_data(SMI130_INT1, INT_ENABLE);
#endif

	memcpy(&client_data->chip_config,
			smi_hw_info[client_data->chip_type].config,
				sizeof(struct smi_chip_config));
	client_data->IRQ = client->irq;

	err = smi_allocate_ring(indio_dev);
	if (err < 0) {
		dev_err(indio_dev->dev.parent,
				"smi configure buffer fail %d\n", err);
		return err;
	}
	err = smi_probe_trigger(indio_dev);
	if (err) {
		dev_err(indio_dev->dev.parent,
				"smi trigger probe fail %d\n", err);
		goto smi_probe_trigger_error;
	}
	smi130_set_bw(3); /*set bandwidth to 47Hz*/

	err = smi130_set_wdt(wdt_en);
	if(err) {
		dev_warn(indio_dev->dev.parent,
			"smi IIO device failed to set WDT %d\n", err);
	}

	err = iio_device_register(indio_dev);
	if (err) {
		dev_err(indio_dev->dev.parent,
				"smi IIO device register failed %d\n", err);
		goto smi_probe_error;
	}


	smi130_set_data_enable(INT_ENABLE);
	dev_notice(indio_dev->dev.parent,
		"IIO device sensor %s probed successfully", SENSOR_NAME);

	return 0;

smi_probe_error:
	smi_remove_trigger(indio_dev);
smi_probe_trigger_error:
	smi_deallocate_ring(indio_dev);
exit_err_clean:
	smi_client = NULL;
	return err;

}


static int smi_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static const struct i2c_device_id smi_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, smi_id);

static struct i2c_driver smi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
	},
	.id_table = smi_id,
	.probe = smi_probe,
	.remove = smi_remove,
};

static int __init smi_init(void)
{
	return i2c_add_driver(&smi_driver);
}

static void __exit smi_exit(void)
{
	i2c_del_driver(&smi_driver);
}

MODULE_AUTHOR("contact@xxxxxxxxxxxxxxxxxxx");
MODULE_DESCRIPTION("driver for " SENSOR_NAME);
MODULE_LICENSE("GPL v2");

module_init(smi_init);
module_exit(smi_exit);
