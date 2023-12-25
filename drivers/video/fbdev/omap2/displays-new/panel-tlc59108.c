/*
 * TLC59108 DPI Panel Driver
 *
 * Copyright (C) 2013 Texas Instruments
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
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
#include <linux/of_irq.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/of_device.h>

#include <video/omapdss.h>
#include <video/omap-panel-data.h>
#include <video/of_display_timing.h>
#include <asm/unaligned.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/timer.h>

#include "serializer.h"
#include "deserializer.h"

#define TLC_NAME		"tlc59108"
#define TLC_I2C_ADDR		0x40

#define TLC59108_MODE1		0x00
#define TLC59108_PWM2		0x04
#define TLC59108_LEDOUT0	0x0c
#define TLC59108_LEDOUT1	0x0d

/*port expander max7310 register*/

#define MAX7310_INPUT		0x00
#define MAX7310_OUTPUT		0x01
#define MAX7310_POLARITY	0x02
#define MAX7310_CONFIG		0x03
#define MAX7310_TIMEOUT		0x04
#define MAX7310_POWER_MASK	0x3f

/*touch screen GT9271*/

#define GT9271_MAX_HEIGHT		4096
#define GT9271_MAX_WIDTH		4096
#define GT9271_INT_TRIGGER		1
#define GT9271_CONTACT_SIZE		8
#define GT9271_CONFIG_MAX_LENGTH	240


#define GT9271_READ_COOR_ADDR		0x814E
#define GT9271_REG_CONFIG_DATA		0x8047
#define GT9271_REG_VERSION		0x8140

#define BUTTON_TIMEOUT (msecs_to_jiffies(200))

#define RESOLUTION_LOC		1
#define TRIGGER_LOC		6

#define DES926_I2C_ADDRESS			(0x3b<<1)
#define GT9271_TOUCH_SCREEN_I2C_ADDRESS		(0x14<<1)
#define TLC59108_LCD_I2C_ADDRESS		(0x40<<1)
#define MAX7310_PORT_EXPANDER_I2C_ADDRESS	(0x1f<<1)
#define BUTTON_STATUS(x,y)			(!((x) & (1<<(y))))

#define GT9271_INFO(fmt, arg...)       pr_info("<<-GTP-INFO->> "fmt"\n", ##arg)
#define GT9271_ERROR(fmt, arg...)      pr_err("<<-GTP-ERROR->> "fmt"\n", ##arg)

enum{
	LVDS_UNLINKED,
	LVDS_LINKED,
};

struct gt9271_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	int abs_x_max;
	int abs_y_max;
	unsigned int max_touch_num;
	unsigned int int_trigger_type;
	unsigned int irq;
	unsigned int x_size;
	unsigned int y_size;
	unsigned int num_mt_slots;
};

struct panel_drv_data {
	struct omap_dss_device dssdev;
	struct omap_dss_device *in;

	struct omap_video_timings videomode;
	struct workqueue_struct *workq;
	struct delayed_work tlc59108_button_timer_work;
	int LVDS_link_status;

	struct gpio_desc *enable_gpio;
	struct regmap *regmap;
	struct i2c_client *ser_i2c_client;
	struct i2c_client *deser_i2c_client;
	struct i2c_client *max7310_i2c_client;
	struct i2c_client *tlc59108_i2c_client;
	struct i2c_client *gt9271_i2c_touch_client;
	struct gt9271_ts_data *ts_data;
	unsigned int irq;
	u32 deser_alias;	/* alias used on this side of FPDlink */
	struct mutex mutex;
	int display_detected;
	const struct tlc_board_data *board_data;
};


static const char *gt9271_ts_name = "GT9271 Capacitive TouchScreen";
static int gt9271_power_on(struct panel_drv_data *ddata);
static int gt9271_probe(struct panel_drv_data *ddata);
static int tlc59108_setup_lcd_alias(struct panel_drv_data *ddata);
static int tlc59108_setup_ts_alias(struct panel_drv_data *ddata);


static const unsigned long gt9271_irq_flags[] = {
	IRQ_TYPE_EDGE_RISING,
	IRQ_TYPE_EDGE_FALLING,
	IRQ_TYPE_LEVEL_LOW,
	IRQ_TYPE_LEVEL_HIGH,
};


struct tlc_board_data {
	struct omap_video_timings timings;
	const unsigned int *init_seq;
	unsigned init_seq_len;
};

static const unsigned int tlc_7_inch_init_seq[] = {
	/* Init the TLC chip */
	TLC59108_MODE1, 0x01,
	/*
	 * set LED1(AVDD) to ON state(default), enable LED2 in PWM mode, enable
	 * LED0 to OFF state
	 */
	TLC59108_LEDOUT0, 0x21,
	/* set LED2 PWM to full freq */
	TLC59108_PWM2, 0xff,
	/* set LED4(UPDN) and LED6(MODE3) to OFF state */
	TLC59108_LEDOUT1, 0x11,
};

static const unsigned int tlc_10_inch_init_seq[] = {
	/* Init the TLC chip */
	TLC59108_MODE1, 0x01,
	/* LDR0: ON, LDR1: OFF, LDR2: PWM, LDR3: OFF */
	TLC59108_LEDOUT0, 0x21,
	/* Set LED2 PWM to full */
	TLC59108_PWM2, 0xff,
	/* LDR4: OFF, LDR5: OFF, LDR6: OFF, LDR7: ON */
	TLC59108_LEDOUT1, 0x44,/*ORIGINAL VALUE 0X40*/
};

static const struct tlc_board_data tlc_7_inch_data = {
	.timings = {
		.x_res		= 800,
		.y_res		= 480,

		.pixelclock	= 29232000,

		.hfp		= 41,
		.hsw		= 49,
		.hbp		= 41,

		.vfp		= 13,
		.vsw		= 4,
		.vbp		= 29,

		.vsync_level	= OMAPDSS_SIG_ACTIVE_LOW,
		.hsync_level	= OMAPDSS_SIG_ACTIVE_LOW,
		.data_pclk_edge	= OMAPDSS_DRIVE_SIG_RISING_EDGE,
		.de_level	= OMAPDSS_SIG_ACTIVE_HIGH,
		.sync_pclk_edge	= OMAPDSS_DRIVE_SIG_RISING_EDGE,
	},
	.init_seq = tlc_7_inch_init_seq,
	.init_seq_len = ARRAY_SIZE(tlc_7_inch_init_seq),
};

static const struct tlc_board_data tlc_10_inch_data = {
	.timings = {
		.x_res          = 1280,
		.y_res          = 800,

		.pixelclock     = 69300404,

		.hfp            = 48,
		.hsw            = 32,
		.hbp            = 44,

		.vfp            = 4,
		.vsw            = 7,
		.vbp            = 12,

		.vsync_level    = OMAPDSS_SIG_ACTIVE_LOW,
		.hsync_level    = OMAPDSS_SIG_ACTIVE_LOW,
		.data_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
		.de_level       = OMAPDSS_SIG_ACTIVE_HIGH,
		.sync_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
	},
	.init_seq = tlc_10_inch_init_seq,
	.init_seq_len = ARRAY_SIZE(tlc_10_inch_init_seq),
};


static unsigned registered_keys[] = {
	/*
	 * Button layout(from bottom to top)  on the LVDS convert card.
	 */
	KEY_HOMEPAGE,	/* Home 		first button*/
	KEY_F11,        /* POWER		second button*/
	KEY_BACK,	/* BACK			third button*/
	KEY_F12,	/* Day/night:		4th botton*/
	KEY_VOLUMEUP,	/* KEY_VOLUMEUP		5th button*/
	KEY_VOLUMEDOWN,	/* KEY_VOLUMEDOWN	6th button */

};

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
	int ret = (int)i2c_smbus_read_byte_data(client, addr);
	if (ret < 0)
		dev_dbg(&client->dev, "Failed to read 0x%02x, Error = %d",
				addr, ret);
	return ret;
}

static int tlc59108_i2c_cmdrsp(struct i2c_client *client,
				__u8 *wbuf, __u16 wlen, __u8 *rbuf, __u16 rlen)
{
	struct i2c_msg msg[2];

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = wbuf;
	msg[0].len = wlen;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = rbuf;
	msg[1].len = rlen;

	return i2c_transfer(client->adapter, msg, sizeof(msg) / sizeof(msg[0]));
}



/**
 * gt9271_i2c_read - read data from a register of the i2c slave device.
 *
 * @client: i2c device.
 * @reg: the register to read from.
 * @buf: raw write data buffer.
 * @len: length of the buffer to write
 */
static int gt9271_i2c_read(struct i2c_client *client,
				u16 reg, u8 *buf, int len)
{
	struct i2c_msg msgs[2];
	u16 wbuf = cpu_to_be16(reg);
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 2;
	msgs[0].buf   = (u8 *) &wbuf;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	return ret < 0 ? ret : (ret != ARRAY_SIZE(msgs) ? -EIO : 0);
}

/**
 * gt9271_i2c_write - write data to the i2c slave device.
 *
 * @client: i2c device.
 * @reg: the register to read to.
 * @buf: raw write data buffer.
 * @len: length of the buffer to write
 */
static int gt9271_i2c_write(struct i2c_client *client,
				u16 reg, u8 *buf, int len)
{
	struct i2c_msg msg;
	int ret;
	u8 *wbuf;

	wbuf = kzalloc(len + 2, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	wbuf[0] = reg >> 8;
	wbuf[1] = reg & 0xFF;
	memcpy(&wbuf[2], buf, len);

	msg.flags = !I2C_M_RD;
	msg.addr  = client->addr;
	msg.len   = len + 2;
	msg.buf   = wbuf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	kfree(wbuf);
	return ret;
}

static int gt9271_ts_read_input_report(struct gt9271_ts_data *ts, u8 *data)
{
	int touch_num;
	int ret;

	ret = gt9271_i2c_read(ts->client, GT9271_READ_COOR_ADDR, data, 10);
	if (ret < 0) {
		GT9271_ERROR("I2C transfer error (%d)\n", ret);
		return ret;
	}

	touch_num = data[0] & 0x0f;
	if (touch_num > (ts->num_mt_slots))
		return -EPROTO;

	if (touch_num > 1) {
		ret = gt9271_i2c_read(ts->client, GT9271_READ_COOR_ADDR + 10,
				   &data[10], 8 * (touch_num - 1));
		if (ret < 0)
			return ret;
	}

	return touch_num;
}


static void gt9271_ts_parse_touch(struct gt9271_ts_data *ts, u8 *coor_data)
{
	int id = coor_data[0] & 0x0F;
	int input_x ;
	int input_y ;
	int input_w = get_unaligned_le16(&coor_data[5]);

	int ori_x,ori_y,rot_x,rot_y;

	/* Read the actual coordinates from the controller */
	ori_x = get_unaligned_le16(&coor_data[1]);
	ori_y = get_unaligned_le16(&coor_data[3]);

/*	printk("gt9271_ts_parse_touch  x = %d y= %d\n",ori_x ,ori_y);*/

	/* Rotate the coordinates clockwise by 90 degrees about the center
         * to match the goodix controllers coordinates
	 * with the low level touch driver.*/
	rot_x = ((ori_x-2048)*0 - (ori_y-2048)*1) + 2048;
	rot_y = ((ori_x-2048)*1 + (ori_y-2048)*0) + 2048;

	rot_x = GT9271_MAX_HEIGHT - rot_x;
	rot_y = GT9271_MAX_WIDTH - rot_y;

	/* Scale the coordinates to match the size of the TSC - 1280 * 800 */
	input_x = ((rot_x * ts->x_size)/GT9271_MAX_HEIGHT);
	input_y = ((rot_y * ts->y_size)/GT9271_MAX_WIDTH);

	input_mt_slot(ts->input_dev, id);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
}

/**
 * gt9271_ts_work_func - Process incoming IRQ
 *
 * @ts: our gt9271_ts_data pointer
 *
 * Called when the IRQ is triggered. Read the current device state, and push
 * the input events to the user space.
 */
static void gt9271_ts_work_func(struct panel_drv_data *ddata)
{
	u8  end_cmd[1] = {0};
	struct gt9271_ts_data *ts= ddata->ts_data;
	if(ts ==NULL){
		printk("gt9271_ts_work_func failed\n");
		return;
	}
	u8  point_data[1 + 8 * (ts->num_mt_slots + 1)];
	int touch_num;
	int i;


	tlc59108_setup_ts_alias(ddata);
	touch_num = gt9271_ts_read_input_report(ts, point_data);
	if (touch_num < 0)
		goto exit_work_func;

	for (i = 0; i < touch_num; i++)
		gt9271_ts_parse_touch(ts, &point_data[1 + 8 * i]);

	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);

exit_work_func:
	if (gt9271_i2c_write(ts->client,
				GT9271_READ_COOR_ADDR, end_cmd, 1) < 0)
		GT9271_INFO("I2C write end_cmd error");
}


static int tlc_init(struct panel_drv_data *ddata)
{
	struct regmap *map = ddata->regmap;
	unsigned i, len;
	const unsigned int *seq;
	unsigned int val;
	int r;

	len = ddata->board_data->init_seq_len;
	seq = ddata->board_data->init_seq;
#ifdef CONFIG_HRB_DISPLAY_SUPPORT
	for (i = 0; i < len; i += 2)
		regmap_write(map, seq[i], seq[i + 1]);
#else
	for (i = 0; i < len; i += 2){

		regmap_read(map, seq[i], &val);

		val = (seq[i + 1] | val);

		r=regmap_write(map, seq[i], val);
		udelay(500);
	}

#endif
	return 0;
}

static int tlc_uninit(struct panel_drv_data *ddata)
{
	struct regmap *map = ddata->regmap;

	/* clear TLC chip regs */
	regmap_write(map, TLC59108_PWM2, 0x0);
	regmap_write(map, TLC59108_LEDOUT0, 0x0);
	regmap_write(map, TLC59108_LEDOUT1, 0x0);

	regmap_write(map, TLC59108_MODE1, 0x0);

	return 0;
}


#define to_panel_data(p) container_of(p, struct panel_drv_data, dssdev)

static int gt9271_power_on(struct panel_drv_data *ddata)
{


	unsigned int read_val = 0;
	unsigned int write_val = 0;
	int ret = 0;

	ret = i2c_write_le8(ddata->tlc59108_i2c_client,0x00,	0x00);
	if(ret <0)
		printk("fail to read tlc59108 mode0\n");

	msleep(2);

	ret = i2c_read_le8(ddata->tlc59108_i2c_client, TLC59108_LEDOUT1);
	if(ret <0)
		printk("fail to read tlc59108 LEDOUT1\n");
	else printk("origial power value  ret= 0x%02x\n",ret);


	/* OUT5 LOW - RST LOW */
	/* OUT6 LOW - INT LOW */
	write_val = ret | 0x04;
	ret = i2c_write_le8(ddata->tlc59108_i2c_client, TLC59108_LEDOUT1,write_val);
	if(ret <0)
		printk("fail to write tlc59108 LEDOUT1\n");

	msleep(20);

	/* OUT5 LOW - RST LOW */
	/* OUT6 HIGH - INT HIGH*/
	write_val = read_val | 0x00;
	ret = i2c_write_le8(ddata->tlc59108_i2c_client, TLC59108_LEDOUT1,write_val);
	if(ret <0)
		printk("fail to write tlc59108 LEDOUT1\n");

	msleep(2);

	/* OUT5 HIGH - RST HIGH */
	/* OUT6 HIGH - INT HIGH*/
	write_val = read_val | 0x04;
	ret = i2c_write_le8(ddata->tlc59108_i2c_client, TLC59108_LEDOUT1,write_val);
	if(ret <0)
		printk("fail to write tlc59108 LEDOUT1\n");

	msleep(6);

	/* Set INT pin as input for FW sync. If the INT is high, it means
	 * there is a pull up resistor attached on INT pin. Pull low the
	 * INT pin manually for FW sync. */

	/* OUT5 HIGH - RST HIGH */
	/* OUT6 LOW - INT LOW */
	write_val = read_val | 0x14;
	ret = i2c_write_le8(ddata->tlc59108_i2c_client, TLC59108_LEDOUT1,write_val);
	if(ret <0)
		printk("fail to write tlc59108 LEDOUT1\n");

	msleep(50);

	/* OUT5 HIGH - RST HIGH */
	/* OUT6 HIGH - INT HIGH*/
	write_val = read_val | 0x44;
	ret = i2c_write_le8(ddata->tlc59108_i2c_client, TLC59108_LEDOUT1,write_val);
	if(ret <0)
		printk("fail to write tlc59108 LEDOUT1\n");

	msleep(2);

	return 0;
}

static int tlc59108_setup_port_expander_alias(struct panel_drv_data *ddata)
{
        int r;
	unsigned data;

	data = MAX7310_PORT_EXPANDER_I2C_ADDRESS;
	r = i2c_write_le8(ddata->ser_i2c_client,
			SER925_I2C_SLAVE_ID, data);
        if (r < 0)
                return r;
	udelay(500);
        r = i2c_write_le8(ddata->ser_i2c_client,
			SER925_I2C_SLAVE_ALIAS, data);

        return r;
}


static int tlc59108_setup_ts_alias(struct panel_drv_data *ddata)
{
        int r;
	unsigned data;

	data = GT9271_TOUCH_SCREEN_I2C_ADDRESS ;
	r = i2c_write_le8(ddata->ser_i2c_client,
			SER925_I2C_SLAVE_ID, data);
        if (r < 0)
                return r;
	udelay(500);
        r = i2c_write_le8(ddata->ser_i2c_client,
			SER925_I2C_SLAVE_ALIAS, data);

        return r;
}


static int tlc59108_setup_lcd_alias(struct panel_drv_data *ddata)
{
        int r;
	unsigned data;

	data = TLC59108_LCD_I2C_ADDRESS;
	r = i2c_write_le8(ddata->ser_i2c_client,
			SER925_I2C_SLAVE_ID, data);
        if (r < 0)
                return r;

        r = i2c_write_le8(ddata->ser_i2c_client,
			SER925_I2C_SLAVE_ALIAS, data);

        return r;
}

static int tlc59108_setup_deser_alias(struct panel_drv_data *ddata)
{
        int r;
	unsigned data;

	data = DES926_I2C_ADDRESS;
	r = i2c_write_le8(ddata->ser_i2c_client,
			SER925_I2C_SLAVE_ID, data);
        if (r < 0)
                return r;

        r = i2c_write_le8(ddata->ser_i2c_client,
			SER925_I2C_SLAVE_ALIAS, data);

        return r;
}

int tlc59108_setup_link(struct panel_drv_data *ddata)
{
	int r;
	int config0;
	int config1;
	int datactl;
	int modests;
	int gensts;
	int rxsts;
	int retry = 5;

	/*
	 * have to shift the received value because the serializer i2c address
	 *	is in the upper 7 bits of the register.
	 */
	r = i2c_read_le8(ddata->ser_i2c_client, SER925_I2C_DEVICE_ID);
	if (r > 0) {
		r >>= 1;
		if (r != ddata->ser_i2c_client->addr) {
			dev_err(&ddata->ser_i2c_client->dev,
					"Ser addr mismatch: r = 0x%02hhx, client addr = %02hhx.",
					r, ddata->ser_i2c_client->addr);
			goto err0;
		}
	} else {
		/*
		 * dump the 7-bit physical address of the serializer,
		 *	since that's how it's known by developers.
		 */
		dev_err(&ddata->ser_i2c_client->dev,
				"Couldn't read Serializer i2c address, r = %d.",
				r >> 1);
		goto err0;
	}

	dev_info(&ddata->ser_i2c_client->dev, "Serializer i2c addr 0x%x.", r);

	r = i2c_read_le8(ddata->ser_i2c_client, SER925_GEN_STS);
	dev_info(&ddata->ser_i2c_client->dev, "SER925_GEN_STS Value %d\n", r);
	if ((r < 0) || ((r & SER925_GEN_STS_PCLK_DETECT) == 0)) {
		dev_err(&ddata->ser_i2c_client->dev, "PCLK not present.");
	/*	goto err0;*/
	}

	dev_info(&ddata->ser_i2c_client->dev, "PCLK detection done.");

	if ((r & SER925_GEN_STS_LINK_DETECT) == 0) {
		dev_err(&ddata->ser_i2c_client->dev, "FPDlink not present.");
		r = -ENODEV;
		goto err0;
	}

	dev_info(&ddata->ser_i2c_client->dev, "FPDLink detected.");

	r = i2c_read_le8(ddata->ser_i2c_client, SER925_I2C_DES_ID);
	if (r > 0) {

		/*
		 * dump the 7-bit physical address of the deserializer,
		 *	since that's how it's known by developers.
		 */
		dev_info(&ddata->ser_i2c_client->dev,
				"Deserializer found at phys address 0x%02hhx",
				r>>1);
	} else {
		dev_err(&ddata->ser_i2c_client->dev,
				"Couldn't read i2c deserializer address.  "
				"r = %d, deser->addr = 0x%x",
				r, ddata->deser_i2c_client->addr);
		goto err0;
	}

	dev_info(&ddata->ser_i2c_client->dev, "Serializer status dump:");

	config0 = i2c_read_le8(ddata->ser_i2c_client, SER925_CONFIG0);
	config1 = i2c_read_le8(ddata->ser_i2c_client, SER925_CONFIG1);
	datactl = i2c_read_le8(ddata->ser_i2c_client, SER925_DATA_CTL);
	modests = i2c_read_le8(ddata->ser_i2c_client, SER925_MODE_STS);
	gensts = i2c_read_le8(ddata->ser_i2c_client, SER925_GEN_STS);

	if ((config0 < 0) || (config1 < 0) || (datactl < 0) ||
			(modests < 0) || (gensts < 0)) {
		dev_err(&ddata->ser_i2c_client->dev,
				"Couldn't read serializer status.");
		goto err0;
	}



	dev_info(&ddata->ser_i2c_client->dev,
			"SER925_CONFIG0 = 0x%02x", config0);
	dev_info(&ddata->ser_i2c_client->dev,
			"SER925_CONFIG1 = 0x%02x", config1);
	dev_info(&ddata->ser_i2c_client->dev,
			"SER925_DATA_CTL = 0x%02x", datactl);
	dev_info(&ddata->ser_i2c_client->dev,
			"SER925_MODE_STS = 0x%02x", modests);
	dev_info(&ddata->ser_i2c_client->dev,
			"SER925_GEN_STS = 0x%02x", gensts);

	/*
	 * setup the deserializer alias to talk to the deserializer,
	 *	and then dump its registers.
	 */
	r = tlc59108_setup_deser_alias(ddata);
	if(r<0)
		dev_err(&ddata->deser_i2c_client->dev,
				"fail to set deserializer alias");

	r = i2c_write_le8(ddata->deser_i2c_client,DES928_CONFIG0,0x90);
	if(r<0)dev_err(&ddata->deser_i2c_client->dev, "fail to set DES928_CONFIG0");
	udelay(500);
	r = i2c_write_le8(ddata->deser_i2c_client,DES928_CONFIG1,0xf8);
	if(r<0)dev_err(&ddata->deser_i2c_client->dev, "fail to set DES928_CONFIG1");
	udelay(500);
	r = i2c_write_le8(ddata->deser_i2c_client,DES928_I2C_CTL1,0x2e);
	if(r<0)dev_err(&ddata->deser_i2c_client->dev, "fail to set DES928_I2C_CTL1");
	udelay(500);
		r = i2c_write_le8(ddata->deser_i2c_client,DES928_SCL_HT,0x14);
	if(r<0)dev_err(&ddata->deser_i2c_client->dev, "fail to set DES928_SCL_HT");
	udelay(500);
	r = i2c_write_le8(ddata->deser_i2c_client,DES928_SCL_LT,0x26);
	if(r<0)dev_err(&ddata->deser_i2c_client->dev, "fail to set DES928_SCL_LT");
	dev_info(&ddata->ser_i2c_client->dev, "Deserializer status dump:");
	udelay(500);
	while(retry--){
		config0 = i2c_read_le8(ddata->deser_i2c_client, DES928_CONFIG0);
		config1 = i2c_read_le8(ddata->deser_i2c_client, DES928_CONFIG1);
		datactl = i2c_read_le8(ddata->deser_i2c_client, DES928_DATA_CTL);
		rxsts = i2c_read_le8(ddata->deser_i2c_client, DES928_RX_STS);
		gensts = i2c_read_le8(ddata->deser_i2c_client, DES928_GEN_STS);

		if ((config0 < 0) || (config1 < 0) || (datactl < 0) ||
				(rxsts < 0) || (gensts < 0)) {
			dev_err(&ddata->ser_i2c_client->dev,
				"Couldn't read deserializer status, addr = 0x%02hhx.",
				ddata->deser_i2c_client->addr);
			udelay(500);
		}
		else break;
	}

	dev_info(&ddata->deser_i2c_client->dev,
			"DES928_CONFIG0 = 0x%02x", config0);
	dev_info(&ddata->deser_i2c_client->dev,
			"DES928_CONFIG1 = 0x%02x", config1);
	dev_info(&ddata->deser_i2c_client->dev,
			"DES928_DATA_CTL = 0x%02x", datactl);
	dev_info(&ddata->deser_i2c_client->dev,
			"DES928_RX_STS = 0x%02x", rxsts);
	dev_info(&ddata->deser_i2c_client->dev,
			"DES928_GEN_STS = 0x%02x", gensts);

	return 0;

err0:
	dev_info(&ddata->ser_i2c_client->dev, "%s ends on error", __func__);
	return -EINVAL;
}


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
#ifdef CONFIG_HRB_DISPLAY_SUPPORT
	tlc_init(ddata);
#endif

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void panel_dpi_disable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (!omapdss_device_is_enabled(dssdev))
		return;

	tlc_uninit(ddata);

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

static const struct of_device_id tlc59108_of_match[] = {
	{
		.compatible = "omapdss,ti,tlc59108-tfcs9700",
		.data = &tlc_7_inch_data,
	},
	{
		.compatible = "omapdss,ti,tlc59108-lp101",
		.data = &tlc_10_inch_data,
	},
	{ }
};

MODULE_DEVICE_TABLE(of, tlc59108_of_match);

void tlc59108_interrupt_handler(struct panel_drv_data *ddata)
{
	__u8 rdregs[1] = { SER925_HDCP_STS };

	/*
	 * regs has space for four registers:
	 *	0xC4 = STS
	 *	0xC5 = dummy
	 *	0xC6 = ICR
	 *	0xc7 = ISR
	 */
	__u8 regs[4] = { 0 };
#define sts (regs[0])
#define icr (regs[2])
#define isr (regs[3])
	int r;
	int is_rx_int;


	r = tlc59108_i2c_cmdrsp(ddata->ser_i2c_client,
			rdregs, sizeof(rdregs), regs, sizeof(regs));
	if (2 == r) {
		is_rx_int = ((isr & icr) & SER925_HDCP_ISR_IS_RX_INT) != 0;
		if (is_rx_int) {
			gt9271_ts_work_func(ddata);
				}
	}
	if (isr & SER925_HDCP_ISR_INT_DETECT) {
		/* downstream receiver detected */
		if(ddata->display_detected == 0)
		{
			/*enable touch interrupt*/
			r = i2c_read_le8(ddata->ser_i2c_client,
					SER925_HDCP_ICR);
			if (r >= 0) {
				r |= SER925_HDCP_ICR_IS_RX_INT;

				i2c_write_le8(
						ddata->ser_i2c_client,
						SER925_HDCP_ICR, r);
			};
			ddata->display_detected = 1;
		}

		dev_info(&ddata->tlc59108_i2c_client->dev,
				"HDCP ISR INT DETECT\n");
	}


}

static irqreturn_t tlc59108_interrupt(int irq, void *dev_id)
{

	struct panel_drv_data *ddata = dev_id;
	tlc59108_interrupt_handler(ddata);
	return IRQ_HANDLED;
}

static int lvds_power_on(struct device *dev)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct device_node *np = dev->of_node;
	int r = 0;
	int ret = 0;

	/* Get I2c node of max7310 */
	np = of_parse_phandle(dev->of_node, "port-expander", 0);
	if (np)
		ddata->max7310_i2c_client = of_find_i2c_device_by_node(np);
	else return -EINVAL;
	dev_info(dev, "max7310_i2c_client I2C ID 0x%x",
			ddata->max7310_i2c_client->addr);

	r= tlc59108_setup_port_expander_alias(ddata);
	if (r < 0) {

		dev_err(&ddata->ser_i2c_client->dev,
				"%s set up port_expander_alias failed.\n\n",__func__);

	}
	udelay(500);
	mutex_lock(&ddata->mutex);
	r = i2c_read_le8(ddata->max7310_i2c_client,MAX7310_CONFIG);

	/* clear max7210 configuration register bit6 and bit7
	 * to configure it as output port*/
	r = r & MAX7310_POWER_MASK;
	ret = i2c_write_le8(ddata->max7310_i2c_client,MAX7310_CONFIG,r);
	if (ret < 0) {

		dev_err(&ddata->max7310_i2c_client->dev,
				"%s config output port failed.\n\n",__func__);

	}
	udelay(500);
	/*Maxim MAX7310 Polarity register to 0 so that input is not inverted*/
	r = 0;
	ret = i2c_write_le8(ddata->max7310_i2c_client,MAX7310_POLARITY,r);
	if (ret < 0) {

		dev_err(&ddata->max7310_i2c_client->dev,
				"%s set Polarity register failed.\n\n",__func__);

	}

	/* Maxim MAX7310 Data_Output register set I/O7 to 1 to power on LVDS,
	 * Set I/06 to 1 to enable backlight*/
	r = i2c_read_le8(ddata->max7310_i2c_client, MAX7310_OUTPUT);

	r = r | (~MAX7310_POWER_MASK);
	ret = i2c_write_le8(ddata->max7310_i2c_client,MAX7310_OUTPUT,r);
	if (ret < 0) {

		dev_err(&ddata->max7310_i2c_client->dev,
				"%s Data_Output register set failed.\n\n",__func__);

	}
	ret = i2c_write_le8(ddata->max7310_i2c_client,MAX7310_TIMEOUT,0);
	if (ret < 0) {

		dev_err(&ddata->max7310_i2c_client->dev,
				"%s timeout register set failed.\n\n",__func__);

	}

	mutex_unlock(&ddata->mutex);

	return 0;
}


/*button timer timeout handler*/
int button_pressed_flag = 0;
static void tlc59108_button_timer_worker(struct work_struct *work)
{
	int r = 0;
	int i = 0;
	__u8 pin_status= 0;
	struct panel_drv_data *ddata = container_of(work,
			struct panel_drv_data, tlc59108_button_timer_work.work);

	mutex_lock(&ddata->mutex);
	r= tlc59108_setup_port_expander_alias(ddata);
	if (r < 0) {

		dev_err(&ddata->ser_i2c_client->dev,
				"%s set up port_expander_alias failed.\n\n",__func__);
		goto err;

	}


	r = i2c_read_le8(ddata->max7310_i2c_client,MAX7310_INPUT);
	if (r <= 0) {
/*commit out for it will flush boot log*/
		/*dev_err(&ddata->ser_i2c_client->dev,
				"%s fail to get port_expander pin status\n\n",__func__);*/
		goto err;

	}

	/*only report press and release message*/
	if((r == 0xff && button_pressed_flag) ||  r != 0xff )
	{
		if(r == 0xff)
			button_pressed_flag = 0; /*release button*/
		else
			button_pressed_flag = 1;      /*press button*/


		for(i=0;i<6;i++)
		{
			/*first 6 bits are button status*/
			pin_status=BUTTON_STATUS(r,i);

			input_report_key(ddata->ts_data->input_dev,
					registered_keys[i], pin_status);
			input_sync(ddata->ts_data->input_dev);

		}

	}
err:
	tlc59108_setup_ts_alias(ddata);
	mutex_unlock(&ddata->mutex);
	queue_delayed_work(ddata->workq,
			&ddata->tlc59108_button_timer_work,
			BUTTON_TIMEOUT);

}


static int tlc_probe_of(struct device *dev)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct device_node *np = dev->of_node;
	const struct of_device_id *of_dev_id;
	int r;

#ifdef CONFIG_HRB_DISPLAY_SUPPORT

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
#endif

	ddata->in = omapdss_of_find_source_for_first_ep(np);
	if (IS_ERR(ddata->in)) {
		dev_err(dev, "failed to find video source\n");
		return PTR_ERR(ddata->in);
	}

	of_dev_id = of_match_device(tlc59108_of_match, dev);
	if (!of_dev_id) {
		dev_err(dev, "Unable to match device\n");
		return -ENODEV;
	}

	ddata->board_data = of_dev_id->data;
	ddata->videomode = ddata->board_data->timings;
#ifndef CONFIG_HRB_DISPLAY_SUPPORT
	/* Get I2c node of serializer */
	np = of_parse_phandle(dev->of_node, "serializer", 0);
	if (np)
		ddata->ser_i2c_client = of_find_i2c_device_by_node(np);
	else
		return -EINVAL;

	dev_info(dev, "Serial I2C ID %x",ddata->ser_i2c_client->addr);

	/*
	 * change Ser925 default I2C address to 0X0c according
	 * to TI's request
	 * still support boards with address 0x17
	 */
	r = i2c_read_le8(ddata->ser_i2c_client,SER925_I2C_DEVICE_ID);
	if (r < 0) {

		ddata->ser_i2c_client->addr = SER925_ALT_I2C_ADDRESS;

		r = i2c_read_le8(ddata->ser_i2c_client,SER925_I2C_DEVICE_ID);

		if (r < 0) {
			dev_err(dev,"fail to read Ser address\n");
			return 	r;
		}
	}

	/* Get I2c node of dserializer */
	np = of_parse_phandle(dev->of_node, "deserializer", 0);
	if (np) {
		ddata->deser_i2c_client = of_find_i2c_device_by_node(np);
	} else {
		return -EINVAL;
	}

	dev_info(dev, "Deserial I2C ID %x",	ddata->deser_i2c_client->addr);

	if (of_property_read_u32(dev->of_node, "deser-alias",
			&ddata->deser_alias)) {
		dev_err(dev, "No deser_alias defined");
		return -EINVAL;
	}

	ddata->deser_i2c_client->addr = ddata->deser_alias;
	dev_info(dev, "Deserializer alias = %02hhx", ddata->deser_alias);
#endif

	return 0;
}

struct regmap_config tlc59108_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

/**
 * gt9271_request_input_dev - Allocate, populate and register the input device
 *
 * @ts: our GT9271_ts_data pointer
 *
 * Must be called during probe
 */
static int gt9271_request_input_dev(struct gt9271_ts_data *ts)
{
	int ret,i;
	int n = 0;
	ts->input_dev = devm_input_allocate_device(&ts->client->dev);
	if (ts->input_dev == NULL) {
		GT9271_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) |
				  BIT_MASK(EV_KEY) |
				  BIT_MASK(EV_ABS);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0,ts->x_size, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0,ts->y_size, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	n = sizeof(registered_keys) / sizeof(registered_keys[0]);
	for (i = 0; i < n; i++)
		__set_bit(registered_keys[i], ts->input_dev->keybit);

	input_mt_init_slots(ts->input_dev, ts->num_mt_slots,
			    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);

	ts->input_dev->name = gt9271_ts_name;
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0x0416;
	ts->input_dev->id.product = 0x1001;
	ts->input_dev->id.version = 10427;
/*	input_set_drvdata(ts->input_dev,ts);*/


	ret = input_register_device(ts->input_dev);
	if (ret) {
		GT9271_ERROR("Failed to register %s input device",
			  ts->input_dev->name);
		input_free_device(ts->input_dev);
		return -ENODEV;
	}
	return 0;
}

static int gt9271_read_version(struct i2c_client *client, u16 *version)
{
	int error;
	u8 buf[6];

	error = gt9271_i2c_read(client, GT9271_REG_VERSION, buf, sizeof(buf));
	if (error) {
		dev_err(&client->dev, "read version failed: %d\n", error);
		return error;
	}

	if (version)
		*version = get_unaligned_le16(&buf[4]);

	dev_info(&client->dev, "IC VERSION: %6ph\n", buf);

	return 0;
}


static void gt9271_read_config(struct gt9271_ts_data *ts)
{
	u8 config[GT9271_CONFIG_MAX_LENGTH];
	int error;
	if(ts == NULL)
	{
		printk("gt9271_read_config fail due to null pointer\n");
		return;
	}
	error = gt9271_i2c_read(ts->client, GT9271_REG_CONFIG_DATA,
			      config,
			   GT9271_CONFIG_MAX_LENGTH);
	if (error) {
		dev_warn(&ts->client->dev,
			 "Error reading config (%d), using defaults\n",
			 error);
		ts->abs_x_max = GT9271_MAX_WIDTH;
		ts->abs_y_max = GT9271_MAX_HEIGHT;
		ts->int_trigger_type = GT9271_INT_TRIGGER;
		return;
	}

	ts->abs_x_max = get_unaligned_le16(&config[RESOLUTION_LOC]);
	ts->abs_y_max = get_unaligned_le16(&config[RESOLUTION_LOC + 2]);
	ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03;
	if (!ts->abs_x_max || !ts->abs_y_max) {
		dev_err(&ts->client->dev,
			"Invalid config, using defaults\n");
		ts->abs_x_max = GT9271_MAX_WIDTH;
		ts->abs_y_max = GT9271_MAX_HEIGHT;
	}
}


/**
 * gt9271_i2c_test - I2C test function to check if the device answers.
 *
 * @client: the i2c client
 */
static int gt9271_i2c_test(struct i2c_client *client)
{
	int retry = 0;
	int error;
	u8 test;

	while (retry++ < 2) {
		error = gt9271_i2c_read(client, GT9271_REG_CONFIG_DATA,
					&test, 1);
		if (!error)
			return 0;

		dev_err(&client->dev, "i2c test failed attempt %d: %d\n",
			retry, error);
		msleep(20);
	}

	return error;
}

int gt9271_of_populate(struct i2c_client *client,
					struct gt9271_ts_data *ts)
{
	struct device_node *node = client->dev.of_node;
	unsigned int val;
	int ret = -EINVAL;

	ret = of_property_read_u32(node, "res-x", &val);
	if (ret)
		goto error;
	ts->x_size = val;

	ret = of_property_read_u32(node, "res-y", &val);
	if (ret)
		goto error;

	ts->y_size = val;

	ret = of_property_read_u32(node, "max-touch-points", &val);
	if (ret)
		goto error;

	ts->num_mt_slots = val;

error:
	return ret;
}


static int gt9271_probe(struct panel_drv_data *ddata)
{
	struct gt9271_ts_data *ts;
	u16 version_info = 0;
	struct i2c_client *ts_client = ddata->gt9271_i2c_touch_client;
	int ret;
	int r;

	ts = devm_kzalloc(&ts_client->dev,
			sizeof(struct gt9271_ts_data), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	/* get dt values to populate platform data */
	ret = gt9271_of_populate(ts_client, ts);
	if (ret)
		return -EINVAL;



	i2c_set_clientdata(ts_client, ts);
	ts->client = ts_client;
	udelay(500);
	tlc59108_setup_lcd_alias(ddata);
	udelay(500);
	gt9271_power_on(ddata);
	msleep(20);
	ret = tlc59108_setup_ts_alias(ddata);
	if(ret<0)
	{
		printk("gt9271  touch screen did not got control \n");
		return -EINVAL;
	}

	udelay(500);

	ret = gt9271_i2c_test(ts_client);
	if (ret < 0) {
		ts_client->addr = 0x5d;
		ret = gt9271_i2c_test(ts_client);
		if (ret < 0) {
			GT9271_ERROR("I2C communication ERROR!");
			return -ENODEV;
		}
		GT9271_INFO("fail to read the address GTP I2C"
				"new Address: 0x%02x", ts_client->addr);
	}
	ret = gt9271_read_version(ts_client, &version_info);
	if (ret < 0) {
		GT9271_ERROR("Read version failed.");
		return ret;
	}
	else printk("gt9271 version is %d\n",version_info);

	gt9271_read_config(ts);

	ret = gt9271_request_input_dev(ts);
	if (ret < 0) {
		GT9271_ERROR("GTP request input dev failed");
		return ret;
	}

	/*enable touch interrupt*/
	if(ddata->display_detected){
		r = i2c_read_le8(ddata->ser_i2c_client,
				SER925_HDCP_ICR);
		if (r >= 0) {
			r |= SER925_HDCP_ICR_IS_RX_INT;

			i2c_write_le8(
					ddata->ser_i2c_client,
					SER925_HDCP_ICR, r);
		}
	}

	ddata->ts_data = ts;
	return ret;
}

static int tlc59108_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int r;
	struct regmap *regmap;
	struct panel_drv_data *ddata;
	struct device *dev = &client->dev;
	struct omap_dss_device *dssdev;
	struct device_node *node = NULL;
	int irq = 0;
	unsigned int val;

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (ddata == NULL)
		return -ENOMEM;

	dev_set_drvdata(dev, ddata);
#ifdef CONFIG_HRB_DISPLAY_SUPPORT
	r = tlc_probe_of(dev);
	if (r)
		return r;

	regmap = devm_regmap_init_i2c(client, &tlc59108_regmap_config);
	if (IS_ERR(regmap)) {
		r = PTR_ERR(regmap);
		dev_err(dev, "Failed to init regmap: %d\n", r);
		goto err_gpio;
	}

	ddata->regmap = regmap;

	usleep_range(10000, 15000);

	/* Try to read a TLC register to verify if i2c works */
	r = regmap_read(ddata->regmap, TLC59108_MODE1, &val);

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

	dev_info(dev, "Successfully initialized %s\n", TLC_NAME);
#else

	ddata->tlc59108_i2c_client = client;

	mutex_init(&ddata->mutex);
	regmap = devm_regmap_init_i2c(client, &tlc59108_regmap_config);
	if (IS_ERR(regmap)) {
		r = PTR_ERR(regmap);
		dev_err(dev, "Failed to init regmap: %d\n", r);
		goto err_gpio;
	}

	ddata->regmap = regmap;
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

	dev_info(dev, "Successfully initialized %s\n", TLC_NAME);

	r = tlc_probe_of(dev);
	if (r)
		goto err_gpio;

	/*
	 * perform minimal startup of serializer.
	 * more initialization will be done when the link is detected.
	 */
	r = i2c_write_le8(ddata->ser_i2c_client, SER925_RESET,
			SER925_RESET_DIGITAL_RESET1);
	if (r < 0) {
		dev_err(&client->dev, "Failed to reset Serializer.");
		r = -EINVAL;
		goto err_gpio;
	}

	dev_info(&client->dev, "Serializer Reset done.");

	r = i2c_write_le8(ddata->ser_i2c_client, SER925_CONFIG0,
			SER925_CONFIG0_BACK_CHAN_CRC |
			SER925_CONFIG0_I2C_PASSTHRU |
			SER925_CONFIG0_PCLK_AUTO);

	if (r < 0) {
		dev_err(&client->dev, "Failed to configure"
				"Serializer (CONFIG0).");
		r = -EINVAL;
		goto err_gpio;
	}

	r = i2c_write_le8(ddata->ser_i2c_client,SER925_CONFIG1,
			SER925_CONFIG1_LFMODE_SEL |
			SER925_CONFIG1_BCMODE_SEL |
			SER925_CONFIG1_FAILSAFE);
	if (r < 0) {
		dev_err(&client->dev, "Failed to configure"
				" Serializer (CONFIG1).");
		r = -EINVAL;
		goto err_gpio;
	}

	i2c_read_le8(ddata->ser_i2c_client, SER925_HDCP_CFG);
	if (r >= 0) {

		r = i2c_write_le8(ddata->ser_i2c_client, SER925_HDCP_CFG,
				r | SER925_HDCP_CFG_RX_DET_SEL);
		if (r < 0)
			dev_err(&ddata->ser_i2c_client->dev, "HDCP CFG not set");

	} else {
		dev_err(&ddata->ser_i2c_client->dev, "HDCP CFG not read");
	}

	dev_info(&client->dev, "Serializer configuration done.");

	r = i2c_read_le8(ddata->ser_i2c_client,SER925_GEN_STS);
	if (r < 0) {
		dev_err(&ddata->ser_i2c_client->dev,
				"fail to read Serializer general status register\n\n");
	}
	else printk("SER925_GEN_STS  VALUE IS %d\n",r);

	r = tlc59108_setup_link(ddata);

	if (r < 0) {

		dev_err(&ddata->ser_i2c_client->dev,
				"%s set up LVDS link failed.\n\n",__func__);

	}
	udelay(500);


	/*power on the display*/
	r= tlc59108_setup_port_expander_alias(ddata);
	if (r < 0) {

		dev_err(&ddata->ser_i2c_client->dev,
				"%s set up port_expander_alias failed.\n\n",__func__);

	}
	if(lvds_power_on(dev))printk("fail to power on the tlc59108 display\n");

	node = of_parse_phandle(dev->of_node, "gt9271-touch-sreen", 0);
	if (node)
		ddata->gt9271_i2c_touch_client = of_find_i2c_device_by_node(node);
	else
		goto err_gpio;
	irq = irq_of_parse_and_map(node, 0);

	if (irq) {
		ddata->irq = irq;
		dev_info(dev, "tlc59108 IRQ = %u", ddata->irq);
	} else {
		pr_err("Could not obtain IRQ for tlc59108 TS, %u", ddata->irq);
		r = -EINVAL;
		goto err_gpio;
	}
	/* We do not set IS_RX_INT (INTB pin asserted)
	  * until after a receiver is detected.*/
	ddata->display_detected = 0;


	/*register touch screen inerrupt*/
	r = devm_request_threaded_irq(dev, ddata->irq,
			NULL, tlc59108_interrupt, IRQF_ONESHOT | IRQF_TRIGGER_LOW,
			dev_name(&client->dev), ddata);
	if (r) {
		dev_err(&client->dev, "Failed to register interrupt");
		r = -EINVAL;
		goto err_gpio;
	}

	r = SER925_HDCP_ICR_IE_RXDET_INT | SER925_HDCP_ICR_INT_ENABLE;
	r = i2c_write_le8(ddata->ser_i2c_client, SER925_HDCP_ICR, r);
	if (r < 0)
		dev_err(&client->dev,
				"Couldn't configure HDCP ICR");
	r = i2c_read_le8(ddata->ser_i2c_client, SER925_HDCP_ISR);
	if (r < 0)
		dev_err(&ddata->ser_i2c_client->dev,
				"%s: Couldn't read back HDCP ISR",
				__func__);
	else
		dev_info(&ddata->ser_i2c_client->dev,
				"%s: ISR = %02hhx",
				__func__, r);


	r= tlc59108_setup_lcd_alias(ddata);
	if (r < 0) {

		dev_err(&ddata->ser_i2c_client->dev,
				"%s set up lcd_alias failed.\n\n",__func__);

	}
	else dev_err(&ddata->ser_i2c_client->dev,
			"%s set up lcd_alias done.\n\n",__func__);

	tlc_init(ddata);
	r = gt9271_probe(ddata);
	if (r) {
		printk("fail to probe gt9271 touch screen");
		goto err_remove_input_device;
	}

	ddata->workq = create_singlethread_workqueue("tlc59108");
	if (!ddata->workq) {
		dev_info(&client->dev, "Unable to create workqueue");
		r = -EINVAL;
		goto err_remove_input_device;
	}
	INIT_DELAYED_WORK(&ddata->tlc59108_button_timer_work,
				tlc59108_button_timer_worker);
	queue_delayed_work(ddata->workq,
				&ddata->tlc59108_button_timer_work,
				BUTTON_TIMEOUT);
#endif
	printk("#############tlc59108_i2c_probe is done###############\n");

	return 0;

err_remove_input_device:
	input_unregister_device(ddata->ts_data->input_dev);
err_reg:
err_gpio:
	omapdss_unregister_display(&ddata->dssdev);
	panel_dpi_disable(dssdev);
	panel_dpi_disconnect(dssdev);
	omap_dss_put_device(ddata->in);
	dev_set_drvdata(&client->dev, NULL);
	return r;
}

static int gt9271_remove(struct i2c_client *client)
{
	struct gt9271_ts_data *data =
			(struct gt9271_ts_data *)i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
	input_unregister_device(input_dev);

	return 0;
}

static int tlc59108_i2c_remove(struct i2c_client *client)
{
	struct panel_drv_data *ddata = dev_get_drvdata(&client->dev);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	struct omap_dss_device *in = ddata->in;

	if (ddata->enable_gpio)
		gpiod_set_value_cansleep(ddata->enable_gpio, 0);
#ifndef CONFIG_HRB_DISPLAY_SUPPORT
	gt9271_remove(ddata->gt9271_i2c_touch_client);
#endif
	omapdss_unregister_display(dssdev);

	panel_dpi_disable(dssdev);
	panel_dpi_disconnect(dssdev);

	omap_dss_put_device(in);

	dev_set_drvdata(&client->dev, NULL);
	return 0;
}

static const struct i2c_device_id tlc59108_id[] = {
	{ TLC_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tlc59108_id);

static struct i2c_driver tlc59108_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= TLC_NAME,
		.of_match_table = tlc59108_of_match,
	},
	.id_table	= tlc59108_id,
	.probe		= tlc59108_i2c_probe,
	.remove		= tlc59108_i2c_remove,
};

module_i2c_driver(tlc59108_i2c_driver);

MODULE_AUTHOR("Archit Taneja  <archit@ti.com>");
MODULE_DESCRIPTION("TLC-59108 DPI Panel Driver");
MODULE_LICENSE("GPL");
