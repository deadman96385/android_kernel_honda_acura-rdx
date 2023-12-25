/*
 * Copyright (C) 2013 Texas Instruments Inc
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Copyright (C) 2014 Denso International America */

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

#include <linux/slab.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/mutex.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/time.h>

#include <video/omapdss.h>

#include "serializer.h"
#include "deserializer.h"
#include "panel-hralvds.h"
#include "panel-hralvds_tjba.h"
#include "hralvds-proto.h"

/* panel-hralvds.c's hralvds_i2c_probe function will refer to this object. */
struct omap_video_timings hralvds_video_timings = {
	/*
	 * set the pixelclock for 57Hz refreash rate.
	 *
	 * make sure that the values are copied properly into the .pixelclock
	 *	calculation from the fields
	 *		.xres
	 *		.hfp
	 *		.hbp
	 *		.hsw
	 *		.yres
	 *		.vfp
	 *		.vbp
	 *		.vsw
	 *
	 * the result of the calculation, in Hz:
	 *	.pixelclock	= 82,441,152
	 */
	.pixelclock	= 57 * (HRALVDS_TJBA_WIDTH + 54 + 4 + 6) *
				(HRALVDS_TJBA_HEIGHT + 3 + 3 + 3),

	.x_res		= HRALVDS_TJBA_WIDTH,
	.hfp		= 54,
	.hbp		= 4,
	.hsw		= 6,

	.y_res		= HRALVDS_TJBA_HEIGHT,
	.vfp		= 3,
	.vbp		= 3,
	.vsw		= 3,

	.vsync_level	= OMAPDSS_SIG_ACTIVE_LOW,
	.hsync_level	= OMAPDSS_SIG_ACTIVE_LOW,
	.data_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
	.de_level	= OMAPDSS_SIG_ACTIVE_HIGH,
	.sync_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
};

/* during display updates when Display Status Notification command is rx'd */
extern void hralvds_display_status_notification(struct panel_drv_data *ddata);

/* handle a status event from the hralvds display */
static void hralvds_status_event(struct panel_drv_data *ddata)
{
	/* buffer to send contains only the REPORT ID STATUS command byte. */
	__u8 wbuf[] = { HRALVDS_REPORT_ID_STATUS };
	/* buffer to receive the response in.  '+ 1' for the checksum. */
	__u8 rbuf[HRALVDS_REPORT_ID_STATUS_SIZE + 1] = { 0 };
	int r;

	/* read the status event and get the information from the hralvds. */
	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
				wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 1, 3);

	if (2 == r) {
#if defined HRALVDS_LOG_DEBUG
		/*
		 * possibly dump the received buffer (done before
		 * checksum).
		 */
		if (ddata->dbg_lvl & HRALVDS_DBG_LEVEL_DUMP) {
			char buf[(sizeof(__func__) + 1) +
					(sizeof(rbuf) * 3) + 2];
			hralvds_dump_msg(&ddata->ts_i2c_client->dev,
						__func__,
						rbuf, sizeof(rbuf),
						buf, sizeof(buf));
		}
#endif

		/* Handle gradual time change complete */
		if (rbuf[0] & (1 << 6))
			hralvds_on_finish_gradual_change(ddata);

		mutex_lock(&ddata->event_status_lock);
		ddata->event_status[0] = rbuf[0];
		ddata->event_status[1] = rbuf[1];
		mutex_unlock(&ddata->event_status_lock);
		sysfs_notify(&ddata->ts_i2c_client->dev.kobj, NULL,
				"hralvds_event_status");
	}
}

/* handle a display mode event from the hralvds display */
static void hralvds_display_mode_event(struct panel_drv_data *ddata)
{
	/* buffer to send contains only the REPORT ID STATUS command byte. */
	__u8 wbuf[] = { HRALVDS_REPORT_ID_CAMERA_DISPLAY_MODE };
	/* buffer to receive the response in.  '+ 1' for the checksum. */
	__u8 rbuf[HRALVDS_REPORT_ID_CAMERA_DISPLAY_MODE_SIZE + 1] = { 0 };
	int r;

	/* read the status event and get the information from the hralvds. */
	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
				wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 1, 3);

	if (2 == r) {
#if defined HRALVDS_LOG_DEBUG
		/*
		 * possibly dump the received buffer (done before
		 * checksum).
		 */
		if (ddata->dbg_lvl & HRALVDS_DBG_LEVEL_DUMP) {
			char buf[(sizeof(__func__) + 1) +
					(sizeof(rbuf) * 3) + 2];
			hralvds_dump_msg(&ddata->ts_i2c_client->dev,
						__func__,
						rbuf, sizeof(rbuf),
						buf, sizeof(buf));
		}
#endif

		mutex_lock(&ddata->display_mode_lock);
		ddata->display_mode = rbuf[0];
		mutex_unlock(&ddata->display_mode_lock);
		sysfs_notify(&ddata->ts_i2c_client->dev.kobj, NULL,
				"hralvds_display_mode_event");
	}
}

/* Handle a display buzzer event from the hralvds display
 *
 * This method just reads the event to comply with spec but doesn't do anything
 * with the data since it's not supported in TJBA
 */
static void hralvds_display_buzzer_event(struct panel_drv_data *ddata)
{
	/* buffer to send contains only the REPORT ID CAMERA BUZZER command byte. */
	__u8 wbuf[] = { HRALVDS_REPORT_ID_CAMERA_BUZZER };
	/* buffer to receive the response in.  '+ 1' for the checksum. */
	__u8 rbuf[HRALVDS_REPORT_ID_CAMERA_BUZZER_SIZE + 1] = { 0 };
	int r;

	/* read the status event and get the information from the hralvds. */
	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
				wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 1, 3);

	if (2 == r) {
#if defined HRALVDS_LOG_DEBUG
		/*
		 * possibly dump the received buffer (done before
		 * checksum).
		 */
		if (ddata->dbg_lvl & HRALVDS_DBG_LEVEL_DUMP) {
			char buf[(sizeof(__func__) + 1) +
					(sizeof(rbuf) * 3) + 2];
			hralvds_dump_msg(&ddata->ts_i2c_client->dev,
						__func__,
						rbuf, sizeof(rbuf),
						buf, sizeof(buf));
		}
#endif
	}
}


static ssize_t show_hralvds_key_event(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	/*
	 * the key event is empty at this time.
	 * the TJBA display only sends day/night key events, and we're only
	 *	interested in the day/night key release.
	 * so the data read by the consumer is empty as all it needs to know
	 *	is that a day/night key release event occurred.
	 * if other keys are needed in the future, they'll need to be processed
	 *	here and in hralvds_key_event, below.
	 */
	*buf++ = 0;
	*buf++ = 0;
	return 1;
}

/*
 * FOR TESTING PURPOSES ONLY:
 *    run "cat hralvds_fake_key_event" to trigger a sysfs_notify on
 *	hralvds_key_event to unblock the listener.
 *    this function should be removed after testing.
 */
static ssize_t set_hralvds_fake_key_event(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	sysfs_notify(&ddata->ts_i2c_client->dev.kobj, NULL,
				"hralvds_key_event");
	return count;
}

static DEVICE_ATTR(hralvds_key_event, S_IRUGO,
	show_hralvds_key_event, NULL);
static DEVICE_ATTR(hralvds_fake_key_event, S_IWUGO,
	NULL, set_hralvds_fake_key_event);

static struct attribute *hralvds_tjba_attributes[] = {
	&dev_attr_hralvds_key_event.attr,
	&dev_attr_hralvds_fake_key_event.attr,
	NULL,
};

static const struct attribute_group tjba_attr_group = {
	.attrs = hralvds_tjba_attributes,
};

/*
 * handle a display key event from the hralvds display.
 * if the key event is the day/night key release, signal syfs
 *	that hralvds_key_event is ready.
 */
static void hralvds_key_event(struct panel_drv_data *ddata)
{
	/* buffer to send contains only the REPORT ID KEY command byte. */
	__u8 wbuf[] = { HRALVDS_REPORT_ID_KEY };
	/* buffer to receive the response in.  '+ 1' for the checksum. */
	__u8 rbuf[HRALVDS_REPORT_ID_KEY_SIZE + 1] = { 0 };
	int r;

	/* read the status event and get the information from the hralvds. */
	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
				wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 1, 3);

	if (2 == r) {
#if defined HRALVDS_LOG_DEBUG
		/*
		 * possibly dump the received buffer (done before
		 * checksum).
		 */
		if (ddata->dbg_lvl & HRALVDS_DBG_LEVEL_DUMP) {
			char buf[(sizeof(__func__) + 1) +
					(sizeof(rbuf) * 3) + 2];
			hralvds_dump_msg(&ddata->ts_i2c_client->dev,
						__func__,
						rbuf, sizeof(rbuf),
						buf, sizeof(buf));
		}

		/*
		 * the day/night key event is in the 0th byte of the rx buffer.
		 * when it's 0, the event is a release.
		 */
		if (rbuf[0] == 0) {
			/*
			 * signal that a release has occurred.
			 */
			sysfs_notify(&ddata->ts_i2c_client->dev.kobj, NULL,
					"hralvds_key_event");

		}
#endif
	}
}

static void hralvds_dispatch_event(struct panel_drv_data *ddata,
	__u8 *rbuf, size_t rbuflen)
{
#if defined HRALVDS_LOG_DEBUG
	/*
	 * possibly dump the received buffer (done before checksum).
	 * this can delay processing and communication with the display,
	 *	so use it carefully.
	 */
	if (ddata->dbg_lvl & HRALVDS_DBG_LEVEL_DUMP) {
		char buf[(sizeof(__func__) + 1) + (rbuflen * 3) + 2];

		hralvds_dump_msg(&ddata->ts_i2c_client->dev,
					__func__,
					rbuf, rbuflen ,
					buf, sizeof(buf));
	}
#endif

	/*
	 * valid checksum, see what the
	 *	touchscreen wants to tell us.
	 */
	switch (rbuf[0]) {
	case HRALVDS_REPORT_ID_STATUS:
		/* process the event before doing any debug */
		hralvds_status_event(ddata);

#if defined HRALVDS_LOG_DEBUG
		if (ddata->dbg_lvl & HRALVDS_DBG_LEVEL_EVENT)
			dev_info(&ddata->ts_i2c_client->dev,
				"%s active: STATUS\n", __func__);

		if ((HRALVDS_REPORT_ID_STATUS_SIZE != rbuf[1]) &&
								ddata->dbg_lvl)
			dev_info(&ddata->ts_i2c_client->dev,
				"%s invalid length reported in "
				"STATUS event: %d, but should be %d\n",
				__func__, rbuf[1],
				HRALVDS_REPORT_ID_STATUS_SIZE);
#endif
		break;

	case HRALVDS_REPORT_ID_CAMERA_DISPLAY_MODE:
		hralvds_display_mode_event(ddata);
		break;

	case HRALVDS_REPORT_ID_DISPLAY_STATUS_NOTIFICATION:
		/*
		 * Back to hralvds panel driver to handle this.
		 * We could process it here, but since the display update
		 *	process isn't a per-display-type function, it gets
		 *	handled in the main driver.
		 */
		hralvds_display_status_notification(ddata);
		break;

	case HRALVDS_REPORT_ID_CAMERA_BUZZER:
		hralvds_display_buzzer_event(ddata);
		break;

	case HRALVDS_REPORT_ID_KEY:
		hralvds_key_event(ddata);
		break;

	default:
		break;
	}
}

/*
 * hralvds_config_input_dev:
 *
 * configure the input device so that key, touch screen, knob and status
 *	events can go to the input system.
 *
 * THIS FUNCTION NEEDS TO BE MOVED TO THE PROPER DISPLAY-SPECIFIC DRIVER FILE.
 */
static int hralvds_config_input_dev(struct device *dev)
{
	int r;
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	ddata->input_dev = devm_input_allocate_device(dev);
	if (!ddata->input_dev)
		return -ENOMEM;

	ddata->input_dev->name = "HRALVDS TJBA Touchscreen Panel";
	ddata->input_dev->id.bustype = BUS_I2C;
	ddata->input_dev->dev.parent = dev;
	input_set_drvdata(ddata->input_dev, ddata);

	input_set_abs_params(ddata->input_dev, ABS_X, 0,
					HRALVDS_TJBA_WIDTH, 0, 0);
	input_set_abs_params(ddata->input_dev, ABS_Y, 0,
					HRALVDS_TJBA_HEIGHT, 0, 0);

	input_set_abs_params(ddata->input_dev, ABS_MT_POSITION_X, 0,
					HRALVDS_TJBA_WIDTH, 0, 0);
	input_set_abs_params(ddata->input_dev, ABS_MT_POSITION_Y, 0,
					HRALVDS_TJBA_HEIGHT, 0, 0);

	r = input_register_device(ddata->input_dev);

	return 0;
}

int hralvds_panel_probe(struct panel_drv_data *ddata)
{
	int r;

	/*
	 * currently, we don't use hralvds_panel_data in the TJBA context.
	 * if it becomes necessary, use:
	 *
	 *	ddata->hralvds_panel_data =
	 *		devm_kzalloc(&ddata->ts_i2c_client->dev,
	 *			sizeof(struct hralvds_tjba_data), GFP_KERNEL);
	 *	if (NULL == ddata->hralvds_panel_data)
	 *		return -ENOMEM;
	 */
	ddata->hralvds_panel_data = NULL;

	r = hralvds_config_input_dev(&ddata->ts_i2c_client->dev);
	if (r < 0) {
		dev_err(&ddata->ts_i2c_client->dev,
			"Failed to allocate/configure input device, r = %d.", r);
		return r;
	}

	r = sysfs_create_group(&ddata->ts_i2c_client->dev.kobj,
		&tjba_attr_group);
	if (r < 0)
		dev_info(&ddata->ts_i2c_client->dev,
			"Unable to create sysfs files.");

	return r;
}

int hralvds_panel_irq(struct panel_drv_data *ddata)
{
	int r;
	__u8 wbuf[1] = { HRALVDS_REPORT_ID_CMD };
	__u8 rbuf[2 + 1] = { 0 };

	/*
	 * read the report ID and get the information
	 *	from the hralvds.
	 */
	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
		wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 1, 3);

	if (2 == r)
		hralvds_dispatch_event(ddata, rbuf, sizeof(rbuf));
	else
		dev_warn(&ddata->ts_i2c_client->dev,
			"Error fetching Report ID from display: r = %d", r);

	return 0;
}

int hralvds_fwif_update_supported(void)
{
	return 1;
}

