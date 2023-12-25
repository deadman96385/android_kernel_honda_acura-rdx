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
#include "panel-hralvds_thra.h"
#include "hralvds-proto.h"

/* panel-hralvds.c's hralvds_i2c_probe function will refer to this object. */

#if 1

struct omap_video_timings hralvds_video_timings = {
	/* Should be 74250000 Hz */
	.pixelclock	=
		/* Horizontal Component */
		((HRALVDS_THRA_WIDTH) +
			HRALVDS_THRA_TIMING_HFP + HRALVDS_THRA_TIMING_HBP +
			HRALVDS_THRA_TIMING_HSW) *
		/* Vertical Component */
		((HRALVDS_THRA_HEIGHT -
			HRALVDS_THRA_DEAD_ZONE_TOP -
			HRALVDS_THRA_DEAD_ZONE_BOTTOM) +
			HRALVDS_THRA_TIMING_VFP + HRALVDS_THRA_TIMING_VBP +
			HRALVDS_THRA_TIMING_VSW) *
		HRALVDS_THRA_REFRESH_RATE,

	/*
	 * N.B.: The horizontal margins are not applied, as the panel can not
	 * display anything but 1280.  Instead the Android Window Manager
	 * Overscan right margin (40 pixels) will be used to compensate.
	 */
	.x_res		= HRALVDS_THRA_WIDTH,
	/* According to Alpine, HFP + HBP must be between [108..248] */
	.hfp		= HRALVDS_THRA_TIMING_HFP,
	.hbp		= HRALVDS_THRA_TIMING_HBP,
	.hsw		= HRALVDS_THRA_TIMING_HSW,
        /* According to Alpine, VFP + VBP must be between [37..192] */
	.y_res		= HRALVDS_THRA_HEIGHT -
		(HRALVDS_THRA_DEAD_ZONE_TOP + HRALVDS_THRA_DEAD_ZONE_BOTTOM),
	.vfp		= HRALVDS_THRA_TIMING_VFP,
	.vbp		= HRALVDS_THRA_TIMING_VBP,
	.vsw		= HRALVDS_THRA_TIMING_VSW,

	.vsync_level	= OMAPDSS_SIG_ACTIVE_LOW,
	.hsync_level	= OMAPDSS_SIG_ACTIVE_LOW,
	.data_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
	.de_level	= OMAPDSS_SIG_ACTIVE_HIGH,
	.sync_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
};

#else

struct omap_video_timings hralvds_video_timings = {
	.pixelclock	= 82446624,			/* Hz */

	.x_res		= 1920,
	.hfp		= 46,
	.hbp		= 4,
	.hsw		= 6,

	.y_res		= 720,
	.vfp		= 10,
	.vbp		= 1,
	.vsw		= 1,

	.vsync_level	= OMAPDSS_SIG_ACTIVE_LOW,
	.hsync_level	= OMAPDSS_SIG_ACTIVE_LOW,
	.data_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
	.de_level	= OMAPDSS_SIG_ACTIVE_HIGH,
	.sync_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
};
#endif

#if 1
static unsigned registered_mscs[] = {
	MSC_RAW,
};
#endif

static unsigned registered_events[] = {
	EV_ABS,	/* abs position events		*/
	EV_SYN,	/* sync events			*/
	EV_KEY,	/* key events			*/
	EV_MSC,	/* interrupt status events	*/
};

static unsigned registered_keys[] = {
	/*
	 * KEYS DAY_NIGHT -> BACK MUST BE IN THE FOLLOWING ORDER:
	 *		Event		Key on display
	 *		KEY_F12		DAY/NIGHT
	 *		KEY_F11		POWER KNOB
	 *		HOMEPAGE	HOME
	 *		MENU		MENU
	 *		BACK		BACK
	 *
	 * This is how they are defined in the Key Command message
	 *		coming from the HRALVDS THRA display.
	 */
	KEY_F12,	/* Day/night key on touchscreen	*/
	KEY_F11,	/* Power button on volume knob	*/
	KEY_HOMEPAGE,	/* Home key on touchscreen	*/
	KEY_MENU,	/* Menu key on touchscreen	*/
	KEY_BACK,	/* Back key on touchscreen	*/

	/* order doesn't matter now. */
	KEY_VOLUMEUP,	/* Knob rotation CW			*/
	KEY_VOLUMEDOWN,	/* Knob rotation CCW			*/
	BTN_TOUCH,	/* touch events (finger down/finger up)	*/
};

/*
 * hralvds_key_event:
 *
 * handle a KEY command from the hralvds display.
 * inject events to the input subsystem.
 */
static void hralvds_key_event(struct panel_drv_data *ddata)
{
	/* buffer to send contains only the REPORT ID KEY command byte. */
	__u8 wbuf[] = { HRALVDS_REPORT_ID_KEY };
	/* buffer to receive the response in.  '+ 1' for the checksum. */
	__u8 rbuf[HRALVDS_REPORT_ID_KEY_SIZE + 1] = { 0 };
	int r;

	/* read the report ID and get the information from the hralvds. */
	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
				wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 1, 0);

	if (hralvds_process_input_event(ddata) && (2 == r)) {
		unsigned i;
#if defined HRALVDS_LOG_DEBUG
		/*
		 * possibly dump the received buffer
		 * 	(done before cksum).
		 */
		if (ddata->dbg_lvl & HRALVDS_DBG_LEVEL_DUMP) {
			char buf[(sizeof(__func__) + 1) +
				(sizeof(rbuf) * 3) + 2];
			hralvds_dump_msg(&ddata->ts_i2c_client->dev, __func__,
					rbuf, sizeof(rbuf),
					buf, sizeof(buf));
		}
#endif

		/*
		 * the KEY Command only has HRALVDS_KEY_MAX_OFFS
		 *	valid elements in it.
		 * note that registered_keys is in the same
		 * 	order as the key values in the KEY
		 * 	message.
		 */
		for (i = 0; i < HRALVDS_KEY_MAX_OFFS; i++)
			if ((0 == rbuf[i]) || (1 == rbuf[i]))
				input_report_key(ddata->input_dev,
					registered_keys[i],
					rbuf[i]);

		/*
		 * TODO: is it an error to input_sync() if
		 * none of the keys were reported?
		 * 	(i.e. they are all non-0 and non-1)
		 */
		input_sync(ddata->input_dev);
	}
}

/*
 * hralvds_knob_event:
 *
 * handle a KNOB command from the hralvds display.
 * inject KEY_VOLUMEUP and KEY_VOLUMEDOWN events to the input system.
 */
static void hralvds_knob_event(struct panel_drv_data *ddata)
{
	/* buffer to send contains only the REPORT ID KNOB command byte. */
	__u8 wbuf[] = { HRALVDS_REPORT_ID_KNOB };
	/* buffer to receive the response in.  '+ 1' for the checksum. */
	__u8 rbuf[HRALVDS_REPORT_ID_KNOB_SIZE + 1] = { 0 };
	int r;

	/* read the report ID and get the information from the hralvds. */
	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
				wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 1, 0);

	if (hralvds_process_input_event(ddata) && (2 == r)) {
		unsigned int ev = 0;
		unsigned int count = 0;
		int i;
		__s8 relpos;

#if defined HRALVDS_LOG_DEBUG
		/* possibly dump the received buffer (done before cksum). */
		if (ddata->dbg_lvl & HRALVDS_DBG_LEVEL_DUMP) {
			char buf[(sizeof(__func__)) + 1 +
				 (sizeof(rbuf) * 3) + 2];
			hralvds_dump_msg(&ddata->ts_i2c_client->dev, __func__,
						rbuf, sizeof(rbuf),
						buf, sizeof(buf));
		}
#endif

		/*
		 * as of 13 Nov 2014, rbuf[0] should be an 8-bit
		 *	signed value, in the range [-63, 64],
		 *	inclusive. it is possible that the HRALVDS
		 *	implementers are going to use a 7 bit signed
		 *	value, in which case we will have to do
		 *	some math here.
		 */
		relpos = rbuf[0];

			/*
			 * we ignore when relpos = 0.  we've already generated
			 *	the right number of key down / key up events
			 *	for either VOLUMEUP or VOLUMEDOWN.
			 * limit the number of events to the max specified in
			 *	the protocol: +63 for right turns (UP) and -64
			 *	for left turns (DOWN).
			 */
		if (relpos != 0) {
			if (relpos > 0) {
				ev = KEY_VOLUMEUP;
				if (relpos > 63)
					relpos = 63;
				count = relpos;
			} else {
				ev = KEY_VOLUMEDOWN;
				if (relpos < -64)
					relpos = -64;
				count = -relpos;
			}

			for (i = 0; i < count; i++) {
				/*
				 * Generate key down event for first
				 * key press, and repeat key down
				 * events for subsequent presses.
				 */
				input_event(ddata->input_dev, EV_KEY,
						ev, i == 0 ? 1 : 2);
			}
			input_report_key(ddata->input_dev, ev, 0);
			input_sync(ddata->input_dev);
		}
	}
}

/*
 * hralvds_touch_event:
 *
 *	handle a touch event from the hralvds display.
 */
static void hralvds_touch_event(struct panel_drv_data *ddata)
{
	/* buffer to send contains only the REPORT ID TOUCH command byte. */
	__u8 wbuf[] = { HRALVDS_REPORT_ID_TOUCH };
	/* buffer to receive the response in.  '+ 1' for the checksum. */
	__u8 rbuf[HRALVDS_REPORT_ID_TOUCH_SIZE + 1] = { 0 };
	int r;

	/* read the touch event and get the information from the display. */
	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
				wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 1, 0);

	if (hralvds_process_input_event(ddata) && (2 == r)) {
		struct hralvds_thra_data *htdata =
			(struct hralvds_thra_data *)
				ddata->hralvds_panel_data;

		unsigned i;
		unsigned offs;

#if defined HRALVDS_LOG_DEBUG
		/*
		 * possibly dump the received buffer (done before
		 *	checksum).
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

		/*
		 * go through each of the 5-byte slots.
		 *	format:
		 *		byte 0: pressed/released
		 *		byte 1: high byte of X coordinate
		 *		byte 2: low byte of X coordinate
		 *		byte 3: high byte of Y coordinate
		 *		byte 4: low byte of Y coordinate
		 */
		for (offs = 0, i = 0;
				i < HRALVDS_NUM_MT_SLOTS;
				i++, offs += 5) {


			__u8 state = rbuf[offs + 0];

			/*
			 * don't process invalid slots.
			 */
			if ((HRALVDS_KEY_PRESSED != state) &&
				(HRALVDS_KEY_NOT_PRESSED != state))
				continue;

			/* set the mt slot for this touch event */
			input_mt_slot(ddata->input_dev, i);

			/*
			 * if the button is pressed,
			 *	report a new press event.
			 */
			if (HRALVDS_KEY_PRESSED == state) {
				int x, y;

				/*
				 * the high byte of the x coordinate
				 *	is in rbuf[offs+1], and the
				 *	low byte is in rbuf[offs+2].
				 * the y coordinate is similarly
				 *	store in offsets 3 and 4.
				 */
				x = (rbuf[offs + 1] << 8) |
					rbuf[offs + 2];

				y = (rbuf[offs + 3] << 8) |
					rbuf[offs + 4];

				/*
				 * this is a finger down state,
				 *	so send the coordinates.
				 */
				input_report_abs(ddata->input_dev,
					ABS_MT_POSITION_X, x);
				input_report_abs(ddata->input_dev,
					ABS_MT_POSITION_Y, y);
			}

			/*
			 * state == 0 means the button isn't
			 *	currently pressed.
			 *
			 * if the current slot state indicates
			 *	the slot had been pressed,
			 *	report the release.
			 */
			else if (HRALVDS_KEY_PRESSED ==
					htdata->slot_state[i]) {
				input_mt_report_slot_state(
					ddata->input_dev,
					MT_TOOL_FINGER,
					HRALVDS_KEY_NOT_PRESSED);
			}

			/* report finger press */
			input_mt_report_slot_state(
				ddata->input_dev,
				MT_TOOL_FINGER,
				state);

			/*
			 * record the current slot state, after
			 *	it's been used to determine the
			 *	old state.
			 */
			htdata->slot_state[i] = state;
		}

		input_mt_report_pointer_emulation(ddata->input_dev,
							false);

		input_sync(ddata->input_dev);
	}
}

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
				wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 1, 0);

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
	 * *** TEMPORARILY *** pause 1msec between Report ID and
	 *	reading the event data.
	 */
	msleep(1);

	/*
	 * valid checksum, see what the
	 *	touchscreen wants to tell us.
	 */
	switch (rbuf[0]) {
	case HRALVDS_REPORT_ID_KEY:
		/* process the event before doing any debug */
		hralvds_key_event(ddata);

#if defined HRALVDS_LOG_DEBUG
		if (ddata->dbg_lvl & HRALVDS_DBG_LEVEL_EVENT)
			dev_info(&ddata->ts_i2c_client->dev,
				"%s active: KEY\n", __func__);

		if ((HRALVDS_REPORT_ID_KEY_SIZE != rbuf[1]) && ddata->dbg_lvl)
			dev_info(&ddata->ts_i2c_client->dev,
				"%s invalid length reported in "
				"KEY event: %d, but should be %d\n",
				__func__, rbuf[1],
				HRALVDS_REPORT_ID_KEY_SIZE);
#endif
		break;

	case HRALVDS_REPORT_ID_KNOB:
		/* process the event before doing any debug */
		hralvds_knob_event(ddata);

#if defined HRALVDS_LOG_DEBUG
		if (ddata->dbg_lvl & HRALVDS_DBG_LEVEL_EVENT)
			dev_info(&ddata->ts_i2c_client->dev,
				"%s active: KNOB\n", __func__);

		if ((HRALVDS_REPORT_ID_KNOB_SIZE != rbuf[1]) &&
								ddata->dbg_lvl)
			dev_info(&ddata->ts_i2c_client->dev,
				"%s invalid length reported in "
				"KNOB event: %d, but should be %d\n",
				__func__, rbuf[1],
				HRALVDS_REPORT_ID_KNOB_SIZE);
#endif

		break;

	case HRALVDS_REPORT_ID_TOUCH:
		/* process the event before doing any debug */
		hralvds_touch_event(ddata);

#if defined HRALVDS_LOG_DEBUG
		if (ddata->dbg_lvl & HRALVDS_DBG_LEVEL_EVENT)
			dev_info(&ddata->ts_i2c_client->dev,
				"%s active: TOUCH\n", __func__);

		if ((HRALVDS_REPORT_ID_TOUCH_SIZE != rbuf[1]) &&
								ddata->dbg_lvl)
			dev_info(&ddata->ts_i2c_client->dev,
				"%s invalid length reported in TOUCH "
				"event: %d, but should be %d\n",
				__func__, rbuf[1],
				HRALVDS_REPORT_ID_TOUCH_SIZE);
#endif

		break;

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
	unsigned i, n;
	int r;
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	ddata->input_dev = devm_input_allocate_device(dev);
	if (!ddata->input_dev)
		return -ENOMEM;

	ddata->input_dev->name = "HRALVDS THRA Touchscreen Panel";
	ddata->input_dev->id.bustype = BUS_I2C;
	ddata->input_dev->dev.parent = dev;
	input_set_drvdata(ddata->input_dev, ddata);

	n = sizeof(registered_events) / sizeof(registered_events[0]);
	for (i = 0; i < n; i++)
		__set_bit(registered_events[i], ddata->input_dev->evbit);

	n = sizeof(registered_keys) / sizeof(registered_keys[0]);
	for (i = 0; i < n; i++)
		__set_bit(registered_keys[i], ddata->input_dev->keybit);

	n = sizeof(registered_mscs) / sizeof(registered_mscs[i]);
	for (i = 0; i < n;  i++)
		__set_bit(registered_mscs[i], ddata->input_dev->mscbit);

	/*
	 * THRA dead zone:
	 * The display's left margin has been shifted and the top and bottom
	 * margins have been reduced per the timing parameters, but the
	 * touch screen will still return coordinates between 1280x720.
	 * Fortunately Android frameworks will rescale the touch coordinates
	 * to compensate for the screen's effective X and Y resolution.
	 */
	input_set_abs_params(ddata->input_dev, ABS_X, 0,
					HRALVDS_THRA_WIDTH, 0, 0);
	input_set_abs_params(ddata->input_dev, ABS_Y, 0,
					HRALVDS_THRA_HEIGHT, 0, 0);

	input_mt_init_slots(ddata->input_dev, HRALVDS_NUM_MT_SLOTS, 0);
	input_set_abs_params(ddata->input_dev, ABS_MT_POSITION_X, 0,
					HRALVDS_THRA_WIDTH, 0, 0);
	input_set_abs_params(ddata->input_dev, ABS_MT_POSITION_Y, 0,
					HRALVDS_THRA_HEIGHT, 0, 0);

	r = input_register_device(ddata->input_dev);

	return 1;
}

/*
 * called by hralvds_i2c_probe().
 */
int hralvds_panel_probe(struct panel_drv_data *ddata)
{
	int r = 0;

	ddata->hralvds_panel_data =
		devm_kzalloc(&ddata->ts_i2c_client->dev,
			sizeof (struct hralvds_thra_data), GFP_KERNEL);

	if (ddata->hralvds_panel_data == NULL)
		return -ENOMEM;

	/* allocate configure the input device. */
	r = hralvds_config_input_dev(&ddata->ts_i2c_client->dev);
	if (r >= 0)
		return 0;

	dev_info(&ddata->ts_i2c_client->dev,
			"Failed to allocate/configure input device.");
	kfree(ddata->hralvds_panel_data);
	return r;
}

int hralvds_panel_irq(struct panel_drv_data *ddata)
{
	int r;
	__u8 cksum;
	__u8 wbuf[1] = { HRALVDS_REPORT_ID_CMD };
	__u8 rbuf[2 + 1] = { 0 };

	/*
	 * read the report ID and get the information
	 *	from the hralvds.
	 */
	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
		wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 1, 0);

	if (2 == r)
		hralvds_dispatch_event(ddata, rbuf,
			sizeof(rbuf));

	return 0;
}

int hralvds_fwif_update_supported(void)
{
	return 0;
}

