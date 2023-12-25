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
*  are Copyright (c) 2014-2017 Honda R&D Americas, Inc.
*
*  Honda R&D Americas, Inc. hereby licenses those modifications
*  under the terms set forth in the file HONDA-NOTICE
*  located in the root of the directory /vendor/honda
*
*********************************************************/

#include <linux/module.h>
#include <linux/platform_device.h>
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
#include <linux/workqueue.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/vmalloc.h>
#include <linux/ratelimit.h>
#include <asm/uaccess.h>

#include <linux/i2c-debug.h>

#include <video/omapdss.h>
#include <video/omap-panel-data.h>
#include <video/of_display_timing.h>

#include "serializer.h"
#include "deserializer.h"
#include "hralvds-proto.h"
#include "panel-hralvds.h"

/*
 * there are two timeouts:
 * The INIT_TIMEOUT_MSEC is for when display_state == DISPLAY_INIT:
 *	we poll the LVDS LOCK status every 20msec per the spec.
 * the POLL_TIMEOUT_MSEC is for when display_state is in either of
 *	the DISPLAY_POLL_* states, and we check every 50 msec.
 */
#define HRALVDS_SUPERVISOR_WORKQ_INIT_TIMEOUT_MSEC	(20)
#define HRALVDS_SUPERVISOR_WORKQ_INIT_TIMEOUT \
	(msecs_to_jiffies(HRALVDS_SUPERVISOR_WORKQ_INIT_TIMEOUT_MSEC))

/*
 * these POLL_TIMEOUT constant are derived from a requirement that the
 *	supervisor message be sent from the FCB every 500 msec +/ 10%.
 * the poll could should cause the supervisor message to be sent out
 *	somewhere around 450-500msec.
 */
#define HRALVDS_SUPERVISOR_WORKQ_POLL_TIMEOUT_MSEC	(50)
#define HRALVDS_SUPERVISOR_WORKQ_POLL_TIMEOUT \
	(msecs_to_jiffies(HRALVDS_SUPERVISOR_WORKQ_POLL_TIMEOUT_MSEC))
#define HRALVDS_SUPERVISOR_POLL_COUNT	\
	((450 / HRALVDS_SUPERVISOR_WORKQ_POLL_TIMEOUT_MSEC) - 1)

/*verify HDCP every 2 seconds*/
#define HRALVDS_HDCP_WORKQ_TIMEOUT (2*HZ)

/*
 * The hardware will retry the HDCP AUTH if there was a failure.
 * Wait a second for a successful AUTH before resetting the display.
 */
#define HRALVDS_HDCP_AUTH_TIMEOUT		(HZ)

/*
 * The cycle time in msec for toggling the display controller OFF then ON.
 * Used for resetting the display for retries.
 */
#define HRALVDS_DISPLAY_RESET_CYCLE_TIME_MSEC	(100)	/* Per HRAO LVDS spec */

/*
 * Number of resets to retry for initializing the serializer, before
 * giving up.
 */
#define HRALVDS_SERIALIZER_RESET_RETRIES	(20)

/*
 * when waiting for the supervisor state to go from Down to OK, check
 *		this often.
 */
#define HRALVDS_SUPERVISOR_WORKQ_SHORT_TIMEOUT (msecs_to_jiffies(20))

/*
 * timeout between polling for IRQ status, and thus events from the
 *		HRALVDS display.
 */
#define HRALVDS_IRQ_WORKQ_TIMEOUT (msecs_to_jiffies(20))

/*
 * Log ratelimiting for hralvds_setup_link().  Allow at most 10 log messages
 * every minute, to prevent a kernel log spam due to frequent repeated retries
 * if the link fails to establish.  This can happen if the PCLK gets turned
 * off, but we are required to keep retrying link setup per protocol specs.
 */
#define HRALVDS_SETUP_LINK_RATELIMIT_PERIOD	(HZ * 60)
#define HRALVDS_SETUP_LINK_RATELIMIT_THRESHOLD	(10)

enum {
	RADIO_FREQ_BAND_FIRST = 1,
	RADIO_FREQ_BAND_AM    = RADIO_FREQ_BAND_FIRST,
	RADIO_FREQ_BAND_FM    = 2,
	RADIO_FREQ_BAND_MAX,
};

enum {
	TOUCH_SENSITIVITY_LOW  = 0,
	TOUCH_SENSITIVITY_MID  = 1,
	TOUCH_SENSITIVITY_HIGH = 2,
};

/*
 * Each hralvds-panel_<model>.c file must define a
 *	struct omap_video_timings hralvds_video_timings.
 * The other functions referenced here are for other functionality.
 */
extern struct omap_video_timings hralvds_video_timings;
extern int hralvds_panel_probe(struct panel_drv_data *ddata);
extern int hralvds_panel_irq(struct panel_drv_data *ddata);

extern int hralvds_fwif_update_supported(void);

static int xmit_cmd_if_changed = 1;
module_param(xmit_cmd_if_changed, int, 0644);
MODULE_PARM_DESC(xmit_cmd_if_changed, "Send command (e.g. video, power) to "
	"the panel only when there is a change.");

static int power_cmd_retries = 3;
module_param(power_cmd_retries, int, 0644);
MODULE_PARM_DESC(power_cmd_retries, "The number of retries to send the "
	"power command before resetting the display controller if "
	"the panel does not acknowledge."
	"0 = no retry, -1 = infinit");

static int reset_retries = 3;
module_param(reset_retries, int, 0644);
MODULE_PARM_DESC(reset_retries, "The number of display controller resets "
	"before giving up.");

extern uint32_t hralvds_i2c_msleep;
uint32_t hralvds_i2c_retries = 0;
uint32_t hralvds_i2c_gave_up = 0;

static void hralvds_power_state_set_timeout(struct panel_drv_data *ddata);
void hralvds_on_finish_gradual_change(struct panel_drv_data *ddata);
static void hralvds_send_video_ts_and_power(struct panel_drv_data *ddata);

static void hralvds_set_power_state(struct panel_drv_data *ddata,
	const char *caller, display_power_state_t new_power_state)
{
	static const char *power_states[] = {
		"POWER_STATE_OFF",
		"POWER_STATE_POWERING_OFF",
		"POWER_STATE_POWERING_OFF_CHECK_ON",
		"POWER_STATE_POWERING_ON_1",
		"POWER_STATE_POWERING_ON_1A",
		"POWER_STATE_POWERING_ON_2",
		"POWER_STATE_ON",
	};

	dev_info(&ddata->ts_i2c_client->dev,
		"%s sets power_state: %s -> %s",
		caller, power_states[ddata->power_state],
		power_states[new_power_state]);
	ddata->power_state = new_power_state;
}

/*
 * hralvds_set_handshake:
 *
 * check for the bit from the HRALVDS display that indicates it has
 *		completed its initialization.
 *
 * NOTE caller must have set the serializer alias to talk to the deserializer.
 */
static int hralvds_set_handshake(struct panel_drv_data *ddata)
{
	int i;
	int r;

	for (i = 0; i < HRALVDS_INIT_STATUS_REG_MAX_POLL; i++) {
		r = i2c_read_le8(ddata->deser_i2c_client,
					HRALVDS_INIT_STATUS_REG);

		if (r >= 0) {
			if (r & HRALVDS_INIT_STATUS_REG_INIT_COMPLETE_BIT)
				/* successful read */
				break;
			else
				dev_err(&ddata->ts_i2c_client->dev,
					"Poll init try %d\n", i);
		}
		msleep(500);
	}

	if (i >= HRALVDS_INIT_STATUS_REG_MAX_POLL) {
		dev_err(&ddata->ts_i2c_client->dev,
			"Deserializer read of init reg failed.\n");
		goto err0;
	} else if (0 == (r & HRALVDS_INIT_STATUS_REG_INIT_COMPLETE_BIT)) {
		dev_err(&ddata->ts_i2c_client->dev,
			"Deserializer INIT COMPLETE not set, %d tries\n", i);
		goto err0;
	} else {
		/* establishment complete, so set bit to dser and start
			supervisory frames */
#if defined HRALVDS_STARTUP_DEBUG
		dev_info(&ddata->ts_i2c_client->dev,
			"Deserializer establishment complete.\n");
#endif
		r = i2c_write_le8(ddata->deser_i2c_client,
			HRALVDS_INIT_STATUS_REG,
			r | HRALVDS_INIT_STATUS_REG_EST_COMPLETE_BIT);

		if (r < 0) { /* failed read */
			dev_err(&ddata->ts_i2c_client->dev,
				"Deserializer establishment complete, "
				"write failed %d\n", r);
			goto err0;
		} else {
			r = i2c_read_le8(ddata->deser_i2c_client,
				HRALVDS_INIT_STATUS_REG);
#if defined HRALVDS_STARTUP_DEBUG
			dev_info(&ddata->ts_i2c_client->dev, "%s data:0x%08x\n",
				__func__, r);
#endif
		}
	}

	return 0;

err0:
	return -EINVAL;
}

/*
 * hralvds_setup_deser_alias:
 *
 * set the serializer's slave device id and slave device alias id to that
 *	of the deserializer.
 */
static int hralvds_setup_deser_alias(struct panel_drv_data *ddata)
{
	int r;
	unsigned phys_addr;
	unsigned alias_addr;

	phys_addr = ddata->deser_phys << 1;
	alias_addr = ddata->deser_alias << 1;

	r = i2c_write_le8(ddata->ser_i2c_client,
			SER925_I2C_SLAVE_ID, phys_addr);

	if (!r)
		r = i2c_write_le8(ddata->ser_i2c_client,
			SER925_I2C_SLAVE_ALIAS, alias_addr);

	return r;
}

/*
 * hralvds_setup_ts_alias:
 *
 * set the serializer's slave device id and slave device alias id to that
 *	of the HRALVDS touchscreen.
 */
static int hralvds_setup_ts_alias(struct panel_drv_data *ddata)
{
	int r;
	unsigned addr;

	addr = ddata->ts_i2c_client->addr << 1;

	r = i2c_write_le8(ddata->ser_i2c_client,
			SER925_I2C_SLAVE_ID, addr);

	if (!r)
		r = i2c_write_le8(ddata->ser_i2c_client,
			SER925_I2C_SLAVE_ALIAS, addr);

	return r;
}

/*
 * hralvds_reset_state:
 *
 * reset various state variables to reflect that a display is not connected.
 */
static void hralvds_reset_state(struct panel_drv_data *ddata)
{
	ddata->display_detected = 0;
	ddata->display_state = DISPLAY_INIT;
	ddata->poll_counter = HRALVDS_SUPERVISOR_POLL_COUNT;
	ddata->supervisor_failure_counter = 0;
	ddata->timer_worker_delay = HRALVDS_SUPERVISOR_WORKQ_INIT_TIMEOUT;
	ddata->supervisor_ok = 0;
	ddata->fwupd.update_reason = DISPLAY_UPDATE_REASON_NULL;
	ddata->fwupd.update_state = DISPLAY_UPDATE_IDLE;
}

/*
 * hralvds_do_supervisor:
 *
 * workqueue task to send a supervisor frame to the display and get
 *		a response.
 * return zero to get rescheduled, or return < 0 to not get rescheduled.
 */
static int hralvds_do_supervisor(struct panel_drv_data *ddata)
{
	/* buffer to send contains only the supervisor command byte */
	__u8 wbuf[] = { HRALVDS_SUPERVISOR_CMD };
	/* buffer to receive the response in.  '+ 1' for the checksum. */
	__u8 rbuf[HRALVDS_SUPERVISOR_RSP_SIZE + 1] = { 0 };
	/* return code */
	int r;

	/*
	 * issue the supervisor command to the hralvds and get the response.
	 * make sure there's a valid checksum, but don't repeat the
	 *	transaction if it fails.
	 */
	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client, wbuf, sizeof(wbuf),
					rbuf, sizeof(rbuf), 1, 0);

	/*
	 * r == 2 is success.
	 * any other value == error.
	 */
	if (2 == r) {

#if defined HRALVDS_LOG_DEBUG
		if ((0 == ddata->supervisor_ok) && ddata->dbg_lvl)
			dev_info(&ddata->ts_i2c_client->dev,
				"%s supervisor OK\n", __func__);
#endif

		if (HRALVDS_SUPERVISOR_RSP == rbuf[0]) {

			return 0;

		} else {

#if defined HRALVDS_LOG_DEBUG
			if (ddata->dbg_lvl)
				dev_err(&ddata->ts_i2c_client->dev,
					"%s bad response "
					"{ %02hhx %02hhx }\n",
					__func__, rbuf[0], rbuf[1]);
#endif

			return -1;
		}
	} else {
		dev_err(&ddata->ts_i2c_client->dev,
			"%s hralvds_i2c_cmdrsp returned %d", __func__, r);
		return r;
	}
}

/*
 * hralvds_power_off
 *
 * Power off the display controller
 */
static void hralvds_power_off(struct panel_drv_data *ddata)
{
	struct hralvds_power_cmd_struct *p = &ddata->cmd_power;
	struct device *dev = &ddata->ts_i2c_client->dev;
	u8 ctrl_mask;

	/* Nothing to do if the panel is already off */
	if (!gpio_get_value(ddata->dispc_gpio)) {
		hralvds_set_power_state(ddata, __func__, POWER_STATE_OFF);
		ddata->power_cmd_processed = 0;
		return;
	}

	/*
	 * If the LCD backlight, vol knob ring, and all buttons are off,
	 * we can just power down the panel.  We don't need to wait for
	 * the gradual brightness to be applied.
	 */
	mutex_lock(&ddata->cmd_power_lock);
	ctrl_mask = (1 << 2)|(1 << 1);
	if (((p->power.ctrl & ctrl_mask) == ctrl_mask) &&
		(p->power.led_face_sw_state == 0xff)) {
		gpio_set_value(ddata->dispc_gpio, 0);
		ddata->power_state = POWER_STATE_OFF;
		ddata->power_cmd_processed = 0;
		mutex_unlock(&ddata->cmd_power_lock);
		return;
	}
	mutex_unlock(&ddata->cmd_power_lock);

	cancel_delayed_work_sync(&ddata->hralvds_power_state_work);
	hralvds_set_power_state(ddata, __func__, POWER_STATE_POWERING_OFF);

	dev_info(dev, "Sending power command for screen off\n");
	p = &ddata->cmd_power_off;
	hralvds_i2c_cmd(ddata->ts_i2c_client, (__u8 *)p, sizeof(*p));

	hralvds_power_state_set_timeout(ddata);
}

/*
 * hralvds_power_on
 *
 * Power on the display controller
 */
static void hralvds_power_on(struct panel_drv_data *ddata)
{
	struct device *dev = &ddata->ts_i2c_client->dev;

	/* Nothing to do if the panel is already on */
	if (gpio_get_value(ddata->dispc_gpio)) {
		if (ddata->power_state == POWER_STATE_OFF) {
			dev_err(dev, "Inconsistent power state: "
				"DISPC is on, but power_state is OFF!\n");
			ddata->power_state = POWER_STATE_ON;
			ddata->power_cmd_processed = 1;
		}
		return;
	}

	/* only change state if power is currently off */
	if (POWER_STATE_OFF == ddata->power_state) {
		/*
		 * Turn the panel on.  The video and power commands will be sent
		 * when deserializer handshake successfully completes.
		 */
		cancel_delayed_work_sync(&ddata->hralvds_power_state_work);
		ddata->gradual_change_retries = 0;
		ddata->reset_retries = 0;
		hralvds_set_power_state(ddata, __func__,
			POWER_STATE_POWERING_ON_1);
		gpio_set_value(ddata->dispc_gpio, 1);
		dev_info(dev, "Powering the panel on.");
	} else
		dev_info(dev, "hralvds_power_on: power on in progress.");
}

/*
 * show_hralvds_display_controller:
 *
 * Show the state of the display controller reset line (DISPC)
 */
static ssize_t show_hralvds_display_controller(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	ssize_t len;

	len = scnprintf(buf, PAGE_SIZE, "%s\n",
		gpio_get_value(ddata->dispc_gpio) ? "on" : "off");

	return len;
}

/*
 * set_hralvds_display_controller:
 *
 * Set the state of the display controller reset line (DISPC)
 */
static ssize_t set_hralvds_display_controller(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	if (strncmp(buf, "on", 2) == 0) {
		ddata->requested_dispc_state = 1;
		hralvds_power_on(ddata);
		return count;
	} else if (strncmp(buf, "off", 3) == 0) {
		ddata->requested_dispc_state = 0;
		hralvds_power_off(ddata);
		return count;
	}

	return -1;
}

/*
 * show_hralvds_link_status:
 *
 * Show the supervisor link status.
 */
static ssize_t show_hralvds_link_status(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	ssize_t len = 0;

	len = scnprintf(buf, PAGE_SIZE,
		"Display detected = %s, Supervisor status = %s\n",
		ddata->display_detected ? "Yes" : "No",
		ddata->supervisor_ok ? "Up" : "Down");

	return len;
}

/*
 * show_hralvds_version:
 *
 * show the contents of the version from the HRALVDS display.
 * contains software, hardware and firmware version information.
 */
static ssize_t show_hralvds_version(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	/* buffer to send contains only the version command byte. */
	__u8 wbuf[] = { HRALVDS_VERSION_CMD };
	/* buffer to receive the response in.  '+ 1' for the cksum. */
	__u8 rbuf[HRALVDS_VERSION_RSP_SIZE + 1] = { 0 };
	int r;
	ssize_t len;

	/*
	 * issue the version read command & get the response.
	 * do checksum, but don't perform any retries.
	 */
	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
				wbuf, sizeof(wbuf),
				rbuf, sizeof(rbuf),
				1, 0);

	if (2 == r)  {
		unsigned int i;

		len = 0;

		/* write out the reply to the output buffer */
		for (i = 0; i < sizeof(rbuf) && len < PAGE_SIZE - 3; i++)
			len += scnprintf(buf+len,
				PAGE_SIZE-len-3,
				"%02hhx ", rbuf[i]);

		if (len < PAGE_SIZE - 2)
			len += scnprintf(buf+len,
				PAGE_SIZE-len-2, "\n");
	} else {
		len = scnprintf(buf, PAGE_SIZE,
			"%s hralvds_i2c_cmdrsp returned %d\n", __func__, r);
	}

	return len;
}

/*
 * show_hralvds_version_text:
 *
 * show the contents of the version from the HRALVDS display.
 * contains software, hardware and firmware version information.
 */
static ssize_t show_hralvds_version_text(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	/* buffer to send contains only the version command byte. */
	__u8 wbuf[] = { HRALVDS_VERSION_CMD };
	/* buffer to receive the response in.  '+ 1' for the cksum. */
	__u8 rbuf[HRALVDS_VERSION_RSP_SIZE + 1] = { 0 };
	int r;
	ssize_t len;

	/*
	 * issue the version read command & get the response.
	 * do checksum, but don't perform any retries.
	 */
	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
				wbuf, sizeof(wbuf),
				rbuf, sizeof(rbuf),
				1, 0);

	if (2 == r)  {
		len = 0;

		/*
		 * write out the reply to the output buffer.
		 *	subtract 1 from sizeof(rbuf) to avoid printing the
		 *       transaction checksum.
		 */
		len += scnprintf(buf+len, PAGE_SIZE-len-3,
			"HW Version %02hhx.%02hhx, ",
			rbuf[HRALVDS_VERSION_HWVER_MAJ_IDX],
			rbuf[HRALVDS_VERSION_HWVER_SUF_IDX]);
		len += scnprintf(buf+len, PAGE_SIZE-len-3,
			"SW Version %02hhx.%02hhx, ",
			rbuf[HRALVDS_VERSION_SWVER_MAJ_IDX],
			rbuf[HRALVDS_VERSION_SWVER_SUF_IDX]);
		len += scnprintf(buf+len, PAGE_SIZE-len-3,
			"Touch IC Version %02hhx.%02hhx, ",
			rbuf[HRALVDS_VERSION_TICVER_MAJ_IDX],
			rbuf[HRALVDS_VERSION_TICVER_SUF_IDX]);
		len += scnprintf(buf+len, PAGE_SIZE-len-3,
			"Option %02hhx\n",
			rbuf[HRALVDS_VERSION_OPTION_IDX]);

		if (len < PAGE_SIZE - 2)
			len += scnprintf(buf+len,
				PAGE_SIZE-len-2, "\n");
	} else {
		len = scnprintf(buf, PAGE_SIZE,
			"%s hralvds_i2c_cmdrsp returned %d\n", __func__, r);
	}

	return len;
}

/*
 * show_hralvds_diag_status:
 *
 * write the diag status command to the hralvds display and report
 *	the results.
 */
static ssize_t show_hralvds_diag_status(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	/*
	 * buffer to send contains only the diagnosis status
	 * command byte.
	 */
	__u8 wbuf[] = { HRALVDS_DIAG_STATUS_CMD };
	/* buffer to receive the response in.  '+ 1' for the cksum. */
	__u8 rbuf[HRALVDS_DIAG_STATUS_RSP_SIZE + 1] = { 0 };
	int len;
	int r;

	/*
	 * issue the diagnosis status read command to the hralvds and
	 * get the response.
	 */
	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
				wbuf, sizeof(wbuf),
				rbuf, sizeof(rbuf),
				1, 0);

	if (2 == r) {
		unsigned i;
		len = 0;

		/* write out the reply to the output buffer */
		for (i = 0; i < sizeof(rbuf) && len < PAGE_SIZE - 3; i++)
			len += scnprintf(buf+len, PAGE_SIZE-len-3,
				"%02hhx ", rbuf[i]);

		if (len < PAGE_SIZE - 2)
			len += scnprintf(buf+len,
				PAGE_SIZE-len-2, "\n");
	} else {
		len = scnprintf(buf, PAGE_SIZE,
			"%s hralvds_i2c_cmdrsp returned %d\n", __func__, r);
	}

	return len;
}

/*
 * set_hralvds_touch_cal:
 *
 * send the touch calibration command and data to the hralvds display.
 */
static ssize_t set_hralvds_touch_cal(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	/* write buffer to send w/ the diag touchh cal */
	__u8 wbuf[1 + HRALVDS_DIAG_TOUCH_CAL_CMD_SIZE + 1] = {
		HRALVDS_DIAG_TOUCH_CAL_CMD };
	__u32 din[HRALVDS_DIAG_TOUCH_CAL_CMD_SIZE / sizeof(__u32)];
	__u32 dout[HRALVDS_DIAG_TOUCH_CAL_CMD_SIZE / sizeof(__u32)];
	int rc;
	int i;

	rc = sscanf(buf, "%x %x %x %x %x %x %x %x %x %x",
			&din[0], &din[1], &din[2], &din[3], &din[4],
			&din[5], &din[6], &din[7], &din[8], &din[9]);
	if (rc < 10)
		return -1;

	for (i = 0; i < sizeof(dout) / sizeof(dout[0]); i++)
		dout[i] = cpu_to_be32(din[i]);
	memcpy(wbuf + 1, dout, HRALVDS_DIAG_TOUCH_CAL_CMD_SIZE);
	wbuf[1 + HRALVDS_DIAG_TOUCH_CAL_CMD_SIZE] =
		~hralvds_cksum(wbuf, sizeof(wbuf) - 1, NULL, 0) + 1;

#if defined HRALVDS_LOG_DEBUG
	if (ddata->dbg_lvl) {
		char dbuf[256];
		hralvds_dump_msg(dev, __func__,
					wbuf, sizeof(wbuf),
					dbuf, sizeof(dbuf));
	}
#endif

	/* send the message to the hralvds display. */
	if (hralvds_i2c_cmd(ddata->ts_i2c_client,
				wbuf, sizeof(wbuf)) > 0)
		return count;
	else
		return -1;
}

/*
 * set_hralvds_touch_sensitivity:
 *
 * send the touch sensitivity command and data to the hralvds display.
 */
static ssize_t set_hralvds_touch_sensitivity(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct hralvds_touch_sensitivity_cmd_struct ts;
	ssize_t ret;
	__u8 din;
	int i;

	i = sscanf(buf, "%hhx", &din);

	if (i < 1)
		return -1;

	if (din > TOUCH_SENSITIVITY_HIGH)
		return -1;

	ts.cmd_id = HRALVDS_DIAG_TOUCH_SENS_CMD;
	ts.touch_sensitivity.level = din;
	ts.checksum = ~hralvds_cksum((const __u8 *)&ts,
					sizeof(ts) - 1, NULL, 0) + 1;

	mutex_lock(&ddata->cmd_touch_sensitivity_lock);

	if (xmit_cmd_if_changed) {
		if (memcmp(&ddata->cmd_touch_sensitivity,
			&ts, sizeof(ts)) == 0) {
			dev_dbg(dev,
				"No change in touch sensitivity command."
				" Not sending command to panel.");

			ret = count;
			goto out_unlock;
		}
	}

	memcpy(&ddata->cmd_touch_sensitivity, &ts, sizeof(ts));

	/* send the message to the hralvds display. */
	if (hralvds_i2c_cmd(ddata->ts_i2c_client,
		(__u8 *)&ddata->cmd_touch_sensitivity,
		sizeof(struct hralvds_touch_sensitivity_cmd_struct)) > 0)
		ret = count;
	else
		ret = -1;

out_unlock:
	mutex_unlock(&ddata->cmd_touch_sensitivity_lock);
	return ret;
}

/*
 * show_hralvds_touch_sensitivity:
 *
 * sysfs interface to show the shadowed touch sensitivity command
 */
static ssize_t show_hralvds_touch_sensitivity(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct hralvds_touch_sensitivity_cmd_struct *ts =
		&ddata->cmd_touch_sensitivity;
	ssize_t len;

	mutex_lock(&ddata->cmd_touch_sensitivity_lock);
	len = scnprintf(buf, PAGE_SIZE, "%i\n", ts->touch_sensitivity.level);
	mutex_unlock(&ddata->cmd_touch_sensitivity_lock);

	return len;
}

/*
 * set_hralvds_power:
 *
 * send the power command and data to the hralvds display.
 */
static ssize_t set_hralvds_power(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct hralvds_power_cmd_struct *p = &ddata->cmd_power;

	/* write buffer to send w/ the power command */
	__u32 din[HRALVDS_POWER_CMD_SIZE / sizeof(__u32)];
	__u32 dout[HRALVDS_POWER_CMD_SIZE / sizeof(__u32)];
	int i;
	int check_if_changed;
	ssize_t ret;

	i = sscanf(buf, "%x %x", &din[0], &din[1]);

	if (i < 2)
		return -1;

	for (i = 0; i < sizeof(dout) / sizeof(dout[0]); i++)
		dout[i] = cpu_to_be32(din[i]);


	/*
	 * There can be a window during boot when user space writes
	 * the power command before the first deserializer
	 * establishment.  Disable fast init to unconditionally apply
	 * the power command from user space.
	 */
	atomic_set(&ddata->fast_init, 0);

	mutex_lock(&ddata->cmd_power_lock);

	if ((ddata->power_state == POWER_STATE_ON) &&
		!ddata->power_cmd_processed) {
		/*
		 * Display is on, but power command had not been applied yet
		 * because of "fast init" mode.  So force power command to
		 * get sent even if there were no change.
		 */
		check_if_changed = 0;
	} else
		check_if_changed = xmit_cmd_if_changed;

	if (check_if_changed) {
		if (memcmp(p->data, dout, sizeof(p->data)) == 0) {
			dev_dbg(dev, "No change in power command. "
				"Not sending command to panel.");

			ret = count;
			goto out_unlock;
		}
	}

	p->cmd_id = HRALVDS_POWER_CMD;
	memcpy(p->data, dout, sizeof(p->data));
	p->checksum = ~hralvds_cksum((__u8 *)p, sizeof(*p) - 1,
							NULL, 0) + 1;

	/*
	 * Don't issue the power command if the power on sequencing is
	 * in progress -- hralvds_power_state_worker() will take care of that
	 * via hralvds_send_video_ts_and_power().
	 *
	 * This avoids racing with hralvds_power_state_worker() and causing
	 * out of order sequencing (e.g. power, DeSer reset) or duplicate
	 * power commands issued (e.g. reset, power, video+ts+power).
	 */
	if (ddata->power_state < POWER_STATE_POWERING_ON_2)
		goto out_unlock;

#if defined HRALVDS_LOG_DEBUG
	if (ddata->dbg_lvl) {
		char dbuf[128];
		hralvds_dump_msg(dev, __func__,
					(__u8 *)p, sizeof(*p),
					dbuf, sizeof(dbuf));
	}
#endif

	/* send the message to the hralvds display. */
	if (hralvds_i2c_cmd(ddata->ts_i2c_client, (__u8 *)p,
						sizeof(*p)) > 0) {
		ddata->gradual_change_retries = 0;
		ddata->reset_retries = 0;
		hralvds_power_state_set_timeout(ddata);
		ret = count;
	} else
		ret = -1;

out_unlock:
	mutex_unlock(&ddata->cmd_power_lock);
	return ret;
}

/*
 * set_hralvds_last_power:
 *
 * sysfs interface to re-send the last power command.
 */
static ssize_t set_hralvds_last_power(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct hralvds_power_cmd_struct *p = NULL;
	ssize_t ret;

	mutex_lock(&ddata->cmd_power_lock);

	if (POWER_STATE_OFF != ddata->power_state) {
		p = &ddata->cmd_power;
	} else {
		p = &ddata->cmd_power_off;
		dev_err(&ddata->ts_i2c_client->dev,
			"%s: send cmd_power_off", __func__);
	}

#if defined HRALVDS_LOG_DEBUG
	if (ddata->dbg_lvl) {
		char dbuf[128];
		hralvds_dump_msg(dev, __func__,
					(__u8 *)p, sizeof(*p),
					dbuf, sizeof(dbuf));
	}
#endif

	/* send the message to the hralvds display. */
	if (hralvds_i2c_cmd(ddata->ts_i2c_client, (__u8 *)p, sizeof(*p)) > 0) {
		ddata->gradual_change_retries = 0;
		ddata->reset_retries = 0;
		hralvds_power_state_set_timeout(ddata);
		ret = count;
	} else
		ret = -1;

	mutex_unlock(&ddata->cmd_power_lock);
	return ret;
}

/*
 * show_hralvds_power:
 *
 * sysfs interface to show the shadowed power command
 */
static ssize_t show_hralvds_power(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct hralvds_power_cmd_struct *p = &ddata->cmd_power;
	ssize_t len;
	u32 d0, d1;

	mutex_lock(&ddata->cmd_power_lock);

	/* This is a lazy and stupid API, but leave it be for consistency
	 * with the "set" syntax and to avoid breaking userspace.
	 */
	d0 = *((u32 *)(&(p->data[0])));
	d1 = *((u32 *)(&(p->data[sizeof(u32)])));
	len = scnprintf(buf, PAGE_SIZE, "0x%08x 0x%08x\n",
			be32_to_cpu(d0), be32_to_cpu(d1));

	mutex_unlock(&ddata->cmd_power_lock);
	return len;
}

/*
 * set_hralvds_video:
 *
 * send the video command and data to the hralvds display.
 */
static ssize_t set_hralvds_video(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct hralvds_video_cmd_struct *v = &ddata->cmd_video;

	/* write buffer to send with the video command */
	__u32 din[1];
	__u32 dout[1];
	int i;
	ssize_t ret;

	/* copy the video command to the write buffer. */
	i = sscanf(buf, "%x", &din[0]);

	if (i < 1)
		return -1;

	dout[0] = cpu_to_be32(din[0]);

	mutex_lock(&ddata->cmd_video_lock);

	if (xmit_cmd_if_changed) {
		if (memcmp(v->data, dout, sizeof(v->data)) == 0) {
			dev_dbg(dev, "No change in video command. "
				"Not sending command to panel.");

			ret = count;
			goto out_unlock;
		}
	}

	v->cmd_id = HRALVDS_VIDEO_CMD;
	memcpy(&v->data, dout, sizeof(v->data));
	v->checksum = ~hralvds_cksum((__u8 *)v, sizeof(*v) - 1,
							NULL, 0) + 1;

#if defined HRALVDS_LOG_DEBUG
	if (ddata->dbg_lvl) {
		char dbuf[128];
		hralvds_dump_msg(dev, __func__,
					(__u8 *)v, sizeof(*v),
					dbuf, sizeof(dbuf));
	}
#endif

	/* send the message to the hralvds display. */
	if (hralvds_i2c_cmd(ddata->ts_i2c_client, (__u8 *)v, sizeof(*v)) > 0)
		ret = count;
	else
		ret = -1;

out_unlock:
	mutex_unlock(&ddata->cmd_video_lock);
	return ret;
}

/*
 * set_hralvds_last_video:
 *
 * sysfs interface to re-send the last video command.
 */
static ssize_t set_hralvds_last_video(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct hralvds_video_cmd_struct *v = &ddata->cmd_video;
	ssize_t ret;

#if defined HRALVDS_LOG_DEBUG
	if (ddata->dbg_lvl) {
		char dbuf[128];
		hralvds_dump_msg(dev, __func__,
					(__u8 *)v, sizeof(*v),
					dbuf, sizeof(dbuf));
	}
#endif

	mutex_lock(&ddata->cmd_video_lock);

	/* send the message to the hralvds display. */
	if (hralvds_i2c_cmd(ddata->ts_i2c_client, (__u8 *)v, sizeof(*v)) > 0)
		ret = count;
	else
		ret = -1;

	mutex_unlock(&ddata->cmd_video_lock);
	return ret;
}

/*
 * show_hralvds_video:
 *
 * sysfs interface to show the shadowed video command
 */
static ssize_t show_hralvds_video(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct hralvds_video_cmd_struct *v = &ddata->cmd_video;
	ssize_t len;
	u32 d0;

	mutex_lock(&ddata->cmd_video_lock);

	/*
	 * This is a lazy and stupid API, but leave it be for
	 *	consistency with the "set" syntax and to avoid breaking
	 *	userspace.
	 */
	d0 = *((u32 *)(&v->data[0]));
	len = scnprintf(buf, PAGE_SIZE, "0x%08x\n", be32_to_cpu(d0));

	mutex_unlock(&ddata->cmd_video_lock);
	return len;
}

/*
 * hralvds_radio_freq_end_of_input:
 *
 * handle end of line input for the hralvds_radio_freq command
 */
static int hralvds_radio_freq_end_of_input(const char *s)
{
	if ('\0' == *s)
		return 0;

	/* trailing space is OK */
	s = skip_spaces(s);

	/* but junk afterwards is not */
	if (*s != '\0')
		return -1;

	return 0;
}

/*
 * hralvds_fill_radio_freq_cmd:
 *
 * parse and fill in the HRA LVDS Radio Frequency command structure
 */
static int hralvds_fill_radio_freq_cmd(
	struct hralvds_radio_freq_cmd_struct *r, const char *cmd_buf)
{
	int band, freq;
	int f, f1, f10, f100;

	const char *s = cmd_buf;
	memset(r, 0x00, sizeof(*r));

	/* Parse the radio band (AM/FM) enumeration */
	s = skip_spaces(s);

	if (!isdigit(*s))
		return -1;

	if (sscanf(s, "%i", &band) != 1)
		return -1;

	if ((band < RADIO_FREQ_BAND_FIRST) ||
		(band >= RADIO_FREQ_BAND_MAX))
		return -1;

	r->radio.band = (band & 0x0f) << 4;

	while (isdigit(*s))
		s++;

	if (!isspace(*s))
		return -1;

	s++;
	s = skip_spaces(s);

	/* Look for the whole number part of the frequency */
	if (!isdigit(*s))
		return -1;

	if (sscanf(s, "%i", &freq) != 1)
		return -1;

	if (freq >= 10000)
		return -1;

	f = freq;
	f1 = f % 10;   /* ones digit */
	f /= 10;
	f10 = f % 10;  /* tens digit */
	f /= 10;
	f100 = f % 10; /* hundreds digit */
	f /= 10;

	r->radio.freq_tens_ones = (f10 << 4) | f1;
	r->radio.freq_thousands_hundreds = ((f & 0x0f) << 4) | f100;

	while (isdigit(*s))
		s++;

	/* Parse the fractional part of the frequency */
	if (*s != '.')
		return hralvds_radio_freq_end_of_input(s);

	s++;
	if (!isdigit(*s))
		return -1;

	r->radio.freq_tenths_hundredths = (*s - '0') << 4;

	s++;
	if (!isdigit(*s))
		return hralvds_radio_freq_end_of_input(s);

	r->radio.freq_tenths_hundredths |= *s - '0';

	s++;
	/* ignore fractional part after the hundredth place */
	while (isdigit(*s))
		s++;

	return hralvds_radio_freq_end_of_input(s);
}

/*
 * set_hralvds_radio_freq:
 *
 * sysfs interface to set the hralvds radio frequency
 */
static ssize_t set_hralvds_radio_freq(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct hralvds_radio_freq_cmd_struct r;
	ssize_t ret;

	if (hralvds_fill_radio_freq_cmd(&r, buf) < 0)
		return -1;

	r.cmd_id = HRALVDS_RADIO_FREQ_CMD;
	r.checksum = ~hralvds_cksum((const __u8 *)&r,
					sizeof(r) - 1, NULL, 0) + 1;

	mutex_lock(&ddata->cmd_radio_freq_lock);

	if (xmit_cmd_if_changed) {
		if (memcmp(&ddata->cmd_radio_freq, &r, sizeof(r)) == 0) {
			dev_dbg(dev, "No change in radio frequency "
				"command. Not sending command to panel.");

			ret = count;
			goto out_unlock;
		}
	}

	memcpy(&ddata->cmd_radio_freq, &r, sizeof(r));

	/* send the message to the hralvds display. */
	if (hralvds_i2c_cmd(ddata->ts_i2c_client,
			(__u8 *)&ddata->cmd_radio_freq,
			sizeof(struct hralvds_radio_freq_cmd_struct)) > 0)
		ret = count;
	else
		ret = -1;

out_unlock:
	mutex_unlock(&ddata->cmd_radio_freq_lock);
	return ret;
}

/*
 * show_hralvds_radio_freq:
 *
 * sysfs interface to show the hralvds radio frequency
 */
static ssize_t show_hralvds_radio_freq(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct hralvds_radio_freq_cmd_struct *r = &ddata->cmd_radio_freq;
	ssize_t len;
	int freq;

	mutex_lock(&ddata->cmd_radio_freq_lock);
	freq = ((r->radio.freq_thousands_hundreds & 0xf0) >> 4) * 1000 +
		(r->radio.freq_thousands_hundreds & 0x0f) * 100 +
		((r->radio.freq_tens_ones & 0xf0) >> 4) * 10 +
		(r->radio.freq_tens_ones & 0x0f);

	len = scnprintf(buf, PAGE_SIZE, "%i %i.%i%i\n",
		(r->radio.band & 0xf0) >> 4,
		freq,
		(r->radio.freq_tenths_hundredths & 0xf0) >> 4,
		(r->radio.freq_tenths_hundredths & 0x0f));
	mutex_unlock(&ddata->cmd_radio_freq_lock);

	return len;
}

/*
 * set_hralvds_kick_supervisor:
 *
 * DEBUG FUNCTION to kick the supervisor by scheduling the timer thread
 *		function hralvds_do_supervisor after it's been shut off.
 */
static ssize_t set_hralvds_kick_supervisor(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	queue_delayed_work(ddata->workq, &ddata->hralvds_timer_work,
				msecs_to_jiffies(10));

	return count;
}

static ssize_t show_hralvds_event_status(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&ddata->event_status_lock);
	len = scnprintf(buf, PAGE_SIZE, "0x%02x 0x%02x\n",
			ddata->event_status[0], ddata->event_status[1]);
	mutex_unlock(&ddata->event_status_lock);
	return len;
}


#if defined HRALVDS_LOG_DEBUG
/*
 * show_hralvds_debug:
 *
 * DEBUG FUNCTION to show the current level for debug messages.
 */
static ssize_t show_hralvds_debug(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "Debug level = %lu\n", ddata->dbg_lvl);
}
#endif

/*
 * show_hralvds_hdcp:
 *
 * DEBUG FUNCTION to show the current status of HDCP registers.
 */
static ssize_t show_hralvds_hdcp(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	int rc;
	__u8 cmd = SER925_HDCP_CTL;
	__u8 rsp[SER925_HDCP_STS - SER925_HDCP_CTL + 1] = { 0 };

	/*
	 * CTL (0xC3) and STS (0xC4) are adjacent registers, and can thus
	 *	be read in one command.
	 */
	rc = hralvds_i2c_cmdrsp(ddata->ser_i2c_client,
			&cmd, sizeof(cmd), rsp, sizeof(rsp), 1, 0);

	if (2 == rc) {
		return scnprintf(buf, PAGE_SIZE,
			"%s: HDCP CTL = %02hhx, HDCP STS = %02hhx\n",
			__func__, rsp[0], rsp[1]);
	} else {
		return scnprintf(buf, PAGE_SIZE,
			"%s: hralvds_i2c_cmdrsp returned %d\n", __func__, rc);
	}
}

#if defined HRALVDS_LOG_DEBUG
/*
 * set_hralvds_debug:
 *
 * set the driver's debug level.
 */
static ssize_t set_hralvds_debug(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	int i;
	unsigned long l;

	/* convert string to unsigned long */
	i = kstrtoul(buf, 0, &l);

	if (i < 0)
		return i;

	/* this is a bit mask */
	ddata->dbg_lvl = l;

	return count;
}
#endif

/*
 * show_hralvds_irq_bad:
 *
 * DEBUG FUNCTION to show the current bad IRQ count.
 */
static ssize_t show_hralvds_irq_bad(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "bad_irq_count = %u\n",
		ddata->bad_irq_count);
}

/*
 * set_hralvds_irq_counts:
 *
 * DEBUG FUNCTION to reset the good/bad IRQ counts.
 */
static ssize_t set_hralvds_irq_counts(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	ddata->bad_irq_count = 0;
	ddata->irq_count = 0;

	return count;
}

/*
 * show_hralvds_irq_counts:
 *
 * DEBUG FUNCTION to show the current good/bad IRQ counts.
 */
static ssize_t show_hralvds_irq_counts(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "irq_count = %u, bad_irq_count = %u\n",
		ddata->irq_count, ddata->bad_irq_count);
}

/*
 * hralvds_read_i2cregs:
 *
 * utility function to read a set of 8-bit registers from an i2c bus
 *	device.
 *
 * @i2c_client: client device to use
 * @regs: the buffer in which to store the values read.
 * @start: starting register to read
 * @nregs: number of registers to read
 */
static int hralvds_read_i2cregs(
	struct i2c_client *i2c_client, u8 *regs, int start, size_t nregs)
{
	u8 reg = start;
	int rc;

	/*
	 * when talking to the 925/928, there is no checksum, and we don't
	 *	want to attempt retries.
	 */
	rc = hralvds_i2c_cmdrsp(i2c_client, &reg, 1, regs, nregs, 0, 0);

	if (2 != rc)
		return rc;

	return 0;
}

/*
 * hralvds_dump_i2cregs:
 *
 * utility function to print the values of a series of registers
 *	from an i2c device.
 */
static ssize_t hralvds_dump_i2cregs (
	char *buf, size_t buflen, u8 *regs, size_t nregs)
{
	ssize_t len = 0;
	int i;
	int j;

	/* adjust so offsets row lines up with data bytes */
	len += scnprintf(buf, buflen, "    ");

	/* print column offsets */
	for (i = 0; i < 16; i++)
		len += scnprintf(buf+len, buflen-len, " %01x ", i);
	len += scnprintf(buf+len, buflen-len, "  0123456789abcdef\n");

	/* for each row, print the row upper nibble offset (e.g. "10: "),
	 * and then the databytes in hex and then ascii. */
	for (i = 0; i < nregs; i += 16) {
		len += scnprintf(buf+len, buflen-len, "%02x: ", i);
		for (j = 0; j < 16 && (i+j < nregs); j++)
			len += scnprintf(buf+len, buflen-len, "%02hhx ",
						regs[i+j]);

		len += scnprintf(buf+len, buflen-len, "  ");

		for (j = 0; j < 16 && (i+j < nregs); j++) {
			int r = regs[i+j];

			if (!r)
				r = '.';
			else if (r < ' ' || r > '~')
				r = '?';

			/* why doesn't isprint() work correctly here? */
			len += scnprintf(buf+len, buflen-len, "%c", r);
		}

		len += scnprintf(buf+len, buflen-len, "\n");
	}

	return len;
}

/*
 * show_hralvds_ser_regs:
 *
 * DEBUG FUNCTION to display all the registers of the display's
 *	deserializer.
 */
static ssize_t show_hralvds_ser_regs(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	u8 regs[256] = { 0 };

	if (0 != hralvds_read_i2cregs(ddata->ser_i2c_client, regs, 0,
					sizeof(regs)))
		return 0;

	return hralvds_dump_i2cregs(buf, PAGE_SIZE, regs, sizeof(regs));
}

/*
 * show_hralvds_deser_regs:
 *
 * DEBUG FUNCTION to display all the registers of the display's
 *	deserializer.
 */
static ssize_t show_hralvds_deser_regs(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t rc;
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	u8 regs[256] = { 0 };

	hralvds_setup_deser_alias(ddata);

	if (0 != hralvds_read_i2cregs(ddata->deser_i2c_client, regs, 0,
					sizeof(regs))) {
		hralvds_setup_ts_alias(ddata);
		return 0;
	}

	rc = hralvds_dump_i2cregs(buf, PAGE_SIZE, regs, sizeof(regs));

	hralvds_setup_ts_alias(ddata);

	return rc;
}

/*
 * show_hralvds_report:
 *
 * read the REPORT ID from the display.
 * reports the last interrupt request (KEY, KNOB, TOUCH) from the
 *	display.
 */
static ssize_t show_hralvds_report(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	int r;

	__u8 wbuf[1] = { HRALVDS_REPORT_ID_CMD };
	__u8 rbuf[2 + 1] = { 0 };

	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
		wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 1, 0);

	if (r > 0)
		return scnprintf(buf, PAGE_SIZE,
			"Report = %02hhx %02hhx %02hhx\n",
			rbuf[0], rbuf[1], rbuf[2]);
	else
		return 0;
}

static ssize_t show_hralvds_pattern(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	printk("color pattern: \n 0: scroll\n1:white/black\n");
	printk("2:Black/white\n3:Red/Cyan\n4:Green/Magenta\n");
	printk("5:Blue/Yellow\n");
	printk("6: Horizontally Scaled Black to white/white to black\n");
	printk("7:Horizontally Scaled black to Red/Cyan to white\n");
	printk("8:Horizontally Scaled Black to Green/Magenta to White\n");
	printk("9:Horizontally Scaled Black to Blue/Yellow to White\n");
	printk("10:Vertically Scaled Black to White/White to Black\n");
	printk("11:Vertically Scaled Black to Red/Cyan to White\n");
	printk("12:Vertically Scaled Black to Green/Magenta to White\n");
	printk("13:Vertically Scaled Black to Blue/Yellow to White\n");
	printk("usage: echo 'color seconds'>hralvds_pattern\n");
	return 0;
}

static ssize_t set_hralvds_pattern(struct device *dev,
		struct device_attribute *attr, const char *buf,size_t count)
{
	int r = 0;
	unsigned char data = 0;
	unsigned int color = 0;
	unsigned int keep_period = 0;
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	/* first byte is color, second byte is showing period */
	r = sscanf(buf,"%d %d", &color, &keep_period);
	if (r <= 0)
	{	printk("usage:echo color second > hralvds_pattern\n");
		return -1;
	}

	if (color > HRALVDS_COLOR_MAX ||
			keep_period > HRALVDS_PATTERN_SHOW_TIME_MAX)
		return -1;

	/* Pattern generator configuration 18-bit */
	if (color == 0) /*auto scroll*/
		data |= (SER925_PG_CONF_AUTO_SCROLL |
			SER925_PG_CONF_18_BIT);
	else
		data |= SER925_PG_CONF_18_BIT;

	r = i2c_write_le8(ddata->ser_i2c_client, SER925_PG_CONF, data);
	if (r < 0)
		return -1;

	/* enable pattern generator control */
	data = color << 4;
	data |= SER925_PG_CTL_ENABLE;
	r = i2c_write_le8(ddata->ser_i2c_client, SER925_PG_CTL, data);
	if (r < 0)
		return -1;

	if (keep_period == 0)
		keep_period = 3; /*default 3 second*/
	dev_info(&ddata->ser_i2c_client->dev,
		"keep for %d seconds\n", keep_period);
	ssleep(keep_period);

	/*diable pattern generator control*/
	data = SER925_PG_CTL_DISABLE;
	r = i2c_write_le8(ddata->ser_i2c_client, SER925_PG_CTL, data);
	if (r < 0)
		return -1;

	dev_info(&ddata->ser_i2c_client->dev,"%s test done! \n",
			__func__);
	return count;

}

static void hralvds_hdcp_change_status(struct panel_drv_data *ddata)
{
	/* TODO:provide a utility for simulate KSV Verify for ic6,
	 * need change to enable/disable HDCP in the future */
	int r = 0;
	int ret = 0;
	__u8 wrctl[2] = {
		SER925_HDCP_CTL,
		SER925_HDCP_CTL_KSV_VALID,
	};

	if (ddata->hdcp_enable)
		ret = hralvds_i2c_cmd(ddata->ser_i2c_client,
					wrctl, sizeof(wrctl));
	else {
		r = i2c_read_le8(ddata->ser_i2c_client, SER925_HDCP_CTL);
		if (r < 0) {
			dev_info(&ddata->ser_i2c_client->dev,
				"%s fail to read SER925_HDCP_CTL \n",
				__func__);
			return;
		}

		r &= ~SER925_HDCP_CTL_KSV_VALID;
		ret = i2c_write_le8(ddata->ser_i2c_client, SER925_HDCP_CTL, r);
	}

	if (ret < 0)
		dev_info(&ddata->ser_i2c_client->dev,
				"%s fail to change HDCP status \n", __func__);

}

static ssize_t set_hralvds_hdcp_enable(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	int r;
	unsigned int hdcp_tmp = 0;
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	r = sscanf(buf, "%d", &hdcp_tmp);
	if (r < 1)
		return -1;

	ddata->hdcp_enable = hdcp_tmp;
	return count;
}

static ssize_t show_hralvds_hardwaretest(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t len;

	len = scnprintf(buf, PAGE_SIZE,
		"Supported Commands: \n%s\n%s\n%s\n",
		"clear_irq - clear all irq counts, clear isr",
		"set_irq - enable RX interrupt, enable interrupts",
		"reset_irq - set isr back to original state");

	return len;
}

/* hralvds_hardwaretest
 * Perform various activities to faciliate testing
 * the hardware connection.
 */
static int save_icr = 0;
static ssize_t set_hralvds_hardwaretest(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct panel_drv_data *ddata;
	int icr; /* interrupt control register */
	int isr; /* interrupt service register */

	if ((buf != NULL) && strlen(buf) > 0) {
		ddata = dev_get_drvdata(dev);
		if (ddata == NULL) {
			return 0;
		}

		/* echo clear_irq > hralvds_hardwaretest */
		if (strncmp(buf, "clear_irq", count)) {
			/* reset irq */
			if (ddata) {
				ddata->irq_count = 0;
				ddata->bad_irq_count = 0;
			}

			/* clear ISR */
			isr = i2c_read_le8(ddata->ser_i2c_client,
				SER925_HDCP_ISR );
			printk("%s READ(ISR) = %d %X\n",
				__func__, isr, isr);
			return count;
		}

		/* echo set_irq > hralvds_hardwaretest */
		else if (strncmp(buf, "set_irq", count)) {
			save_icr = i2c_read_le8(
				ddata->ser_i2c_client,
				SER925_HDCP_ICR);
			icr = save_icr;
			icr |= SER925_HDCP_ICR_IS_RX_INT;
			icr |= SER925_HDCP_ICR_INT_ENABLE;
			icr = i2c_write_le8(ddata->ser_i2c_client,
				SER925_HDCP_ICR, icr);
			printk("%s READ(ICR) = %d %X\n",
				__func__, icr, icr);
		}

		/* echo reset_irq > hralvds_hardwaretest */
		else if (strncmp(buf, "reset_irq", count)) {
			if (save_icr != 0) {
				icr = i2c_write_le8(
					ddata->ser_i2c_client,
					SER925_HDCP_ICR,
					save_icr);
				save_icr = 0;
				printk("%s READ(ICR) = %d %X\n",
					__func__, icr, icr);
			}
		}
	}

	return 0;
}

/*
 * show the current status of any update file transfer.
 */
static ssize_t show_hralvds_fwif_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	ssize_t len = 0;

	static const char *fw_update_state_names[] = {
		"IDLE",
		"LOAD FILE",
		"WAIT FOR START",
		"TRANSFER",
		"100msec WAIT",
		"CHECK STATUS",
		"STATUS WAIT",
		"FIN1 - UPDATE SENT",
		"FIN2 - WAIT FOR INTERRUPT",
	};

	/*
	 * hralvds_fwif_update_supported is defined by the panel driver for
	 *	the specific model (thra, tjba, etc).
	 */
	if (hralvds_fwif_update_supported()) {
		/*
		 * show the status of the firmware update here.
		 */
		len = scnprintf(buf, PAGE_SIZE,
			"Firmware update supported with this display.\n");
		len += scnprintf(buf+len, PAGE_SIZE-len,
			"Firmware update state = %s\n",
			fw_update_state_names[ddata->fwupd.update_state]);
		if (ddata->fwupd.buf) {
			len += scnprintf(buf+len, PAGE_SIZE-len,
				"File size = %u", ddata->fwupd.size);

			if (DISPLAY_UPDATE_WAIT_FOR_READY <
						ddata->fwupd.update_state)
				len += scnprintf(buf+len, PAGE_SIZE-len,
					", Offset = %u", ddata->fwupd.offs);

			buf[len++] = '\n';
			buf[len++] = '\0';
		}
	} else {
		len = scnprintf(buf, PAGE_SIZE,
			"Firmware update not supported with this display.\n");
	}

	return len;
}

static ssize_t show_hralvds_display_mode_event(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&ddata->display_mode_lock);
	len = scnprintf(buf, PAGE_SIZE, "0x%02x\n",
			ddata->display_mode);
	mutex_unlock(&ddata->display_mode_lock);
	return len;
}

/*
 * send a request to the display to change the display mode.
 */
static ssize_t set_hralvds_display_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	int mode;
	uint8_t wbuf[HRALVDS_DISPLAY_MODE_REQUEST_SIZE + 1] = {
		HRALVDS_DISPLAY_MODE_REQUEST_CMD, };
	int rc;

	rc = sscanf(buf, "%x", &mode);

	if (rc < 1)
		return -2;

	wbuf[1] = mode << 4;
	wbuf[2] = ~hralvds_cksum( wbuf, sizeof(wbuf) - 1, NULL, 0) + 1;

	if (hralvds_i2c_cmd(ddata->ts_i2c_client, wbuf, sizeof(wbuf)) > 0)
		return count;
	else
		return -1;
}

static ssize_t set_hralvds_touch_cmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	uint8_t status;
	uint16_t x, y;
	uint16_t xy[2];
	uint8_t wbuf[HRALVDS_TOUCH_CMD_SIZE + 1] = {
		HRALVDS_TOUCH_CMD, };
	int rc;

	rc = sscanf(buf, "%hhu %hu %hu", &status, &x, &y);

	if (rc < 3)
		return -2;

	xy[0] = cpu_to_be16(x);
	xy[1] = cpu_to_be16(y);

	wbuf[1] = status;

	memcpy(wbuf + 2, xy, sizeof(xy));

	wbuf[6] = ~hralvds_cksum(wbuf, sizeof(wbuf) - 1, NULL, 0) + 1;

	if (hralvds_i2c_cmd(ddata->ts_i2c_client,
			wbuf, sizeof(wbuf)) > 0)
		return count;
	else
		return -1;
}

static ssize_t set_hralvds_setting_status(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	uint8_t language, color;
	uint8_t wbuf[HRALVDS_SETTING_STATUS_SIZE + 1] = {
		HRALVDS_SETTING_STATUS_CMD, };
	int rc;

	rc = sscanf(buf, "%hhx %hhx", &language, &color);

	if (rc < 1)
		return -2;

	wbuf[1] = language;
	wbuf[2] = color;
	wbuf[3] = ~hralvds_cksum(wbuf, sizeof(wbuf) - 1, NULL, 0) + 1;

	if (hralvds_i2c_cmd(ddata->ts_i2c_client, wbuf, sizeof(wbuf)) > 0)
		return count;
	return -1;
}

static ssize_t set_hralvds_camera_setting(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	uint8_t status;
	uint8_t wbuf[HRALVDS_WRITE_CAMERA_SETTING_SIZE + 1] = {
		HRALVDS_WRITE_CAMERA_SETTING_CMD, };
	int rc;

	rc = sscanf(buf, "%hhx", &status);

	if (rc < 1)
		return -2;

	wbuf[1] = status;
	wbuf[2] = ~hralvds_cksum(wbuf, sizeof(wbuf) - 1, NULL, 0) + 1;

	if (hralvds_i2c_cmd(ddata->ts_i2c_client, wbuf, sizeof(wbuf)) > 0)
		return count;
	return -1;
}

static ssize_t show_hralvds_camera_setting(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	uint8_t wbuf[1] = { HRALVDS_READ_CAMERA_SETTING_CMD, };
	uint8_t rbuf[1 + 1] = { 0 };
	int rc;

	rc = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
		wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 1, 3);

	if (2 == rc)
		return scnprintf(buf, PAGE_SIZE, "%hhx\n", rbuf[0]);
	else
		return -1;
}

static ssize_t set_hralvds_touchpad_error_status(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	uint8_t status;
	uint8_t wbuf[HRALVDS_TOUCHPAD_ERROR_STATUS_SIZE + 1] = {
		HRALVDS_TOUCHPAD_ERROR_STATUS_CMD, };
	int rc;

	rc = sscanf(buf, "%hhx", &status);

	if (rc < 1)
		return -2;

	wbuf[1] = status;
	wbuf[2] = ~hralvds_cksum(wbuf, sizeof(wbuf) - 1, NULL, 0) + 1;

	if (hralvds_i2c_cmd(ddata->ts_i2c_client, wbuf, sizeof(wbuf)) > 0)
		return count;
	return -1;
}

static ssize_t set_hralvds_camera_configuration(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	uint8_t status;
	uint8_t wbuf[HRALVDS_CAMERA_CONFIGURATION_SIZE + 1] = {
		HRALVDS_CAMERA_CONFIGURATION_CMD, };
	int rc;

	rc = sscanf(buf, "%hhx", &status);

	if (rc < 1)
		return -2;

	wbuf[1] = status;
	wbuf[2] = ~hralvds_cksum(wbuf, sizeof(wbuf) - 1, NULL, 0) + 1;

	if (hralvds_i2c_cmd(ddata->ts_i2c_client, wbuf, sizeof(wbuf)) > 0)
		return count;
	return -1;
}

static ssize_t show_hralvds_camera_diagnosis(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	uint8_t wbuf[1] = { HRALVDS_CAMERA_DIAGNOSIS_CMD, };
	uint8_t rbuf[2 + 1] = { 0 };
	int rc;

	rc = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
		wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 1, 3);

	if (2 == rc)
		return scnprintf(buf, PAGE_SIZE,
			"%hhx %hhx\n", rbuf[0], rbuf[1]);
	else
		return -1;
}

static ssize_t set_hralvds_mvc_dtc_detect_cancel(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	uint8_t request;
	uint8_t wbuf[HRALVDS_MVC_DTC_DETECT_CANCEL_REQUEST_SIZE + 1] = {
		HRALVDS_MVC_DTC_DETECT_CANCEL_REQUEST_CMD, };
	int rc;

	rc = sscanf(buf, "%hhx", &request);

	if (rc < 1)
		return -2;

	wbuf[1] = request;
	wbuf[2] = ~hralvds_cksum(wbuf, sizeof(wbuf) - 1, NULL, 0) + 1;

	if (hralvds_i2c_cmd(ddata->ts_i2c_client, wbuf, sizeof(wbuf)) > 0)
		return count;
	return -1;
}

static ssize_t set_hralvds_i2c_msleep(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	uint32_t ul;

	rc = sscanf(buf, "%d", &ul);

	if (rc != 1)
		return -1;

	hralvds_i2c_msleep = ul;

	return count;
}

static ssize_t show_hralvds_i2c_msleep(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "i2c msleep = %d\n",
		hralvds_i2c_msleep);
}

static ssize_t set_hralvds_i2c_retries(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	/*
	 * all writes to this file result in clearing the counters.
	 */
	hralvds_i2c_retries = 0;
	hralvds_i2c_gave_up = 0;

	return count;
}

static ssize_t show_hralvds_i2c_retries(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "r = %d g = %d\n",
		hralvds_i2c_retries, hralvds_i2c_gave_up);
}

static ssize_t show_hralvds_i2c_stats(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u64 now;
	u64 kernel_time;
	u64 max_delta_t;
	unsigned long rem_nsec_kt, rem_nsec_now;

	now = local_clock();
	spin_lock(&max_i2c_wtr_turnaround_0x55.lock);
	kernel_time = max_i2c_wtr_turnaround_0x55.kernel_time;
	max_delta_t = max_i2c_wtr_turnaround_0x55.max_delta_t;
	spin_unlock(&max_i2c_wtr_turnaround_0x55.lock);

	rem_nsec_kt = do_div(kernel_time, 1000000000);
	rem_nsec_now = do_div(now, 1000000000);

	return scnprintf(buf, PAGE_SIZE, "max = "
		"%llu @ %lu.%06lu, now = %lu.%06lu\n", max_delta_t,
		(unsigned long) kernel_time, rem_nsec_kt / 1000,
		(unsigned long) now, rem_nsec_now / 1000);
}

static ssize_t show_hralvds_i2c_info(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u64 max_delta_t;

	spin_lock(&max_i2c_wtr_turnaround_0x55.lock);
	max_delta_t = max_i2c_wtr_turnaround_0x55.max_delta_t;
	spin_unlock(&max_i2c_wtr_turnaround_0x55.lock);

	return scnprintf(buf, PAGE_SIZE, "%llu %d %d\n", max_delta_t,
		hralvds_i2c_retries, hralvds_i2c_gave_up);
}

/*
 * load the firmware file from HRALVDS_UPDATE_FILE_PATH.
 */
static int hralvds_update_load_file(struct panel_drv_data *ddata)
{
	struct file *fp;
	mm_segment_t fs;
	struct kstat stat;
	ssize_t size;
	int status = 0;

	/*
	 * set the data segment so that vfs_read can access the buffer in the
	 *	kernel as though it was in userspace.
	 */
	fs = get_fs();
	set_fs(KERNEL_DS);

	/*
	 * get the size of the file (too bad this isn't in *fp somewhere).
	 */
	if (vfs_stat(HRALVDS_UPDATE_FILE_PATH, &stat)) {
		dev_err(&ddata->ts_i2c_client->dev,
			"Can't stat update file %s.",
			HRALVDS_UPDATE_FILE_PATH);
		status = -ENOENT;
		goto done0;
	}

	/*
	 * open the file.
	 */
	fp = filp_open(HRALVDS_UPDATE_FILE_PATH, O_RDONLY, 0);
	if (!fp) {
		dev_err(&ddata->ts_i2c_client->dev,
			"Can't load update file %s.",
			HRALVDS_UPDATE_FILE_PATH);
		status = -ENOENT;
		goto done0;
	}

	/*
	 * store the file size.
	 * this is size of the WHOLE file, including the header.
	 */
	ddata->fwupd.size = (u32) stat.size;
	dev_info(&ddata->ts_i2c_client->dev, "Update file size is %u.",
		ddata->fwupd.size);

	/*
	 * in the event of a restart, reset the buffer.  it's possible
	 *	that the update file has changed, and the buffer required may
	 *	not be the correct size.
	 */
	if (ddata->fwupd.buf) {
		vfree(ddata->fwupd.buf);
		ddata->fwupd.buf = NULL;
	}

	/*
	 * allocate a block to store the file.
	 * vmalloc can access > 4MB, which is necessary for some update files.
	 */
	ddata->fwupd.buf = vmalloc(ddata->fwupd.size);
	if (!ddata->fwupd.buf) {
		dev_err(&ddata->ts_i2c_client->dev,
			"No memory to load update file %s.",
			HRALVDS_UPDATE_FILE_PATH);
		status = -ENOMEM;
		goto done1;
	}

	/*
	 * if the read succeeds we fall through to where we close the file and
	 *	restore the data segment.
	 */
	size = vfs_read(fp, ddata->fwupd.buf, ddata->fwupd.size,
								&fp->f_pos);
	if (size != ddata->fwupd.size) {
		dev_err(&ddata->ts_i2c_client->dev,
			"Error while reading update file %s, "
			"bytes read = %d, f_pos = %llu.",
			HRALVDS_UPDATE_FILE_PATH, size, fp->f_pos);
		vfree(ddata->fwupd.buf);
		ddata->fwupd.buf = NULL;
		ddata->fwupd.size = 0;
		status = -EIO;

	} else {

#if defined HRALVDS_STORE_RAW_I2C_DATA
		/*
		 * open a file to allow storage of all the i2c
		 *	data packets sent to the display.  this is used to
		 *	verify that the individual transfer packets are
		 *	valid.
		 * data received is not stored, nor is any of the status
		 *	data (UPDATE STATUS, DISPLAY NOTIFICATION STATUS, etc).
		 */
		ddata->rawfp = filp_open("/data/local/tmp/update.raw",
			O_WRONLY | O_CREAT | O_TRUNC, 0644);

		if (ddata->rawfp)
			dev_err(&ddata->ts_i2c_client->dev,
				"Created file /data/local/tmp/update.raw");
		else
			dev_err(&ddata->ts_i2c_client->dev,
				"Couldn't create update.raw");
#endif

		/*
		 * successful load.
		 */
		return 0;
	}

done1:
	filp_close(fp, NULL);

done0:
	set_fs(fs);
	return status;
}

/*
 * kick off the update process.
 * can be called automatically from an interrupt if we get an update.
 */
static int hralvds_begin_update(struct panel_drv_data *ddata)
{
	int status = -1;

	/*
	 * start by loading the file.
	 */
	if (hralvds_update_load_file(ddata) == 0) {
		/*
		 * update the various pointers into the firmware
		 *	file:
		 * data points to the region of the buffer where the update
		 *	data is, past the header.
		 * data_size is the size of the update data, not including
		 *	the header.  this is what needs to be sent in the first
		 *	update packet to the display.
		 *
		 * offs has a range of [0,size).
		 */
		ddata->fwupd.data = ddata->fwupd.buf +
			HRALVDS_UPDATE_FILE_HEADER_SIZE;
		ddata->fwupd.data_size = ddata->fwupd.size -
			HRALVDS_UPDATE_FILE_HEADER_SIZE;
		ddata->fwupd.offs = 0;
		ddata->fwupd.last_offs = 0;
		ddata->fwupd.last_ackd_offs = 0;

		/*
		 * clear counters before starting.
		 */
		ddata->fwupd.status_pkt_count = 0;
		ddata->fwupd.busy_count = 0;
		ddata->fwupd.i2c_error_count = 0;

		status = 0;
	}

	return status;
}


/*
 * process userspace command to perform certain firmware update commands.
 */
static ssize_t set_hralvds_fwif_control(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	/*
	 * kick off the firmware upload.
	 * don't do this if a file upload is already in progress.
	 */
	if ((strncmp(buf, "begin", 5) == 0) &&
			(DISPLAY_UPDATE_IDLE == ddata->fwupd.update_state)) {
		ddata->fwupd.update_reason = DISPLAY_UPDATE_REASON_USER;
		ddata->fwupd.update_state = DISPLAY_UPDATE_LOAD;
	} else
		dev_warn(&ddata->ts_i2c_client->dev,
			"Error: display update already in progress.");

	return count;
}

/*
 * hralvds_irq_worker:
 *
 * this function is invoked by the interrupt thread.
 */
static void hralvds_irq_worker(struct panel_drv_data *ddata)
{
	/*
	 * valid_irq is set if this function was invoked and a valid
	 *	interrupt was detected.  if no interrupt was detected,
	 *	then a counter is incremented to track invalid interrupts.
	 *	if "enough" invalid interrupts are detected in a row,
	 *	the interrupt is disabled until they are re-enabled by
	 *	writing to a sysfs file.
	 */
	int valid_irq = 0;
	static int ksv_fifo_idx = 0;
	static __u8 ksv_fifo[512] = { 0 };

	/*
	 * transaction for reading the HDCP STS, HDCP ICR and HDCP ISR
	 *	registers.  encode three reads in one transaction to reduce
	 *	bus access time.
	 */
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

	/*
	 * increment the IRQ count for statistics.
	 */
	ddata->irq_count++;

	/*
	 * when communicating with the 925, we don't use a checksum and we
	 *	don't ask for retries.
	 */
	r = hralvds_i2c_cmdrsp(ddata->ser_i2c_client,
		rdregs, sizeof(rdregs), regs, sizeof(regs), 0, 0);

	/* return value of two means two messages were transferred */
	if (2 == r) {
		/*
		 * we have successfully read the STS and ISR
		 * registers.
		 */

		/*
		 * for the ISR register, the bit is non-zero when the
		 *	interrupt is active.  ISR has to be masked with
		 *	ICR for this to work, which is not in the datasheet.
		 */
		is_rx_int = ((isr & icr) & SER925_HDCP_ISR_IS_RX_INT) != 0;

		if (is_rx_int) {
			/*
			 * interrupt on receiver - a downstream
			 *	receiver has asserted the interrupt.
			 */
			valid_irq = 1;

			/*
			 * Call the HRALVDS panel-specific IRQ code.
			 */
			r = hralvds_panel_irq(ddata);
		}

		if (isr & SER925_HDCP_ISR_IS_IND_ACCESS) {
			/* indirect register access */
			valid_irq = 1;
			dev_info(&ddata->ts_i2c_client->dev,
				"HDCP ISR IS IND ACCESS\n");
		}

		if (isr & SER925_HDCP_ISR_INT_DETECT) {
			/* downstream receiver detected */

#if defined HRALVDS_HDCP_ENABLED
			/*
			 * upon receiving RX LOCK (which
			 *	generates this interrupt),
			 *	set the HDCP EN bit in
			 *	the HDCP CTL register.
			 * all of the bits in this register
			 *	are self-clearing or ignored
			 *	when set to zero.
			 */
			__u8 wrctl[2] = {
				SER925_HDCP_CTL,
				SER925_HDCP_CTL_HDCP_EN,
			};

			r = hralvds_i2c_cmd(
				ddata->ser_i2c_client,
				wrctl, sizeof(wrctl));

			dev_info(&ddata->ts_i2c_client->dev,
				"set HDCP EN, rc = %d\n", r);
#endif

			ddata->display_detected = 1;
			valid_irq = 1;

			dev_info(&ddata->ts_i2c_client->dev,
				"HDCP ISR INT DETECT\n");
		}

		if (isr & SER925_HDCP_ISR_IS_LIST_RDY) {
			__u8 rdctl[1] = { SER925_RX_BCAPS };
			__u8 rdrsp[1] = { 0 };

			/*
			 * when communicating with the 925, we don't use a
			 *	checksum and we don't attempt retries.
			 */
			r = hralvds_i2c_cmdrsp (
				ddata->ser_i2c_client,
				rdctl, sizeof rdctl,
				rdrsp, sizeof rdrsp,
				0, 0);

			if (r > 0) {
				if (rdrsp[0] & SER925_RX_BCAPS_KSV_FIFO) {
					rdctl[0] = SER925_KSV_FIFO;

					/*
					 * when communicating with the 925,
					 *	we don't use a checksum and
					 *	we don't attempt retries.
					 */
					r = hralvds_i2c_cmdrsp (
						ddata->ser_i2c_client,
						rdctl, sizeof rdctl,
						rdrsp, sizeof rdrsp,
						0, 0);

					if (r > 0) {
						ksv_fifo[ksv_fifo_idx++] =
							rdrsp[0];

						dev_info(&ddata->ts_i2c_client->dev,
							"KSV FIFO = %02hhx",
							rdrsp[0]);
					}
				}
			}

			/* local KSV list ready */
			valid_irq = 1;
			dev_info(&ddata->ts_i2c_client->dev,
				"HDCP ISR IS LIST RDY\n");
		}

		if (isr & SER925_HDCP_ISR_IS_KSV_RDY) {

			dev_info(&ddata->ts_i2c_client->dev,
				"HDCP ISR IS KSV RDY\n");
			if (ddata->hdcp_enable) {

				__u8 wrctl[2] = {
					SER925_HDCP_CTL,
					SER925_HDCP_CTL_KSV_VALID,
				};

				r = hralvds_i2c_cmd(
						ddata->ser_i2c_client,
						wrctl, sizeof(wrctl));
			}
			valid_irq = 1;
		}

		if (isr & SER925_HDCP_ISR_IS_AUTH_FAIL) {
			/* HDCP authentication failure */
			valid_irq = 1;
			dev_info(&ddata->ts_i2c_client->dev,
				"HDCP ISR IS AUTH FAIL\n");

			/*
			 * The hardware will attempt a retry; the next HDCP
			 * interrupt event should be a HDCP AUTH PASS.
			 * Set a timeout to reset the display if HDCP AUTH
			 * PASS was not achieved.
			 */
			switch (ddata->power_state) {
			case POWER_STATE_POWERING_OFF:
			case POWER_STATE_OFF:
				/*
				 * Do not wait for HDCP AUTH PASS if already
				 * off.
				 */
				break;

			case POWER_STATE_POWERING_ON_1:
			case POWER_STATE_POWERING_ON_1A:
			case POWER_STATE_POWERING_ON_2:
			case POWER_STATE_ON:
				queue_delayed_work(ddata->workq,
					&ddata->hralvds_hdcp_auth_retry_work,
					HRALVDS_HDCP_AUTH_TIMEOUT);
				break;
			default:
				break;
			}
		}

		if (isr & SER925_HDCP_ISR_IS_AUTH_PASS) {
			/* HDCP authentication successful */
			valid_irq = 1;
			dev_info(&ddata->ts_i2c_client->dev,
				"HDCP ISR IS AUTH PASS\n");

			cancel_delayed_work_sync(
				&ddata->hralvds_hdcp_auth_retry_work);
		}
	}

	if (!valid_irq) {
		ddata->bad_irq_count++;
		dev_err(&ddata->ts_i2c_client->dev,
			"Bad IRQ detected (#%d), STS = %02hhx, ISR = %02hhx\n",
			ddata->bad_irq_count, sts, isr);
	}

#undef sts
#undef icr
#undef isr
}

/*
 * returns 0 if we're able to adjust the value of the BCC WDT on the
 *	deserializer.
 *
 * caller must ensure the deserializer alias is set.
 */
static int hralvds_adjust_display_bcc_wdt(struct panel_drv_data *ddata,
					u8 new_bcc_wdt)
{
	int r;
	int tries = 0;

	do {
		r = i2c_write_le8(ddata->deser_i2c_client,
					DES928_BCC_WDT, new_bcc_wdt);

		if (!r)
			dev_info(&ddata->deser_i2c_client->dev,
				"Adjusted display BCC WDG to %hhu msec.",
				new_bcc_wdt);
		else
			dev_warn(&ddata->deser_i2c_client->dev,
				"Adjusting failed; retrying.");

	} while (r && ++tries < HRALVDS_DES_BCC_WDT_ADJ_MAX_TRIES);

	return r;
}

/*
 * hralvds_setup_link:
 *	setup the serializer fpdlink.
 */
/*#define HRALVDS_STARTUP_DEBUG*/
int hralvds_setup_link(struct panel_drv_data *ddata)
{
	int r;
	int config0;
	int config1;
	int datactl;
	int modests;
	int gensts;
	int rxsts;
	int bccwdt;
	int tries;

	/*
	 * have to shift the received value because the serializer i2c address
	 *	is in the upper 7 bits of the register.
	 */
	r = i2c_read_le8(ddata->ser_i2c_client, SER925_I2C_DEVICE_ID);
	if (r > 0) {
		r >>= 1;
		if (r != ddata->ser_i2c_client->addr) {
			if (__ratelimit(&ddata->setup_link_rs))
				dev_err(&ddata->ts_i2c_client->dev,
					"Ser addr mismatch: r = 0x%02hhx,"
					" client addr = %02hhx.",
					r, ddata->ser_i2c_client->addr);

			goto err0;
		}
	} else {
		/*
		 * dump the 7-bit physical address of the serializer,
		 *	since that's how it's known by developers.
		 */
		if (__ratelimit(&ddata->setup_link_rs))
			dev_err(&ddata->ts_i2c_client->dev,
				"Couldn't read Serializer i2c address, r = %d.",
				r >> 1);

		goto err0;
	}

#if defined HRALVDS_STARTUP_DEBUG
	if (__ratelimit(&ddata->setup_link_rs))
		dev_info(&ddata->ts_i2c_client->dev,
			"Serializer i2c addr 0x%x.", r);
#endif
	r = i2c_read_le8(ddata->ser_i2c_client, SER925_GEN_STS);
	if ((r < 0) || ((r & SER925_GEN_STS_PCLK_DETECT) == 0)) {
		if (__ratelimit(&ddata->setup_link_rs))
			dev_err(&ddata->ts_i2c_client->dev,
				"PCLK not present.");

		goto err0;
	}

#if defined HRALVDS_STARTUP_DEBUG
	if (__ratelimit(&ddata->setup_link_rs))
		dev_info(&ddata->ts_i2c_client->dev, "PCLK detection done.");
#endif

	if ((r & SER925_GEN_STS_LINK_DETECT) == 0) {
		if (__ratelimit(&ddata->setup_link_rs))
			dev_err(&ddata->ts_i2c_client->dev,
				"FPDlink not present.");

		r = -ENODEV;
		goto err0;
	}

#if defined HRALVDS_STARTUP_DEBUG
	if (__ratelimit(&ddata->setup_link_rs))
		dev_info(&ddata->ts_i2c_client->dev, "FPDLink detected.");
#endif

#ifndef HRALVDS_DSER_I2C_FROM_OF
	r = i2c_read_le8(ddata->ser_i2c_client, SER925_I2C_DES_ID);
	if (r > 0) {
		ddata->deser_phys = r >> 1;

		/*
		 * dump the 7-bit physical address of the deserializer,
		 *	since that's how it's known by developers.
		 */
		if (__ratelimit(&ddata->setup_link_rs))
			dev_info(&ddata->ts_i2c_client->dev,
				"Deserializer found at phys address 0x%02hhx.",
				ddata->deser_phys);
	} else {
		if (__ratelimit(&ddata->setup_link_rs))
			dev_err(&ddata->ts_i2c_client->dev,
				"Couldn't read i2c deserializer address: "
				"r = %d, deser->addr = 0x%x.",
				r, ddata->deser_i2c_client->addr);
		goto err0;
	}
#endif

#if defined HRALVDS_STARTUP_DEBUG
	if (__ratelimit(&ddata->setup_link_rs))
		dev_info(&ddata->ts_i2c_client->dev, "Serializer status dump:");
#endif

	config0 = i2c_read_le8(ddata->ser_i2c_client, SER925_CONFIG0);
	config1 = i2c_read_le8(ddata->ser_i2c_client, SER925_CONFIG1);
	datactl = i2c_read_le8(ddata->ser_i2c_client, SER925_DATA_CTL);
	modests = i2c_read_le8(ddata->ser_i2c_client, SER925_MODE_STS);
	gensts = i2c_read_le8(ddata->ser_i2c_client, SER925_GEN_STS);

	if ((config0 < 0) || (config1 < 0) || (datactl < 0) ||
		(modests < 0) || (gensts < 0)) {
		if (__ratelimit(&ddata->setup_link_rs))
			dev_err(&ddata->ts_i2c_client->dev,
				"Couldn't read serializer status.");

		goto err0;
	}

	if (__ratelimit(&ddata->setup_link_rs)) {
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
	}

	/*
	 * setup the deserializer alias to talk to the deserializer,
	 *	and then dump its registers.
	 */
	hralvds_setup_deser_alias(ddata);

#if defined HRALVDS_STARTUP_DEBUG
	if (__ratelimit(&ddata->setup_link_rs))
		dev_info(&ddata->ts_i2c_client->dev,
			"Deserializer status dump:");
#endif

	config0 = i2c_read_le8(ddata->deser_i2c_client, DES928_CONFIG0);
	config1 = i2c_read_le8(ddata->deser_i2c_client, DES928_CONFIG1);
	datactl = i2c_read_le8(ddata->deser_i2c_client, DES928_DATA_CTL);
	rxsts = i2c_read_le8(ddata->deser_i2c_client, DES928_RX_STS);
	gensts = i2c_read_le8(ddata->deser_i2c_client, DES928_GEN_STS);

	if ((config0 < 0) || (config1 < 0) || (datactl < 0) ||
		(rxsts < 0) || (gensts < 0)) {
		if (__ratelimit(&ddata->setup_link_rs))
			dev_err(&ddata->ts_i2c_client->dev,
				"Couldn't read deserializer status, addr = 0x%02hhx.",
				ddata->deser_i2c_client->addr);

		goto err0;
	}

	if (__ratelimit(&ddata->setup_link_rs)) {
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
	}

	/*
	 * force the display's BCC WDG timer to be at least 20msec.
	 *
	 * the register is in 2msec units, and is in bits 7:1.
	 * bit 0 is the enable flag (enabled = 0, disabled = 1).
	 * so by writing 20, bits 7:1 = 10, 10 * 2 msec/lsb = 20msec.
	 */
	tries = 0;
	do {
		bccwdt = i2c_read_le8(ddata->deser_i2c_client, DES928_BCC_WDT);
		if (bccwdt >= 0) {
			if (bccwdt < DES928_BCC_WDT_MIN) {
				r = hralvds_adjust_display_bcc_wdt(ddata,
					DES928_BCC_WDT_MIN);
				if (r < 0) {
					if (__ratelimit(&ddata->setup_link_rs))
						dev_warn(&ddata->deser_i2c_client->dev,
							"Unable to adjust DES BCC WDT");
				}
			}
		}

	} while (bccwdt < 0 && ++tries < HRALVDS_DES_BCC_WDT_ADJ_MAX_TRIES);

	/*
	 * reset the alias to the touchscreen.
	 */
	hralvds_setup_ts_alias(ddata);

	ddata->supervisor_ok = 0;

	return 0;

err0:
	if (__ratelimit(&ddata->setup_link_rs))
		dev_info(&ddata->ts_i2c_client->dev, "%s ends on error",
			__func__);

	return -EINVAL;
}
/*#undef HRALVDS_STARTUP_DEBUG*/

static void hralvds_hdcp_auth_retry_worker(struct work_struct *work)
{
	struct panel_drv_data *ddata = container_of(work,
		struct panel_drv_data, hralvds_hdcp_auth_retry_work.work);

	dev_err(&ddata->ts_i2c_client->dev, "HDCP AUTH PASS event was not "
		"received after HDCP AUTH FAIL timeout. "
		"Resetting the display.\n");

	/*
	 * Cancel any power on sequencing that may be in progress and force
	 * the display to reset.
	 */
	cancel_delayed_work_sync(&ddata->hralvds_power_state_work);
	ddata->power_state = POWER_STATE_OFF;
	ddata->reset_retries = 0;
	ddata->gradual_change_retries = 0;
	gpio_set_value(ddata->dispc_gpio, 0);

	msleep(HRALVDS_DISPLAY_RESET_CYCLE_TIME_MSEC);

	/*
	 * Power the display back on again.  The HDCP AUTH will be retried
	 * as part of the power on sequencing.
	 */
        ddata->power_state = POWER_STATE_POWERING_ON_1;
        gpio_set_value(ddata->dispc_gpio, 1);
}

static void hralvds_hdcp_worker(struct work_struct *work)
{
	struct panel_drv_data *ddata = container_of(work,
		struct panel_drv_data, hralvds_hdcp_work.work);

	if (ddata->hdcp_enable != ddata->hdcp_previous_state) {

		ddata->hdcp_previous_state = ddata->hdcp_enable;
		hralvds_hdcp_change_status(ddata);

	}

	queue_delayed_work(ddata->workq,
				&ddata->hralvds_hdcp_work,
				HRALVDS_HDCP_WORKQ_TIMEOUT);
}


/*
 * send the "normal" packet for a display update: i.e. not the first packet.
 * these packets contain just the frag offset and the data.
 */
static int hralvds_display_update_send_packet(struct panel_drv_data *ddata)
{
	int r = 1;
	u32 xfer_len;
	u32 u;

	u8 *buf;
	u8 *p;

	buf = kmalloc(HRALVDS_UPDATE_MAX_PACKET_SIZE + 1, GFP_KERNEL);
	p = buf;

	/*
	 * before changing ddata->fwupd.offs, grab a copy in case of an error
	 *	that requires us to resend this packet.
	 */
	ddata->fwupd.last_offs = ddata->fwupd.offs;

	/*
	 * first byte is always the UPDATE TRANSMISSION COMMAND.
	 */
	*p++ = HRALVDS_UPDATE_TRANSMISSION_CMD;

	/*
	 * next four bytes are the fragment offset.
	 */
	u = cpu_to_be32(ddata->fwupd.offs);
	memcpy(p, &u, sizeof(u));
	p += sizeof(u);

	/*
	 * if this is the first packet of the transfer, we have to insert
	 *	the overall file transfer size (not including the header),
	 *	and the update file header (first N bytes of the update file).
	 */
	if (0 == ddata->fwupd.offs) {
		u = cpu_to_be32(ddata->fwupd.data_size);
		memcpy(p, &u, sizeof(u));
		p += sizeof(u);

		/*
		 * copy the update file header.
		 * it's in the very front of the firmware data file.
		 */
		memcpy(p, ddata->fwupd.buf, HRALVDS_UPDATE_FILE_HEADER_SIZE);
		p += HRALVDS_UPDATE_FILE_HEADER_SIZE;
	}

	/*
	 * compute the number of bytes to transfer.
	 * assume we're going to send the maximum per-packet data that is
	 *	specified by the protocol.
	 */
	xfer_len = HRALVDS_UPDATE_MAX_DATA_SIZE;

	/*
	 * if we run past the end of the data, truncate the count.
	 */
	if (ddata->fwupd.data_size - ddata->fwupd.offs < xfer_len)
		xfer_len = ddata->fwupd.data_size - ddata->fwupd.offs;

	/*
	 * (sigh) copy the data.
	 */
	memcpy(p, ddata->fwupd.data + ddata->fwupd.offs, xfer_len);

	p += xfer_len;

	/*
	 * add the checksum.
	 * p - buf is the number of bytes that have been placed into the
	 *	buffer.
	 */
	*p = ~hralvds_cksum(buf, p - buf, NULL, 0) + 1;

	/*
	 * send the data to the display.
	 */
	r = hralvds_i2c_cmd(ddata->ts_i2c_client, buf, p - buf + 1);

	if (r > 0) {
		ddata->fwupd.offs += xfer_len;

#if defined HRALVDS_STORE_RAW_I2C_DATA
		/*
		 * store the raw i2c message to the raw file.
		 */
		if (ddata->rawfp) {
			mm_segment_t old_fs = get_fs();

			set_fs(KERNEL_DS);
			vfs_write(ddata->rawfp, buf, p - buf + 1,
							&ddata->rawfp->f_pos);
			set_fs(old_fs);
		}
#endif
	}

	kfree(buf);

	return r;
}

/*
 * called from one of the vehicle-specific panel driver's irq handler
 *	to decode & process a Display Status Notification interrupt message.
 * three possible values can come back:
 *	0:	normal (display has switched back to normal mode)
 *		this could be an error case, but isn't likely to occur
 *		because the display's flash is cleared when the reprogramming
 *		begins.
 *	1:	reprogramming ready (start the update messages)
 *		we expect this after we send the Update Indication command.
 *	2:	reprogramming fail (switch back to normal mode?)
 *		in this case, we restart the programming back at the beginning.
 *		set the file offset to zero and continue the flashing process.
 *		in the event the FCB restarts, we have to find a way to
 *		determine what mode the display is in.
 */
void hralvds_display_status_notification(struct panel_drv_data *ddata)
{
	/*
	 * NOTE: when the FCB restarts after a display update has been
	 *	started, this interrupt will fire from the display
	 *	after a supervisory frame has been exchanged with the
	 *	display.  This function must handle the recovery of that
	 *	update.
	 */

	/*
	 * TODO: this needs to be able to generically handle this message
	 *	and must take into account the current state of the display
	 *	and the display update state.
	 * So the current code below needs to be a bit smarter and not just
	 *	dive into the reprogramming process.
	 */

	__u8 wbuf[] = { HRALVDS_REPORT_ID_DISPLAY_STATUS_NOTIFICATION };
	__u8 rbuf[HRALVDS_REPORT_ID_DISPLAY_STATUS_NOTIFICATION_SIZE + 1] =
									{ 0 };
	int r;

	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
		wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 1, 3);

	if (2 == r) {
		switch(rbuf[0]) {
		case HRALVDS_DISPLAY_STATUS_NOTIFICATION_NORMAL:
			/*
			 * the display is in normal mode, not in
			 *	reprogramming mode.
			 *
			 * according to the spec, this means that
			 *	either the reprogramming was
			 *	successful, or the file header
			 *	information is incorrect.
			 *
			 * so, processing here consists of setting a
			 *	flag indicating that the display has
			 *	reported that it is done with the
			 *	update.
			 */
			ddata->fwupd.display_update_done = 1;
			break;

		case HRALVDS_DISPLAY_STATUS_NOTIFICATION_READY:
			/*
			 * according to Section 5.1.2.2 of the spec,
			 *	after the command is sent to tell the
			 *	display to start the upload,
			 *	this interrupt will be received.
			 * it can also happen after a reprogramming
			 *	restart.
			 * the update_state will already be
			 *	WAIT FOR START ACK
			 */
			ddata->fwupd.update_ready = 1;
			break;

		case HRALVDS_DISPLAY_STATUS_NOTIFICATION_FAILED:
			/*
			 * some kind of programming error occurred:
			 * restart the reprogramming operation, as
			 *	though "begin" was written to the
			 *	hralvds_fwif_control file.
			 */
			dev_err(&ddata->ts_i2c_client->dev,
				"Display reports reprogramming failed;"
				" restart update.");
			ddata->fwupd.update_reason =
				DISPLAY_UPDATE_REASON_FAILURE;
			ddata->fwupd.update_state = DISPLAY_UPDATE_LOAD;
			break;
		}
	}
}

/*
 * hralvds_update_check_status sends an UPDATE STATUS command to the display
 *	and returns one of four values:
 *		HRALVDS_UPDATE_STATUS_OK	- all is well
 *		HRALVDS_UPDATE_STATUS_BUSY	- try again in 500msec
 *		HRALVDS_UPDATE_STATUS_ERROR	- failure (what to do?)
 *		-EINVAL				- invalid reply (what to do?)
 */
static int hralvds_update_check_status(struct panel_drv_data *ddata)
{
	/* buffer to send contains only the UPDATE STATUS command byte. */
	__u8 wbuf[] = { HRALVDS_UPDATE_STATUS_CMD };
	/* buffer to receive the response in. '+ 1' for the checksum. */
	__u8 rbuf[HRALVDS_UPDATE_STATUS_SIZE + 1] = { 0 };
	int r;
	int status = -EINVAL;

	/*
	 * read the update status from the display.
	 * use up to 3 retries.
	 */
	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
			wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 1, 3);

	if (2 == r)
		status = rbuf[0];

	return status;
}

/*
 * hralvds_update_cleanup cleans up the fw update state variables.
 */
static void hralvds_update_cleanup(struct panel_drv_data *ddata)
{
	ddata->fwupd.update_state = DISPLAY_UPDATE_IDLE;
	ddata->fwupd.display_update_done = 0;
	if (ddata->fwupd.buf)
		vfree(ddata->fwupd.buf);
	ddata->fwupd.buf = NULL;
	ddata->fwupd.data = NULL;
	ddata->fwupd.size = 0;
	ddata->fwupd.data_size = 0;
	ddata->fwupd.offs = 0;
}

static void hralvds_display_update_load(struct panel_drv_data *ddata)
{
	int r;

	/*
	 * in the event of a failed update, cancel any work that might
	 *	be going on to manage the power state.  this could
	 *	cause the display to be powered off during an update.
	 * we don't need to be more discriminating here.  no matter
	 *	what else is going on, we don't want the power state
	 *	machine to turn off the display.
	 */
	cancel_delayed_work_sync(&ddata->hralvds_power_state_work);
	ddata->power_state = POWER_STATE_ON;
	ddata->reset_retries = 0;
	ddata->gradual_change_retries = 0;

	/*
	 * this is the start stimulus.
	 * we're here for one moment and immediately transition out.
	 */
	dev_info(&ddata->ts_i2c_client->dev, "Update load state.");

	if (hralvds_begin_update(ddata) == 0) {
		u8 wbuf[3] = { 0 };

		dev_info(&ddata->ts_i2c_client->dev,
			"Update load successful; sending UPDATE START.");

		/*
		 * clear update_ready before sending UPDATE INDICATION START
		 *	so that we avoid a race condition.
		 */
		ddata->fwupd.update_ready = 0;
		ddata->fwupd.transfer_complete = 0;
		ddata->fwupd.update_state = DISPLAY_UPDATE_WAIT_FOR_READY;
		ddata->fwupd.wait_for_rr_jiffies = jiffies;

		/*
		 * message must have a checksum in it.
		 */
		wbuf[0] = HRALVDS_UPDATE_INDICATION_CMD;
		wbuf[1] = HRALVDS_UPDATE_INDICATION_START;
		wbuf[2] = 1 + ~hralvds_cksum(wbuf, sizeof(wbuf) - 1, NULL, 0);

		/*
		 * send the Update Indication command to alert the display that
		 *	an update is coming.
		 */
		r = hralvds_i2c_cmd(ddata->ts_i2c_client, wbuf, sizeof(wbuf));

		/*
		 * we sent one message, so we should get a return of 1.
		 */
		if (1 == r) {
			/*
			 * successfully sent the Update Indication command.
			 * set the main display_state to UPDATE, which prevents
			 *	sending supervisor messages and such.
			 */
			ddata->display_state = DISPLAY_UPDATE;

			dev_info(&ddata->ser_i2c_client->dev,
				"Notified display that update is starting.");
		} else {
			/*
			 * for whatever reason, we weren't able to send the
			 *	Update Indication command.
			 * note that at this point, display_state has NOT BEEN
			 *	MODIFIED, and so on return from this function,
			 *	the poll_counter will be updated.
			 * after the update process is complete, a restart of
			 *	some kind will be performed and so any
			 *	necessary reset of the polling counter/state
			 *	will be done then.
			 */

			/*
			 * update_cleanup will handle freeing the firmware
			 *	buffer and resetting the update state machine.
			 */
			hralvds_update_cleanup(ddata);

			dev_warn(&ddata->ser_i2c_client->dev,
				"Update start returned %d; "
				"terminating update.", r);
		}
	} else {
		/*
		 * hralvds_begin_update() returned non-zero.
		 *
		 * some error occurred while trying to load the update file.
		 *	this is potentially catastrophic because if we began
		 *	the update process due to an automatic restart of an
		 *	interrupted update, we could be stuck forever.
		 *
		 * since the protocol doesn't give us another action to perform
		 *	in this case, we will simply wait and retry until we
		 *	either timeout or we succeed.
		 *
		 * in the case of DISPLAY_UPDATE_REASON_FAILURE, we will simply
		 *	allow the main display update state machine to call
		 *	this function again, since
		 *	update_state != DISPLAY_UPDATE_IDLE.
		 *	we will again invoke this function.
		 */
		switch (ddata->fwupd.update_reason) {
		case DISPLAY_UPDATE_REASON_USER:
			dev_err(&ddata->ts_i2c_client->dev,
				"Update file load failed; "
				"terminating update.");
			ddata->fwupd.update_state = DISPLAY_UPDATE_IDLE;
			break;

		case DISPLAY_UPDATE_REASON_FAILURE:
			dev_err(&ddata->ts_i2c_client->dev,
				"Failed to load update file; "
				"try again.");
			break;

		default:
			break;
		}
	}
}

static void hralvds_display_update_wait_for_ready(struct panel_drv_data *ddata)
{
	/*
	 * Wait state for an ACK from display that it has switched modes.
	 */
	dev_info(&ddata->ts_i2c_client->dev,
		"Waiting for REPROGRAMMING READY.");

	if (ddata->fwupd.update_ready) {
		/*
		 * at this point, we stop sending supervisor frames to the
		 *	display, because it has reported REPROGRAMMING READY.
		 *
		 * at this point, when we leave this state machine we will fall
		 *	through into the display_state state machine and send
		 *	the first update packet.
		 */
		ddata->fwupd.update_state = DISPLAY_UPDATE_TRANSFER;

		dev_info(&ddata->ts_i2c_client->dev,
			"Notification received from display to "
			"start reprogramming; beginning transfer.");

	} else if (jiffies - ddata->fwupd.wait_for_rr_jiffies >=
					msecs_to_jiffies(500)) {
		/*
		 * 500msec has elapsed and the display did not acknowledge the
		 *	update.
		 */
		hralvds_update_cleanup(ddata);
		hralvds_reset_state(ddata);

		dev_warn(&ddata->ts_i2c_client->dev,
			"Timeout for REPROGRAMMING READY "
			"elapsed; terminating update (%u msec)",
			jiffies_to_msecs(jiffies -
				ddata->fwupd.wait_for_rr_jiffies));
	}

}

static void hralvds_display_update_transfer(struct panel_drv_data *ddata)
{
	int r;

	/*
	 * If needed, send the next block.
	 * When the data is all sent, the display will send an interrupt, which
	 *	we will use to finish the update processing (free buffers,
	 *	reset state, etc).
	 */
	if (ddata->fwupd.offs < ddata->fwupd.data_size) {
		if (ddata->fwupd.display_update_done) {
			/*
			 * we received a Normal packet during the middle of the
			 *	update.
			 * according to the protocol, this means that the
			 *	display got a bad file header.  go back to
			 *	normal mode.
			 */
			hralvds_update_cleanup(ddata);
			hralvds_reset_state(ddata);

			dev_err(&ddata->ts_i2c_client->dev,
				"Bad file header reported by the "
				"display.  Terminating update.");
		} else {
			/*
			 * initiate the next transfer.
			 */
			r = hralvds_display_update_send_packet(ddata);

			if (r > 0) {
				ddata->fwupd.update_state = DISPLAY_UPDATE_WAIT;

				/*
				 * a successful transmission; clear the i2c
				 *	error count.
				 */
				ddata->fwupd.i2c_error_count = 0;
			} else {
				/*
				 * if there's an error sending this packet,
				 *	resend it next time instead of a new
				 *	packet.
				 */
				ddata->fwupd.offs = ddata->fwupd.last_offs;
				dev_err(&ddata->ts_i2c_client->dev,
					"Error sending update packet, "
					"r=%d; resending from offset %u.",
					r, ddata->fwupd.last_offs);
				if (++ddata->fwupd.i2c_error_count >=
					DISPLAY_UPDATE_MAX_I2C_ERRORS) {
					dev_err(&ddata->ts_i2c_client->dev,
						"Too many i2c errors; "
						"aborting update.");
					hralvds_update_cleanup(ddata);
					hralvds_reset_state(ddata);
				}
			}
		}
	} else if (!ddata->fwupd.transfer_complete) {

#if defined HRALVDS_STORE_RAW_I2C_DATA
		/*
		 * clean up the raw binary file.
		 */
		if (ddata->rawfp) {
			filp_close(ddata->rawfp, NULL);
			ddata->rawfp = NULL;
			dev_info(&ddata->ts_i2c_client->dev,
				"Closing update file.");
		}
#endif

		/*
		 * the transfer is complete.
		 * the protocol requires an update status message before it
		 *	will generate the display notification status interrupt.
		 * enter UPDATE_FIN1 which will generate a 100msec delay and
		 *	then transition to the CHECK STATUS state.  control
		 *	will come back here and the state will stay
		 *	UPDATE TRANSFER.
		 */
		ddata->fwupd.update_state = DISPLAY_UPDATE_FIN1;
		ddata->fwupd.transfer_complete = 1;
	}
}

static void hralvds_display_update_wait(struct panel_drv_data *ddata)
{
	/*
	 * at this point, 50msec has already passed.
	 * switch back to the UPDATE_TRANSFER state so that in another 50msec,
	 *	a total of 100msec will have elapsed.
	 * should the protocol change and more than 100msec is necessary, a
	 *	wait counter will have to be added.
	 */

	/*
	 * if we're due to send an UPDATE STATUS command, do it
	 *	now.
	 */
	if (ddata->fwupd.status_pkt_count < HRALVDS_UPDATE_STATUS_DELAY) {
		ddata->fwupd.status_pkt_count++;
		ddata->fwupd.update_state = DISPLAY_UPDATE_TRANSFER;
	} else {
		ddata->fwupd.status_pkt_count = 0;
		ddata->fwupd.busy_delay_counter = 0;
		ddata->fwupd.update_state = DISPLAY_UPDATE_CHECK_STATUS;
	}
}

static void hralvds_display_update_check_status(struct panel_drv_data *ddata)
{
	int r;

	/*
	 * check the update status.
	 * if busy, we have to wait 500msec before we try again.
	 */
	r = hralvds_update_check_status(ddata);

	switch (r) {
	case HRALVDS_UPDATE_STATUS_OK:
		/*
		 * is there more to send?
		 */
		if (ddata->fwupd.offs < ddata->fwupd.data_size) {
			/*
			 * more to send, go back to the UPDATE TRANSFER
			 *	state.
			 */
			ddata->fwupd.update_state =
				DISPLAY_UPDATE_TRANSFER;

			/*
			 * we clear the busy count whenever we have a
			 *	successful update response.
			 * the BUSY error case is when we received the
			 *	BUSY message X times IN A ROW.
			 *	(according to Mohit Kumar, in an email
			 *	dated 25 July 2016).
			 */
			ddata->fwupd.busy_count = 0;

			/*
			 * display has acknowledged everything up
			 *	to this offs.
			 */
			ddata->fwupd.last_ackd_offs = ddata->fwupd.offs;

			dev_info(&ddata->ts_i2c_client->dev,
				"Update status OK, continue transfer "
				"@ offset %u", ddata->fwupd.offs);
		} else {
			/*
			 * all the data has been transferred, and the
			 *	display reports UPDATE STATUS OK.
			 * UPDATE FIN2 will wait until the display
			 *	sends the interrupt indicating the
			 *	transfer has been successful.
			 */
			ddata->fwupd.update_state = DISPLAY_UPDATE_FIN2;
			dev_info(&ddata->ts_i2c_client->dev,
				"Transfer complete, waiting for "
				"final interrupt.");
		}
		break;

	case HRALVDS_UPDATE_STATUS_BUSY:
		ddata->fwupd.busy_count++;

		/*
		 * check the busy count, and if its limit has not
		 *	been reached, continue; else restart the
		 *	programming sequence.
		 *
		 * the LVDS spec specifies that the ICU should start
		 *	at offset 0, but it does NOT say that the ICU
		 *	is supposed to retransmit the first packet
		 *	with the size and the file header.
		 */
		if (ddata->fwupd.busy_count <
				HRALVDS_UPDATE_STATUS_MAX_BUSY) {
			/*
			 * we got a busy response, but we haven't
			 *	had too many, so we have to wait
			 *	and retry the UPDATE STATUS command.
			 */
			ddata->fwupd.update_state =
				DISPLAY_UPDATE_CHECK_STATUS_BUSY_WAIT;
			ddata->fwupd.busy_delay_counter = 0;
			dev_info(&ddata->ts_i2c_client->dev,
				"Update status BUSY (%u/%d); "
				"wait and re-check.",
				ddata->fwupd.busy_count,
				HRALVDS_UPDATE_STATUS_MAX_BUSY);
		} else {
			/*
			 * we're supposed to start the reprogramming
			 *	over at frag offset 0.
			 */
			dev_info(&ddata->ts_i2c_client->dev,
				"Update status BUSY too many times; "
				"restart reprogramming.");
			ddata->fwupd.offs = 0;
			ddata->fwupd.update_state =
				DISPLAY_UPDATE_TRANSFER;
		}
		break;

	case HRALVDS_UPDATE_STATUS_ERROR:
		/*
		 * rewind to the last ack'd offset and continue.
		 * last_ackd_offs remains unchanged.
		 */
		ddata->fwupd.offs = ddata->fwupd.last_ackd_offs;
		ddata->fwupd.update_state = DISPLAY_UPDATE_TRANSFER;
		dev_err(&ddata->ts_i2c_client->dev,
			"Display reports reprogramming error; "
			"restarting at offset %u.",
			ddata->fwupd.offs);

		break;

	default:
		/*
		 * not sure what to do except continue.
		 */
		dev_err(&ddata->ts_i2c_client->dev,
			"Invalid response for UPDATE STATUS "
			"command = 0x%02hhx; trying to continue.", r);
		ddata->fwupd.update_state = DISPLAY_UPDATE_TRANSFER;
		break;
	}
}

static void hralvds_display_update_check_status_busy_wait(
	struct panel_drv_data *ddata)
{
	ddata->fwupd.busy_delay_counter++;
	if (DISPLAY_UPDATE_BUSY_WAIT_COUNT == ddata->fwupd.busy_delay_counter)
		ddata->fwupd.update_state = DISPLAY_UPDATE_CHECK_STATUS;
}

static void hralvds_display_update_fin1(struct panel_drv_data *ddata)
{
	/*
	 * we've waited 50msec from transfer complete til now.
	 * next time we come through it will have been 100msec since the
	 *	transfer was complete.
	 * set state to DISPLAY_UPDATE_CHECK_STATUS and check the update status.
	 * if the status comes back OK, state will then be set to
	 *	DISPLAY_UPDATE_TRANSFER, and we will test for the interrupt
	 *	indicating that the transfer is now complete.
	 */
	ddata->fwupd.update_state = DISPLAY_UPDATE_CHECK_STATUS;
	dev_info(&ddata->ts_i2c_client->dev,
		"Last packet transferred; going to send final UPDATE STATUS.");
}

static void hralvds_display_update_fin2(struct panel_drv_data *ddata)
{
	/*
	 * the state machine will stay in this state until the display sends
	 *	the DISPLAY UPDATE NOTIFICATION interrupt.  this is signalled
	 *	by display_update_done being set.
	 */
	if (ddata->fwupd.display_update_done) {
		/*
		 * the display has reported the the update is done.
		 * it is time to release the firmware image.
		 *
		 * note: this could also be done with a completion variable.
		 *	however, there is no spec that indicates how long it
		 *	has to take for the display to generate the interrupt
		 *	indicating completion.
		 */
		hralvds_update_cleanup(ddata);
		hralvds_reset_state(ddata);
		dev_info(&ddata->ts_i2c_client->dev,
			"Display signalled update finished.");
	}
}

/*
 * run the update state machine.
 *
 * unfortunately, there is some complicated interaction between the
 *	update state machine and the display state machine.
 *	the update state is a state in the display state machine, but it's
 *	broken out into its own function because it's somewhat complex.
 *	that means that there are cases where the update state machine may
 *	have to manipulate the display state machine, in the case of error
 *	recovery.
 */
static void hralvds_display_update_worker(struct panel_drv_data *ddata)
{
	/*
	 * at the start of the firmware update process, we still need to
	 *	send supervisor messages.
	 * track the firmware update state here.
	 */
	switch (ddata->fwupd.update_state) {
	case DISPLAY_UPDATE_IDLE:
		/*
		 * we don't actually ever see this case here, because
		 *	hralvds_timer_worker() doesn't call this function
		 *	unless update_state != DISPLAY_UPDATE_IDLE
		 */
		break;

	case DISPLAY_UPDATE_LOAD:
		hralvds_display_update_load(ddata);
		break;

	case DISPLAY_UPDATE_WAIT_FOR_READY:
		hralvds_display_update_wait_for_ready(ddata);
		break;

	case DISPLAY_UPDATE_TRANSFER:
		hralvds_display_update_transfer(ddata);
		break;

	case DISPLAY_UPDATE_WAIT:
		hralvds_display_update_wait(ddata);
		break;

	case DISPLAY_UPDATE_CHECK_STATUS:
		hralvds_display_update_check_status(ddata);
		break;

	case DISPLAY_UPDATE_CHECK_STATUS_BUSY_WAIT:
		hralvds_display_update_check_status_busy_wait(ddata);
		break;

	case DISPLAY_UPDATE_FIN1:
		hralvds_display_update_fin1(ddata);
		break;

	case DISPLAY_UPDATE_FIN2:
		hralvds_display_update_fin2(ddata);
		break;
	}
}

/*
 * hralvds_timer_worker:
 *
 * this function manages the communications with the display:
 *	either the display is being updated, or the supervisor state is being
 *	managed, or the display communications are being initialized.
 */
static void hralvds_timer_worker(struct work_struct *work)
{
	struct panel_drv_data *ddata = container_of(work,
		struct panel_drv_data, hralvds_timer_work.work);
	int r;

	switch (ddata->display_state) {
	case DISPLAY_POLL:
		/*
		 * next state will either be DISPLAY_POLL_WAIT or it will
		 *	be DISPLAY_INIT in the event the supervisor
		 *	failure count is met.
		 * it's safe to set DISPLAY_POLL_WAIT because there's no
		 *	path out of this case where the display_state
		 *	is unchanged.
		 */
		ddata->display_state = DISPLAY_POLL_WAIT;
		if (0 == (r = hralvds_do_supervisor(ddata))) {
			/*
			 * in case there were failures, there aren't now.
			 */
			ddata->supervisor_ok = 1;
			ddata->supervisor_failure_counter = 0;
		} else {
			/*
			 * no response to the supervisor message.
			 * increment the failure count and check to see if the
			 *	limit has been exceeded.
			 */
			dev_info(&ddata->ts_i2c_client->dev,
				"No response to supervisor message (%d).", r);

			++ddata->supervisor_failure_counter;
			if (ddata->supervisor_failure_counter >=
					HRALVDS_SUPERVISOR_FAILURE_LIMIT) {
				/*
				 * we didn't get a response too many times in a
				 *	row: we have no comms with the display.
				 *
				 * turn off the IS_RX_INT (disable display
				 *	interrupts) then reset the state
				 *	machine.
				 *
				 * the DISPLAY_INIT state will look for lock and
				 *	restart.
				 */
				r = i2c_read_le8(ddata->ser_i2c_client,
					SER925_HDCP_ICR);
				if (r > 0) {

					r &= ~SER925_HDCP_ICR_IS_RX_INT;

					i2c_write_le8(
						ddata->ser_i2c_client,
						SER925_HDCP_ICR, r);
				}

				dev_info(&ddata->ts_i2c_client->dev,
					"Too many supervisor timeouts.");

				/* reset the state on error */
				hralvds_reset_state(ddata);
			}
		}
		break;

	case DISPLAY_POLL_WAIT:
		/*
		 * if the update_state is changed to DISPLAY_UPDATE_LOAD,
		 *	we switch display_state to the DISPLAY_UPDATE state.
		 * in order to speed up the process, call
		 *	hralvds_display_update_worker() for the first time
		 *	from this function.
		 * in the event that the file isn't loaded, we will end up
		 *	resetting the state.
		 *
		 * NOTE the following side effect to this state machine:
		 * if the file is loaded, display_state will be set to
		 *	DISPLAY_UPDATE and the update worker will be invoked
		 *	from that state (below) until the update is complete
		 *	or is otherwise terminated.
		 *
		 * also note that update_state changes asynchronously to
		 *	this function.
		 */
		if (DISPLAY_UPDATE_IDLE != ddata->fwupd.update_state)
			/*
			 * MAY change display_state:
			 */
			hralvds_display_update_worker(ddata);

		/*
		 * if we haven't switched states, continue with the POLL_WAIT
		 *	processing.
		 */
		if (DISPLAY_UPDATE != ddata->display_state) {
			/*
			 * Make the display_detected flag more responsive by
			 *	polling it roughly every 100msec.  We don't
			 *	want to do it too often, as it consumes extra
			 *	bandwidth on the i2c bus.
			 */
			if ((ddata->poll_counter & 1)) {
				r = i2c_read_le8(ddata->ser_i2c_client,
							SER925_GEN_STS);
				ddata->display_detected =
					(r & SER925_GEN_STS_LINK_DETECT) != 0;
			}

			if (0 == --ddata->poll_counter) {
				ddata->poll_counter =
					HRALVDS_SUPERVISOR_POLL_COUNT;
				ddata->display_state = DISPLAY_POLL;
			}
		}
		break;

	case DISPLAY_INIT:
		/*
		 * check to see if we have a valid link.
		 * if so, bring up the link.
		 */
		r = i2c_read_le8(ddata->ser_i2c_client, SER925_GEN_STS);

		if ((r & SER925_GEN_STS_LINK_DETECT)) {
			r = hralvds_setup_link(ddata);

			if (r >= 0) {
				r = hralvds_setup_deser_alias(ddata);

				if (r < 0) {
					dev_err(&ddata->ser_i2c_client->dev,
						"Can't set deser alias");
					return;
				}

				r = hralvds_set_handshake(ddata);

				/*
				 * set the serializer alias back to the
				 *	touchscreen.
				 */
				hralvds_setup_ts_alias(ddata);

				if (r >= 0) {
					/*
					 * The handshake is complete, so at
					 *	least we have seen the display.
					 *	We don't have supervisor comms
					 *	yet, but at least the display
					 *	had set the handshake bit, and
					 *	we reciprocated.
					 */
					ddata->display_detected = 1;

					/*
					 * Enable the receiver interrupt for
					 *	display touch, knob and button
					 *	events.
					 */
					r = i2c_read_le8(ddata->ser_i2c_client,
						SER925_HDCP_ICR);
					if (r >= 0) {
						r |= SER925_HDCP_ICR_IS_RX_INT;

						i2c_write_le8(
							ddata->ser_i2c_client,
							SER925_HDCP_ICR, r);
					}

					/*
					 * initialization successful:
					 *	go to polling wait mode
					 * we have a little wait to allow the
					 *	display to wake up before we
					 *	send the supervisor message
					 *	to it.  2 * the normal timeout
					 *	of 500 msec is 1 second, which
					 *	is enough time for the display
					 *	to be fully up.
					 */
					ddata->poll_counter = 2 *
						HRALVDS_SUPERVISOR_POLL_COUNT;
					ddata->display_state =
						DISPLAY_POLL_WAIT;

					/*
					 * Switch the polling interval to the
					 *	slower WORKQ_POLL timeout.
					 */
					ddata->timer_worker_delay =
						HRALVDS_SUPERVISOR_WORKQ_POLL_TIMEOUT;

					/*
					 * Wait until panel is fully up before
					 * sending the first video and power
					 * commands.
					 */
					hralvds_set_power_state(
						ddata, __func__,
						POWER_STATE_POWERING_ON_1);
					ddata->power_cmd_processed = 0;
					hralvds_power_state_set_timeout(ddata);
				} else {
					dev_err(&ddata->ts_i2c_client->dev,
						"set_handshake() failed.");
				}
			} else {
				if (__ratelimit(&ddata->setup_link_rs))
					dev_err(&ddata->ts_i2c_client->dev,
						"setup_link() failed.");
			}
		}
		break;

	case DISPLAY_UPDATE:
		hralvds_display_update_worker(ddata);
		break;
	}

	queue_delayed_work(ddata->workq, &ddata->hralvds_timer_work,
		ddata->timer_worker_delay);
}

/*
 * hralvds_power_state_set_timeout:
 *
 * Set the command sequencing interval or retry timeout depending on the
 * power state.
 */
static void hralvds_power_state_set_timeout(struct panel_drv_data *ddata)
{
	unsigned long timeout;

	switch (ddata->power_state) {
	case POWER_STATE_POWERING_OFF:
		/*
		 * 1 second timeout for powering off, as there is a 800 msec
		 * gradual change.
		 */
		timeout = msecs_to_jiffies(MSEC_PER_SEC);
		break;

	case POWER_STATE_POWERING_OFF_CHECK_ON:
		/*
		 * From the OFF state, wait the 100 msec duty cycle and
		 * check if the user requested the display controller to
		 * go back on again.
		 */
		timeout = msecs_to_jiffies(
			HRALVDS_DISPLAY_RESET_CYCLE_TIME_MSEC);
		break;

	case POWER_STATE_POWERING_ON_1:
		/*
		 * Time to wait in between when the deserializer link has been
		 * established and when to send the video and power commands
		 * to allow the panel to fully initialize.
		 */
		timeout = msecs_to_jiffies(500);
		break;

	case POWER_STATE_POWERING_ON_1A:
		/*
		 * Wait 50 msec before transitioning to state 1A.
		 * At this time, we're going to send a reset command to the
		 *	deserializer.
		 */
		timeout = msecs_to_jiffies(50);
		break;

	case POWER_STATE_POWERING_ON_2:
		/* Powering on always has a 800 msec gradual change time */
		timeout = msecs_to_jiffies(MSEC_PER_SEC);
		break;

	case POWER_STATE_ON:
		/*
		 * 1000 msec timeout for 800 msec gradual change.
		 *  500 msec timeout for 200 msec gradual change.
		 */
		timeout = msecs_to_jiffies(
			ddata->cmd_power.power.ctrl & (1 << 5) ? 1000 : 500);
		break;

	default:
		timeout = 0;
		break;
	}

	if (timeout) {
		queue_delayed_work(ddata->workq,
				&ddata->hralvds_power_state_work, timeout);
	}
}


/*
 * It's possible the user turned the display back on during the POWERING_OFF
 * state (up to 1 second window).
 */
static void hralvds_check_requested_dispc_state(struct panel_drv_data *ddata)
{
	if (ddata->requested_dispc_state) {
		/*
		 * Wait the duty cycle time (100 msec), and power the
		 * display back on.
		 */
		hralvds_set_power_state(ddata, __func__,
			POWER_STATE_POWERING_OFF_CHECK_ON);
		hralvds_power_state_set_timeout(ddata);
	}
}

/*
 * hralvds_on_finish_gradual_time_change:
 *
 * Manage the power state transition when the panel acknowledges that the
 * gradual time change has been finished.
 *
 * this needs to be exposed to outside modules because it's called from the
 * panel IRQ function.
 */
void hralvds_on_finish_gradual_change(struct panel_drv_data *ddata)
{
	struct device *dev = &ddata->ts_i2c_client->dev;

	dev_info(dev, "Gradual change finished.\n");

	cancel_delayed_work_sync(&ddata->hralvds_power_state_work);
	ddata->reset_retries = 0;
	ddata->gradual_change_retries = 0;

	switch (ddata->power_state) {
	case POWER_STATE_POWERING_OFF:
		/* Complete power off sequence by setting DISPC to low */
		dev_info(dev, "Powering off display controller.\n");
		hralvds_set_power_state(ddata, __func__, POWER_STATE_OFF);
		ddata->power_cmd_processed = 0;
		gpio_set_value(ddata->dispc_gpio, 0);
		hralvds_check_requested_dispc_state(ddata);
		break;

	case POWER_STATE_POWERING_ON_1:
	case POWER_STATE_POWERING_ON_1A:
	case POWER_STATE_POWERING_ON_2:
		dev_info(dev, "Power on sequence completed\n");
		hralvds_set_power_state(ddata, __func__, POWER_STATE_ON);
		/* fallthrough */
	case POWER_STATE_ON:
		ddata->power_cmd_processed = 1;
		break;

	default:
		break;
	}
}

/*
 * hralvds_send_video_ts_and_power:
 *
 * Send the video, touch sensitivity, and power commands and set timeout.
 */
static void hralvds_send_video_ts_and_power(struct panel_drv_data *ddata)
{
	struct hralvds_power_cmd_struct *p = &ddata->cmd_power;
	struct hralvds_video_cmd_struct *v = &ddata->cmd_video;
	struct hralvds_touch_sensitivity_cmd_struct *ts =
		&ddata->cmd_touch_sensitivity;
	struct hralvds_power_cmd_struct tmp_power;

	/* Only send the video command if the panel is powering on. */
	switch (ddata->power_state) {
	case POWER_STATE_POWERING_ON_1:
		break;

	case POWER_STATE_POWERING_ON_1A:
	case POWER_STATE_POWERING_ON_2:
		mutex_lock(&ddata->cmd_video_lock);
		hralvds_i2c_cmd(ddata->ts_i2c_client, (__u8 *)v, sizeof(*v));
		mutex_unlock(&ddata->cmd_video_lock);

		mutex_lock(&ddata->cmd_touch_sensitivity_lock);
		hralvds_i2c_cmd(ddata->ts_i2c_client,
				(__u8 *)ts, sizeof(*ts));
		mutex_unlock(&ddata->cmd_touch_sensitivity_lock);
		break;

	default:
		break;
	}

	if (!atomic_read(&ddata->fast_init)) {
		mutex_lock(&ddata->cmd_power_lock);
		memcpy(&tmp_power, p, sizeof(tmp_power));
		mutex_unlock(&ddata->cmd_power_lock);

		/* Force 800 msec gradual time change if powering on */
		switch (ddata->power_state) {
		case POWER_STATE_POWERING_ON_1A:
		case POWER_STATE_POWERING_ON_2:
			tmp_power.power.ctrl |= 1 << 5;
			tmp_power.checksum = ~hralvds_cksum(
				(__u8 *)&tmp_power, sizeof(tmp_power) - 1,
				NULL, 0) + 1;
			break;

		default:
			break;
		}

		hralvds_i2c_cmd(ddata->ts_i2c_client,
				(__u8 *)&tmp_power, sizeof(tmp_power));

		hralvds_power_state_set_timeout(ddata);
	}
}

/*
 * hralvds_power_state_power_on_check:
 *
 * We're now in the OFF state, but check if the user turned the display
 * controller back on while we were powering off.
 */
static void hralvds_power_state_powering_off_check_on(
	struct panel_drv_data *ddata)
{
	hralvds_set_power_state(ddata, __func__, POWER_STATE_OFF);
	if (ddata->requested_dispc_state) {
		ddata->gradual_change_retries = 0;
		ddata->reset_retries = 0;
		hralvds_set_power_state(ddata, __func__,
			POWER_STATE_POWERING_ON_1);
		gpio_set_value(ddata->dispc_gpio, 1);
	}
}

/*
 * hralvds_power_state_worker:
 *
 * This worker handles the video and power command sequencing when a timeout
 * happens (before the gradual change finishes) while the panel is powering
 * on or off.
 */
static void hralvds_power_state_worker(struct work_struct *work)
{
	struct panel_drv_data *ddata = container_of(work,
		struct panel_drv_data, hralvds_power_state_work.work);
	struct device *dev = &ddata->ts_i2c_client->dev;

	if (DISPLAY_UPDATE == ddata->display_state) {
		ddata->power_state = POWER_STATE_ON;
		ddata->reset_retries = 0;
		ddata->gradual_change_retries = 0;
		return;
	}

	switch (ddata->power_state) {
	case POWER_STATE_POWERING_OFF:
		/* No retries on powering off. */
		dev_warn(dev, "Did not receive gradual change finish event "
			"from the panel after timeout while powering off.\n");
		hralvds_set_power_state(ddata, __func__, POWER_STATE_OFF);
		gpio_set_value(ddata->dispc_gpio, 0);
		hralvds_check_requested_dispc_state(ddata);
		break;

	case POWER_STATE_POWERING_OFF_CHECK_ON:
		hralvds_power_state_powering_off_check_on(ddata);
		break;

	case POWER_STATE_POWERING_ON_1:
		/*
		 * force a RESET0 on the deserializer.
		 * this muct be done before the power command is sent to the
		 *	display.
		 */

		/* setup alias to talk to deserializer not touchscreen. */
		hralvds_setup_deser_alias(ddata);

		/*
		 * the value written (0x05 == 0b0101) means:
		 *	0b0100 == BC_ENABLE
		 *	0b0001 == DIGITAL RESET0
		 *
		 * so we're enabling the BC ENABLE and we're setting
		 *	DIGITAL RESET0 (which is self-clearing).
		 */
		i2c_write_le8(ddata->deser_i2c_client, DES928_RESET, 0x5);

		/* setup alias to talk to touchscreen again */
		hralvds_setup_ts_alias(ddata);

		/* move to the next state */
		hralvds_set_power_state(ddata, __func__,
			POWER_STATE_POWERING_ON_1A);
		hralvds_power_state_set_timeout(ddata);
		dev_info(dev, "Send RESET0 to deserializer.");
		break;

	case POWER_STATE_POWERING_ON_1A:
		if (atomic_read(&ddata->fast_init)) {
			dev_info(dev, "Fast initialization. Sending only "
				"video and touch sensitivity commands.");
			hralvds_send_video_ts_and_power(ddata);
			atomic_set(&ddata->fast_init, 0);
			hralvds_set_power_state(
				ddata, __func__, POWER_STATE_ON);
		} else {
			dev_info(dev, "Sending video, touch sensitivity, and "
				"power commands");
			hralvds_set_power_state(ddata, __func__,
				POWER_STATE_POWERING_ON_2);
			hralvds_send_video_ts_and_power(ddata);
		}
		break;

	case POWER_STATE_POWERING_ON_2:
	case POWER_STATE_ON:
		if (0 == power_cmd_retries) {
			dev_info(dev, "No ACK for power command. Not retrying.\n");
			break;
		}

		ddata->gradual_change_retries++;

		if (power_cmd_retries < 0) {
			dev_warn(dev, "No ACK for power command. Resending "
				"power command. Retries = %i\n",
				ddata->gradual_change_retries);

			hralvds_send_video_ts_and_power(ddata);
		} else {
			if (ddata->gradual_change_retries > power_cmd_retries) {
				ddata->reset_retries++;
				dev_info(dev, "Resetting display "
					"controller. Retries = %i/%i",
					ddata->reset_retries,
					reset_retries);
				if (ddata->reset_retries >= reset_retries) {
					dev_err(dev, "No ACK for power command. "
						"Giving up after %i retries.\n",
						power_cmd_retries);
				} else {
					gpio_set_value(ddata->dispc_gpio, 0);
					ddata->gradual_change_retries = 0;
					hralvds_set_power_state(
						ddata, __func__,
						POWER_STATE_POWERING_ON_1);
					msleep(HRALVDS_DISPLAY_RESET_CYCLE_TIME_MSEC);
					gpio_set_value(ddata->dispc_gpio, 1);
					hralvds_power_state_set_timeout(ddata);
				}
			} else {
				dev_warn(dev, "No ACK for power command. "
					"Resending power command. "
					"Retries = %i\n",
					ddata->gradual_change_retries);
				hralvds_send_video_ts_and_power(ddata);
			}
		}
		break;

	default:
		break;
	}
}

/*
 * hralvds_process_input_event
 *
 * As part of the power on sequencing, the panel will only apply illumination
 * after it had received a power command.  An input event can be received
 * before illumination is fully applied, i.e. before the power command is
 * processed.  This check is intended to prevent touch, button, and knob
 * events from being processed while the panel appears to be "off", while
 * waiting for the power command.
 *
 * Also, drop input events if the panel is not fully powered on.  This
 * prevents the user from interacting with the system as the display is
 * fading out while shutting down.
 *
 * Return "true" if the input event should be processed.
 */
int hralvds_process_input_event(struct panel_drv_data *ddata)
{
	return ddata->power_cmd_processed &&
		(ddata->power_state == POWER_STATE_ON);
}

static irqreturn_t hralvds_interrupt(int irq, void *dev_id)
{
	struct panel_drv_data *ddata = dev_id;
	hralvds_irq_worker(ddata);
	return IRQ_HANDLED;
}


#define to_panel_data(p) container_of(p, struct panel_drv_data, dssdev)

static int hralvds_dpi_connect(struct omap_dss_device *dssdev)
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

static void hralvds_dpi_disconnect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (!omapdss_device_is_connected(dssdev))
		return;

	in->ops.dpi->disconnect(in, dssdev);
}

static int hralvds_dpi_enable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	if (!omapdss_device_is_connected(dssdev))
		return -ENODEV;

	if (omapdss_device_is_enabled(dssdev))
		return 0;

	in->ops.dpi->set_timings(in, &hralvds_video_timings);

	r = in->ops.dpi->enable(in);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	/*
	 * Enable the display controller now so that user space does not
	 * need to wait for deserializer and firmware initialization.
	 * This allows the panel to receive and process power/video commands
	 * sooner during boot.
	 */
	hralvds_power_on(ddata);

	return 0;
}

static void hralvds_dpi_disable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dssdev->dev);
	struct omap_dss_device *in = ddata->in;

	if (!omapdss_device_is_enabled(dssdev))
		return;

	in->ops.dpi->disable(in);
	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static void hralvds_dpi_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dssdev->dev);
	struct omap_dss_device *in = ddata->in;

	dssdev->panel.timings = *timings;
	in->ops.dpi->set_timings(in, timings);
}

static void hralvds_dpi_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = hralvds_video_timings;
}

static int hralvds_dpi_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	r = in->ops.dpi->check_timings(in, timings);
	return r;
}

static struct omap_dss_driver hralvds_dpi_ops = {
	.connect	= hralvds_dpi_connect,
	.disconnect	= hralvds_dpi_disconnect,

	.enable		= hralvds_dpi_enable,
	.disable	= hralvds_dpi_disable,

	.set_timings	= hralvds_dpi_set_timings,
	.get_timings	= hralvds_dpi_get_timings,
	.check_timings	= hralvds_dpi_check_timings,

	.get_resolution	= omapdss_default_get_resolution,
};

static int hralvds_probe_of(struct device *dev)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct device_node *node = dev->of_node;
	int r, datalines, irq;
	const char *val;

	r = of_property_read_u32(node, "data-lines", &datalines);
	if (r) {
		dev_err(dev, "Failed to parse data-lines.");
		return r;
	}

	r = of_property_read_string(node, "pixel_clock_rising_edge", &val);
	if (!r && !strnicmp(val, "true", 4)) {

		ddata->pixel_clock_rising_edge = true;
		dev_info(dev, "pixel_clock_rising_edge\n");
	}

	ddata->dssdev.phy.dpi.data_lines = datalines;

	irq = irq_of_parse_and_map(node, 0);

	if (irq) {
		ddata->irq = irq;
		dev_info(dev, "HRALVDS IRQ = %u", ddata->irq);
	} else {
		pr_err("Could not obtain IRQ for HRALVDS TS, %u", ddata->irq);
		return -EINVAL;
	}

	/*
	 * Get I2c node of serializer.  The serializer's actual I2C address
	 * will be probed later on after it had been resetted.
	 */
	node = of_parse_phandle(dev->of_node, "serializer", 0);
	if (node)
		ddata->ser_i2c_client = of_find_i2c_device_by_node(node);
	else
		return -EINVAL;

#if defined HRALVDS_STARTUP_DEBUG
	dev_info(dev, "Serial I2C ID %x",
		ddata->ser_i2c_client->addr);
#endif

	/* Get I2c node of dserializer */
	node = of_parse_phandle(dev->of_node, "deserializer", 0);
	if (node)
		ddata->deser_i2c_client = of_find_i2c_device_by_node(node);
	else
		return -EINVAL;

#if defined HRALVDS_STARTUP_DEBUG
	dev_info(dev, "Deserial I2C ID %x",
			ddata->deser_i2c_client->addr);
#endif

#if defined HRALVDS_STARTUP_DEBUG
	dev_info(dev, "HRALVDS I2C ID %x",
		ddata->ts_i2c_client->addr);
#endif

	if (of_property_read_u32(dev->of_node, "deser-alias",
			&ddata->deser_alias)) {
		dev_err(dev, "No deser_alias defined");
		return -EINVAL;
	}

	/*
	 * override the deserializer's i2c client address with the
	 *	deserializer alias.
	 */
	ddata->deser_i2c_client->addr = ddata->deser_alias;

	/*
	 * set the names of the devices so log messages are a little
	 *	clearer.
	 */
	dev_set_name(&ddata->ser_i2c_client->dev, "ser 0-00%02hhx",
		ddata->ser_i2c_client->addr);
	dev_set_name(&ddata->deser_i2c_client->dev, "des 0-00%02hhx",
		ddata->deser_i2c_client->addr);

#if defined HRALVDS_STARTUP_DEBUG
	dev_info(dev, "Deserializer alias = %02hhx", ddata->deser_alias);
#endif

	ddata->in = omapdss_of_find_source_for_first_ep(dev->of_node);

	if (IS_ERR(ddata->in)) {
		dev_err(dev, "Failed to find video source");
		return PTR_ERR(ddata->in);
	}

	r = of_property_read_u32(dev->of_node, "reset-gpio",
				 &ddata->dispc_gpio);
	if (r) {
		dev_err(dev, "Failed to parse reset-gpio");
		return r;
	}

	r = of_property_read_u32(dev->of_node, "serializer-reset-gpio",
				&ddata->serializer_reset_gpio);
	if (r) {
		dev_err(dev, "Failed to parse serializer-reset-gpio");
		return r;
	}

#if defined HRALVDS_STARTUP_DEBUG
	dev_info(dev, "HRALVDS DISPC GPIO: %i", ddata->dispc_gpio);
	dev_info(dev, "LVDS Serializer Reset GPIO: %i",
		ddata->serializer_reset_gpio);
#endif

	r = devm_gpio_request_one(dev, ddata->dispc_gpio,
					GPIOF_OUT_INIT_LOW, "HRALVDS DISPC");
	if (r < 0) {
		dev_err(dev, "Failed to get DISPC GPIO");
		return r;
	}

	/*
	 * Even though PWR_ON_LVDS is high at boot, force the serializer
	 * into reset to ensure that it is re-enabled in to a sane state
	 * when we try to initialize it later on.
	 */
	r = devm_gpio_request_one(dev, ddata->serializer_reset_gpio,
			GPIOF_OUT_INIT_LOW, "LVDS Serializer Reset GPIO");
	if (r < 0) {
		dev_err(dev, "Failed to get LVDS serializer reset GPIO");
		return r;
	}

	return 0;
}

/* files in /sys/bus/omapdss/devices/display0 */
static DEVICE_ATTR(hralvds_display_controller, S_IRUGO | S_IWUSR,
	show_hralvds_display_controller, set_hralvds_display_controller);
static DEVICE_ATTR(hralvds_link_status, S_IRUGO,
	show_hralvds_link_status, NULL);
static DEVICE_ATTR(hralvds_version, S_IRUGO,
	show_hralvds_version, NULL);
static DEVICE_ATTR(hralvds_version_text, S_IRUGO,
	show_hralvds_version_text, NULL);
static DEVICE_ATTR(hralvds_diag_status, S_IRUGO,
	show_hralvds_diag_status, NULL);
static DEVICE_ATTR(hralvds_touch_cal, S_IWUSR | S_IWGRP,
	NULL, set_hralvds_touch_cal);
static DEVICE_ATTR(hralvds_touch_sensitivity, S_IRUGO | S_IWUSR,
	show_hralvds_touch_sensitivity, set_hralvds_touch_sensitivity);
static DEVICE_ATTR(hralvds_power, S_IRUGO | S_IWUSR,
	show_hralvds_power, set_hralvds_power);
static DEVICE_ATTR(hralvds_last_power, S_IWUSR,
	NULL, set_hralvds_last_power);
static DEVICE_ATTR(hralvds_video, S_IRUGO | S_IWUSR,
	show_hralvds_video, set_hralvds_video);
static DEVICE_ATTR(hralvds_last_video, S_IWUSR,
	NULL, set_hralvds_last_video);
static DEVICE_ATTR(hralvds_radio_freq, S_IRUGO | S_IWUSR,
	show_hralvds_radio_freq, set_hralvds_radio_freq);
static DEVICE_ATTR(hralvds_kick_supervisor, S_IWUSR | S_IWGRP,
	NULL, set_hralvds_kick_supervisor);
static DEVICE_ATTR(hralvds_event_status, S_IRUGO,
	show_hralvds_event_status, NULL);
#if defined HRALVDS_LOG_DEBUG
static DEVICE_ATTR(hralvds_debug, S_IRUGO | S_IWUSR | S_IWGRP,
	show_hralvds_debug, set_hralvds_debug);
#endif
static DEVICE_ATTR(hralvds_hdcp, S_IRUGO,
	show_hralvds_hdcp, NULL);
static DEVICE_ATTR(hralvds_irq_bad, S_IRUGO,
	show_hralvds_irq_bad, NULL);
static DEVICE_ATTR(hralvds_irq_counts, S_IRUGO | S_IWUSR,
	show_hralvds_irq_counts, set_hralvds_irq_counts);
static DEVICE_ATTR(hralvds_ser_regs, S_IRUGO,
	show_hralvds_ser_regs, NULL);
static DEVICE_ATTR(hralvds_deser_regs, S_IRUGO,
	show_hralvds_deser_regs, NULL);
static DEVICE_ATTR(hralvds_report, S_IRUGO,
	show_hralvds_report, NULL);
static DEVICE_ATTR(hralvds_hdcp_enable, S_IWUSR | S_IWGRP,
	NULL,set_hralvds_hdcp_enable);
static DEVICE_ATTR(hralvds_pattern, S_IRUGO| S_IWUSR,
	show_hralvds_pattern,set_hralvds_pattern);
static DEVICE_ATTR(hralvds_hardwaretest, S_IRUGO | S_IWUSR,
	show_hralvds_hardwaretest, set_hralvds_hardwaretest);
static DEVICE_ATTR(hralvds_fwif_status, S_IRUGO,
	show_hralvds_fwif_status, NULL);
static DEVICE_ATTR(hralvds_fwif_control, S_IWUSR,
	NULL, set_hralvds_fwif_control);

/*
 * there is an interrupt for the display mode request changes that come
 *	from the display.  we could use that here, too.
 */
static DEVICE_ATTR(hralvds_display_mode, S_IWUSR,
	NULL, set_hralvds_display_mode);
static DEVICE_ATTR(hralvds_display_mode_event, S_IRUGO,
	show_hralvds_display_mode_event, NULL);
static DEVICE_ATTR(hralvds_touch_cmd, S_IWUSR,
	NULL, set_hralvds_touch_cmd);
static DEVICE_ATTR(hralvds_setting_status, S_IWUSR,
	NULL, set_hralvds_setting_status);
static DEVICE_ATTR(hralvds_camera_setting, S_IRUGO | S_IWUSR,
	show_hralvds_camera_setting, set_hralvds_camera_setting);
static DEVICE_ATTR(hralvds_touchpad_error_status, S_IWUSR,
	NULL, set_hralvds_touchpad_error_status);
static DEVICE_ATTR(hralvds_camera_configuration, S_IWUSR,
	NULL, set_hralvds_camera_configuration);
static DEVICE_ATTR(hralvds_camera_diagnosis, S_IRUGO,
	show_hralvds_camera_diagnosis, NULL);
static DEVICE_ATTR(hralvds_mvc_dtc_detect_cancel, S_IWUSR,
	NULL, set_hralvds_mvc_dtc_detect_cancel);

static DEVICE_ATTR(hralvds_i2c_msleep, S_IRUGO | S_IWUSR,
	show_hralvds_i2c_msleep, set_hralvds_i2c_msleep);
static DEVICE_ATTR(hralvds_i2c_stats, S_IRUGO,
	show_hralvds_i2c_stats, NULL);
static DEVICE_ATTR(hralvds_i2c_retries, S_IRUGO | S_IWUSR,
	show_hralvds_i2c_retries, set_hralvds_i2c_retries);
static DEVICE_ATTR(hralvds_i2c_info, S_IRUGO,
	show_hralvds_i2c_info, NULL);

static struct attribute *hralvds_attributes[] = {
	&dev_attr_hralvds_display_controller.attr,
	&dev_attr_hralvds_link_status.attr,
	&dev_attr_hralvds_version.attr,
	&dev_attr_hralvds_version_text.attr,
	&dev_attr_hralvds_diag_status.attr,
	&dev_attr_hralvds_touch_cal.attr,
	&dev_attr_hralvds_touch_sensitivity.attr,
	&dev_attr_hralvds_power.attr,
	&dev_attr_hralvds_last_power.attr,
	&dev_attr_hralvds_video.attr,
	&dev_attr_hralvds_last_video.attr,
	&dev_attr_hralvds_radio_freq.attr,
	&dev_attr_hralvds_kick_supervisor.attr,
	&dev_attr_hralvds_event_status.attr,
#if defined HRALVDS_LOG_DEBUG
	&dev_attr_hralvds_debug.attr,
#endif
	&dev_attr_hralvds_hdcp.attr,
	&dev_attr_hralvds_irq_bad.attr,
	&dev_attr_hralvds_irq_counts.attr,
	&dev_attr_hralvds_ser_regs.attr,
	&dev_attr_hralvds_deser_regs.attr,
	&dev_attr_hralvds_report.attr,
	&dev_attr_hralvds_pattern.attr,
	&dev_attr_hralvds_hdcp_enable.attr,
	&dev_attr_hralvds_hardwaretest.attr,
	&dev_attr_hralvds_fwif_status.attr,
	&dev_attr_hralvds_fwif_control.attr,
	&dev_attr_hralvds_display_mode.attr,
	&dev_attr_hralvds_display_mode_event.attr,
	&dev_attr_hralvds_touch_cmd.attr,
	&dev_attr_hralvds_setting_status.attr,
	&dev_attr_hralvds_camera_setting.attr,
	&dev_attr_hralvds_touchpad_error_status.attr,
	&dev_attr_hralvds_camera_configuration.attr,
	&dev_attr_hralvds_camera_diagnosis.attr,
	&dev_attr_hralvds_mvc_dtc_detect_cancel.attr,
	&dev_attr_hralvds_i2c_msleep.attr,
	&dev_attr_hralvds_i2c_stats.attr,
	&dev_attr_hralvds_i2c_retries.attr,
	&dev_attr_hralvds_i2c_info.attr,
	NULL
};

static const struct attribute_group serlink_attr_group = {
	.attrs = hralvds_attributes,
};

static void hralvds_init_shadow_registers(struct device *dev)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct hralvds_power_cmd_struct *p = &ddata->cmd_power;
	struct hralvds_video_cmd_struct *v = &ddata->cmd_video;
	struct hralvds_radio_freq_cmd_struct *f = &ddata->cmd_radio_freq;
	struct hralvds_touch_sensitivity_cmd_struct *ts =
		&ddata->cmd_touch_sensitivity;

	p->cmd_id = HRALVDS_POWER_CMD;
	/* LCD=on, LED(Hard Keys)=on, Day/Night=Day, Gradual Change=200 msec*/
	p->power.ctrl = 0x00;
	/* all LED capactive buttons are on */
	p->power.led_face_sw_state = 0x00;
	/* default day time LCD backlight illumination */
	p->power.lcd_backlight_lum = cpu_to_be16(2210);
	/* default day time FaceSW (capacitive buttons) illumination */
	p->power.led_face_sw_lum = cpu_to_be16(1971);
	/* default day time vol ring knob illumination */
	p->power.led_hard_key_lum = cpu_to_be16(491);
	p->checksum = ~hralvds_cksum((const __u8 *)p, sizeof(*p) - 1,
					NULL, 0) + 1;

	/* Configure power command to enter display off mode */
	p = &ddata->cmd_power_off;
	p->cmd_id = HRALVDS_POWER_CMD;
	p->power.ctrl = (1 << 5) |  /* 800 msec gradual change time */
			(1 << 2) |  /* LCD backlight off */
			(1 << 1);   /* vol ring knob off */
	p->power.led_face_sw_state = 0xff; /* all buttons off */
	p->power.lcd_backlight_lum = cpu_to_be16(0);
	p->power.led_face_sw_lum = cpu_to_be16(0);
	p->power.led_hard_key_lum = cpu_to_be16(0);
	p->checksum = ~hralvds_cksum((const __u8 *)p, sizeof(*p) - 1,
					NULL, 0) + 1;

	v->cmd_id = HRALVDS_VIDEO_CMD;
	v->video.contrast = 5;
	v->video.black_level = 5;
	v->video.color = 5;
	v->video.tint = 5;
	v->checksum = ~hralvds_cksum((const __u8 *)v, sizeof(*v) - 1,
					NULL, 0) + 1;

	f->cmd_id = HRALVDS_RADIO_FREQ_CMD;
	f->radio.band = RADIO_FREQ_BAND_AM;
	f->radio.freq_thousands_hundreds = 0;
	f->radio.freq_tens_ones = 0;
	f->radio.freq_tenths_hundredths = 0;
	f->checksum = ~hralvds_cksum((const __u8 *)f, sizeof(*f) - 1,
					NULL, 0) + 1;

	ts->cmd_id = HRALVDS_DIAG_TOUCH_SENS_CMD;
	ts->touch_sensitivity.level = TOUCH_SENSITIVITY_MID;
	ts->checksum = ~hralvds_cksum((const __u8 *)ts, sizeof(*ts) - 1,
					NULL, 0) + 1;

	mutex_init(&ddata->cmd_radio_freq_lock);
	mutex_init(&ddata->cmd_power_lock);
	mutex_init(&ddata->cmd_video_lock);
	mutex_init(&ddata->cmd_touch_sensitivity_lock);
}

static int hralvds_init_serializer_i2c(struct panel_drv_data *ddata)
{
	struct device *ser_dev = &ddata->ser_i2c_client->dev;
	int r;
	u32 ser925_config0;
	/*
	 * The serializer on newer boards (DAN-1 and later?) is on I2C
	 * address 0x0c instead of 0x17.  This change was done per TI's
	 * request.  To maintain backwards compatibility with existing
	 * boards the serializer will be probled if it is on address
	 * 0x0c or 0x17.
	 */
	r = i2c_read_le8(ddata->ser_i2c_client,SER925_I2C_DEVICE_ID);

	if (r < 0) {

		ddata->ser_i2c_client->addr = SER925_ALT_I2C_ADDRESS;

		r = i2c_read_le8(ddata->ser_i2c_client,SER925_I2C_DEVICE_ID);

		if (r < 0) {
			dev_err(&ddata->ts_i2c_client->dev,
				"fail to read Ser address\n");
			return r;
		}
	}

#if defined HRALVDS_STARTUP_DEBUG
	dev_info(&ddata->ts_i2c_client->dev, "Serializer I2C ID %x",
		ddata->ser_i2c_client->addr);
#endif

	r = i2c_write_le8(ddata->ser_i2c_client, SER925_RESET,
		SER925_RESET_DIGITAL_RESET1);
	if (r < 0) {
		dev_err(ser_dev, "Failed to reset Serializer.");
		return r;
	}

	dev_info(ser_dev, "Serializer Reset done.");

	ser925_config0 = SER925_CONFIG0_BACK_CHAN_CRC |
				SER925_CONFIG0_I2C_PASSTHRU |
				SER925_CONFIG0_PCLK_AUTO;
	/*
	 * Address 0x03 bit0 : TRFB should be set on TJBA/TG7X board
	 */
       if (ddata->pixel_clock_rising_edge)
               ser925_config0 |= SER925_CONFIG0_TRFB;

	r = i2c_write_le8(ddata->ser_i2c_client, SER925_CONFIG0,ser925_config0);
	if (r < 0) {
		dev_err(ser_dev, "Failed to configure Serializer (CONFIG0).");
		return r;
	}

	r = i2c_write_le8(ddata->ser_i2c_client, SER925_CONFIG1,
		SER925_CONFIG1_LFMODE_SEL |
		SER925_CONFIG1_BCMODE_SEL |
		SER925_CONFIG1_FAILSAFE);
	if (r < 0) {
		dev_err(ser_dev, "Failed to configure Serializer (CONFIG1).");
		return r;
	}

	r = i2c_read_le8(ddata->ser_i2c_client, SER925_HDCP_CFG);
	if (r >= 0) {
		r = i2c_write_le8(ddata->ser_i2c_client, SER925_HDCP_CFG,
			r | SER925_HDCP_CFG_RX_DET_SEL);
		if (r < 0)
			dev_err(ser_dev, "HDCP CFG not set");
	} else
		dev_err(ser_dev, "HDCP CFG not read");

	dev_info(ser_dev, "Serializer configuration done.");
	return 0;
}

static inline void hralvds_reset_serializer(struct panel_drv_data *ddata,
					int serializer_state)
{
	/* If the serializer is not in reset, put it in reset */
	if (serializer_state) {
		gpio_set_value(ddata->serializer_reset_gpio, 0);
		msleep(10);
	}

	gpio_set_value(ddata->serializer_reset_gpio, 1);
	msleep(10); /* 10 msec should be plenty for serializer to start */
}

/*
 * perform minimal startup of serializer with retries.
 * more initialization will be done when the link is detected.
 */
static int hralvds_init_serializer(struct panel_drv_data *ddata)
{
	struct device *ser_dev = &ddata->ser_i2c_client->dev;
	int serializer_state, i, r = 0;

	/* Get initial state of PWR_ON_LVDS */
	serializer_state = gpio_get_value(ddata->serializer_reset_gpio);

	for (i = HRALVDS_SERIALIZER_RESET_RETRIES; i; i--) {
		hralvds_reset_serializer(ddata, serializer_state);

		r = hralvds_init_serializer_i2c(ddata);
		if (!r)
			break;

		/* Serializer is already enabled from above */
		serializer_state = 1;
		dev_err(ser_dev, "Serializer failed to initialize with error "
			"code %i. Retry=%i", r, i);
	}
	if (!i)
		dev_crit(ser_dev, "Could not initialize serializer after "
			"%i retries. Giving up.",
			HRALVDS_SERIALIZER_RESET_RETRIES);

	return r;
}

static int hralvds_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int r;
	struct panel_drv_data *ddata;
	struct device *dev = &client->dev;

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	dev_set_drvdata(dev, ddata);

	ddata->ts_i2c_client = client;

	ddata->dssdev.dev = dev;
	ddata->dssdev.driver = &hralvds_dpi_ops;
	ddata->dssdev.type = OMAP_DISPLAY_TYPE_DPI;
	ddata->dssdev.owner = THIS_MODULE;
	ddata->dssdev.panel.timings = hralvds_video_timings;

	ddata->pixel_clock_rising_edge = false;
	ddata->requested_dispc_state = 1;
	atomic_set(&ddata->fast_init, 1);

	hralvds_init_shadow_registers(dev);

	ratelimit_state_init(&ddata->setup_link_rs,
		HRALVDS_SETUP_LINK_RATELIMIT_PERIOD,
		HRALVDS_SETUP_LINK_RATELIMIT_THRESHOLD);

	mutex_init(&ddata->event_status_lock);
	mutex_init(&ddata->display_mode_lock);

	/*
	 * call the HRALVDS panel-specific probe() function.
	 */
	r = hralvds_panel_probe(ddata);
	if (r) {
		dev_err(&client->dev,
			"Device-specific probe() failed, %d\n", r);
		goto err_reg;
	}

	r = omapdss_register_display(&ddata->dssdev);
	if (r) {
		dev_err(&client->dev, "Failed to register panel");
		goto err_reg;
	}

	r = hralvds_probe_of(dev);
	if (r)
		goto err_remove_dssdev;

#if defined HRALVDS_LOG_DEBUG
	ddata->dbg_lvl = HRALVDS_LOG_DEBUG_DEFAULT_LEVEL;
#endif

	/* the display state is initially DISPLAY_INIT. */
	hralvds_reset_state(ddata);

	r = hralvds_init_serializer(ddata);
	if (r)
		goto err_remove_dssdev;

	/* Register sysfs hooks */
	r = sysfs_create_group(&client->dev.kobj, &serlink_attr_group);
	if (r < 0) {
		dev_info(&client->dev, "Couldn't create group for serlink.");
		goto err_remove_dssdev;
	}

	/* HDCP enable flag, default enable */
	ddata->hdcp_enable = ddata->hdcp_previous_state = 1;
	ddata->workq = create_singlethread_workqueue("hralvds");
	if (!ddata->workq) {
		dev_info(&client->dev, "Unable to create workqueue");
		r = -EINVAL;
	}

	hralvds_set_power_state(
		ddata, __func__, POWER_STATE_OFF);
	INIT_DELAYED_WORK(&ddata->hralvds_power_state_work,
			hralvds_power_state_worker);

	INIT_DELAYED_WORK(&ddata->hralvds_hdcp_auth_retry_work,
			hralvds_hdcp_auth_retry_worker);

	/*
	 * Allocate a delayed work queue to send supervisor frames to the
	 *	display to keep it alive.  hralvds_timer_worker() does this
	 *	work.
	 * ddata->timer_worker_delay is set in hralvds_reset_state, which
	 *	is called before execution reaches this point.
	 */
	INIT_DELAYED_WORK(&ddata->hralvds_timer_work,
				hralvds_timer_worker);
	queue_delayed_work(ddata->workq,
				&ddata->hralvds_timer_work,
				ddata->timer_worker_delay);

	INIT_DELAYED_WORK(&ddata->hralvds_hdcp_work,
				hralvds_hdcp_worker);
	queue_delayed_work(ddata->workq,
				&ddata->hralvds_hdcp_work,
				HRALVDS_HDCP_WORKQ_TIMEOUT);

	dev_info(&client->dev, "Using hard HRALVDS interrupt");

	/*
	 * attach the interrupt to the interrupt handler, which will
	 *	schedule the work. hralvds_interrupt will then call
	 *	hralvds_irq worker().
	 */
	r = devm_request_threaded_irq(dev, ddata->irq,
		NULL, hralvds_interrupt, IRQF_ONESHOT | IRQF_TRIGGER_LOW,
		dev_name(&client->dev), ddata);
	if (r) {
		dev_err(&client->dev, "Failed to register interrupt");
		r = -EINVAL;
		goto err_cancel_work;
	}

	/*
	 * Configure the Serializer's HDCP interrupt control
	 *	register.
	 * We set the INT_ENABLE bit to enable interrupts, and the RXDET_INT
	 *	bit to generate an interrupt when a receiver is detected.
	 * We do not set IS_RX_INT (INTB pin asserted) until after a
	 *	receiver is detected.
	 */
#if defined HRALVDS_HDCP_ENABLED
	/* take out RX INT on startup */
	r = 0xff & ~SER925_HDCP_ICR_IS_RX_INT;
#else
	r = SER925_HDCP_ICR_IE_RXDET_INT | SER925_HDCP_ICR_INT_ENABLE;
#endif

	r = i2c_write_le8(ddata->ser_i2c_client, SER925_HDCP_ICR, r);
	if (r < 0)
		/* not fatal, but log it. */
		dev_err(&client->dev,
			"Couldn't configure HDCP ICR");

	/*
	 * The 928Q datasheet indicates that the serializer
	 *	ISR register should be read after setting
	 *	up the ICR.
	 */
	r = i2c_read_le8(ddata->ser_i2c_client, SER925_HDCP_ISR);
	if (r < 0)
		/* not fatal, but log it. */
		dev_err(&ddata->ser_i2c_client->dev,
			"%s: Couldn't read back HDCP ISR",
			__func__);
	else
		dev_info(&ddata->ser_i2c_client->dev,
			"%s: ISR = %02hhx",
			__func__, r);

	dev_info(&client->dev, "hralvds_probe probe successful.");

	return 0;

err_cancel_work:
	cancel_delayed_work_sync(&ddata->hralvds_power_state_work);
	cancel_delayed_work_sync(&ddata->hralvds_timer_work);
	cancel_delayed_work_sync(&ddata->hralvds_hdcp_work);
	destroy_workqueue(ddata->workq);

	input_free_device(ddata->input_dev);

	sysfs_remove_group(&client->dev.kobj, &serlink_attr_group);

err_remove_dssdev:
	omapdss_unregister_display(&ddata->dssdev);
	hralvds_dpi_disable(&ddata->dssdev);
	hralvds_dpi_disconnect(&ddata->dssdev);
	omap_dss_put_device(ddata->in);
	dev_set_drvdata(&client->dev, NULL);
	kfree(ddata);

err_reg:
	return r;
}

static int hralvds_i2c_remove(struct i2c_client *client)
{
	struct panel_drv_data *ddata = dev_get_drvdata(&client->dev);

	gpio_set_value(ddata->dispc_gpio, 0);

	cancel_delayed_work_sync(&ddata->hralvds_power_state_work);
	cancel_delayed_work_sync(&ddata->hralvds_timer_work);
	cancel_delayed_work_sync(&ddata->hralvds_hdcp_work);
	destroy_workqueue(ddata->workq);

	input_free_device(ddata->input_dev);
	sysfs_remove_group(&client->dev.kobj, &serlink_attr_group);
	omapdss_unregister_display(&ddata->dssdev);
	hralvds_dpi_disable(&ddata->dssdev);
	hralvds_dpi_disconnect(&ddata->dssdev);
	omap_dss_put_device(ddata->in);

	dev_set_drvdata(&client->dev, NULL);
	kfree(ddata);

	return 0;
}

static const struct i2c_device_id hralvds_id[] = {
	{ "hralvds,thra", 0, },
	{ "hralvds,tjba", 0, },
	{ }
};

static const struct of_device_id hralvds_of_match[] = {
	{
		.compatible = "hralvds,thra",
		.data = "thra",
	},
	{
		.compatible = "hralvds,tjba",
		.data = "tjba",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, hralvds_of_match);


static struct i2c_driver hralvds_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "hralvds",
		.of_match_table = hralvds_of_match,
	},
	.id_table	= hralvds_id,
	.probe		= hralvds_i2c_probe,
	.remove		= hralvds_i2c_remove,
};

module_i2c_driver(hralvds_i2c_driver);

MODULE_LICENSE("GPL");
