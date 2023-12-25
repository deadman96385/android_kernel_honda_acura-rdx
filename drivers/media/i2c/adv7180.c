/*
 * adv7180.c Analog Devices ADV7180 video decoder driver
 * Copyright (c) 2009 Intel Corporation
 * Copyright (C) 2013 Cogent Embedded, Inc.
 * Copyright (C) 2013 Renesas Solutions Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-device.h>
#include <linux/gpio.h>
//#include <linux/mutex.h>

#include <media/soc_camera.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>

#include <linux/pm_runtime.h>

#include <linux/of_gpio.h>
#include <linux/of_device.h>

#define ADV7180_INPUT_CONTROL_REG			0x00
/* ADV7180 Input Control Registry Values*/
#define ADV7180_INPUT_CONTROL_AD_PAL_BG_NTSC_J_SECAM	0x00
#define ADV7180_INPUT_CONTROL_AD_PAL_BG_NTSC_J_SECAM_PED 0x10
#define ADV7180_INPUT_CONTROL_AD_PAL_N_NTSC_J_SECAM	0x20
#define ADV7180_INPUT_CONTROL_AD_PAL_N_NTSC_M_SECAM	0x30
#define ADV7180_INPUT_CONTROL_NTSC_J			0x40
#define ADV7180_INPUT_CONTROL_NTSC_M			0x50
#define ADV7180_INPUT_CONTROL_PAL60			0x60
#define ADV7180_INPUT_CONTROL_NTSC_443			0x70
#define ADV7180_INPUT_CONTROL_PAL_BG			0x80
#define ADV7180_INPUT_CONTROL_PAL_N			0x90
#define ADV7180_INPUT_CONTROL_PAL_M			0xa0
#define ADV7180_INPUT_CONTROL_PAL_M_PED			0xb0
#define ADV7180_INPUT_CONTROL_PAL_COMB_N		0xc0
#define ADV7180_INPUT_CONTROL_PAL_COMB_N_PED		0xd0
#define ADV7180_INPUT_CONTROL_PAL_SECAM			0xe0
#define ADV7180_INPUT_CONTROL_PAL_SECAM_PED		0xf0
#define ADV7180_INPUT_CONTROL_INSEL_MASK		0x0f
/* Denso RCG: INSEL for 40-Lead pkg */
#define ADV7180_INPUT_CONTROL_INSEL_AIN1		0x00
#define ADV7180_INPUT_CONTROL_INSEL_AIN2		0x03
#define ADV7180_INPUT_CONTROL_INSEL_AIN3		0x04
#define ADV7180_INPUT_CONTROL_INSEL_SVIDEO_YAIN1_CAIN2	0x06
#define ADV7180_INPUT_CONTROL_INSEL_YAIN1_PRAIN3_PBAIN2	0x09
#define ADV7180_INPUT_CONTROL_INSEL_DEF 		(ADV7180_INPUT_CONTROL_INSEL_AIN3)

#define ADV7180_OUTPUT_CONTROL_REG		        0x03
/* ADV7180 Output Control registry for 40-Lead pkg */
#define ADV7180_OUTPUT_CONTROL_SD_DUP_AV	        0x01 /* Invalid on 40-Lead pkg */
#define ADV7180_OUTPUT_CONTROL_RESERVED		        0x02
#define ADV7180_OUTPUT_CONTROL_TOD		        0x40
#define ADV7180_OUTPUT_CONTROL_VBI_EN		        0x80
/* OF_SEL [3:0] for 40-Lead pkg */
#define ADV7180_OUTPUT_CTRL_OF_SEL_8B_LLC_422_BT656     0x0C


#define ADV7180_EXTENDED_OUTPUT_CONTROL_REG		0x04
#define ADV7180_EXTENDED_OUTPUT_CONTROL_NTSCDIS		0xC5
#define ADV7180_EXTENDED_OUTPUT_CONTROL_BT656_3 	0x45 // Denso RCG: 0x45 Blank Cr and Cb, Extended range - ITU-R BT.656-3 compatible TIM_OE = Disable

#define ADV7180_AUTODETECT_ENABLE_REG			0x07
#define ADV7180_AUTODETECT_DEFAULT			0x7f
/* Contrast */
#define ADV7180_CON_REG		0x08	/*Unsigned */
#define ADV7180_CON_MIN		0
#define ADV7180_CON_DEF		128
#define ADV7180_CON_MAX		255
/* Brightness*/
#define ADV7180_BRI_REG		0x0a	/*Signed */
#define ADV7180_BRI_MIN		-128
#define ADV7180_BRI_DEF		0
#define ADV7180_BRI_MAX		127
/* Hue */
#define ADV7180_HUE_REG		0x0b	/*Signed, inverted */
#define ADV7180_HUE_MIN		-127
#define ADV7180_HUE_DEF		0
#define ADV7180_HUE_MAX		128

#define ADV7180_ADI_CTRL_REG				0x0e
#define ADV7180_ADI_CTRL_IRQ_SPACE			0x20

#define ADV7180_PWR_MAN_REG		0x0f
#define ADV7180_PWR_MAN_ON		0x04
#define ADV7180_PWR_MAN_OFF		0x24
#define ADV7180_PWR_MAN_RES		0x80

#define ADV7180_STATUS1_REG		0x10
#define ADV7180_STATUS1_IN_LOCK		0x01
#define ADV7180_STATUS1_AUTOD_MASK	0x70
#define ADV7180_STATUS1_AUTOD_NTSC_M_J	0x00
#define ADV7180_STATUS1_AUTOD_NTSC_4_43 0x10
#define ADV7180_STATUS1_AUTOD_PAL_M	0x20
#define ADV7180_STATUS1_AUTOD_PAL_60	0x30
#define ADV7180_STATUS1_AUTOD_PAL_B_G	0x40
#define ADV7180_STATUS1_AUTOD_SECAM	0x50
#define ADV7180_STATUS1_AUTOD_PAL_COMB	0x60
#define ADV7180_STATUS1_AUTOD_SECAM_525	0x70

#define ADV7180_IDENT_REG               0x11
#define ADV7180_IDENT_40_LEAD           0x1C
#define ADV7180_ID_7180                 0x18

#define ADV7180_STATUS2_REG		0x12

#define ADV7180_STATUS3_REG		0x13
#define ADV7180_STATUS3_SD_OP_50HZ      0x04

#define ADV7180_ICONF1_ADI		0x40
#define ADV7180_ICONF1_ACTIVE_LOW	0x01
#define ADV7180_ICONF1_PSYNC_ONLY	0x10
#define ADV7180_ICONF1_ACTIVE_TO_CLR	0xC0
/* Saturation */
#define ADV7180_SD_SAT_CB_REG	0xe3	/*Unsigned */
#define ADV7180_SD_SAT_CR_REG	0xe4	/*Unsigned */
#define ADV7180_SAT_MIN		0
#define ADV7180_SAT_DEF		128
#define ADV7180_SAT_MAX		255

#define ADV7180_IRQ1_LOCK	0x01
#define ADV7180_IRQ1_UNLOCK	0x02
#define ADV7180_ISR1_ADI	0x42
#define ADV7180_ICR1_ADI	0x43
#define ADV7180_IMR1_ADI	0x44
#define ADV7180_IMR2_ADI	0x48
#define ADV7180_IRQ3_AD_CHANGE	0x08
#define ADV7180_ISR3_ADI	0x4A
#define ADV7180_ICR3_ADI	0x4B
#define ADV7180_IMR3_ADI	0x4C
#define ADV7180_IMR4_ADI	0x50

#define ADV7180_NTSC_V_BIT_END_REG	0xE6

#define ADV7180_VSYNC_FIELD_CTL_1_REG   0x31
#define ADV7180_VSYNC_FIELD_CTL_2_REG   0x32
#define ADV7180_VSYNC_FIELD_CTL_3_REG   0x33

#define ADV7180_MANUAL_WIN_CTL_REG      0x3d	/* Manual Window Control */

#define ADV7180_CVBS_TRIM_REG           0x52	/* CVBS_TRIM register */
#define RECOMMENDED_AFE_BIAS_CURRENT    0xd     /* automotive temperature range */

/* Denso RCG:
 *     Extracted from TVP5158
 */
/*
#define ADV_DECODER_1		(1<<0)
#define ADV_VIDEO_PORT_ENABLE	(1<<0)
#define ADV_OUT_CLK_P_EN	(1<<2)
#define ADV_FIELD_RATE		(1<<5)
#define ADV_SIGNAL_PRESENT	(1<<7)
#define ADV_VIDEO_STANDARD_MASK	(0x07)
*/
/* Number of pixels and number of lines per frame for different standards */
#define NTSC_NUM_ACTIVE_PIXELS  (720)
#define NTSC_NUM_ACTIVE_LINES   (480)
#define PAL_NUM_ACTIVE_PIXELS   (720)
#define PAL_NUM_ACTIVE_LINES    (576)
#define AUTO_NUM_ACTIVE_PIXELS  (720)
#define AUTO_NUM_ACTIVE_LINES   (576)

static inline void
adv7180_write(struct i2c_client *client,
        unsigned char addr, unsigned char value);

static enum adv7180_std
adv7180_get_video_std(struct i2c_client *client);


/* Debug functions */
static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

enum adv7180_std {
        STD_NTSC_M_J =	0x00,
        STD_NTSC_4_43 = 0x01,
        /*STD_PAL_M =	0x02,
        STD_PAL_60 =	0x03,
        STD_PAL_B_G =	0x04,
        STD_SECAM =	0x05,
        STD_PAL_COMB =	0x06,
        STD_SECAM_525 =	0x07,
        STD_INVALID =   0x0F */// State un-lock
        STD_INVALID =   0x02 // State un-lock
};


/*#define ADV7180_STD_MAX		(ADV7180_PAL + 1) adv7180 POC*/
#define ADV7180_STD_MAX		(STD_PAL_COMB + 1)

struct adv7180_std_info {
	unsigned long width;
	unsigned long height;
	struct v4l2_standard standard;
};

struct adv7180_color_format {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
};


static const struct adv7180_color_format adv7180_cfmts[] = { // Denso RCG: TODO: review pixel/color formats
	{
		.code           = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace     = V4L2_COLORSPACE_SMPTE170M,
	},
/* Denso RCG: color format not available for ADV7180 40-Lead
        {
		.code           = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace     = V4L2_COLORSPACE_SMPTE170M,
	},*/
};

/*  Denso RCG: this member is irrelevant even at TVP5158
enum tvp5158_signal_present {
	TVP5158_SIGNAL_DEAD = 0,
	TVP5158_SIGNAL_PRESENT
};*/

struct adv7180_priv {

        struct v4l2_subdev	subdev;
        struct v4l2_async_subdev	asd;
	struct v4l2_ctrl_handler        hdl;
	int                             power;
	int                             model;
	int                             revision;
	int                             width;
	int                             height;
	const char			*sensor_name;

        /* Denso RCG: this GPIOs are not in use.
	int				cam_fpd_mux_s0_gpio;
	int				sel_tvp_fpd_s0;
	int				vin2_s0_gpio;
	int				vin2_s2_gpio;*/

        enum adv7180_std                        current_std;

        /* Denso RCG: this member is irrelevant even at TVP5158
        enum				tvp5158_signal_present signal_present;*/

	const struct adv7180_std_info           *std_list;
	const struct adv7180_color_format       *cfmt;
};


static const struct adv7180_std_info adv7180_std_list[] = {
/* Standard: STD_NTSC_MJ */
[STD_NTSC_M_J] = {
	.width = NTSC_NUM_ACTIVE_PIXELS,
	.height = NTSC_NUM_ACTIVE_LINES,
	.standard = {
		.index = 0,
		.id = V4L2_STD_NTSC,
		.name = "NTSC",
		.frameperiod = {1001, 30000},
		.framelines = 525
		},
	},
[STD_NTSC_4_43] = {
	.width = NTSC_NUM_ACTIVE_PIXELS,
	.height = NTSC_NUM_ACTIVE_LINES,
	.standard = {
		.index = 1,
		.id = V4L2_STD_NTSC,
		.name = "NTSC",
		.frameperiod = {1001, 30000},
		.framelines = 525
		},
	},
/*! (B, G, H, I, N) PAL */
/*[STD_PAL_M] = {
        .width = PAL_NUM_ACTIVE_PIXELS,
	.height = PAL_NUM_ACTIVE_LINES,
        .standard = {
                .index = 2,
                .id = V4L2_STD_PAL,
                .name = "PAL",
                .frameperiod = {1001, 30000},
                .framelines = 625,
		},
	 },*/
	/* Standard: need to add for additional standard */
};


#define to_adv7180_sd(_ctrl) (&container_of(_ctrl->handler,		\
					    struct adv7180_priv,	\
					    hdl)->subdev)

static v4l2_std_id adv7180_std_to_v4l2(u8 status1)
{
	/* in case V4L2_IN_ST_NO_SIGNAL */
	if (!(status1 & ADV7180_STATUS1_IN_LOCK))
		return V4L2_STD_UNKNOWN;

	switch (status1 & ADV7180_STATUS1_AUTOD_MASK) {
	case ADV7180_STATUS1_AUTOD_NTSC_M_J:
		return V4L2_STD_NTSC;
	case ADV7180_STATUS1_AUTOD_NTSC_4_43:
		return V4L2_STD_NTSC_443;
	case ADV7180_STATUS1_AUTOD_PAL_M:
		return V4L2_STD_PAL_M;
	case ADV7180_STATUS1_AUTOD_PAL_60:
		return V4L2_STD_PAL_60;
	case ADV7180_STATUS1_AUTOD_PAL_B_G:
		return V4L2_STD_PAL;
	case ADV7180_STATUS1_AUTOD_SECAM:
		return V4L2_STD_SECAM;
	case ADV7180_STATUS1_AUTOD_PAL_COMB:
		return V4L2_STD_PAL_Nc | V4L2_STD_PAL_N;
	case ADV7180_STATUS1_AUTOD_SECAM_525:
		return V4L2_STD_SECAM;
	default:
		return V4L2_STD_UNKNOWN;
	}
}

static u32 adv7180_status_to_v4l2(u8 status1)
{
	if (!(status1 & ADV7180_STATUS1_IN_LOCK))
		return V4L2_IN_ST_NO_SIGNAL;

	return 0;
}

static int __adv7180_status(struct i2c_client *client, u32 *status,
			    v4l2_std_id *std)
{
	int status1;

        v4l_info(client, "__adv7180_status \n");

	status1 = i2c_smbus_read_byte_data(client, ADV7180_STATUS1_REG);
	if (status1 < 0){
		return status1;
        }

	if (status){
		*status = adv7180_status_to_v4l2(status1);
        }
	if (std){
		*std = adv7180_std_to_v4l2(status1);
        }

	return 0;
}


/* Denso RCG:
 * imported from TVP5158
 * original name to_tvp5158
 */
static struct adv7180_priv *to_adv7180(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct adv7180_priv,
			subdev);
}


static int adv7180_querystd(struct v4l2_subdev *sd, v4l2_std_id *std_id)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        struct adv7180_priv *priv = to_adv7180(client);

        v4l_info(client, "adv7180_querystd \n");

	if (std_id == NULL)
		return -EINVAL;
	*std_id = V4L2_STD_UNKNOWN;

	adv7180_get_video_std(client);
	if (priv->current_std == STD_INVALID)
		return -EINVAL;
	*std_id = priv->std_list[0].standard.id;

        return 0;
}


/* Denso RCG:
 * imported from TVP5158
 * original name tvp5158_start_streaming
 */
static void
adv7180_start_streaming(struct i2c_client *client)
{
	/*unsigned char adv_reg_val = 0;*/

        v4l_info(client, "adv7180_start_streaming \n");

        /* Denso RCG: removing, ADV7180 has 1 video decoder channel */
	/* adv7180_write(client, REG_DEC_WR_EN, core); Decoder Write Enable */

	/* Enable output stream on */
	/*adv_reg_val = ADV_VIDEO_PORT_ENABLE | ADV_OUT_CLK_P_EN ;
	adv7180_write(client, REG_OFM_CTRL, adv_reg_val);*/

        //adv7180_write(client, ADV7180_OUTPUT_CONTROL_REG, ADV7180_OUTPUT_CTRL_OF_SEL_8B_LLC_422_BT656);
        adv7180_write(client, 0x03, 0x0C);


         //adv7180_write(client, ADV7180_EXTENDED_OUTPUT_CONTROL_REG, ADV7180_EXTENDED_OUTPUT_CONTROL_BT656_3);
         //adv7180_write(client, 0x04, 0x45);
}


/* Denso RCG:
 * imported from TVP5158
 * for adv7180_video_ops
 * original name tvp5158_enum_fmt
 */
static int adv7180_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			enum v4l2_mbus_pixelcode *code)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);

        v4l_info(client, "adv7180_enum_fmt \n");

	if (index >= ARRAY_SIZE(adv7180_cfmts)){
		return -EINVAL;
        }

	*code = adv7180_cfmts[0].code;

	return 0;
}


/* Denso RCG:
 * imported from TVP5158
 * for adv7180_video_ops
 * original name tvp5158_enum_framesizes
 * code inside of fuction imported form adv7180_POC
 */
 /*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int adv7180_enum_framesizes(struct v4l2_subdev *sd,
			struct v4l2_frmsizeenum *fsize)
{
        //struct i2c_client *client = v4l2_get_subdevdata(sd);
	/* For now, hard coded resolutions for TVP5158 NTSC decoder */
	int cam_width[] =	{ 720,	640,};
	int cam_height[] =	{ 240,	240,};

        /*v4l2_info(client, "adv7180_enum_framesizes \n");*/

	/*if (fsize->index >= 2){ This is how it was on TVP5158 */
	if (fsize->index >= 2){
                /*dev_err(&client->dev, "adv7180_enum_framesizes (fsize->index >= 2) fsize->index = 0x%02x \n", fsize->index);*/
		return -EINVAL;
        }

        /* Denso RCG: from adv7180_POC ioctl_enum_framesizes */
        /*if (fsize->index > ADV7180_STD_MAX)
		return -EINVAL;*/

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = cam_width[fsize->index];
	/*fsize->discrete.width = adv7180_std_list[fsize->index].width;*/
	/* tvp5158 sensor outputs interlaced data, hence, in the timings
	* we listed down the field height. In the framesize query we need
	* to publish the frame height, so multiply the field height by 2 */
	fsize->discrete.height = 2 * cam_height[fsize->index];
	/*fsize->discrete.height = adv7180_std_list[fsize->index].height;*/

        /* Denso RCG: from adv7180_POC ioctl_enum_framesizes */
        /*fsize->pixel_format = adv7180_data.pix.pixelformat;
	fsize->discrete.width = video_fmts[fsize->index].active_width;
	fsize->discrete.height = video_fmts[fsize->index].active_height;*/

	return 0;
}


/* Denso RCG:
 * imported from TVP5158
 * for adv7180_video_ops
 * original name tvp5158_g_parm
 */
/**
 * adv7180_g_parm() - V4L2 decoder interface handler for g_parm
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the decoder's video CAPTURE parameters.
 */
static int adv7180_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7180_priv *state = to_adv7180(client);
	struct v4l2_captureparm *cparm;
	enum adv7180_std current_std;

        v4l2_info(&state->subdev, "adv7180_g_parm \n");

	if (parms == NULL){
		return -EINVAL;
        }

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE){
		/* only capture is supported */
		return -EINVAL;
        }

	/* get the current standard */
	current_std = state->current_std;
	cparm = &parms->parm.capture;
	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe =
		state->std_list[current_std].standard.frameperiod;

	return 0;
}


/* Denso RCG:
 * imported from TVP5158
 * for adv7180_video_ops
 * original name tvp5158_s_parm
 */
/**
 * adv7180_s_parm() - V4L2 decoder interface handler for s_parm
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the decoder to use the input parameters, if possible. If
 * not possible, returns the appropriate error code.
 */
static int adv7180_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7180_priv *state = to_adv7180(client);
	struct v4l2_fract *timeperframe;
	enum adv7180_std current_std;

        v4l2_info(&state->subdev, "adv7180_s_parm \n");

	if (parms == NULL){
		return -EINVAL;
        }

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE){
		/* only capture is supported */
		return -EINVAL;
        }

	timeperframe = &parms->parm.capture.timeperframe;

	/* get the current standard */
	current_std = state->current_std;

	*timeperframe =
	    state->std_list[current_std].standard.frameperiod;

	return 0;
}


/* Denso RCG:
 * imported from TVP5158
 * for adv7180_video_ops
 * original name tvp5158_s_stream
 */
/**
 * adv7180_s_stream() - V4L2 decoder i/f handler for s_stream
 * @sd: pointer to standard V4L2 sub-device structure
 * @enable: streaming enable or disable
 *
 * Sets streaming to enable or disable.
 */
static int adv7180_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

        v4l_info(client, "adv7180_s_stream enable:%s \n", (enable)? "True" : "False" );

	if (enable) {
                /* Denso RCG: VBUS programing removal */
		/*if (vbus_prog == 1)
                        v4l_info(client, "VBUS_PROG is enabled at adv7180_s_stream \n"); */
                        /* Denso RCG: VBUS programing removal */
			/* VBUS address value setting */
			/*adv7180_set_int_regs(client, adv_vbus_addr_value_set,
					ARRAY_SIZE(adv_vbus_addr_value_set));*/

		adv7180_start_streaming(client);
	} else {
		/*adv7180_write(client, REG_OFM_CTRL, 0x0); define REG_OFM_CTRL   0xB2 */
                //adv7180_write(client, ADV7180_OUTPUT_CONTROL_REG, (ADV7180_OUTPUT_CTRL_OF_SEL_8B_LLC_422_BT656 | ADV7180_OUTPUT_CONTROL_TOD));
                adv7180_write(client, 0x03, 0x4C);
        }

	return 0;
}


/* Denso RCG:
 * imported from TVP5158
 * for adv7180_video_ops
 * original name tvp5158_g_fmt
 */
static int adv7180_g_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7180_priv *state = to_adv7180(client);
	enum adv7180_std current_std;

        //v4l2_info(&state->subdev, "adv7180_g_fmt state->current_std: %d \n", state->current_std);
        //v4l2_info(&state->subdev, "adv7180_g_fmt state->std_list[%d].standard.name: %s \n", state->current_std, state->std_list[current_std].standard.name);

	if ((state->current_std == STD_INVALID) && (adv7180_get_video_std(client) == STD_INVALID)){
                dev_err(&client->dev, "adv7180_g_fmt state->current_std == STD_INVALID \n");
		return -EINVAL;
        }
	/* Calculate height and width based on current standard */
	current_std = state->current_std;
	/* both fields alternating into separate buffers */
	mf->field = V4L2_FIELD_ALTERNATE;
	mf->code = adv7180_cfmts[0].code;
	mf->width = state->std_list[current_std].width;
	mf->height = state->std_list[current_std].height;
	mf->colorspace = adv7180_cfmts[0].colorspace;

	return 0;
}


/* Denso RCG:
 * imported from TVP5158
 * for adv7180_video_ops
 * original name tvp5158_s_fmt
 */
/* set the format we will capture in */
static int adv7180_s_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7180_priv *state = to_adv7180(client);

        //v4l2_info(client, "adv7180_s_fmt  \n");
	/* adv7180_set_gpios(client);  Denso RCG: No GPIOS needed */
	v4l2_info(&state->subdev, "adv7180_s_fmt: Currently only D1 resolution is supported\n");

	return 0;
}


static void adv7180_exit_controls(struct adv7180_priv *state)
{
	v4l2_ctrl_handler_free(&state->hdl);
}

static int adv7180_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				 enum v4l2_mbus_pixelcode *code)
{
	if (index > 0)
		return -EINVAL;

	*code = V4L2_MBUS_FMT_UYVY8_2X8;

	return 0;
}

static int adv7180_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *cfg)
{
	/*
	 * The ADV7180 sensor supports BT.601/656 output modes.
	 * The BT.656 is default and not yet configurable by s/w.
	 */
	cfg->flags = V4L2_MBUS_MASTER | V4L2_MBUS_PCLK_SAMPLE_RISING |
		     V4L2_MBUS_DATA_ACTIVE_HIGH;
	cfg->type = V4L2_MBUS_BT656;

	return 0;
}

static const struct v4l2_subdev_video_ops adv7180_video_ops = {
	.querystd = adv7180_querystd, /* Denso RCG: present in tvp5158*/
	//.g_input_status = adv7180_g_input_status,
	//.s_routing = adv7180_s_routing,

        /* Denso RCG: Parameters exported from tvp5158 */
        .enum_mbus_fmt	= adv7180_enum_fmt,
        .enum_framesizes = adv7180_enum_framesizes,
        .g_parm		= adv7180_g_parm,
        .s_parm		= adv7180_s_parm,
	.s_stream	= adv7180_s_stream,
	.g_mbus_fmt	= adv7180_g_fmt,
	.s_mbus_fmt	= adv7180_s_fmt,
	.try_mbus_fmt	= adv7180_g_fmt,
};


static const struct v4l2_subdev_ops adv7180_ops = {
        /* Denso RCG: Removing v4l2_subdev_core_ops Not needed at TVP5158 */
	/*.core = &adv7180_core_ops,*/
	.video = &adv7180_video_ops,
};


static int adv7180_read(struct i2c_client *client, unsigned char addr)
{
	/*unsigned char buffer[1];*/
	int rc;

        rc = i2c_smbus_read_byte_data(client, addr);
        if (rc < 0) {
		dev_err(&client->dev, "read_i2c_smbus_read_byte_data i/o error: rc == %d (should be 1)\n"
		, rc);
		/*return rc;*/
	}

	/*buffer[0] = addr;

	rc = i2c_master_send(client, buffer, 1);
	if (rc < 0) {
		dev_err(&client->dev, "read_i2c_master_send i/o error: rc == %d (should be 1)\n"
		, rc);
		return rc;
	}

	rc = i2c_master_recv(client, buffer, 1);
	if (rc < 0) {
		dev_err(&client->dev, "read_i2c_master_recv i/o error: rc == %d (should be 1)\n"
		, rc);
		return rc;
	}

	return buffer[0];*/
	return rc;
}

static inline void adv7180_write(struct i2c_client *client, unsigned char addr,
			unsigned char value)
{
	unsigned char buffer[2];
	int rc;

	buffer[0] = addr;
	buffer[1] = value;
	rc = i2c_master_send(client, buffer, 2);
	if (rc != 2)
		dev_err(&client->dev, "write_i2c_master_send i/o error: rc == 0x%02x (should be 2)\n", rc);
}


static int adv7180_set_default(struct i2c_client *client, struct
adv7180_priv *state) {
	int ret;
        v4l2_std_id std;


        v4l_info(client, "Manual Detection (MD) chip @ 0x%02x (%s)\n",
                client->addr, client->adapter->name);

        /* Denso RCG: Device status is performed by adv7180_get_video_std */
        /*ret = v4l2_std_to_adv7180(state->curr_norm);
        if (ret < 0){
                v4l_info(client, "state get fail MD \n");
                return ret;
        }*/

        /*ret =
            i2c_smbus_write_byte_data(client, ADV7180_INPUT_CONTROL_REG,
                                      ret | state->input);*/
        ret = i2c_smbus_write_byte_data(client,
                ADV7180_INPUT_CONTROL_REG,
                (ADV7180_INPUT_CONTROL_AD_PAL_BG_NTSC_J_SECAM |
                ADV7180_INPUT_CONTROL_INSEL_DEF));
        if (ret < 0){
                v4l_info(client, "Input Control Reg write fail MD \n");
                return ret;
        }

	/* Denso RCG
        Extended_Output_Control_Reg (0x04) from (0xC5) to (0x45)
        Extended range
        SFL output is disabled
        Blank Cr and Cb
        HS, VS, FIELD three-stated
        ITU-R BT.656-3 compatible */
	ret = i2c_smbus_write_byte_data(client,
			ADV7180_EXTENDED_OUTPUT_CONTROL_REG,
			ADV7180_EXTENDED_OUTPUT_CONTROL_BT656_3);
	if (ret < 0){
                v4l_info(client, "Extended Output Control Reg write fail MD \n");
		return ret;
        }

	/* Denso RCG
	NVEND[4:0] = 0x0F: 15 lines
	NVENDSIGN = 0: + (Manual)
	NVENDDELE = 1: Additional delay by one line on even fields
	NVENDDELO = 0: No additional delay on odd fields
	*/
	adv7180_write(client, ADV7180_NTSC_V_BIT_END_REG, 0x4F); // +16 even, +15 odd

	/* read current norm */
	__adv7180_status(client, NULL, &std);

	adv7180_write(client, 0xF4, 0x3F); // DR_STR[1:0] High drive strength (4×), DR_STR_C[1:0] High drive strength (4×), DR_STR_S[1:0] High drive strength (4×)

        /* Denso RCG: 
         *   VS/FIELD CONTROL 1 REGISTRY
         *   Default value 0x12   -- Reserved Bit 7:6:5, 2:1:0
         *     Bit 3 HVSTIM: 0 - start of line relative to HSE
         *     Bit 4 NEWAVMODE: 1 Manual VS/FIELD position controlled by reg 0x32 and 0x33
         *
         *   VS/FIELD CONTROL 2 REGISTRY
         *   Default value 0x41
         *     Bit 6 VSBHE: 1 - VS changes state at the start of the line (even field)
         *     Bit 7 VSBHO: 0 - VS goes high in the middle of the line (odd field)
         *
         *   VS/FIELD CONTROL 3 REGISTRY
         *   Default value 0x84
         *     Bit 6 VSEHE: 0 - VS goes low in middle of the line (even field)
         *     Bit 7 VSEHO: 1 - VS changes state at the start of the line (odd field)
         */
        adv7180_write(client, ADV7180_VSYNC_FIELD_CTL_1_REG, 0x12); // 0x31
        adv7180_write(client, ADV7180_VSYNC_FIELD_CTL_2_REG, 0x41); // 0x32
        adv7180_write(client, ADV7180_VSYNC_FIELD_CTL_3_REG, 0x84); // 0x33




        /* Manual Window Control */
        adv7180_write(client, ADV7180_MANUAL_WIN_CTL_REG, 0xa2); // 0x3d

	/* Adjust the video shift */
	adv7180_write(client, 0x58, 0x01); // VSYNC signal enable
	adv7180_write(client, 0x35, 0x12); // Adjust HS 272 (0x110)
	adv7180_write(client, 0x36, 0x10); // See ADV7180 Data sheet HS configuration for detail
	adv7180_write(client, 0x34, 0x11); // refer Figure 37

	/* THRA-13075: Change of resistor value for ADV7180WBCPZ */
	/* setting for automotive temperature range. */
	adv7180_write(client, ADV7180_CVBS_TRIM_REG, RECOMMENDED_AFE_BIAS_CURRENT);

	return 0;
}


static enum adv7180_std
adv7180_get_video_std(struct i2c_client *client)
{
	struct adv7180_priv *state	= to_adv7180(client);
	int adv_status1 = 0, adv_status3 = 0;

        //v4l_info(client, "adv7180_get_video_std \n");

	/* Core Read Enable */ /* Denso RCG: Not needed for ADV7180 */
	/* adv7180_write(client, REG_DEC_RD_EN, core); */

        /* Get Video Status */
	adv_status1 = i2c_smbus_read_byte_data(client, ADV7180_STATUS1_REG);
	if (adv_status1 < 0){
                dev_err(&client->dev, "ERROR: Reading ADV7180_STATUS1_REG in adv7180_get_video_std ret=0x%02x \n", adv_status1) ;
		return adv_status1;
        }

        v4l2_dbg(1, debug, &state->subdev, "%s\n",
		(adv_status1 & ADV7180_STATUS1_IN_LOCK) ?
		"Signal Present" : "Signal not present");

        if (adv_status1 & ADV7180_STATUS1_IN_LOCK) {
                v4l_info(client, "adv7180_get_video_std: ADV7180_SIGNAL_PRESENT \n");
                state->current_std = STD_NTSC_M_J;
        }
        else{
                /*V4L2_IN_ST_NO_SIGNAL ;*/
                v4l_info(client, "adv7180_get_video_std: ADV7180_STATUS1_REG  STATUS1_IN_LOCK=0 ret=0x%02x \n", adv_status1) ;
                state->current_std = STD_INVALID;
		//return state->current_std;
        }

	/* Get Video Standard
         * Denso RCG: This infortation is part of ADV7180_STATUS1_REG */
	/* ret = adv7180_read(client, REG_VID_STAND);*/
	adv_status3 = adv7180_read(client, ADV7180_STATUS3_REG);
	v4l2_dbg(1, debug, &state->subdev, "Video Standard : %s %dHz AD_RESULT[2:0] = 0x%02x \n",
		(adv_status1 & ADV7180_STATUS1_AUTOD_MASK) ==  (ADV7180_STATUS1_AUTOD_NTSC_M_J)?
                "NTSC 720x240 @" : "Unknown",
		(adv_status3 & ADV7180_STATUS3_SD_OP_50HZ) ? 50 : 60,
                (adv_status1 & ADV7180_STATUS1_AUTOD_MASK));

        state->std_list = adv7180_std_list;
        v4l_info(client, "adv7180_get_video_std: (ret & ADV7180_STATUS1_AUTOD_MASK) = 0x%02x \n", (adv_status1 & ADV7180_STATUS1_AUTOD_MASK));

        if ((adv_status1 & ADV7180_STATUS1_IN_LOCK) &&
            (adv_status1 & ADV7180_STATUS1_AUTOD_MASK) == ADV7180_STATUS1_AUTOD_NTSC_M_J) {
		state->current_std = STD_NTSC_M_J;
                //v4l_info(client, "adv7180_get_video_std (ret & ADV7180_STATUS1_AUTOD_MASK) = 0x%02x \n", (adv_status1 & ADV7180_STATUS1_AUTOD_MASK));
	} else{
                //dev_err(&client->dev, "ERROR: ADV7180_STATUS1_REG AUTOD=0 in adv7180_get_video_std ret=0x%02x \n", adv_status1) ;
		state->current_std = STD_INVALID;
        }

        /*state->std_list = adv7180_std_list;

        switch ((ret & ADV7180_STATUS1_AUTOD_MASK)>>4) {
	case ADV7180_STATUS1_AUTOD_NTSC_M_J:
		state->current_std = STD_NTSC_M_J;
                v4l_info(client, "adv7180_get_video_std AUTOD: STD_NTSC_M_J \n");
		break;
	case ADV7180_STATUS1_AUTOD_NTSC_4_43:
		state->current_std = STD_NTSC_4_43;
                v4l_info(client, "adv7180_get_video_std AUTOD: STD_NTSC_4_43 \n");
		break;
	case ADV7180_STATUS1_AUTOD_PAL_M:
		//state->current_std = STD_PAL_M;
		state->current_std = STD_INVALID;
                v4l_info(client, "adv7180_get_video_std AUTOD: STD_PAL_M \n");
		break;
	case ADV7180_STATUS1_AUTOD_PAL_60:
		//state->current_std = STD_PAL_60;
		state->current_std = STD_INVALID;
                v4l_info(client, "adv7180_get_video_std AUTOD: STD_PAL_60 \n");
		break;
	case ADV7180_STATUS1_AUTOD_PAL_B_G:
		//state->current_std = STD_PAL_B_G;
		state->current_std = STD_INVALID;
                v4l_info(client, "adv7180_get_video_std AUTOD: STD_PAL_B_G \n");
		break;
	case ADV7180_STATUS1_AUTOD_SECAM:
		//state->current_std = STD_SECAM;
		state->current_std = STD_INVALID;
                v4l_info(client, "adv7180_get_video_std AUTOD: STD_SECAM \n");
		break;
	case ADV7180_STATUS1_AUTOD_PAL_COMB:
		//state->current_std = STD_PAL_COMB;
		state->current_std = STD_INVALID;
                v4l_info(client, "adv7180_get_video_std AUTOD: STD_PAL_COMB \n");
		break;
	case ADV7180_STATUS1_AUTOD_SECAM_525:
		//state->current_std = STD_SECAM_525;
		state->current_std = STD_INVALID;
                v4l_info(client, "adv7180_get_video_std STD_SECAM_525 \n");
                break;
	}*/

	return state->current_std;
}


static int adv7180_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct adv7180_priv *state;
	struct v4l2_subdev *sd;
	int ret;
        int i2c_read;


        printk("adv7180: +adv7180_probe()\n");

	/* Check if the adapter supports the needed features */
	/*if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
		 client->addr, client->adapter->name);*/

        state = devm_kzalloc(&client->dev, sizeof(*state), GFP_KERNEL); // from TI tvp
	if (!state)
		return -ENOMEM;

        i2c_set_clientdata(client, state); //from TI tvp

        state->cfmt = &adv7180_cfmts[0]; //from TI tvp

        sd = &state->subdev;

	v4l2_i2c_subdev_init(sd, client, &adv7180_ops);

        /* Denso RCG: not GPIO's needed by ADV7180 */
	/*ret = adv7180_get_gpios(node, client);
	if (ret) {
		dev_err(&client->dev, "Unable to get gpios\n");
		return ret;
	}

	ret = adv7180_set_gpios(client);  //from TI tvp
	if (ret) {
		dev_err(&client->dev, "failed to set gpios ERR %d\n", ret);
		return ret;
	}*/

	state->current_std = STD_NTSC_M_J; /*STD_INVALID*/

        /* Get Chip ID and register with v4l2*/
	i2c_read = adv7180_read(client, ADV7180_IDENT_REG);   // Denso RCG: continue here.
	if (i2c_read == ADV7180_IDENT_40_LEAD) {
		v4l2_dbg(1, debug, sd, "Chip id : %x\n", i2c_read);
                /* Denso RCG: removing, ADV7180 has 1 video decoder channel */
		/*adv7180_write(client, REG_DEC_WR_EN, ADV_DECODER_1);*/

                /*adv7180_write(client, REG_OFM_CTRL,
		(TVP_VIDEO_PORT_ENABLE | TVP_OUT_CLK_P_EN));*/
	} else {
		dev_err(&client->dev, "ERROR: Chip id is not ADV7180");
		return -ENODEV;
	}

        ret = adv7180_set_default(client, state);
        if (ret)
		goto err_free_ctrl;

	adv7180_get_video_std(client);

        /* Denso RCG: this code does not seems to be needed */
        /*ret = adv7180_init_controls(state);
	if (ret)
		goto err_unreg_subdev;*/

        /* Denso RCG: This function should be remplaced by  adv7180_set_default */
        /*ret = init_device(client, state);
	if (ret)
		goto err_free_ctrl;*/

	/* V4l2 asyn subdev register */
	sd->dev = &client->dev;
	ret = v4l2_async_register_subdev(sd);
	if (!ret)
		v4l2_info(&state->subdev, "Camera sensor driver registered\n");
	else
		goto err_unreg_subdev;

	/* TO DO: Should be passed from DTS instead of hard-coding here */
	/* gpio11_0 => GPIO496 => BIT0 for camera mode */
	gpio_request(496, "BIT0_CAMERA_MODE");
	gpio_direction_output(496, 1);
	gpio_set_value_cansleep(496,1);
	gpio_export(496, false);
	printk("GPIO 496 BIT0 was exported\n");

	/* gpio11_1 => GPIO497 => BIT1 for camera mode */
	gpio_request(497, "BIT1_CAMERA_MODE");
	gpio_direction_output(497, 0);
	gpio_set_value_cansleep(497,0);
	gpio_export(497, false);
	printk("GPIO 497 BIT1 was exported\n");

	return 0;

err_free_ctrl:
	adv7180_exit_controls(state);
err_unreg_subdev:
	v4l2_device_unregister_subdev(sd);
	kfree(state);
//err:
	printk(KERN_ERR KBUILD_MODNAME ": Failed to probe: %d\n", ret);
	return ret;
}

static int adv7180_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	//struct adv7180_priv *state = to_adv7180(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_adv7180(client));
	return 0;
}


static const struct i2c_device_id adv7180_id[] = {
	{"ad,adv7180", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, adv7180_id);

static const struct of_device_id adv7180_dt_id[] = {
	{
	.compatible   = "ad,adv7180", .data = "adv7180"
	},
	{
	}
};

static struct i2c_driver adv7180_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   /*.name = KBUILD_MODNAME,*/
		   .name = "adv7180",
                   .of_match_table = adv7180_dt_id,
		   },
	.probe = adv7180_probe,
	.remove = adv7180_remove,
/*#ifdef CONFIG_PM
	.suspend = adv7180_suspend,
	.resume = adv7180_resume,
#endif*/
	.id_table = adv7180_id,
};

module_i2c_driver(adv7180_driver);

MODULE_DESCRIPTION("Analog Devices ADV7180 video decoder driver");
MODULE_AUTHOR("Mocean Laboratories");
MODULE_LICENSE("GPL v2");
