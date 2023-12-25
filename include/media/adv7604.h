/*
 * adv7604 - Analog Devices ADV7604 video decoder driver
 *
 * Copyright 2012 Cisco Systems, Inc. and/or its affiliates. All rights reserved.
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef _ADV7604_
#define _ADV7604_

#include <linux/types.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-of.h>

#define ADV7611_REVISION_ID 0x2051

/* IO Registers definition */
enum adv7611_io_reg {
	ADV7611_HDMI_LVL_RAW_STATUS_2 = 0x65,
	ADV7611_IO_MAX_REG_OFFSET = 0xFF,
	ADV7611_RD_INFO = 0xEA,
	ADV7611_RD_INFO_2 = 0xEB,
};

/* DPLL Afe */
enum adv76xx_dpll_reg {
	ADV76XX_DPLL_MAX_REG_OFFSET = 0xC8,
	ADV76XX_AUDIO_MISC = 0xA0,
	ADV76XX_MCLK_FS = 0xB5,
};

/* Specifics HDMI registers for ADV7611 */
enum adv7611_hdmi_reg {
	ADV7611_PACKETS_DETECTED_2 = 0x18,
	ADV7611_PACKETS_DETECTED_3 = 0x19,
	ADV7611_REGISTER_3CH = 0x3C,
	ADV7611_REGISTER_4CH = 0x4C,
	ADV7611_HPA_DELAY_SEL_3_0 = 0x6C,
	ADV7611_DSD_MAP_ROT_2_0 = 0x6D,
	ADV7611_DST_MAP_ROT_2_0 = 0x6E,
	ADV7611_DDC_PAD = 0x73,
	ADV7611_HDMI_REGISTER_02 = 0x83,
};

/* HDMI Registers definition */
enum adv76xx_hdmi_reg {
	ADV76XX_HDMI_MAX_REG_OFFSET = 0x96,
	ADV76XX_HDMI_REGISTER_00 = 0x00,
	ADV76XX_HDMI_REGISTER_01 = 0x01,
	ADV76XX_HDMI_REGISTER_03 = 0x03,
	ADV76XX_HDMI_REGISTER_04 = 0x04,
	ADV76XX_HDMI_REGISTER_05 = 0x05,
	ADV76XX_LINE_WIDTH_1 = 0x07,
	ADV76XX_LINE_WIDTH_2 = 0x08,
	ADV76XX_FIELD0_HEIGHT_1 = 0x09,
	ADV76XX_FIELD0_HEIGHT_2 = 0x0A,
	ADV76XX_FIELD1_HEIGHT_1 = 0x0B,
	ADV76XX_FIELD1_HEIGHT_2 = 0x0C,
	ADV76XX_HDMI_REGISTER_0D = 0x0D,
	ADV76XX_AUDIO_MUTE_SPEED = 0X0F,
	ADV76XX_HDMI_REGISTER_10 = 0x10,
	ADV76XX_AUDIO_FIFO_ALM_OST_FULL_THRES_HOLD = 0x11,
	ADV76XX_AUDIO_FIFO_ALM_OST_EMPTY_THRE_SHOLD = 0x12,
	ADV76XX_AUDIO_COAST_MASK = 0x13,
	ADV76XX_MUTE_MASK_21_16 = 0x14,
	ADV76XX_MUTE_MASK_15_8 = 0x15,
	ADV76XX_MUTE_MASK_7_0 = 0x16,
	ADV76XX_MUTE_CTRL = 0x1A,
	ADV76XX_DEEPCOLOR_FIFO_DEBUG_1 = 0x1B,
	ADV76XX_DEEPCOLOR_FIFO_DEBUG_2 = 0x1C,
	ADV76XX_REGISTER_1DH = 0x1D,
	ADV76XX_TOTAL_LINE_WIDTH_1 = 0x1E,
	ADV76XX_TOTAL_LINE_WIDTH_2 = 0x1F,
	ADV76XX_HSYNC_FRONT_PORCH_1 = 0x20,
	ADV76XX_HSYNC_FRONT_PORCH_2 = 0x21,
	ADV76XX_HSYNC_PULSE_WIDTH_1 = 0x22,
	ADV76XX_HSYNC_PULSE_WIDTH_2 = 0x23,
	ADV76XX_HSYNC_BACK_PORCH_1 = 0x24,
	ADV76XX_HSYNC_BACK_PORCH_2 = 0x25,
	ADV76XX_FIELD0_TOTAL_HEIGHT_1 = 0x26,
	ADV76XX_FIELD0_TOTAL_HEIGHT_2 = 0x27,
	ADV76XX_FIELD1_TOTAL_HEIGHT_1 = 0x28,
	ADV76XX_FIELD1_TOTAL_HEIGHT_2 = 0x29,
	ADV76XX_FIELD0_VS_FRONT_PORCH_1 = 0x2A,
	ADV76XX_FIELD0_VS_FRONT_PORCH_2 = 0x2B,
	ADV76XX_FIELD1_VS_FRONT_PORCH_1 = 0x2C,
	ADV76XX_FIELD1_VS_FRONT_PORCH_2 = 0x2D,
	ADV76XX_FIELD0_VS_PULSE_WIDTH_1 = 0x2E,
	ADV76XX_FIELD0_VS_PULSE_WIDTH_2 = 0x2F,
	ADV76XX_FIELD1_VS_PULSE_WIDTH_1 = 0x30,
	ADV76XX_FIELD1_VS_PULSE_WIDTH_2 = 0x31,
	ADV76XX_FIELD0_VS_BACK_PORCH_1 = 0x32,
	ADV76XX_FIELD0_VS_BACK_PORCH_2 = 0x33,
	ADV76XX_FIELD1_VS_BACK_PORCH_1 = 0x34,
	ADV76XX_FIELD1_VS_BACK_PORCH_2 = 0x35,
	ADV76XX_CHANNEL_STATUS_DATA_1 = 0x36,
	ADV76XX_CHANNEL_STATUS_DATA_2 = 0x37,
	ADV76XX_CHANNEL_STATUS_DATA_3 = 0x38,
	ADV76XX_CHANNEL_STATUS_DATA_4 = 0x39,
	ADV76XX_CHANNEL_STATUS_DATA_5 = 0x3A,
	ADV76XX_REGISTER_40H = 0x40,
	ADV76XX_REGISTER_41H = 0x41,
	ADV76XX_REGISTER_47H = 0x47,
	ADV76XX_REGISTER_48H = 0x48,
	ADV76XX_HDMI_REGISTER_50 = 0x50,
	ADV76XX_TMDSFREQ_8_1 = 0x51,
	ADV76XX_TMDSFREQ_FRAC = 0x52,
	ADV76XX_HDMI_COLORSPACE = 0x53,
	ADV76XX_FILT_5V_DET_REG = 0x56,
	ADV76XX_REGISTER_5AH = 0x5A,
	ADV76XX_CTS_N_1 = 0x5B,
	ADV76XX_CTS_N_2 = 0x5C,
	ADV76XX_CTS_N_3 = 0x5D,
	ADV76XX_CTS_N_4 = 0x5E,
	ADV76XX_CTS_N_5 = 0x5F,
	ADV76XX_HPA_DELAY_SEL_3_0 = 0x6C,
	ADV76XX_DSD_MAP_ROT_2_0 = 0x6D,
	ADV76XX_DST_MAP_ROT_2_0 = 0x6E,
	ADV76XX_EQ_DYNAMIC_FREQ  = 0x8C,
	ADV76XX_EQ_DYN1_LF = 0x8D,
	ADV76XX_EQ_DYN1_HF = 0x8E,
	ADV76XX_EQ_DYN2_LF = 0x90,
	ADV76XX_EQ_DYN2_HF = 0x91,
	ADV76XX_EQ_DYN3_LF = 0x93,
	ADV76XX_EQ_DYN3_HF = 0x94,
	ADV76XX_EQ_DYNAMIC_ENABLE = 0x96,
};

/* Analog input muxing modes (AFE register 0x02, [2:0]) */
enum adv7604_ain_sel {
	ADV7604_AIN1_2_3_NC_SYNC_1_2 = 0,
	ADV7604_AIN4_5_6_NC_SYNC_2_1 = 1,
	ADV7604_AIN7_8_9_NC_SYNC_3_1 = 2,
	ADV7604_AIN10_11_12_NC_SYNC_4_1 = 3,
	ADV7604_AIN9_4_5_6_SYNC_2_1 = 4,
};

/*
 * Bus rotation and reordering. This is used to specify component reordering on
 * the board and describes the components order on the bus when the ADV7604
 * outputs RGB.
 */
enum adv7604_bus_order {
	ADV7604_BUS_ORDER_RGB,		/* No operation	*/
	ADV7604_BUS_ORDER_GRB,		/* Swap 1-2	*/
	ADV7604_BUS_ORDER_RBG,		/* Swap 2-3	*/
	ADV7604_BUS_ORDER_BGR,		/* Swap 1-3	*/
	ADV7604_BUS_ORDER_BRG,		/* Rotate right	*/
	ADV7604_BUS_ORDER_GBR,		/* Rotate left	*/
};

/* Input Color Space (IO register 0x02, [7:4]) */
enum adv76xx_inp_color_space {
	ADV76XX_INP_COLOR_SPACE_LIM_RGB = 0,
	ADV76XX_INP_COLOR_SPACE_FULL_RGB = 1,
	ADV76XX_INP_COLOR_SPACE_LIM_YCbCr_601 = 2,
	ADV76XX_INP_COLOR_SPACE_LIM_YCbCr_709 = 3,
	ADV76XX_INP_COLOR_SPACE_XVYCC_601 = 4,
	ADV76XX_INP_COLOR_SPACE_XVYCC_709 = 5,
	ADV76XX_INP_COLOR_SPACE_FULL_YCbCr_601 = 6,
	ADV76XX_INP_COLOR_SPACE_FULL_YCbCr_709 = 7,
	ADV76XX_INP_COLOR_SPACE_AUTO = 0xf,
};

/* Select output format (IO register 0x03, [4:2]) */
enum adv7604_op_format_mode_sel {
	ADV7604_OP_FORMAT_MODE0 = 0x00,
	ADV7604_OP_FORMAT_MODE1 = 0x04,
	ADV7604_OP_FORMAT_MODE2 = 0x08,
};

/* Select output format (HDMI register 0x03, [6:5]) */
enum adv76xx_i2s_mode_sel {
	ADV76XX_I2SOUTMODE_MASK = 0x60,
	ADV76XX_I2SOUTMODE_I2S = 0x00,
	ADV76XX_I2SOUTMODE_RJ = 0x01 << 5,
	ADV76XX_I2SOUTMODE_LJ = 0x02 << 5,
	ADV76XX_I2SOUTMODE_SPDIF = 0x03 << 5,
};

/* Audio Package detection (HDMI register 0x18, [3:0]) */
enum adv76xx_audio_pckt_det {
	ADV76XX_AUDIO_SAMPLE_PCKT_DET = 0x01,
	ADV76XX_DSD_PACKET_DET = 0x02,
	ADV7611_DST_AUDIO_PCKT_DET = 0x04,
	ADV76XX_HBR_AUDIO_PCKT_DET = 0x08,
};

enum adv76xx_drive_strength {
	ADV76XX_DR_STR_MEDIUM_LOW = 1,
	ADV76XX_DR_STR_MEDIUM_HIGH = 2,
	ADV76XX_DR_STR_HIGH = 3,
};

/* INT1 Configuration (IO register 0x40, [1:0]) */
enum adv76xx_int1_config {
	ADV76XX_INT1_CONFIG_OPEN_DRAIN,
	ADV76XX_INT1_CONFIG_ACTIVE_LOW,
	ADV76XX_INT1_CONFIG_ACTIVE_HIGH,
	ADV76XX_INT1_CONFIG_DISABLED,
};

enum adv76xx_page {
	ADV76XX_PAGE_IO,
	ADV7604_PAGE_AVLINK,
	ADV76XX_PAGE_CEC,
	ADV76XX_PAGE_INFOFRAME,
	ADV7604_PAGE_ESDP,
	ADV7604_PAGE_DPP,
	ADV76XX_PAGE_AFE,
	ADV76XX_PAGE_REP,
	ADV76XX_PAGE_EDID,
	ADV76XX_PAGE_HDMI,
	ADV76XX_PAGE_TEST,
	ADV76XX_PAGE_CP,
	ADV7604_PAGE_VDP,
	ADV76XX_PAGE_MAX,
};

/* Platform dependent definition */
struct adv76xx_platform_data {
	/* DIS_PWRDNB: 1 if the PWRDNB pin is unused and unconnected */
	unsigned disable_pwrdnb:1;

	/* DIS_CABLE_DET_RST: 1 if the 5V pins are unused and unconnected */
	unsigned disable_cable_det_rst:1;

	int default_input;

	/* Analog input muxing mode */
	enum adv7604_ain_sel ain_sel;

	/* Bus rotation and reordering */
	enum adv7604_bus_order bus_order;

	/* Select output format mode */
	enum adv7604_op_format_mode_sel op_format_mode_sel;

	/* Configuration of the INT1 pin */
	enum adv76xx_int1_config int1_config;

	/* IO register 0x02 */
	unsigned alt_gamma:1;
	unsigned op_656_range:1;
	unsigned alt_data_sat:1;

	/* IO register 0x05 */
	unsigned blank_data:1;
	unsigned insert_av_codes:1;
	unsigned replicate_av_codes:1;

	/* IO register 0x06 */
	unsigned inv_vs_pol:1;
	unsigned inv_hs_pol:1;
	unsigned inv_llc_pol:1;

	/* IO register 0x14 */
	enum adv76xx_drive_strength dr_str_data;
	enum adv76xx_drive_strength dr_str_clk;
	enum adv76xx_drive_strength dr_str_sync;

	/* IO register 0x30 */
	unsigned output_bus_lsb_to_msb:1;

	/* Free run */
	unsigned hdmi_free_run_mode;

	/* i2c addresses: 0 == use default */
	u8 i2c_addresses[ADV76XX_PAGE_MAX];
};

enum adv76xx_pad {
	ADV76XX_PAD_HDMI_PORT_A = 0,
	ADV7604_PAD_HDMI_PORT_B = 1,
	ADV7604_PAD_HDMI_PORT_C = 2,
	ADV7604_PAD_HDMI_PORT_D = 3,
	ADV7604_PAD_VGA_RGB = 4,
	ADV7604_PAD_VGA_COMP = 5,
	/* The source pad is either 1 (ADV7611) or 6 (ADV7604) */
	ADV7604_PAD_SOURCE = 6,
	ADV7611_PAD_SOURCE = 1,
	ADV76XX_PAD_MAX = 7,
};

#define V4L2_CID_ADV_RX_ANALOG_SAMPLING_PHASE	(V4L2_CID_DV_CLASS_BASE + 0x1000)
#define V4L2_CID_ADV_RX_FREE_RUN_COLOR_MANUAL	(V4L2_CID_DV_CLASS_BASE + 0x1001)
#define V4L2_CID_ADV_RX_FREE_RUN_COLOR		(V4L2_CID_DV_CLASS_BASE + 0x1002)
#define V4L2_CID_PRIVATE_HDMI_PORT_ID		(V4L2_CID_USER_ADV7611_BASE + 0)
#define V4L2_CID_PRIVATE_HDMI_DEVICE_ID		(V4L2_CID_USER_ADV7611_BASE + 1)

/* notify events */
enum adv76xx_notify_events {
	HDMI_UNPLUG,
 	HDMI_PLUG,
};

enum adv76xx_type {
	ADV7604,
	ADV7611,
};

struct adv76xx_reg_seq {
	unsigned int reg;
	u8 val;
};

struct adv76xx_format_info {
	u32 code;
	u8 op_ch_sel;
	bool rgb_out;
	bool swap_cb_cr;
	u8 op_format_sel;
};

struct adv76xx_cfg_read_infoframe {
	const char *desc;
	u8 present_mask;
	u8 head_addr;
	u8 payload_addr;
};

struct adv76xx_chip_info {
	enum adv76xx_type type;

	bool has_afe;
	unsigned int max_port;
	unsigned int num_dv_ports;

	unsigned int edid_enable_reg;
	unsigned int edid_status_reg;
	unsigned int lcf_reg;

	unsigned int cable_det_mask;
	unsigned int tdms_lock_mask;
	unsigned int fmt_change_digital_mask;
	unsigned int cp_csc;

	const struct adv76xx_format_info *formats;
	unsigned int nformats;

	void (*set_termination)(struct v4l2_subdev *sd, bool enable);
	void (*setup_irqs)(struct v4l2_subdev *sd);
	unsigned int (*read_hdmi_pixelclock)(struct v4l2_subdev *sd);
	unsigned int (*read_cable_det)(struct v4l2_subdev *sd);

	/* 0 = AFE, 1 = HDMI */
	const struct adv76xx_reg_seq *recommended_settings[2];
	unsigned int num_recommended_settings[2];

	unsigned long page_mask;

	/* Masks for timings */
	unsigned int linewidth_mask;
	unsigned int field0_height_mask;
	unsigned int field1_height_mask;
	unsigned int hfrontporch_mask;
	unsigned int hsync_mask;
	unsigned int hbackporch_mask;
	unsigned int field0_vfrontporch_mask;
	unsigned int field1_vfrontporch_mask;
	unsigned int field0_vsync_mask;
	unsigned int field1_vsync_mask;
	unsigned int field0_vbackporch_mask;
	unsigned int field1_vbackporch_mask;
};

struct adv76xx_state {
	const struct adv76xx_chip_info *info;
	struct adv76xx_platform_data pdata;
	struct platform_device *pdev_snd_codec;

	struct gpio_desc *hpd_gpio[4];

	struct v4l2_subdev sd;
	struct media_pad pads[ADV76XX_PAD_MAX];
	unsigned int source_pad;

	struct v4l2_ctrl_handler hdl;

	enum adv76xx_pad selected_input;

	struct v4l2_dv_timings timings;
	const struct adv76xx_format_info *format;

	struct {
		u8 edid[256];
		u32 present;
		unsigned blocks;
	} edid;
	u16 spa_port_a[2];
	struct v4l2_fract aspect_ratio;
	u32 rgb_quantization_range;
	struct workqueue_struct *work_queues;
	struct delayed_work delayed_work_enable_hotplug;
	bool restart_stdi_once;

	/* Regmaps */
	struct regmap *regmap[ADV76XX_PAGE_MAX];

	/* i2c clients */
	struct i2c_client *i2c_clients[ADV76XX_PAGE_MAX];

	/* controls */
	struct v4l2_ctrl *detect_tx_5v_ctrl;
	struct v4l2_ctrl *analog_sampling_phase_ctrl;
	struct v4l2_ctrl *free_run_color_manual_ctrl;
	struct v4l2_ctrl *free_run_color_ctrl;
	struct v4l2_ctrl *rgb_quantization_range_ctrl;
	struct v4l2_ctrl *hdmi_port_id_ctrl;
	struct v4l2_ctrl *hdmi_device_id_ctrl;
};

/* Bus rotation and reordering (IO register 0x04, [7:5]) */
enum adv7604_op_ch_sel {
	ADV7604_OP_CH_SEL_GBR = 0,
	ADV7604_OP_CH_SEL_GRB = 1,
	ADV7604_OP_CH_SEL_BGR = 2,
	ADV7604_OP_CH_SEL_RGB = 3,
	ADV7604_OP_CH_SEL_BRG = 4,
	ADV7604_OP_CH_SEL_RBG = 5,
};

/* Input Color Space (IO register 0x02, [7:4]) */
enum adv7604_inp_color_space {
	ADV7604_INP_COLOR_SPACE_LIM_RGB = 0,
	ADV7604_INP_COLOR_SPACE_FULL_RGB = 1,
	ADV7604_INP_COLOR_SPACE_LIM_YCbCr_601 = 2,
	ADV7604_INP_COLOR_SPACE_LIM_YCbCr_709 = 3,
	ADV7604_INP_COLOR_SPACE_XVYCC_601 = 4,
	ADV7604_INP_COLOR_SPACE_XVYCC_709 = 5,
	ADV7604_INP_COLOR_SPACE_FULL_YCbCr_601 = 6,
	ADV7604_INP_COLOR_SPACE_FULL_YCbCr_709 = 7,
	ADV7604_INP_COLOR_SPACE_AUTO = 0xf,
};

/* Select output format (IO register 0x03, [7:0]) */
enum adv7604_op_format_sel {
	ADV7604_OP_FORMAT_SEL_SDR_ITU656_8 = 0x00,
	ADV7604_OP_FORMAT_SEL_SDR_ITU656_10 = 0x01,
	ADV7604_OP_FORMAT_SEL_SDR_ITU656_12_MODE0 = 0x02,
	ADV7604_OP_FORMAT_SEL_SDR_ITU656_12_MODE1 = 0x06,
	ADV7604_OP_FORMAT_SEL_SDR_ITU656_12_MODE2 = 0x0a,
	ADV7604_OP_FORMAT_SEL_DDR_422_8 = 0x20,
	ADV7604_OP_FORMAT_SEL_DDR_422_10 = 0x21,
	ADV7604_OP_FORMAT_SEL_DDR_422_12_MODE0 = 0x22,
	ADV7604_OP_FORMAT_SEL_DDR_422_12_MODE1 = 0x23,
	ADV7604_OP_FORMAT_SEL_DDR_422_12_MODE2 = 0x24,
	ADV7604_OP_FORMAT_SEL_SDR_444_24 = 0x40,
	ADV7604_OP_FORMAT_SEL_SDR_444_30 = 0x41,
	ADV7604_OP_FORMAT_SEL_SDR_444_36_MODE0 = 0x42,
	ADV7604_OP_FORMAT_SEL_DDR_444_24 = 0x60,
	ADV7604_OP_FORMAT_SEL_DDR_444_30 = 0x61,
	ADV7604_OP_FORMAT_SEL_DDR_444_36 = 0x62,
	ADV7604_OP_FORMAT_SEL_SDR_ITU656_16 = 0x80,
	ADV7604_OP_FORMAT_SEL_SDR_ITU656_20 = 0x81,
	ADV7604_OP_FORMAT_SEL_SDR_ITU656_24_MODE0 = 0x82,
	ADV7604_OP_FORMAT_SEL_SDR_ITU656_24_MODE1 = 0x86,
	ADV7604_OP_FORMAT_SEL_SDR_ITU656_24_MODE2 = 0x8a,
};

enum adv7604_drive_strength {
	ADV7604_DR_STR_MEDIUM_LOW = 1,
	ADV7604_DR_STR_MEDIUM_HIGH = 2,
	ADV7604_DR_STR_HIGH = 3,
};


enum adv7604_int1_config {
	ADV7604_INT1_CONFIG_OPEN_DRAIN,
	ADV7604_INT1_CONFIG_ACTIVE_LOW,
	ADV7604_INT1_CONFIG_ACTIVE_HIGH,
	ADV7604_INT1_CONFIG_DISABLED,
};

/* Jackie Porting from 3.18 */
/* Platform dependent definition */
struct adv7604_platform_data {
	/* DIS_PWRDNB: 1 if the PWRDNB pin is unused and unconnected */
	unsigned disable_pwrdnb:1;

	/* DIS_CABLE_DET_RST: 1 if the 5V pins are unused and unconnected */
	unsigned disable_cable_det_rst:1;

	int default_input;

	/* Analog input muxing mode */
	enum adv7604_ain_sel ain_sel;

	/* Bus rotation and reordering */
	enum adv7604_bus_order bus_order;

	/* Select output format mode */
	enum adv7604_op_format_mode_sel op_format_mode_sel;

	/* Bus rotation and reordering */
	enum adv7604_op_ch_sel op_ch_sel;

	/* Select output format */
	enum adv7604_op_format_sel op_format_sel;

	/* Configuration of the INT1 pin *//* Jackie Porting from 3.18 */
	enum adv7604_int1_config int1_config;

	/* IO register 0x02 */
	unsigned alt_gamma:1;
	unsigned op_656_range:1;
//	unsigned rgb_out:1;
	unsigned alt_data_sat:1;

	/* IO register 0x05 */
	unsigned blank_data:1;
	unsigned insert_av_codes:1;
	unsigned replicate_av_codes:1;
//	unsigned invert_cbcr:1;

	/* IO register 0x06 */
	unsigned inv_vs_pol:1;
	unsigned inv_hs_pol:1;
	unsigned inv_llc_pol:1;   /* Jackie porting from 3.18 */

	/* IO register 0x14 */
	enum adv7604_drive_strength dr_str_data;
	enum adv7604_drive_strength dr_str_clk;
	enum adv7604_drive_strength dr_str_sync;

	/* IO register 0x30 */
	unsigned output_bus_lsb_to_msb:1;

	/* Free run */
	unsigned hdmi_free_run_mode;

	/* i2c addresses: 0 == use default */
	u8 i2c_avlink;
	u8 i2c_cec;
	u8 i2c_infoframe;
	u8 i2c_esdp;
	u8 i2c_dpp;
	u8 i2c_afe;
	u8 i2c_repeater;
	u8 i2c_edid;
	u8 i2c_hdmi;
	u8 i2c_test;
	u8 i2c_cp;
	u8 i2c_vdp;

	/* i2c addresses: 0 == use default */ /* Jackie porting from 3.18 */
	u8 i2c_addresses[ADV76XX_PAGE_MAX];
	
};

enum adv7604_input_port {
	ADV7604_INPUT_HDMI_PORT_A,
	ADV7604_INPUT_HDMI_PORT_B,
	ADV7604_INPUT_HDMI_PORT_C,
	ADV7604_INPUT_HDMI_PORT_D,
	ADV7604_INPUT_VGA_RGB,
	ADV7604_INPUT_VGA_COMP,
};

#define ADV7604_EDID_PORT_A 0
#define ADV7604_EDID_PORT_B 1
#define ADV7604_EDID_PORT_C 2
#define ADV7604_EDID_PORT_D 3

/* notify events */
#define ADV7604_HOTPLUG		1
#define ADV7604_FMT_CHANGE	2


//-----------------------

struct adv7604_video_standards {
	struct v4l2_dv_timings timings;
	u8 vid_std;
	u8 v_freq;
};


/*
 * Mode of operation.
 * This is used as the input argument of the s_routing video op.
 */
enum adv7604_mode {
	ADV7604_MODE_COMP,
	ADV7604_MODE_GR,
	ADV7604_MODE_HDMI,
};


#define ADV7611_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE |\
			SNDRV_PCM_FMTBIT_S32_LE)

//#define DIGITAL_INPUT                       (state->mode == ADV7604_MODE_HDMI)

#define ADV7604_REG(page, offset)           (((page) << 8) | (offset))
#define ADV7604_REG_SEQ_TERM                0xffff

/* VERT_FILTER_LOCKED and DE_REGEN_FILTER_LOCKED flags */
#define adv7611_HDMI_F_LOCKED(v)            (((v) & 0xa0) == 0xa0)               // Jackie Added

/* Maximum supported resolution */
#define adv7611_MAX_WIDTH                    640         //1920            // Jackie Added
#define adv7611_MAX_HEIGHT                   480         //    1080        // Jackie Added

struct adv7611_format_info {
    enum v4l2_mbus_pixelcode code;
    u8 op_ch_sel;
    bool rgb_out;
    bool swap_cb_cr;
    u8 op_format_sel;
};

struct adv7611_color_format {
    enum v4l2_mbus_pixelcode code;
    enum v4l2_colorspace colorspace;
};

/* Supported color format list */
static const struct adv7611_color_format adv7611_cfmts[] = {
    {
        .code = V4L2_MBUS_FMT_RGB888_1X24,
        .colorspace = V4L2_COLORSPACE_SRGB,
    },
};

struct adv7611_reg_seq {
    unsigned int reg;
    u8 val;
};

struct adv7611_chip_info {
    enum adv76xx_type type;

    bool has_afe;
    unsigned int max_port;
    unsigned int num_dv_ports;

    unsigned int edid_enable_reg;
    unsigned int edid_status_reg;
    unsigned int lcf_reg;

    unsigned int cable_det_mask;
    unsigned int tdms_lock_mask;
    unsigned int fmt_change_digital_mask;

    const struct adv7611_format_info *formats;
    unsigned int nformats;

    void (*set_termination)(struct v4l2_subdev *sd, bool enable);
    void (*setup_irqs)(struct v4l2_subdev *sd);
    unsigned int (*read_hdmi_pixelclock)(struct v4l2_subdev *sd);
    unsigned int (*read_cable_det)(struct v4l2_subdev *sd);

    /* 0 = AFE, 1 = HDMI */
    const struct adv7611_reg_seq *recommended_settings[2];
    unsigned int num_recommended_settings[2];

    unsigned long page_mask;
};


struct gpio_desc {
	struct gpio_chip	*chip;
	unsigned long		flags;
/* flag symbols are bit numbers */
#define FLAG_REQUESTED	0
#define FLAG_IS_OUT	1
#define FLAG_EXPORT	2	/* protected by sysfs_lock */
#define FLAG_SYSFS	3	/* exported via /sys/class/gpio/control */
#define FLAG_TRIG_FALL	4	/* trigger on falling edge */
#define FLAG_TRIG_RISE	5	/* trigger on rising edge */
#define FLAG_ACTIVE_LOW	6	/* value has active low */
#define FLAG_OPEN_DRAIN	7	/* Gpio is open drain type */
#define FLAG_OPEN_SOURCE 8	/* Gpio is open source type */
#define FLAG_USED_AS_IRQ 9	/* GPIO is connected to an IRQ */
#define FLAG_IS_HOGGED	10	/* GPIO is hogged */

#define ID_SHIFT	16	/* add new flags before this one */

#define GPIO_FLAGS_MASK		((1 << ID_SHIFT) - 1)
#define GPIO_TRIGGER_MASK	(BIT(FLAG_TRIG_FALL) | BIT(FLAG_TRIG_RISE))

#ifdef CONFIG_DEBUG_FS
	const char		*label;
#endif
};


struct adv7611_state {
	
    const struct adv7611_chip_info   *info;                 //[JACKIE]  new
    enum adv76xx_pad                 selected_input;        //[JACKIE]  new
    struct gpio_desc                 *hpd_gpio[4];          //[JACKIE]  new
    const struct adv7611_format_info *format;               //[JACKIE]  new
    unsigned int                     source_pad;            //[JACKIE]  new
    struct media_pad                 pads[ADV76XX_PAD_MAX]; //[JACKIE]  new
    struct work_struct               interrupt_service;     // Jackie Added 2015.03.16
    struct workqueue_struct          *work_queue;           // Jackie Added 2015.03.16
    u32                               status;   // Jackie Added 2015.03.17
    u32                               width;    // Jackie Added 2015.03.17
    u32                               height;   // Jackie Added 2015.03.17
    enum v4l2_field                   scanmode; // Jackie Added 2015.03.17
    const struct adv7611_color_format *cfmt;    // Jackie Added 2015.03.17
    int gpio;                                   // Jackie Added 2015.03.17

    /* Regmaps */
    struct regmap *regmap[ADV76XX_PAGE_MAX];    // Jackie added for Audio 2015.05.07

    struct adv7604_platform_data    pdata;
    struct v4l2_subdev              sd;
    struct media_pad                pad;
    struct v4l2_ctrl_handler        hdl;
    enum adv7604_mode               mode;
    struct v4l2_dv_timings          timings;

    struct {                                    // Jackie added
        u8          edid[256];
        u32         present;
        unsigned    blocks;
    } edid;
    //u8 edid[256];                             //old
	u16 spa_port_a[2];

    unsigned                edid_blocks;
    struct v4l2_fract       aspect_ratio;
    u32                     rgb_quantization_range;
    struct workqueue_struct *work_queues;
    struct delayed_work     delayed_work_enable_hotplug;
    bool                    connector_hdmi;
    bool                    restart_stdi_once;

    /* i2c clients */
    struct i2c_client       *i2c_avlink;
    struct i2c_client       *i2c_cec;
    struct i2c_client       *i2c_infoframe;
    struct i2c_client       *i2c_esdp;
    struct i2c_client       *i2c_dpp;
    struct i2c_client       *i2c_afe;
    struct i2c_client       *i2c_repeater;
    struct i2c_client       *i2c_edid;
    struct i2c_client       *i2c_hdmi;
    struct i2c_client       *i2c_test;
    struct i2c_client       *i2c_cp;
    struct i2c_client       *i2c_vdp;
    struct i2c_client       *i2c_clients[ADV76XX_PAGE_MAX]; /*[JACKIE] new i2c clients */

    /* controls */
    struct v4l2_ctrl        *detect_tx_5v_ctrl;
    struct v4l2_ctrl        *analog_sampling_phase_ctrl;
    struct v4l2_ctrl        *free_run_color_manual_ctrl;
    struct v4l2_ctrl        *free_run_color_ctrl;
    struct v4l2_ctrl        *rgb_quantization_range_ctrl;
};


/* Number of pixels and number of lines per frame for different standards */
#define NTSC_NUM_ACTIVE_PIXELS      (720)
#define NTSC_NUM_ACTIVE_LINES       (480)
#define PAL_NUM_ACTIVE_PIXELS       (720)
#define PAL_NUM_ACTIVE_LINES        (576)
#define AUTO_NUM_ACTIVE_PIXELS      (720)
#define AUTO_NUM_ACTIVE_LINES       (576)

struct adv7604_reg_seq
{
    unsigned int    reg;
    u8              val;
};

#endif
