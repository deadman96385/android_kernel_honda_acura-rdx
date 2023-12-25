/*********************************************************
*
*  Copyright (c) 2014-2016 Honda R&D Americas, Inc.
*  All rights reserved. You may not copy, distribute, publicly display,
*  create derivative works from or otherwise use or modify this
*  software without first obtaining a license from Honda R&D Americas, Inc.
*
*********************************************************/

#ifndef __DESERIALIZER_H
#define __DESERIALIZER_H

/*
 * Definitions below originally from Texas Instruments.
 *
 * lvds based serial FPDLINK interface for onboard communication.
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 * Authors: Dandawate Saket <dsaket@ti.com>
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

/* NOTE: not every register is necessarily defined here. */
/* Future deserializers can be added here as well. */

/* definitions for the DS90UH928Q deserializer */
#define DES928_I2C_DEVICE_ID	0x00
#define DES928_RESET		0x01

#define DES928_CONFIG0		0x02
#define DES928_CONFIG1		0x03
#define DES928_BCC_WDT		0x04
#define DES928_BCC_WDT_MIN	(80)
#define DES928_I2C_CTL1		0x05
#define DES928_I2C_CTL2		0x06
#define DES928_SER_I2C_ID	0x07
#define DES928_SLAVE_ID0	0x08
#define DES928_SLAVE_ID1	0x09
#define DES928_SLAVE_ID2	0x0A
#define DES928_SLAVE_ID3	0x0B
#define DES928_SLAVE_ID4	0x0C
#define DES928_SLAVE_ID5	0x0D
#define DES928_SLAVE_ID6	0x0E
#define DES928_SLAVE_ID7	0x0F

#define DES928_SLAVE_AL_ID0	0x10
#define DES928_SLAVE_AL_ID1	0x11
#define DES928_SLAVE_AL_ID2	0x12
#define DES928_SLAVE_AL_ID3	0x13
#define DES928_SLAVE_AL_ID4	0x14
#define DES928_SLAVE_AL_ID5	0x15
#define DES928_SLAVE_AL_ID6	0x16
#define DES928_SLAVE_AL_ID7	0x17

#define DES928_MB0		0x18
#define DES928_MB1		0x19
#define DES928_FR_CNT		0x1B

#define DES928_GEN_STS		0x1C
#define DES928_GPIO_0		0x1D
#define DES928_GPIO_1_2		0x1E
#define DES928_GPIO_3		0x1F
#define DES928_GPIO_5_6		0x20
#define DES928_GPIO_7_8		0x21

#define DES928_DATA_CTL		0x22
#define DES928_RX_STS		0x23
#define DES928_BIST_CTL		0x24
#define DES928_BIST_CTL_ENABLE	(1<<0)
#define DES928_BIST_CTL_CONFIG	(1<<3)

#define DES928_BIST_ERR		0x25
#define DES928_SCL_HT		0x26
#define DES928_SCL_LT		0x27

#define DES928_DATA_CTL2	0x28
#define DES928_FRC_CTL		0x29
#define DES928_WB_CTL		0x2A
#define DES928_I2S_CTL		0x2B

#define DES928_AEQ_CTL		0x35
#define DES928_CLK_EN		0x39
#define DES928_I2S_DIVSEL	0x3A
#define DES928_AEQ_STS		0x3B
#define DES928_AEQ_BYPASS	0x44
#define DES928_AEQ_MIN_MAX	0x45
#define DES928_MAP_SEL		0x49

#define DES928_LOOP_DRV		0x56
#define DES928_PATTERN_GEN	0x64
#define DES928_PATTERN_GEN_ENABLE	(1 << 0)

#define DES928_PATTERN_CONF	0x65
#define DES928_PATTERN_CONF_AUTO_SCROLL	(1 << 0)
#define DES928_PATTERN_CONF_18_BIT	(1 << 4)

#define DES928_HDCP_RX_ID0	0xF0
#define DES928_HDCP_RX_ID1	0xF1
#define DES928_HDCP_RX_ID2	0xF2
#define DES928_HDCP_RX_ID3	0xF3
#define DES928_HDCP_RX_ID4	0xF4
#define DES928_HDCP_RX_ID5	0xF5

#endif	/* __DESERIALIZER_H */
