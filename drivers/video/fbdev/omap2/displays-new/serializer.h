/*********************************************************
*
*  Copyright (c) 2014-2016 Honda R&D Americas, Inc.
*  All rights reserved. You may not copy, distribute, publicly display,
*  create derivative works from or otherwise use or modify this
*  software without first obtaining a license from Honda R&D Americas, Inc.
*
*********************************************************/

#ifndef __SERIALIZER_H
#define __SERIALIZER_H

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
/* Future serializers can be added here as well. */

/* definitions for the DS90UH925Q serializer */
#define SER925_ALT_I2C_ADDRESS	0x17
#define SER925_I2C_DEVICE_ID	0x00
#define SER925_RESET		0x01
#define SER925_RESET_DIGITAL_RESET0	(1 << 0)
#define SER925_RESET_DIGITAL_RESET1	(1 << 1)

#define SER925_CONFIG0		0x03
#define SER925_CONFIG0_BACK_CHAN_CRC	(1 << 7)
#define SER925_CONFIG0_RES6		(1 << 6)
#define SER925_CONFIG0_I2C_AUTO		(1 << 5)
#define SER925_CONFIG0_FILTER_EN	(1 << 4)
#define SER925_CONFIG0_I2C_PASSTHRU	(1 << 3)
#define SER925_CONFIG0_RES2		(1 << 2)
#define SER925_CONFIG0_PCLK_AUTO	(1 << 1)
#define SER925_CONFIG0_TRFB		(1 << 0)
#define SER925_CONFIG1		0x04
#define SER925_CONFIG1_
#define SER925_CONFIG1_FAILSAFE		(1 << 7)
#define SER925_CONFIG1_RES6		(1 << 6)
#define SER925_CONFIG1_CRC_ERR_RESET	(1 << 5)
#define SER925_CONFIG1_RES4		(1 << 4)
#define SER925_CONFIG1_BCMODE_SEL	(1 << 3)
#define SER925_CONFIG1_BCMODE		(1 << 2)
#define SER925_CONFIG1_LFMODE_SEL	(1 << 1)
#define SER925_CONFIG1_LFMODE		(1 << 0)
#define SER925_I2C_CONTROL	0x05
#define SER925_I2C_DES_ID	0x06
#define SER925_I2C_SLAVE_ID	0x07
#define SER925_I2C_SLAVE_ALIAS	0x08
#define SER925_CRC_LSB		0x0A
#define SER925_CRC_MSB		0x0B
#define SER925_GEN_STS		0x0C
#define SER925_GEN_STS_PCLK_DETECT	(1 << 2)
#define SER925_GEN_STS_LINK_DETECT	(1 << 0)

/* GPIO settings*/
#define SER925_ID_GPIO0		0x0D
#define SER925_GPIO1_GPIO2	0x0E
#define SER925_GPIO3_GPIO4	0x0F
#define SER925_GPIO5_GPIO6	0x10
#define SER925_GPIO7_GPIO8	0x11

/* Setup SER925 modes and configuration */
#define SER925_DATA_CTL		0x12
#define SER925_MODE_STS		0x13
#define SER925_MODE_OSC		0x14
#define SER925_MODE_OSC_BIST_ENABLE	(1<<0)

#define SER925_WDG_CTL		0x16

/* I2C Timing */
#define SER925_I2C_CTL		0x17
#define SER925_I2C_SCL_HT	0x18
#define SER925_I2C_SCL_LT	0x19

#define SER925_BC_ERR		0x1B

/* Pattern Generation */
#define SER925_PG_CTL		0x64
#define SER925_PG_CTL_ENABLE		(1 << 0)
#define SER925_PG_CTL_DISABLE	0x10

#define SER925_PG_CONF		0x65
#define SER925_PG_CONF_AUTO_SCROLL	(1 << 0)
#define SER925_PG_CONF_18_BIT		(1 << 4)


#define SER925_PG_IND_ADDR	0x66
#define SER925_PG_IND_DATA	0x67

/* Derializer KSV */
#define SER925_RX_BKSV0		0x80
#define SER925_RX_BKSV1		0x81
#define SER925_RX_BKSV2		0x82
#define SER925_RX_BKSV3		0x83
#define SER925_RX_BKSV4		0x84
/* Serializer KSV */
#define SER925_TX_KSV0		0x90
#define SER925_TX_KSV1		0x91
#define SER925_TX_KSV2		0x92
#define SER925_TX_KSV3		0x93
#define SER925_TX_KSV4		0x94

#define SER925_RX_BCAPS		0xA0
#define SER925_RX_BCAPS_KSV_FIFO	(1 << 5)
#define SER925_RX_BSTS0		0xA1
#define SER925_RX_BSTS1		0xA2
#define SER925_KSV_FIFO		0xA3
#define SER925_HDCP_DBG		0xC0

/* HDCP */
#define SER925_HDCP_CFG		0xC2
#define	SER925_HDCP_CFG_RX_DET_SEL	(1 << 1)

#define SER925_HDCP_CTL		0xC3
#define	SER925_HDCP_CTL_HDCP_RST	(1 << 7)
#define	SER925_HDCP_CTL_RESERVED6	(1 << 6)
#define	SER925_HDCP_CTL_KSV_LIST_VALID	(1 << 5)
#define	SER925_HDCP_CTL_KSV_VALID	(1 << 4)
#define	SER925_HDCP_CTL_HDCP_ENC_DIS	(1 << 3)
#define	SER925_HDCP_CTL_HDCP_ENC_EN	(1 << 2)
#define	SER925_HDCP_CTL_HDCP_DIS	(1 << 1)
#define	SER925_HDCP_CTL_HDCP_EN		(1 << 0)

#define SER925_HDCP_STS		0xC4
#define SER925_HDCP_STS_I2C_ERR_DET	(1 << 7)
#define SER925_HDCP_STS_RX_INT		(1 << 6)
#define SER925_HDCP_STS_RX_LOCK_DET	(1 << 5)
#define	SER925_HDCP_STS_DOWN_HPD	(1 << 4)
#define	SER925_HDCP_STS_RX_DET		(1 << 3)
#define	SER925_HDCP_STS_KSV_LIST_READY	(1 << 2)
#define	SER925_HDCP_STS_KSY_READY	(1 << 1)
#define	SER925_HDCP_STS_AUTHED		(1 << 0)

#define SER925_HDCP_ICR		0xC6
#define	SER925_HDCP_ICR_IE_IND_ACC	(1 << 7)
#define	SER925_HDCP_ICR_IE_RXDET_INT	(1 << 6)
#define	SER925_HDCP_ICR_IS_RX_INT	(1 << 5)
#define	SER925_HDCP_ICR_IE_LIST_RDY	(1 << 4)
#define	SER925_HDCP_ICR_IE_KSV_RDY	(1 << 3)
#define	SER925_HDCP_ICR_IE_AUTH_FAIL	(1 << 2)
#define	SER925_HDCP_ICR_IE_AUTH_PASS	(1 << 1)
#define	SER925_HDCP_ICR_INT_ENABLE	(1 << 0)

#define SER925_HDCP_ISR		0xC7
#define	SER925_HDCP_ISR_IS_IND_ACCESS	(1 << 7)
#define	SER925_HDCP_ISR_INT_DETECT	(1 << 6)
#define	SER925_HDCP_ISR_IS_RX_INT	(1 << 5)
#define	SER925_HDCP_ISR_IS_LIST_RDY	(1 << 4)
#define	SER925_HDCP_ISR_IS_KSV_RDY	(1 << 3)
#define	SER925_HDCP_ISR_IS_AUTH_FAIL	(1 << 2)
#define	SER925_HDCP_ISR_IS_AUTH_PASS	(1 << 1)
#define	SER925_HDCP_ISR_INT		(1 << 0)

#define SER925_HDCP_TX_ID0	0xF0
#define SER925_HDCP_TX_ID1	0xF1
#define SER925_HDCP_TX_ID2	0xF2
#define SER925_HDCP_TX_ID3	0xF3
#define SER925_HDCP_TX_ID4	0xF4
#define SER925_HDCP_TX_ID5	0xF5

#endif	/* __SERIALIZER_H */
