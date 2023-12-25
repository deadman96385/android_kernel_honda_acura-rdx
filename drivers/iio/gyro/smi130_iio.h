/*
 * (C) Copyright 2013 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 *	SMI130 IIO driver releated definition
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

#ifndef __SMI130_IIO_H__
#define __SMI130_IIO_H__

#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#ifdef __linux__
#define SMI130_U16 unsigned short       /* 16 bit achieved with short */
#define SMI130_S16 signed short
#define SMI130_S32 signed int           /* 32 bit achieved with int   */
#else
#include <linux/limits.h> /*needed to test integer limits */


/* find correct data type for signed/unsigned 16 bit variables \
by checking max of unsigned variant */
#if USHRT_MAX == 0xFFFF
		/* 16 bit achieved with short */
		#define SMI130_U16 unsigned short
		#define SMI130_S16 signed short
#elif UINT_MAX == 0xFFFF
		/* 16 bit achieved with int */
		#define SMI130_U16 unsigned int
		#define SMI130_S16 signed int
#else
		#error SMI130_U16 and SMI130_S16 could not be
		#error defined automatically, please do so manually
#endif

/* find correct data type for signed 32 bit variables */
#if INT_MAX == 0x7FFFFFFF
		/* 32 bit achieved with int */
		#define SMI130_S32 signed int
#elif LONG_MAX == 0x7FFFFFFF
		/* 32 bit achieved with long int */
		#define SMI130_S32 signed long int
#else
		#error SMI130_S32 could not be
		#error defined automatically, please do so manually
#endif
#endif

/**\brief defines the calling parameter types of the SMI130_WR_FUNCTION */
#define SMI130_BUS_WR_RETURN_TYPE char

/**\brief links the order of parameters defined in
SMI130_BUS_WR_PARAM_TYPE to function calls used inside the API*/
#define SMI130_BUS_WR_PARAM_TYPES unsigned char, unsigned char,\
unsigned char *, unsigned char

/**\brief links the order of parameters defined in
SMI130_BUS_WR_PARAM_TYPE to function calls used inside the API*/
#define SMI130_BUS_WR_PARAM_ORDER(device_addr, register_addr,\
register_data, wr_len)

/**\brief defines the return parameter type of the SMI130_RD_FUNCTION
*/
#define SMI130_BUS_RD_RETURN_TYPE char
/**\brief defines the calling parameter types of the SMI130_RD_FUNCTION
*/
#define SMI130_BUS_RD_PARAM_TYPES unsigned char, unsigned char,\
unsigned char *, unsigned char
/**\brief links the order of parameters defined in \
SMI130_BUS_RD_PARAM_TYPE to function calls used inside the API
*/
#define SMI130_BUS_RD_PARAM_ORDER (device_addr, register_addr,\
register_data)
/**\brief defines the return parameter type of the SMI130_RD_FUNCTION
*/
#define SMI130_BURST_RD_RETURN_TYPE char
/**\brief defines the calling parameter types of the SMI130_RD_FUNCTION
*/
#define SMI130_BURST_RD_PARAM_TYPES unsigned char,\
unsigned char, unsigned char *, signed int
/**\brief links the order of parameters defined in \
SMI130_BURST_RD_PARAM_TYPE to function calls used inside the API
*/
#define SMI130_BURST_RD_PARAM_ORDER (device_addr, register_addr,\
register_data)

/**\brief defines the return parameter type of the delay_funcTION
*/
#define SMI130_DELAY_RETURN_TYPE void
/**\brief defines the calling parameter types of the delay_funcTION
*/
#define SMI130_DELAY_PARAM_TYPES SMI130_U16

#define	SMI130_I2C_ADDR1				0x68
#define	SMI130_I2C_ADDR					SMI130_I2C_ADDR1
#define	SMI130_I2C_ADDR2				0x69



/*Define of registers*/

/* Hard Wired */
#define SMI130_CHIP_ID_ADDR						0x00
/**<Address of Chip ID Register*/


/* Data Register */
#define SMI130_RATE_X_LSB_ADDR                   0x02
/**<        Address of X axis Rate LSB Register       */
#define SMI130_RATE_X_MSB_ADDR                   0x03
/**<        Address of X axis Rate MSB Register       */
#define SMI130_RATE_Y_LSB_ADDR                   0x04
/**<        Address of Y axis Rate LSB Register       */
#define SMI130_RATE_Y_MSB_ADDR                   0x05
/**<        Address of Y axis Rate MSB Register       */
#define SMI130_RATE_Z_LSB_ADDR                   0x06
/**<        Address of Z axis Rate LSB Register       */
#define SMI130_RATE_Z_MSB_ADDR                   0x07
/**<        Address of Z axis Rate MSB Register       */
#define SMI130_TEMP_ADDR                        0x08
/**<        Address of Temperature Data LSB Register  */

/* Status Register */
#define SMI130_INT_STATUS0_ADDR                 0x09
/**<        Address of Interrupt status Register 0    */
#define SMI130_INT_STATUS1_ADDR                 0x0A
/**<        Address of Interrupt status Register 1    */
#define SMI130_INT_STATUS2_ADDR                 0x0B
/**<        Address of Interrupt status Register 2    */
#define SMI130_INT_STATUS3_ADDR                 0x0C
/**<        Address of Interrupt status Register 3    */
#define SMI130_FIFO_STATUS_ADDR                 0x0E
/**<        Address of FIFO status Register           */

/* Control Register */
#define SMI130_RANGE_ADDR                  0x0F
/**<        Address of Range address Register     */
#define SMI130_BW_ADDR                     0x10
/**<        Address of Bandwidth Register         */
/*
#define SMI130_MODE_LPM1_ADDR              0x11
*/
/**<        Address of Mode LPM1 Register         */
/*
#define SMI130_MODE_LPM2_ADDR              0x12
*/
/**<        Address of Mode LPM2 Register         */
#define SMI130_RATED_HBW_ADDR              0x13
/**<        Address of Rate HBW Register          */
#define SMI130_BGW_SOFTRESET_ADDR          0x14
/**<        Address of BGW Softreset Register      */
#define SMI130_INT_ENABLE0_ADDR            0x15
/**<        Address of Interrupt Enable 0             */
#define SMI130_INT_ENABLE1_ADDR            0x16
/**<        Address of Interrupt Enable 1             */
#define SMI130_INT_MAP_0_ADDR              0x17
/**<        Address of Interrupt MAP 0                */
#define SMI130_INT_MAP_1_ADDR              0x18
/**<        Address of Interrupt MAP 1                */
#define SMI130_INT_MAP_2_ADDR              0x19
/**<        Address of Interrupt MAP 2                */
#define SMI130_INT_0_ADDR                  0x1A
/**<        Address of Interrupt 0 register   */
#define SMI130_INT_1_ADDR                  0x1B
/**<        Address of Interrupt 1 register   */
#define SMI130_INT_2_ADDR                  0x1C
/**<        Address of Interrupt 2 register   */
#define SMI130_INT_4_ADDR                  0x1E
/**<        Address of Interrupt 4 register   */
#define SMI130_RST_LATCH_ADDR              0x21
/**<        Address of Reset Latch Register           */
#define SMI130_HIGH_TH_X_ADDR              0x22
/**<        Address of High Th x Address register     */
#define SMI130_HIGH_DUR_X_ADDR             0x23
/**<        Address of High Dur x Address register    */
#define SMI130_HIGH_TH_Y_ADDR              0x24
/**<        Address of High Th y  Address register    */
#define SMI130_HIGH_DUR_Y_ADDR             0x25
/**<        Address of High Dur y Address register    */
#define SMI130_HIGH_TH_Z_ADDR              0x26
/**<        Address of High Th z Address register  */
#define SMI130_HIGH_DUR_Z_ADDR             0x27
/**<        Address of High Dur z Address register  */
#define SMI130_SOC_ADDR                        0x31
/**<        Address of SOC register        */
#define SMI130_A_FOC_ADDR                      0x32
/**<        Address of A_FOC Register        */
#define SMI130_TRIM_NVM_CTRL_ADDR          0x33
/**<        Address of Trim NVM control register      */
#define SMI130_BGW_SPI3_WDT_ADDR           0x34
/**<        Address of BGW SPI3,WDT Register           */


/* Trim Register */
#define SMI130_OFC1_ADDR                   0x36
/**<        Address of OFC1 Register          */
#define SMI130_OFC2_ADDR                       0x37
/**<        Address of OFC2 Register          */
#define SMI130_OFC3_ADDR                   0x38
/**<        Address of OFC3 Register          */
#define SMI130_OFC4_ADDR                   0x39
/**<        Address of OFC4 Register          */
#define SMI130_TRIM_GP0_ADDR               0x3A
/**<        Address of Trim GP0 Register              */
#define SMI130_TRIM_GP1_ADDR               0x3B
/**<        Address of Trim GP1 Register              */
#define SMI130_SELF_TEST_ADDR              0x3C
/**<        Address of BGW Selftest Register           */

/* Control Register */
#define SMI130_FIFO_CGF1_ADDR              0x3D
/**<        Address of FIFO CGF0 Register             */
#define SMI130_FIFO_CGF0_ADDR              0x3E


/* Data Register */
#define SMI130_FIFO_DATA_ADDR              0x3F


/* Rate X LSB Register */
#define SMI130_RATE_X_LSB_VALUEX__POS        0

/**< Last 8 bits of RateX LSB Registers */
#define SMI130_RATE_X_LSB_VALUEX__LEN        8
#define SMI130_RATE_X_LSB_VALUEX__MSK        0xFF
#define SMI130_RATE_X_LSB_VALUEX__REG        SMI130_RATE_X_LSB_ADDR

/* Rate Y LSB Register */
/**<  Last 8 bits of RateY LSB Registers */
#define SMI130_RATE_Y_LSB_VALUEY__POS        0
#define SMI130_RATE_Y_LSB_VALUEY__LEN        8
#define SMI130_RATE_Y_LSB_VALUEY__MSK        0xFF
#define SMI130_RATE_Y_LSB_VALUEY__REG        SMI130_RATE_Y_LSB_ADDR

/* Rate Z LSB Register */
/**< Last 8 bits of RateZ LSB Registers */
#define SMI130_RATE_Z_LSB_VALUEZ__POS        0
#define SMI130_RATE_Z_LSB_VALUEZ__LEN        8
#define SMI130_RATE_Z_LSB_VALUEZ__MSK        0xFF
#define SMI130_RATE_Z_LSB_VALUEZ__REG        SMI130_RATE_Z_LSB_ADDR

/* Interrupt status 0 Register */
   /**< 2th bit of Interrupt status 0 register */
#define SMI130_INT_STATUS0_ANY_INT__POS     2
#define SMI130_INT_STATUS0_ANY_INT__LEN     1
#define SMI130_INT_STATUS0_ANY_INT__MSK     0x04
#define SMI130_INT_STATUS0_ANY_INT__REG     SMI130_INT_STATUS0_ADDR

/**< 1st bit of Interrupt status 0 register */
#define SMI130_INT_STATUS0_HIGH_INT__POS    1
#define SMI130_INT_STATUS0_HIGH_INT__LEN    1
#define SMI130_INT_STATUS0_HIGH_INT__MSK    0x02
#define SMI130_INT_STATUS0_HIGH_INT__REG    SMI130_INT_STATUS0_ADDR

 /**< 1st and 2nd bit of Interrupt status 0 register */
#define SMI130_INT_STATUSZERO__POS    1
#define SMI130_INT_STATUSZERO__LEN    2
#define SMI130_INT_STATUSZERO__MSK    0x06
#define SMI130_INT_STATUSZERO__REG    SMI130_INT_STATUS0_ADDR

/* Interrupt status 1 Register */
/**< 7th bit of Interrupt status 1 register */
#define SMI130_INT_STATUS1_DATA_INT__POS           7
#define SMI130_INT_STATUS1_DATA_INT__LEN           1
#define SMI130_INT_STATUS1_DATA_INT__MSK           0x80
#define SMI130_INT_STATUS1_DATA_INT__REG           SMI130_INT_STATUS1_ADDR

 /**< 6th bit of Interrupt status 1 register */
#define SMI130_INT_STATUS1_AUTO_OFFSET_INT__POS    6
#define SMI130_INT_STATUS1_AUTO_OFFSET_INT__LEN    1
#define SMI130_INT_STATUS1_AUTO_OFFSET_INT__MSK    0x40
#define SMI130_INT_STATUS1_AUTO_OFFSET_INT__REG    SMI130_INT_STATUS1_ADDR

/**< 5th bit of Interrupt status 1 register */
#define SMI130_INT_STATUS1_FAST_OFFSET_INT__POS    5
#define SMI130_INT_STATUS1_FAST_OFFSET_INT__LEN    1
#define SMI130_INT_STATUS1_FAST_OFFSET_INT__MSK    0x20
#define SMI130_INT_STATUS1_FAST_OFFSET_INT__REG    SMI130_INT_STATUS1_ADDR

/**< 4th bit of Interrupt status 1 register */
#define SMI130_INT_STATUS1_FIFO_INT__POS           4
#define SMI130_INT_STATUS1_FIFO_INT__LEN           1
#define SMI130_INT_STATUS1_FIFO_INT__MSK           0x10
#define SMI130_INT_STATUS1_FIFO_INT__REG           SMI130_INT_STATUS1_ADDR

/**< MSB 4 bits of Interrupt status1 register */
#define SMI130_INT_STATUSONE__POS           4
#define SMI130_INT_STATUSONE__LEN           4
#define SMI130_INT_STATUSONE__MSK           0xF0
#define SMI130_INT_STATUSONE__REG           SMI130_INT_STATUS1_ADDR

/* Interrupt status 2 Register */
/**< 3th bit of Interrupt status 2 register */
#define SMI130_INT_STATUS2_ANY_SIGN_INT__POS     3
#define SMI130_INT_STATUS2_ANY_SIGN_INT__LEN     1
#define SMI130_INT_STATUS2_ANY_SIGN_INT__MSK     0x08
#define SMI130_INT_STATUS2_ANY_SIGN_INT__REG     SMI130_INT_STATUS2_ADDR

/**< 2th bit of Interrupt status 2 register */
#define SMI130_INT_STATUS2_ANY_FIRSTZ_INT__POS   2
#define SMI130_INT_STATUS2_ANY_FIRSTZ_INT__LEN   1
#define SMI130_INT_STATUS2_ANY_FIRSTZ_INT__MSK   0x04
#define SMI130_INT_STATUS2_ANY_FIRSTZ_INT__REG   SMI130_INT_STATUS2_ADDR

/**< 1st bit of Interrupt status 2 register */
#define SMI130_INT_STATUS2_ANY_FIRSTY_INT__POS   1
#define SMI130_INT_STATUS2_ANY_FIRSTY_INT__LEN   1
#define SMI130_INT_STATUS2_ANY_FIRSTY_INT__MSK   0x02
#define SMI130_INT_STATUS2_ANY_FIRSTY_INT__REG   SMI130_INT_STATUS2_ADDR

/**< 0th bit of Interrupt status 2 register */
#define SMI130_INT_STATUS2_ANY_FIRSTX_INT__POS   0
#define SMI130_INT_STATUS2_ANY_FIRSTX_INT__LEN   1
#define SMI130_INT_STATUS2_ANY_FIRSTX_INT__MSK   0x01
#define SMI130_INT_STATUS2_ANY_FIRSTX_INT__REG   SMI130_INT_STATUS2_ADDR

/**< 4 bits of Interrupt status 2 register */
#define SMI130_INT_STATUSTWO__POS   0
#define SMI130_INT_STATUSTWO__LEN   4
#define SMI130_INT_STATUSTWO__MSK   0x0F
#define SMI130_INT_STATUSTWO__REG   SMI130_INT_STATUS2_ADDR

/* Interrupt status 3 Register */
/**< 3th bit of Interrupt status 3 register */
#define SMI130_INT_STATUS3_HIGH_SIGN_INT__POS     3
#define SMI130_INT_STATUS3_HIGH_SIGN_INT__LEN     1
#define SMI130_INT_STATUS3_HIGH_SIGN_INT__MSK     0x08
#define SMI130_INT_STATUS3_HIGH_SIGN_INT__REG     SMI130_INT_STATUS3_ADDR

/**< 2th bit of Interrupt status 3 register */
#define SMI130_INT_STATUS3_HIGH_FIRSTZ_INT__POS   2
#define SMI130_INT_STATUS3_HIGH_FIRSTZ_INT__LEN   1
#define SMI130_INT_STATUS3_HIGH_FIRSTZ_INT__MSK   0x04
#define SMI130_INT_STATUS3_HIGH_FIRSTZ_INT__REG  SMI130_INT_STATUS3_ADDR

/**< 1st bit of Interrupt status 3 register */
#define SMI130_INT_STATUS3_HIGH_FIRSTY_INT__POS   1
#define SMI130_INT_STATUS3_HIGH_FIRSTY_INT__LEN   1
#define SMI130_INT_STATUS3_HIGH_FIRSTY_INT__MSK   0x02
#define SMI130_INT_STATUS3_HIGH_FIRSTY_INT__REG   SMI130_INT_STATUS3_ADDR

/**< 0th bit of Interrupt status 3 register */
#define SMI130_INT_STATUS3_HIGH_FIRSTX_INT__POS   0
#define SMI130_INT_STATUS3_HIGH_FIRSTX_INT__LEN   1
#define SMI130_INT_STATUS3_HIGH_FIRSTX_INT__MSK   0x01
#define SMI130_INT_STATUS3_HIGH_FIRSTX_INT__REG   SMI130_INT_STATUS3_ADDR

/**< LSB 4 bits of Interrupt status 3 register */
#define SMI130_INT_STATUSTHREE__POS   0
#define SMI130_INT_STATUSTHREE__LEN   4
#define SMI130_INT_STATUSTHREE__MSK   0x0F
#define SMI130_INT_STATUSTHREE__REG   SMI130_INT_STATUS3_ADDR

/* SMI130 FIFO Status Register */
/**< 7th bit of FIFO status Register */
#define SMI130_FIFO_STATUS_OVERRUN__POS         7
#define SMI130_FIFO_STATUS_OVERRUN__LEN         1
#define SMI130_FIFO_STATUS_OVERRUN__MSK         0x80
#define SMI130_FIFO_STATUS_OVERRUN__REG         SMI130_FIFO_STATUS_ADDR

/**< First 7 bits of FIFO status Register */
#define SMI130_FIFO_STATUS_FRAME_COUNTER__POS   0
#define SMI130_FIFO_STATUS_FRAME_COUNTER__LEN   7
#define SMI130_FIFO_STATUS_FRAME_COUNTER__MSK   0x7F
#define SMI130_FIFO_STATUS_FRAME_COUNTER__REG   SMI130_FIFO_STATUS_ADDR

/**< First 3 bits of range Registers */
#define SMI130_RANGE_ADDR_RANGE__POS           0
#define SMI130_RANGE_ADDR_RANGE__LEN           3
#define SMI130_RANGE_ADDR_RANGE__MSK           0x07
#define SMI130_RANGE_ADDR_RANGE__REG           SMI130_RANGE_ADDR

/**< Last bit of Bandwidth Registers */
#define SMI130_BW_ADDR_HIGH_RES__POS       7
#define SMI130_BW_ADDR_HIGH_RES__LEN       1
#define SMI130_BW_ADDR_HIGH_RES__MSK       0x80
#define SMI130_BW_ADDR_HIGH_RES__REG       SMI130_BW_ADDR

/**< First 3 bits of Bandwidth Registers */
#define SMI130_BW_ADDR__POS             0
#define SMI130_BW_ADDR__LEN             3
#define SMI130_BW_ADDR__MSK             0x07
#define SMI130_BW_ADDR__REG             SMI130_BW_ADDR

/**< 6th bit of Bandwidth Registers */
#define SMI130_BW_ADDR_IMG_STB__POS             6
#define SMI130_BW_ADDR_IMG_STB__LEN             1
#define SMI130_BW_ADDR_IMG_STB__MSK             0x40
#define SMI130_BW_ADDR_IMG_STB__REG             SMI130_BW_ADDR

/**< 7th bit of HBW Register */
#define SMI130_RATED_HBW_ADDR_DATA_HIGHBW__POS         7
#define SMI130_RATED_HBW_ADDR_DATA_HIGHBW__LEN         1
#define SMI130_RATED_HBW_ADDR_DATA_HIGHBW__MSK         0x80
#define SMI130_RATED_HBW_ADDR_DATA_HIGHBW__REG         SMI130_RATED_HBW_ADDR

/**< 6th bit of HBW Register */
#define SMI130_RATED_HBW_ADDR_SHADOW_DIS__POS          6
#define SMI130_RATED_HBW_ADDR_SHADOW_DIS__LEN          1
#define SMI130_RATED_HBW_ADDR_SHADOW_DIS__MSK          0x40
#define SMI130_RATED_HBW_ADDR_SHADOW_DIS__REG          SMI130_RATED_HBW_ADDR

/**< 7th bit of Interrupt Enable 0 Registers */
#define SMI130_INT_ENABLE0_DATAEN__POS               7
#define SMI130_INT_ENABLE0_DATAEN__LEN               1
#define SMI130_INT_ENABLE0_DATAEN__MSK               0x80
#define SMI130_INT_ENABLE0_DATAEN__REG               SMI130_INT_ENABLE0_ADDR

/**< 6th bit of Interrupt Enable 0 Registers */
#define SMI130_INT_ENABLE0_FIFOEN__POS               6
#define SMI130_INT_ENABLE0_FIFOEN__LEN               1
#define SMI130_INT_ENABLE0_FIFOEN__MSK               0x40
#define SMI130_INT_ENABLE0_FIFOEN__REG               SMI130_INT_ENABLE0_ADDR

/**< 1st bit of Interrupt Enable 1 Registers */
#define SMI130_INT_ENABLE1_IT1_OD__POS               1
#define SMI130_INT_ENABLE1_IT1_OD__LEN               1
#define SMI130_INT_ENABLE1_IT1_OD__MSK               0x02
#define SMI130_INT_ENABLE1_IT1_OD__REG               SMI130_INT_ENABLE1_ADDR

/**< 0th bit of Interrupt Enable 1 Registers */
#define SMI130_INT_ENABLE1_IT1_LVL__POS              0
#define SMI130_INT_ENABLE1_IT1_LVL__LEN              1
#define SMI130_INT_ENABLE1_IT1_LVL__MSK              0x01
#define SMI130_INT_ENABLE1_IT1_LVL__REG              SMI130_INT_ENABLE1_ADDR

/**< 3rd bit of Interrupt MAP 0 Registers */
#define SMI130_INT_MAP_0_INT1_HIGH__POS            3
#define SMI130_INT_MAP_0_INT1_HIGH__LEN            1
#define SMI130_INT_MAP_0_INT1_HIGH__MSK            0x08
#define SMI130_INT_MAP_0_INT1_HIGH__REG            SMI130_INT_MAP_0_ADDR

/**< 1st bit of Interrupt MAP 0 Registers */
#define SMI130_INT_MAP_0_INT1_ANY__POS             1
#define SMI130_INT_MAP_0_INT1_ANY__LEN             1
#define SMI130_INT_MAP_0_INT1_ANY__MSK             0x02
#define SMI130_INT_MAP_0_INT1_ANY__REG             SMI130_INT_MAP_0_ADDR

/**< 7th bit of MAP_1Registers */
#define SMI130_MAP_1_INT2_DATA__POS                  7
#define SMI130_MAP_1_INT2_DATA__LEN                  1
#define SMI130_MAP_1_INT2_DATA__MSK                  0x80
#define SMI130_MAP_1_INT2_DATA__REG                  SMI130_INT_MAP_1_ADDR

/**< 6th bit of MAP_1Registers */
#define SMI130_MAP_1_INT2_FAST_OFFSET__POS           6
#define SMI130_MAP_1_INT2_FAST_OFFSET__LEN           1
#define SMI130_MAP_1_INT2_FAST_OFFSET__MSK           0x40
#define SMI130_MAP_1_INT2_FAST_OFFSET__REG           SMI130_INT_MAP_1_ADDR

/**< 5th bit of MAP_1Registers */
#define SMI130_MAP_1_INT2_FIFO__POS                  5
#define SMI130_MAP_1_INT2_FIFO__LEN                  1
#define SMI130_MAP_1_INT2_FIFO__MSK                  0x20
#define SMI130_MAP_1_INT2_FIFO__REG                  SMI130_INT_MAP_1_ADDR

/**< 4th bit of MAP_1Registers */
#define SMI130_MAP_1_INT2_AUTO_OFFSET__POS           4
#define SMI130_MAP_1_INT2_AUTO_OFFSET__LEN           1
#define SMI130_MAP_1_INT2_AUTO_OFFSET__MSK           0x10
#define SMI130_MAP_1_INT2_AUTO_OFFSET__REG           SMI130_INT_MAP_1_ADDR

/**< 3rd bit of MAP_1Registers */
#define SMI130_MAP_1_INT1_AUTO_OFFSET__POS           3
#define SMI130_MAP_1_INT1_AUTO_OFFSET__LEN           1
#define SMI130_MAP_1_INT1_AUTO_OFFSET__MSK           0x08
#define SMI130_MAP_1_INT1_AUTO_OFFSET__REG           SMI130_INT_MAP_1_ADDR

/**< 2nd bit of MAP_1Registers */
#define SMI130_MAP_1_INT1_FIFO__POS                  2
#define SMI130_MAP_1_INT1_FIFO__LEN                  1
#define SMI130_MAP_1_INT1_FIFO__MSK                  0x04
#define SMI130_MAP_1_INT1_FIFO__REG                  SMI130_INT_MAP_1_ADDR

/**< 1st bit of MAP_1Registers */
#define SMI130_MAP_1_INT1_FAST_OFFSET__POS           1
#define SMI130_MAP_1_INT1_FAST_OFFSET__LEN           1
#define SMI130_MAP_1_INT1_FAST_OFFSET__MSK           0x02
#define SMI130_MAP_1_INT1_FAST_OFFSET__REG           SMI130_INT_MAP_1_ADDR

/**< 0th bit of MAP_1Registers */
#define SMI130_MAP_1_INT1_DATA__POS                  0
#define SMI130_MAP_1_INT1_DATA__LEN                  1
#define SMI130_MAP_1_INT1_DATA__MSK                  0x01
#define SMI130_MAP_1_INT1_DATA__REG                  SMI130_INT_MAP_1_ADDR

/**< 3rd bit of Interrupt Map 2 Registers */
#define SMI130_INT_MAP_2_INT2_HIGH__POS            3
#define SMI130_INT_MAP_2_INT2_HIGH__LEN            1
#define SMI130_INT_MAP_2_INT2_HIGH__MSK            0x08
#define SMI130_INT_MAP_2_INT2_HIGH__REG            SMI130_INT_MAP_2_ADDR

/**< 1st bit of Interrupt Map 2 Registers */
#define SMI130_INT_MAP_2_INT2_ANY__POS             1
#define SMI130_INT_MAP_2_INT2_ANY__LEN             1
#define SMI130_INT_MAP_2_INT2_ANY__MSK             0x02
#define SMI130_INT_MAP_2_INT2_ANY__REG             SMI130_INT_MAP_2_ADDR

/**< 5th bit of Interrupt 0 Registers */
#define SMI130_INT_0_ADDR_SLOW_OFFSET_UNFILT__POS          5
#define SMI130_INT_0_ADDR_SLOW_OFFSET_UNFILT__LEN          1
#define SMI130_INT_0_ADDR_SLOW_OFFSET_UNFILT__MSK          0x20
#define SMI130_INT_0_ADDR_SLOW_OFFSET_UNFILT__REG          SMI130_INT_0_ADDR

/**< 3rd bit of Interrupt 0 Registers */
#define SMI130_INT_0_ADDR_HIGH_UNFILT_DATA__POS            3
#define SMI130_INT_0_ADDR_HIGH_UNFILT_DATA__LEN            1
#define SMI130_INT_0_ADDR_HIGH_UNFILT_DATA__MSK            0x08
#define SMI130_INT_0_ADDR_HIGH_UNFILT_DATA__REG            SMI130_INT_0_ADDR

/**< 1st bit of Interrupt 0 Registers */
#define SMI130_INT_0_ADDR_ANY_UNFILT_DATA__POS             1
#define SMI130_INT_0_ADDR_ANY_UNFILT_DATA__LEN             1
#define SMI130_INT_0_ADDR_ANY_UNFILT_DATA__MSK             0x02
#define SMI130_INT_0_ADDR_ANY_UNFILT_DATA__REG             SMI130_INT_0_ADDR

/**< 7th bit of INT_1  Registers */
#define SMI130_INT_1_ADDR_FAST_OFFSET_UNFILT__POS            7
#define SMI130_INT_1_ADDR_FAST_OFFSET_UNFILT__LEN            1
#define SMI130_INT_1_ADDR_FAST_OFFSET_UNFILT__MSK            0x80
#define SMI130_INT_1_ADDR_FAST_OFFSET_UNFILT__REG            SMI130_INT_1_ADDR

/**< First 7 bits of INT_1  Registers */
#define SMI130_INT_1_ADDR_ANY_TH__POS                       0
#define SMI130_INT_1_ADDR_ANY_TH__LEN                       7
#define SMI130_INT_1_ADDR_ANY_TH__MSK                       0x7F
#define SMI130_INT_1_ADDR_ANY_TH__REG                       SMI130_INT_1_ADDR

/**< Last 2 bits of INT 2Registers */
#define SMI130_INT_2_ADDR_AWAKE_DUR__POS          6
#define SMI130_INT_2_ADDR_AWAKE_DUR__LEN          2
#define SMI130_INT_2_ADDR_AWAKE_DUR__MSK          0xC0
#define SMI130_INT_2_ADDR_AWAKE_DUR__REG          SMI130_INT_2_ADDR

/**< 4th & 5th bit of INT 2Registers */
#define SMI130_INT_2_ADDR_ANY_DURSAMPLE__POS      4
#define SMI130_INT_2_ADDR_ANY_DURSAMPLE__LEN      2
#define SMI130_INT_2_ADDR_ANY_DURSAMPLE__MSK      0x30
#define SMI130_INT_2_ADDR_ANY_DURSAMPLE__REG      SMI130_INT_2_ADDR

/**< 2nd bit of INT 2Registers */
#define SMI130_INT_2_ADDR_ANY_EN_Z__POS           2
#define SMI130_INT_2_ADDR_ANY_EN_Z__LEN           1
#define SMI130_INT_2_ADDR_ANY_EN_Z__MSK           0x04
#define SMI130_INT_2_ADDR_ANY_EN_Z__REG           SMI130_INT_2_ADDR

/**< 1st bit of INT 2Registers */
#define SMI130_INT_2_ADDR_ANY_EN_Y__POS           1
#define SMI130_INT_2_ADDR_ANY_EN_Y__LEN           1
#define SMI130_INT_2_ADDR_ANY_EN_Y__MSK           0x02
#define SMI130_INT_2_ADDR_ANY_EN_Y__REG           SMI130_INT_2_ADDR

/**< 0th bit of INT 2Registers */
#define SMI130_INT_2_ADDR_ANY_EN_X__POS           0
#define SMI130_INT_2_ADDR_ANY_EN_X__LEN           1
#define SMI130_INT_2_ADDR_ANY_EN_X__MSK           0x01
#define SMI130_INT_2_ADDR_ANY_EN_X__REG           SMI130_INT_2_ADDR

/**< Last bit of INT 4 Registers */
#define SMI130_INT_4_FIFO_WM_EN__POS           7
#define SMI130_INT_4_FIFO_WM_EN__LEN           1
#define SMI130_INT_4_FIFO_WM_EN__MSK           0x80
#define SMI130_INT_4_FIFO_WM_EN__REG           SMI130_INT_4_ADDR

/**< Last bit of Reset Latch Registers */
#define SMI130_RST_LATCH_ADDR_RESET_INT__POS           7
#define SMI130_RST_LATCH_ADDR_RESET_INT__LEN           1
#define SMI130_RST_LATCH_ADDR_RESET_INT__MSK           0x80
#define SMI130_RST_LATCH_ADDR_RESET_INT__REG           SMI130_RST_LATCH_ADDR

/**< 6th bit of Reset Latch Registers */
#define SMI130_RST_LATCH_ADDR_OFFSET_RESET__POS        6
#define SMI130_RST_LATCH_ADDR_OFFSET_RESET__LEN        1
#define SMI130_RST_LATCH_ADDR_OFFSET_RESET__MSK        0x40
#define SMI130_RST_LATCH_ADDR_OFFSET_RESET__REG        SMI130_RST_LATCH_ADDR

/**< 4th bit of Reset Latch Registers */
#define SMI130_RST_LATCH_ADDR_LATCH_STATUS__POS        4
#define SMI130_RST_LATCH_ADDR_LATCH_STATUS__LEN        1
#define SMI130_RST_LATCH_ADDR_LATCH_STATUS__MSK        0x10
#define SMI130_RST_LATCH_ADDR_LATCH_STATUS__REG        SMI130_RST_LATCH_ADDR

/**< First 4 bits of Reset Latch Registers */
#define SMI130_RST_LATCH_ADDR_LATCH_INT__POS           0
#define SMI130_RST_LATCH_ADDR_LATCH_INT__LEN           4
#define SMI130_RST_LATCH_ADDR_LATCH_INT__MSK           0x0F
#define SMI130_RST_LATCH_ADDR_LATCH_INT__REG           SMI130_RST_LATCH_ADDR

/**< Last 2 bits of HIGH_TH_X Registers */
#define SMI130_HIGH_HY_X__POS        6
#define SMI130_HIGH_HY_X__LEN        2
#define SMI130_HIGH_HY_X__MSK        0xC0
#define SMI130_HIGH_HY_X__REG        SMI130_HIGH_TH_X_ADDR

/**< 5 bits of HIGH_TH_X Registers */
#define SMI130_HIGH_TH_X__POS        1
#define SMI130_HIGH_TH_X__LEN        5
#define SMI130_HIGH_TH_X__MSK        0x3E
#define SMI130_HIGH_TH_X__REG        SMI130_HIGH_TH_X_ADDR

/**< 0th bit of HIGH_TH_X Registers */
#define SMI130_HIGH_EN_X__POS        0
#define SMI130_HIGH_EN_X__LEN        1
#define SMI130_HIGH_EN_X__MSK        0x01
#define SMI130_HIGH_EN_X__REG        SMI130_HIGH_TH_X_ADDR

/**< Last 2 bits of HIGH_TH_Y Registers */
#define SMI130_HIGH_HY_Y__POS        6
#define SMI130_HIGH_HY_Y__LEN        2
#define SMI130_HIGH_HY_Y__MSK        0xC0
#define SMI130_HIGH_HY_Y__REG        SMI130_HIGH_TH_Y_ADDR

/**< 5 bits of HIGH_TH_Y Registers */
#define SMI130_HIGH_TH_Y__POS        1
#define SMI130_HIGH_TH_Y__LEN        5
#define SMI130_HIGH_TH_Y__MSK        0x3E
#define SMI130_HIGH_TH_Y__REG        SMI130_HIGH_TH_Y_ADDR

/**< 0th bit of HIGH_TH_Y Registers */
#define SMI130_HIGH_EN_Y__POS        0
#define SMI130_HIGH_EN_Y__LEN        1
#define SMI130_HIGH_EN_Y__MSK        0x01
#define SMI130_HIGH_EN_Y__REG        SMI130_HIGH_TH_Y_ADDR

/**< Last 2 bits of HIGH_TH_Z Registers */
#define SMI130_HIGH_HY_Z__POS        6
#define SMI130_HIGH_HY_Z__LEN        2
#define SMI130_HIGH_HY_Z__MSK        0xC0
#define SMI130_HIGH_HY_Z__REG        SMI130_HIGH_TH_Z_ADDR

/**< 5 bits of HIGH_TH_Z Registers */
#define SMI130_HIGH_TH_Z__POS        1
#define SMI130_HIGH_TH_Z__LEN        5
#define SMI130_HIGH_TH_Z__MSK        0x3E
#define SMI130_HIGH_TH_Z__REG        SMI130_HIGH_TH_Z_ADDR

/**< 0th bit of HIGH_TH_Z Registers */
#define SMI130_HIGH_EN_Z__POS        0
#define SMI130_HIGH_EN_Z__LEN        1
#define SMI130_HIGH_EN_Z__MSK        0x01
#define SMI130_HIGH_EN_Z__REG        SMI130_HIGH_TH_Z_ADDR

/**< Last 3 bits of INT OFF0 Registers */
#define SMI130_SLOW_OFFSET_TH__POS          6
#define SMI130_SLOW_OFFSET_TH__LEN          2
#define SMI130_SLOW_OFFSET_TH__MSK          0xC0
#define SMI130_SLOW_OFFSET_TH__REG          SMI130_SOC_ADDR

/**< 2  bits of INT OFF0 Registers */
#define SMI130_SLOW_OFFSET_DUR__POS         3
#define SMI130_SLOW_OFFSET_DUR__LEN         3
#define SMI130_SLOW_OFFSET_DUR__MSK         0x38
#define SMI130_SLOW_OFFSET_DUR__REG         SMI130_SOC_ADDR

/**< 2nd bit of INT OFF0 Registers */
#define SMI130_SLOW_OFFSET_EN_Z__POS        2
#define SMI130_SLOW_OFFSET_EN_Z__LEN        1
#define SMI130_SLOW_OFFSET_EN_Z__MSK        0x04
#define SMI130_SLOW_OFFSET_EN_Z__REG        SMI130_SOC_ADDR

/**< 1st bit of INT OFF0 Registers */
#define SMI130_SLOW_OFFSET_EN_Y__POS        1
#define SMI130_SLOW_OFFSET_EN_Y__LEN        1
#define SMI130_SLOW_OFFSET_EN_Y__MSK        0x02
#define SMI130_SLOW_OFFSET_EN_Y__REG        SMI130_SOC_ADDR

/**< 0th bit of INT OFF0 Registers */
#define SMI130_SLOW_OFFSET_EN_X__POS        0
#define SMI130_SLOW_OFFSET_EN_X__LEN        1
#define SMI130_SLOW_OFFSET_EN_X__MSK        0x01
#define SMI130_SLOW_OFFSET_EN_X__REG        SMI130_SOC_ADDR

/**< Last 2 bits of INT OFF1 Registers */
#define SMI130_AUTO_OFFSET_WL__POS        6
#define SMI130_AUTO_OFFSET_WL__LEN        2
#define SMI130_AUTO_OFFSET_WL__MSK        0xC0
#define SMI130_AUTO_OFFSET_WL__REG        SMI130_A_FOC_ADDR

/**< 2  bits of INT OFF1 Registers */
#define SMI130_FAST_OFFSET_WL__POS        4
#define SMI130_FAST_OFFSET_WL__LEN        2
#define SMI130_FAST_OFFSET_WL__MSK        0x30
#define SMI130_FAST_OFFSET_WL__REG        SMI130_A_FOC_ADDR

/**< 3nd bit of INT OFF1 Registers */
#define SMI130_FAST_OFFSET_EN__POS        3
#define SMI130_FAST_OFFSET_EN__LEN        1
#define SMI130_FAST_OFFSET_EN__MSK        0x08
#define SMI130_FAST_OFFSET_EN__REG        SMI130_A_FOC_ADDR

/**< 2nd bit of INT OFF1 Registers */
#define SMI130_FAST_OFFSET_EN_Z__POS      2
#define SMI130_FAST_OFFSET_EN_Z__LEN      1
#define SMI130_FAST_OFFSET_EN_Z__MSK      0x04
#define SMI130_FAST_OFFSET_EN_Z__REG      SMI130_A_FOC_ADDR

/**< 1st bit of INT OFF1 Registers */
#define SMI130_FAST_OFFSET_EN_Y__POS      1
#define SMI130_FAST_OFFSET_EN_Y__LEN      1
#define SMI130_FAST_OFFSET_EN_Y__MSK      0x02
#define SMI130_FAST_OFFSET_EN_Y__REG      SMI130_A_FOC_ADDR

/**< 0th bit of INT OFF1 Registers */
#define SMI130_FAST_OFFSET_EN_X__POS      0
#define SMI130_FAST_OFFSET_EN_X__LEN      1
#define SMI130_FAST_OFFSET_EN_X__MSK      0x01
#define SMI130_FAST_OFFSET_EN_X__REG      SMI130_A_FOC_ADDR

/**< 0 to 2 bits of INT OFF1 Registers */
#define SMI130_FAST_OFFSET_EN_XYZ__POS      0
#define SMI130_FAST_OFFSET_EN_XYZ__LEN      3
#define SMI130_FAST_OFFSET_EN_XYZ__MSK      0x07
#define SMI130_FAST_OFFSET_EN_XYZ__REG      SMI130_A_FOC_ADDR

/**< Last 4 bits of Trim NVM control Registers */
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_REMAIN__POS        4
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_REMAIN__LEN        4
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_REMAIN__MSK        0xF0
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_REMAIN__REG        \
SMI130_TRIM_NVM_CTRL_ADDR

/**< 3rd bit of Trim NVM control Registers */
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_LOAD__POS          3
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_LOAD__LEN          1
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_LOAD__MSK          0x08
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_LOAD__REG          \
SMI130_TRIM_NVM_CTRL_ADDR

/**< 2nd bit of Trim NVM control Registers */
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_RDY__POS           2
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_RDY__LEN           1
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_RDY__MSK           0x04
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_RDY__REG           \
SMI130_TRIM_NVM_CTRL_ADDR

 /**< 1st bit of Trim NVM control Registers */
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_PROG_TRIG__POS     1
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_PROG_TRIG__LEN     1
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_PROG_TRIG__MSK     0x02
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_PROG_TRIG__REG     \
SMI130_TRIM_NVM_CTRL_ADDR

/**< 0th bit of Trim NVM control Registers */
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_PROG_MODE__POS     0
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_PROG_MODE__LEN     1
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_PROG_MODE__MSK     0x01
#define SMI130_TRIM_NVM_CTRL_ADDR_NVM_PROG_MODE__REG     \
SMI130_TRIM_NVM_CTRL_ADDR

 /**< 2nd bit of SPI3 WDT Registers */
#define SMI130_BGW_SPI3_WDT_ADDR_I2C_WDT_EN__POS      2
#define SMI130_BGW_SPI3_WDT_ADDR_I2C_WDT_EN__LEN      1
#define SMI130_BGW_SPI3_WDT_ADDR_I2C_WDT_EN__MSK      0x04
#define SMI130_BGW_SPI3_WDT_ADDR_I2C_WDT_EN__REG      \
SMI130_BGW_SPI3_WDT_ADDR

 /**< 1st bit of SPI3 WDT Registers */
#define SMI130_BGW_SPI3_WDT_ADDR_I2C_WDT_SEL__POS     1
#define SMI130_BGW_SPI3_WDT_ADDR_I2C_WDT_SEL__LEN     1
#define SMI130_BGW_SPI3_WDT_ADDR_I2C_WDT_SEL__MSK     0x02
#define SMI130_BGW_SPI3_WDT_ADDR_I2C_WDT_SEL__REG     \
SMI130_BGW_SPI3_WDT_ADDR

/**< 0th bit of SPI3 WDT Registers */
#define SMI130_BGW_SPI3_WDT_ADDR_SPI3__POS            0
#define SMI130_BGW_SPI3_WDT_ADDR_SPI3__LEN            1
#define SMI130_BGW_SPI3_WDT_ADDR_SPI3__MSK            0x01
#define SMI130_BGW_SPI3_WDT_ADDR_SPI3__REG            \
SMI130_BGW_SPI3_WDT_ADDR

/**< 4th bit of Self test Registers */
#define SMI130_SELF_TEST_ADDR_RATEOK__POS            4
#define SMI130_SELF_TEST_ADDR_RATEOK__LEN            1
#define SMI130_SELF_TEST_ADDR_RATEOK__MSK            0x10
#define SMI130_SELF_TEST_ADDR_RATEOK__REG            \
SMI130_SELF_TEST_ADDR

/**< 2nd bit of Self test Registers */
#define SMI130_SELF_TEST_ADDR_BISTFAIL__POS          2
#define SMI130_SELF_TEST_ADDR_BISTFAIL__LEN          1
#define SMI130_SELF_TEST_ADDR_BISTFAIL__MSK          0x04
#define SMI130_SELF_TEST_ADDR_BISTFAIL__REG          \
SMI130_SELF_TEST_ADDR

/**< 1st bit of Self test Registers */
#define SMI130_SELF_TEST_ADDR_BISTRDY__POS           1
#define SMI130_SELF_TEST_ADDR_BISTRDY__LEN           1
#define SMI130_SELF_TEST_ADDR_BISTRDY__MSK           0x02
#define SMI130_SELF_TEST_ADDR_BISTRDY__REG           \
SMI130_SELF_TEST_ADDR

/**< 0th bit of Self test Registers */
#define SMI130_SELF_TEST_ADDR_TRIGBIST__POS          0
#define SMI130_SELF_TEST_ADDR_TRIGBIST__LEN          1
#define SMI130_SELF_TEST_ADDR_TRIGBIST__MSK          0x01
#define SMI130_SELF_TEST_ADDR_TRIGBIST__REG          \
SMI130_SELF_TEST_ADDR

/**< 7th bit of FIFO CGF1 Registers */
#define SMI130_FIFO_CGF1_ADDR_TAG__POS     7
#define SMI130_FIFO_CGF1_ADDR_TAG__LEN     1
#define SMI130_FIFO_CGF1_ADDR_TAG__MSK     0x80
#define SMI130_FIFO_CGF1_ADDR_TAG__REG     SMI130_FIFO_CGF1_ADDR

/**< First 7 bits of FIFO CGF1 Registers */
#define SMI130_FIFO_CGF1_ADDR_WML__POS     0
#define SMI130_FIFO_CGF1_ADDR_WML__LEN     7
#define SMI130_FIFO_CGF1_ADDR_WML__MSK     0x7F
#define SMI130_FIFO_CGF1_ADDR_WML__REG     SMI130_FIFO_CGF1_ADDR

/**< Last 2 bits of FIFO CGF0 Addr Registers */
#define SMI130_FIFO_CGF0_ADDR_MODE__POS         6
#define SMI130_FIFO_CGF0_ADDR_MODE__LEN         2
#define SMI130_FIFO_CGF0_ADDR_MODE__MSK         0xC0
#define SMI130_FIFO_CGF0_ADDR_MODE__REG         SMI130_FIFO_CGF0_ADDR

/**< First 2 bits of FIFO CGF0 Addr Registers */
#define SMI130_FIFO_CGF0_ADDR_DATA_SEL__POS     0
#define SMI130_FIFO_CGF0_ADDR_DATA_SEL__LEN     2
#define SMI130_FIFO_CGF0_ADDR_DATA_SEL__MSK     0x03
#define SMI130_FIFO_CGF0_ADDR_DATA_SEL__REG     SMI130_FIFO_CGF0_ADDR

 /**< Last 2 bits of INL Offset MSB Registers */
#define SMI130_OFC1_ADDR_OFFSET_X__POS       6
#define SMI130_OFC1_ADDR_OFFSET_X__LEN       2
#define SMI130_OFC1_ADDR_OFFSET_X__MSK       0xC0
#define SMI130_OFC1_ADDR_OFFSET_X__REG       SMI130_OFC1_ADDR

/**< 3 bits of INL Offset MSB Registers */
#define SMI130_OFC1_ADDR_OFFSET_Y__POS       3
#define SMI130_OFC1_ADDR_OFFSET_Y__LEN       3
#define SMI130_OFC1_ADDR_OFFSET_Y__MSK       0x38
#define SMI130_OFC1_ADDR_OFFSET_Y__REG       SMI130_OFC1_ADDR

/**< First 3 bits of INL Offset MSB Registers */
#define SMI130_OFC1_ADDR_OFFSET_Z__POS       0
#define SMI130_OFC1_ADDR_OFFSET_Z__LEN       3
#define SMI130_OFC1_ADDR_OFFSET_Z__MSK       0x07
#define SMI130_OFC1_ADDR_OFFSET_Z__REG       SMI130_OFC1_ADDR

/**< 4 bits of Trim GP0 Registers */
#define SMI130_TRIM_GP0_ADDR_GP0__POS            4
#define SMI130_TRIM_GP0_ADDR_GP0__LEN            4
#define SMI130_TRIM_GP0_ADDR_GP0__MSK            0xF0
#define SMI130_TRIM_GP0_ADDR_GP0__REG            SMI130_TRIM_GP0_ADDR

/**< 2 bits of Trim GP0 Registers */
#define SMI130_TRIM_GP0_ADDR_OFFSET_X__POS       2
#define SMI130_TRIM_GP0_ADDR_OFFSET_X__LEN       2
#define SMI130_TRIM_GP0_ADDR_OFFSET_X__MSK       0x0C
#define SMI130_TRIM_GP0_ADDR_OFFSET_X__REG       SMI130_TRIM_GP0_ADDR

/**< 1st bit of Trim GP0 Registers */
#define SMI130_TRIM_GP0_ADDR_OFFSET_Y__POS       1
#define SMI130_TRIM_GP0_ADDR_OFFSET_Y__LEN       1
#define SMI130_TRIM_GP0_ADDR_OFFSET_Y__MSK       0x02
#define SMI130_TRIM_GP0_ADDR_OFFSET_Y__REG       SMI130_TRIM_GP0_ADDR

/**< First bit of Trim GP0 Registers */
#define SMI130_TRIM_GP0_ADDR_OFFSET_Z__POS       0
#define SMI130_TRIM_GP0_ADDR_OFFSET_Z__LEN       1
#define SMI130_TRIM_GP0_ADDR_OFFSET_Z__MSK       0x01
#define SMI130_TRIM_GP0_ADDR_OFFSET_Z__REG       SMI130_TRIM_GP0_ADDR


/* For Axis Selection   */
/**< It refers SMI130 X-axis */
#define SMI130_X_AXIS           0
/**< It refers SMI130 Y-axis */
#define SMI130_Y_AXIS           1
/**< It refers SMI130 Z-axis */
#define SMI130_Z_AXIS           2

/* For Mode Settings    */
#define SMI130_MODE_NORMAL              0
#define SMI130_MODE_DEEPSUSPEND         1
#define SMI130_MODE_SUSPEND             2
#define SMI130_MODE_FASTPOWERUP			3
#define SMI130_MODE_ADVANCEDPOWERSAVING 4

/* Constants */

#define SMI130_NULL                             0
/**< constant declaration of NULL */
#define SMI130_DISABLE                          0
/**< It refers SMI130 disable */
#define SMI130_ENABLE                           1
/**< It refers SMI130 enable */
#define SMI130_OFF                              0
/**< It refers SMI130 OFF state */
#define SMI130_ON                               1
/**< It refers SMI130 ON state  */


#define SMI130_TURN1                            0
/**< It refers SMI130 TURN1 */
#define SMI130_TURN2                            1
/**< It refers SMI130 TURN2 */

#define SMI130_INT1                             0
/**< It refers SMI130 INT1 */
#define SMI130_INT2                             1
/**< It refers SMI130 INT2 */

#define SMI130_SLOW_OFFSET                      0
/**< It refers SMI130 Slow Offset */
#define SMI130_AUTO_OFFSET                      1
/**< It refers SMI130 Auto Offset */
#define SMI130_FAST_OFFSET                      2
/**< It refers SMI130 Fast Offset */
#define SMI130_S_TAP                            0
/**< It refers SMI130 Single Tap */
#define SMI130_D_TAP                            1
/**< It refers SMI130 Double Tap */
#define SMI130_INT1_DATA                        0
/**< It refers SMI130 Int1 Data */
#define SMI130_INT2_DATA                        1
/**< It refers SMI130 Int2 Data */
#define SMI130_TAP_UNFILT_DATA                   0
/**< It refers SMI130 Tap unfilt data */
#define SMI130_HIGH_UNFILT_DATA                  1
/**< It refers SMI130 High unfilt data */
#define SMI130_CONST_UNFILT_DATA                 2
/**< It refers SMI130 Const unfilt data */
#define SMI130_ANY_UNFILT_DATA                   3
/**< It refers SMI130 Any unfilt data */
#define SMI130_SHAKE_UNFILT_DATA                 4
/**< It refers SMI130 Shake unfilt data */
#define SMI130_SHAKE_TH                         0
/**< It refers SMI130 Shake Threshold */
#define SMI130_SHAKE_TH2                        1
/**< It refers SMI130 Shake Threshold2 */
#define SMI130_AUTO_OFFSET_WL                   0
/**< It refers SMI130 Auto Offset word length */
#define SMI130_FAST_OFFSET_WL                   1
/**< It refers SMI130 Fast Offset word length */
#define SMI130_I2C_WDT_EN                       0
/**< It refers SMI130 I2C WDT En */
#define SMI130_I2C_WDT_SEL                      1
/**< It refers SMI130 I2C WDT Sel */
#define SMI130_EXT_MODE                         0
/**< It refers SMI130 Ext Mode */
#define SMI130_EXT_PAGE                         1
/**< It refers SMI130 Ext page */
#define SMI130_START_ADDR                       0
/**< It refers SMI130 Start Address */
#define SMI130_STOP_ADDR                        1
/**< It refers SMI130 Stop Address */
#define SMI130_SLOW_CMD                         0
/**< It refers SMI130 Slow Command */
#define SMI130_FAST_CMD                         1
/**< It refers SMI130 Fast Command */
#define SMI130_TRIM_VRA                         0
/**< It refers SMI130 Trim VRA */
#define SMI130_TRIM_VRD                         1
/**< It refers SMI130 Trim VRD */
#define SMI130_LOGBIT_EM                        0
/**< It refers SMI130 LogBit Em */
#define SMI130_LOGBIT_VM                        1
/**< It refers SMI130 LogBit VM */
#define SMI130_GP0                              0
/**< It refers SMI130 GP0 */
#define SMI130_GP1                              1
/**< It refers SMI130 GP1*/
#define SMI130_LOW_SPEED                        0
/**< It refers SMI130 Low Speed Oscilator */
#define SMI130_HIGH_SPEED                       1
/**< It refers SMI130 High Speed Oscilator */
#define SMI130_DRIVE_OFFSET_P                   0
/**< It refers SMI130 Drive Offset P */
#define SMI130_DRIVE_OFFSET_N                   1
/**< It refers SMI130 Drive Offset N */
#define SMI130_TEST_MODE_EN                     0
/**< It refers SMI130 Test Mode Enable */
#define SMI130_TEST_MODE_REG                    1
/**< It refers SMI130 Test Mode reg */
#define SMI130_IBIAS_DRIVE_TRIM                 0
/**< It refers SMI130 IBIAS Drive Trim */
#define SMI130_IBIAS_RATE_TRIM                  1
/**< It refers SMI130 IBIAS Rate Trim */
#define SMI130_BAA_MODE                         0
/**< It refers SMI130 BAA Mode Trim */
#define SMI130_BMA_MODE                         1
/**< It refers SMI130 BMA Mode Trim */
#define SMI130_PI_KP                            0
/**< It refers SMI130 PI KP */
#define SMI130_PI_KI                            1
/**< It refers SMI130 PI KI */


#define C_SMI130_SUCCESS						0
/**< It refers SMI130 operation is success */
#define C_SMI130_FAILURE						1
/**< It refers SMI130 operation is Failure */

#define SMI130_SPI_RD_MASK                      0x80
/**< Read mask **/
#define SMI130_READ_SET                         0x01
/**< Setting for rading data **/

#define SMI130_SHIFT_1_POSITION                 1
/**< Shift bit by 1 Position **/
#define SMI130_SHIFT_2_POSITION                 2
/**< Shift bit by 2 Position **/
#define SMI130_SHIFT_3_POSITION                 3
/**< Shift bit by 3 Position **/
#define SMI130_SHIFT_4_POSITION                 4
/**< Shift bit by 4 Position **/
#define SMI130_SHIFT_5_POSITION                 5
/**< Shift bit by 5 Position **/
#define SMI130_SHIFT_6_POSITION                 6
/**< Shift bit by 6 Position **/
#define SMI130_SHIFT_7_POSITION                 7
/**< Shift bit by 7 Position **/
#define SMI130_SHIFT_8_POSITION                 8
/**< Shift bit by 8 Position **/
#define SMI130_SHIFT_12_POSITION                12
/**< Shift bit by 12 Position **/

#define         C_SMI130_Null_U8X                              0
#define         C_SMI130_Zero_U8X                              0
#define         C_SMI130_One_U8X                               1
#define         C_SMI130_Two_U8X                               2
#define         C_SMI130_Three_U8X                             3
#define         C_SMI130_Four_U8X                              4
#define         C_SMI130_Five_U8X                              5
#define         C_SMI130_Six_U8X                               6
#define         C_SMI130_Seven_U8X                             7
#define         C_SMI130_Eight_U8X                             8
#define         C_SMI130_Nine_U8X                              9
#define         C_SMI130_Ten_U8X                               10
#define         C_SMI130_Eleven_U8X                            11
#define         C_SMI130_Twelve_U8X                            12
#define         C_SMI130_Thirteen_U8X                          13
#define         C_SMI130_Fifteen_U8X                           15
#define         C_SMI130_Sixteen_U8X                           16
#define         C_SMI130_TwentyTwo_U8X                         22
#define         C_SMI130_TwentyThree_U8X                       23
#define         C_SMI130_TwentyFour_U8X                        24
#define         C_SMI130_TwentyFive_U8X                        25
#define         C_SMI130_ThirtyTwo_U8X                         32
#define         C_SMI130_Hundred_U8X                           100
#define         C_SMI130_OneTwentySeven_U8X                    127
#define         C_SMI130_OneTwentyEight_U8X                    128
#define         C_SMI130_TwoFiftyFive_U8X                      255
#define         C_SMI130_TwoFiftySix_U16X                      256

#define E_SMI130_NULL_PTR               (char)(-127)
#define E_SMI130_COMM_RES               (char)(-1)
#define E_SMI130_OUT_OF_RANGE           (signed char)(-2)

#define C_SMI130_No_Filter_U8X			0
#define	C_SMI130_BW_230Hz_U8X			1
#define	C_SMI130_BW_116Hz_U8X			2
#define	C_SMI130_BW_47Hz_U8X			3
#define	C_SMI130_BW_23Hz_U8X			4
#define	C_SMI130_BW_12Hz_U8X			5
#define	C_SMI130_BW_64Hz_U8X			6
#define	C_SMI130_BW_32Hz_U8X			7

#define C_SMI130_No_AutoSleepDur_U8X	0
#define	C_SMI130_4ms_AutoSleepDur_U8X	1
#define	C_SMI130_5ms_AutoSleepDur_U8X	2
#define	C_SMI130_8ms_AutoSleepDur_U8X	3
#define	C_SMI130_10ms_AutoSleepDur_U8X	4
#define	C_SMI130_15ms_AutoSleepDur_U8X	5
#define	C_SMI130_20ms_AutoSleepDur_U8X	6
#define	C_SMI130_40ms_AutoSleepDur_U8X	7



#define SMI130_WR_FUNC_PTR char (*bus_write)\
(unsigned char, unsigned char, unsigned char *, unsigned char)
#define SMI130_RD_FUNC_PTR char (*bus_read)\
(unsigned char, unsigned char, unsigned char *, unsigned char)
#define SMI130_BRD_FUNC_PTR char (*burst_read)\
(unsigned char, unsigned char, unsigned char *, SMI130_S32)
#define SMI130_MDELAY_DATA_TYPE SMI130_S32



/*user defined Structures*/
struct smi130_data_t {
		SMI130_S16 datax;
		SMI130_S16 datay;
		SMI130_S16 dataz;
		char intstatus[5];
};


struct smi130_offset_t {
		SMI130_U16 datax;
		SMI130_U16 datay;
		SMI130_U16 dataz;
};


struct smi130_t {
		unsigned char chip_id;
		unsigned char dev_addr;
		SMI130_BRD_FUNC_PTR;
		SMI130_WR_FUNC_PTR;
		SMI130_RD_FUNC_PTR;
		void(*delay_msec)(SMI130_MDELAY_DATA_TYPE);
};

int smi130_init(struct smi130_t *p_smi130);
int smi130_get_dataxyz(struct smi130_data_t *data);
int smi130_get_bw(unsigned char *bandwidth);
int smi130_set_bw(unsigned char bandwidth);
int smi130_get_wdt(unsigned char *wdt);
int smi130_set_wdt(unsigned char wdt);
int smi130_set_data_enable(unsigned char data_en);
int smi130_get_int_od(unsigned char param,
	unsigned char *int_od);
int smi130_set_int_od(unsigned char param,
	unsigned char int_od);
int smi130_get_int_data(unsigned char axis,
	unsigned char *int_data);
int smi130_set_int_data(unsigned char axis,
	unsigned char int_data);
int smi130_set_reset_int
(unsigned char reset_int);
int smi130_get_offset
(unsigned char axis, SMI130_S16 *offset);
int smi130_set_offset
(unsigned char axis, SMI130_S16 offset);
int smi130_get_range_reg
(unsigned char *range);
int smi130_set_range_reg
(unsigned char range);

int smi130_selftest(unsigned char *result);
int smi130_get_autosleepdur(unsigned char *duration);
int smi130_set_autosleepdur(unsigned char duration,
unsigned char bandwith);


#define SMI_SENSOR_UP_TIME           15
#define SMI_TIME_STAMP_TOR                        5
#define SMI130_BYTES_PER_3AXIS_SENSOR 6
#define SMI130_OUTPUT_DATA_SIZE         16
#define INT_PIN_PUSH_PULL     0
#define INT_PIN_OPEN_DRAIN    1
#define INT_DISABLE           0
#define INT_ENABLE            1

/*Related with output data type*/
enum SMI_SCAN_INDEX {
		SMI_SCAN_GYRO_X,
		SMI_SCAN_GYRO_Y,
		SMI_SCAN_GYRO_Z,
		SMI_SCAN_TIMESTAMP,
};

/*Related with output data rate*/
enum SMI_FILTER_BW {
		SMI_523HZ_Unfilter = 0,
		SMI_FILTER_230HZ,
		SMI_FILTER_116HZ,
		SMI_FILTER_47HZ,
		SMI_FILTER_23HZ,
		SMI_FILTER_12HZ,
		SMI_FILTER_64HZ,
		SMI_FILTER_32HZ,
		NUM_SMI_FILTER
};

/*full scale releated definition*/
enum SMI_FSR_VAL {
		SMI_FSR_2000DPS_VAL = 0,
		SMI_FSR_1000DPS_VAL,
		SMI_FSR_500DPS_VAL,
		SMI_FSR_250DPS_VAL,
		SMI_FSR_125DPS_VAL,
		NUM_SMI_FSR_VAL
};

#define SMI_FULLSCALE_AVL_MAX		10
#define SMI_FS_AVL_125DPS			125
#define SMI_FS_AVL_250DPS			250
#define SMI_FS_AVL_500DPS			500
#define SMI_FS_AVL_1000DPS		1000
#define SMI_FS_AVL_2000DPS		2000

/**
 *	struct smi_fullscale_avl - smi full scale releated feture.
 *	@gyro_fs_value: the reg value.
 *	@gyro_fs_dps: full scale range.
 *	@gyro_fs_rslt: scale resolution.
 */

struct smi_fullscale_avl {
		u8 gyro_fs_value;
		unsigned int gyro_fs_dps;
		unsigned int gyro_fs_rslt;
};


enum smi_devices {
		SMI130,
		NUM_DEVICE_PARTS
};

/**
 *	struct smi_chip_config - chip configuration data.
 *	@fsr:		full scale range.
 *	@filter_bw:	bandwidth frequency.
 *	@gyro_fifo_enable: gyro data ready enable.
 */
struct smi_chip_config {
	unsigned int fsr:3;
	unsigned int filter_bw:3;
	unsigned int gyro_fifo_enable:1;
};

/**
 *	struct smi_hw - Other important hardware information.
 *	@name:		name of the chip.
 *	@chip_id:	value of chip id.
 *	@config:	configuration of the chip.
 */
struct smi_hw {
	u8 *name;
	u8 chip_id;
	const struct smi_chip_config *config;
};

struct smi_config {
			struct smi_chip_config chip_config;
			const struct smi_hw *hw;
			enum   smi_devices chip_type;
};

struct smi_client_data {
	struct smi130_t device;
	struct i2c_client *client;
	struct smi_fullscale_avl *current_fullscale;
	struct iio_trigger	*trig;
	struct smi_chip_config chip_config;
	const struct smi_hw *hw;
	enum smi_devices chip_type;
	struct device *dev;
	atomic_t delay;
	unsigned int gpio_interrupt_pin;
	struct smi130_data_t value;
	u8 enable:1;
	int IRQ;
};


/* CONFIG_IIO_BUFFER */
#ifdef CONFIG_IIO_BUFFER
int smi_allocate_ring(struct iio_dev *indio_dev);
void smi_deallocate_ring(struct iio_dev *indio_dev);

#else
static inline int smi_allocate_ring(struct iio_dev *indio_dev)
{
	return 0;
}
static inline void smi_deallocate_ring(struct iio_dev *indio_dev)
{
}
#endif /* CONFIG_IIO_BUFFER */

#ifdef CONFIG_IIO_TRIGGER
int smi_probe_trigger(struct iio_dev *indio_dev);

void smi_remove_trigger(struct iio_dev *indio_dev);

#else
static inline int smi_probe_trigger(struct iio_dev *indio_dev)
{
	return 0;
}
static inline void smi_remove_trigger(struct iio_dev *indio_dev)
{
	return;
}
#endif

#endif	/*END FILE __SMI130_IIO_H__ */
