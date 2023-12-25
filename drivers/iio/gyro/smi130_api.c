/*
 * (C) Copyright 2013 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * SMI130 API functions
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

#include "smi130_iio.h"
struct smi130_t *p_smi130;


/*****************************************************************************
 * Description: *//**\brief API Initialization routine
  *  \param smi130_t *smi130
 *      Pointer to a structure.
 *
 *       structure members are
 *
 *       unsigned char chip_id;
 *       unsigned char dev_addr;
 *       SMI130_BRD_FUNC_PTR;
 *       SMI130_WR_FUNC_PTR;
 *       SMI130_RD_FUNC_PTR;
 *       void(*delay_msec)( SMI130_MDELAY_DATA_TYPE );
  *
 *  \return result of communication routines
 *
 *****************************************************************************/
int smi130_init(struct smi130_t *smi130)
{
	int comres = 0;
	unsigned char a_data_u8r;
	p_smi130 = smi130;

	p_smi130->dev_addr = SMI130_I2C_ADDR;

	/*Read CHIP_ID */
	comres = p_smi130->bus_read(p_smi130->dev_addr,
	 SMI130_CHIP_ID_ADDR, &a_data_u8r, 1);
	p_smi130->chip_id = a_data_u8r;
	return comres;
}
/*****************************************************************************
 * Description: *//**\brief Reads Rate dataX from location 02h and 03h
 * registers
 *  \param
 *      SMI130_S16  *data_x   :  Address of data_x
 *  \return
 *      result of communication routines
 *
 *****************************************************************************/
int smi130_get_dataxyz(struct smi130_data_t *data)
{
	int comres;
	unsigned char a_data_u8r[6];
	if (p_smi130 == SMI130_NULL) {
		comres = E_SMI130_NULL_PTR;
	} else {
		comres = p_smi130->bus_read(p_smi130->dev_addr,
		 SMI130_RATE_X_LSB_VALUEX__REG, a_data_u8r, 6);

		/* Data X */
		a_data_u8r[0] = ((a_data_u8r[0] & SMI130_RATE_X_LSB_VALUEX__MSK) >> SMI130_RATE_X_LSB_VALUEX__POS);
		data->datax = (SMI130_S16)
		((((SMI130_S16)((signed char)a_data_u8r[1]))
		<< SMI130_SHIFT_8_POSITION) | (a_data_u8r[0]));

		/* Data Y */
		a_data_u8r[2] = ((a_data_u8r[2] & SMI130_RATE_Y_LSB_VALUEY__MSK) >> SMI130_RATE_Y_LSB_VALUEY__POS);
		data->datay = (SMI130_S16)
		((((SMI130_S16)((signed char)a_data_u8r[3]))
		<< SMI130_SHIFT_8_POSITION) | (a_data_u8r[2]));

		/* Data Z */
		a_data_u8r[4] = ((a_data_u8r[4] & SMI130_RATE_Z_LSB_VALUEZ__MSK) >> SMI130_RATE_Z_LSB_VALUEZ__POS);
		data->dataz = (SMI130_S16)
		((((SMI130_S16)((signed char)a_data_u8r[5]))
		<< SMI130_SHIFT_8_POSITION) | (a_data_u8r[4]));
	}
	return comres;
}
/*****************************************************************************
 * Description: *//**\brief Reads data X,Y,Z and Interrupts
 *							from location 02h to 07h
 *  \param
 *      smi130_data_t *data   :  Address of smi130_data_t
 *  \return
 *      result of communication routines
 *
 *****************************************************************************/
int smi130_get_range_reg(unsigned char *range)
{
	int comres;
	unsigned char v_data_u8r;
	if (p_smi130 == SMI130_NULL) {
		comres = E_SMI130_NULL_PTR;
	} else {
		comres = p_smi130->bus_read
		(p_smi130->dev_addr,
		SMI130_RANGE_ADDR_RANGE__REG, &v_data_u8r, 1);
		*range =
		((v_data_u8r & SMI130_RANGE_ADDR_RANGE__MSK) >> SMI130_RANGE_ADDR_RANGE__POS);
	}
	return comres;
}
/*****************************************************************************
 * Description: *//**\brief This API sets the range register 0x0Fh
 * (0 to 2 bits)
 *
 *  \param unsigned char range
 *
 *      Range[0....7]
 *      0 2000/s
 *      1 1000/s
 *      2 500/s
 *      3 250/s
 *      4 125/s
 *  \return Communication results
 *****************************************************************************/
int smi130_set_range_reg(unsigned char range)
{
	int comres;
	unsigned char v_data_u8r;
	if (p_smi130 == SMI130_NULL) {
		comres = E_SMI130_NULL_PTR;
	} else {
		if (range < C_SMI130_Five_U8X) {
			comres = p_smi130->bus_read
			(p_smi130->dev_addr,
			SMI130_RANGE_ADDR_RANGE__REG, &v_data_u8r, 1);
			v_data_u8r = ((v_data_u8r & ~SMI130_RANGE_ADDR_RANGE__MSK) | ((range << SMI130_RANGE_ADDR_RANGE__POS) & SMI130_RANGE_ADDR_RANGE__MSK));// RCG: Replacing SMI130_SET_BITSLICE
			comres = p_smi130->bus_write
			(p_smi130->dev_addr,
			SMI130_RANGE_ADDR_RANGE__REG, &v_data_u8r, 1);
		} else {
			comres = E_SMI130_OUT_OF_RANGE;
		}
	}
	return comres;
}
/*****************************************************************************
 * Description: *//**\brief This API reads the high resolution bit of 0x10h
 * Register 7th bit
 *
 *  \param unsigned char *high_res
 *                      Pointer to a variable passed as a parameter
 *  \return communication results
 *
 *****************************************************************************/
int smi130_get_bw(unsigned char *bandwidth)
{
	int comres;
	unsigned char v_data_u8r;
	if (p_smi130 == SMI130_NULL) {
		comres = E_SMI130_NULL_PTR;
	} else {
		comres = p_smi130->bus_read
		(p_smi130->dev_addr, SMI130_BW_ADDR__REG, &v_data_u8r, 1);
		*bandwidth = ((v_data_u8r & SMI130_BW_ADDR__MSK) >> SMI130_BW_ADDR__POS);
	}
	return comres;
}

/*****************************************************************************
 * Description: *//**\brief This API writes the Bandwidth register (0x10h of 0
 * to 3 bits)
 *
 *  \param unsigned char bandwidth,
 *              The bandwidth to be set passed as a parameter
 *
 *              0 no filter(523 Hz)
 *              1 230Hz
 *              2 116Hz
 *              3 47Hz
 *              4 23Hz
 *              5 12Hz
 *              6 64Hz
 *              7 32Hz
 *
 *  \return communication results
 *****************************************************************************/
int smi130_set_bw(unsigned char bandwidth)
{
	int comres;
	unsigned char v_data_u8r;
	if (p_smi130 == SMI130_NULL) {
		comres = E_SMI130_NULL_PTR;
	} else {
		if (bandwidth < C_SMI130_Eight_U8X) {
			comres = p_smi130->bus_write
			(p_smi130->dev_addr,
			SMI130_BW_ADDR__REG, &v_data_u8r, 1);
			v_data_u8r = ((v_data_u8r & ~SMI130_BW_ADDR__MSK) | ((bandwidth << SMI130_BW_ADDR__POS) & SMI130_BW_ADDR__MSK));// RCG: Replacing SMI130_SET_BITSLICE
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_BW_ADDR__REG, &v_data_u8r, 1);
		} else {
			comres = E_SMI130_OUT_OF_RANGE;
		}
	}
	return comres;
}

/*****************************************************************************
 * Description: *//**\brief This API is used to get the output type status
 *
 *
 *  \param unsigned char channel,unsigned char *int_od
 *                  SMI130_INT1    ->   0
 *                  SMI130_INT2    ->   1
 *                  int_od : open drain   ->   1
 *                           push pull    ->   0
 *
 *
 *  \return
 *
 *
 *****************************************************************************/
int smi130_get_int_od(unsigned char param, unsigned char *int_od)
{
	int comres;
	unsigned char v_data_u8r;

	if (p_smi130 == SMI130_NULL) {
		comres = E_SMI130_NULL_PTR;
	} else {
		switch (param) {
			case SMI130_INT1:
				comres = p_smi130->bus_read(p_smi130->dev_addr, SMI130_INT_ENABLE1_IT1_OD__REG, &v_data_u8r, 1);
				*int_od =((v_data_u8r & SMI130_INT_ENABLE1_IT1_OD__MSK) >> SMI130_INT_ENABLE1_IT1_OD__POS);
			break;

			default:
				comres = E_SMI130_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}

/*****************************************************************************
 * Description: *//**\brief This API is used to set the output type status
 *
 *  \param unsigned char channel,unsigned char *int_od
 *                  SMI130_INT1    ->   0
 *                  SMI130_INT2    ->   1
 *                  int_od : open drain   ->   1
 *                           push pull    ->   0
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
int smi130_set_int_od(unsigned char param, unsigned char int_od)
{
	int comres;
	unsigned char v_data_u8r;

	if (p_smi130 == SMI130_NULL) {
		comres = E_SMI130_NULL_PTR;
	} else {
		switch (param) {

			case SMI130_INT1:
				comres = p_smi130->bus_read(p_smi130->dev_addr,	SMI130_INT_ENABLE1_IT1_OD__REG, &v_data_u8r, 1);
				v_data_u8r = ((v_data_u8r & ~SMI130_INT_ENABLE1_IT1_OD__MSK) | ((int_od << SMI130_INT_ENABLE1_IT1_OD__POS) & SMI130_INT_ENABLE1_IT1_OD__MSK));
				comres = p_smi130->bus_write(p_smi130->dev_addr, SMI130_INT_ENABLE1_IT1_OD__REG, &v_data_u8r, 1);
			break;

			default:
				comres = E_SMI130_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}

/*****************************************************************************
 * Description: *//**\brief This API is used to get data Interrupt1 and data
 * Interrupt2
 *
 *  \param unsigned char axis,unsigned char *int_data
 *                       axis :
 *                       SMI130_INT1_DATA -> 0
 *                       SMI130_INT2_DATA -> 1
 *                       int_data :
 *                       Disable     -> 0
 *                       Enable      -> 1
 *
 *
 *  \return
 *
 *
 *****************************************************************************/
int smi130_get_int_data(unsigned char axis, unsigned char *int_data)
{
	int comres;
	unsigned char v_data_u8r;
	if (p_smi130 == SMI130_NULL) {
		comres = E_SMI130_NULL_PTR;
	} else {
		switch (axis) {
		case SMI130_INT1_DATA:
			comres = p_smi130->bus_read(p_smi130->dev_addr, SMI130_MAP_1_INT1_DATA__REG, &v_data_u8r, 1);

			*int_data = ((v_data_u8r & SMI130_MAP_1_INT1_DATA__MSK) >> SMI130_MAP_1_INT1_DATA__POS);
			break;

		default:
			comres = E_SMI130_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/*****************************************************************************
 * Description: *//**\brief This API is used to set data Interrupt1 and data
 * Interrupt2
 *
 *  \param unsigned char axis,unsigned char *int_data
 *                       axis :
 *                       SMI130_INT1_DATA -> 0
 *                       SMI130_INT2_DATA -> 1
 *                       int_data :
 *                       Disable     -> 0
 *                       Enable      -> 1
 *
 *
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
int smi130_set_int_data(unsigned char axis, unsigned char int_data)
{
	int comres;
	unsigned char v_data_u8r;
	if (p_smi130 == SMI130_NULL) {
		comres = E_SMI130_NULL_PTR;
	} else {
		switch (axis) {
		case SMI130_INT1_DATA:
			comres = p_smi130->bus_read(p_smi130->dev_addr, SMI130_MAP_1_INT1_DATA__REG, &v_data_u8r, 1);

			v_data_u8r = ((v_data_u8r & ~SMI130_MAP_1_INT1_DATA__MSK) | ((int_data << SMI130_MAP_1_INT1_DATA__POS) & SMI130_MAP_1_INT1_DATA__MSK));

			comres = p_smi130->bus_write(p_smi130->dev_addr, SMI130_MAP_1_INT1_DATA__REG, &v_data_u8r, 1);
			break;

		default:
			comres = E_SMI130_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/*****************************************************************************
 * Description: *//**\brief This API is used to set the Interrupt Reset
 *
 *
 *
 *
 *  \param unsigned char reset_int
 *                    1 -> Reset All Interrupts
 *
 *
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
int smi130_set_reset_int(unsigned char reset_int)
{
	int comres;
	unsigned char v_data_u8r;
	if (p_smi130 == SMI130_NULL) {
		comres = E_SMI130_NULL_PTR;
	} else {
		comres = p_smi130->bus_read(p_smi130->dev_addr,
		SMI130_RST_LATCH_ADDR_RESET_INT__REG, &v_data_u8r, 1);
		v_data_u8r = ((v_data_u8r & ~SMI130_RST_LATCH_ADDR_RESET_INT__MSK) | ((reset_int << SMI130_RST_LATCH_ADDR_RESET_INT__POS) & SMI130_RST_LATCH_ADDR_RESET_INT__MSK));// RCG: Replacing SMI130_SET_BITSLICE
		comres = p_smi130->bus_write(p_smi130->dev_addr,
		SMI130_RST_LATCH_ADDR_RESET_INT__REG, &v_data_u8r, 1);
	}
	return comres;
}
/*****************************************************************************
 * Description: *//**\brief This API is used to get the status of offset
 *
 *
 *
 *
 *  \param unsigned char axis,unsigned char *offset
 *                         axis ->
 *                   SMI130_X_AXIS     ->      0
 *                   SMI130_Y_AXIS     ->      1
 *                   SMI130_Z_AXIS     ->      2
 *                   offset -> Any valid value
 *
 *  \return
 *
 *
 *****************************************************************************/
int smi130_get_offset(unsigned char axis, SMI130_S16 *offset)
{
	int comres;
	unsigned char v_data1_u8r, v_data2_u8r;
	if (p_smi130 == SMI130_NULL) {
		comres = E_SMI130_NULL_PTR;
	} else {
		switch (axis) {
		case SMI130_X_AXIS:
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_TRIM_GP0_ADDR_OFFSET_X__REG, &v_data1_u8r, 1);
			v_data1_u8r = ((v_data1_u8r & SMI130_TRIM_GP0_ADDR_OFFSET_X__MSK) >> SMI130_TRIM_GP0_ADDR_OFFSET_X__POS);
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_OFC1_ADDR_OFFSET_X__REG, &v_data2_u8r, 1);
			v_data2_u8r = ((v_data2_u8r & SMI130_OFC1_ADDR_OFFSET_X__MSK) >> SMI130_OFC1_ADDR_OFFSET_X__POS);
			v_data2_u8r = ((v_data2_u8r <<
			SMI130_SHIFT_2_POSITION) | v_data1_u8r);
			comres = p_smi130->bus_write
			(p_smi130->dev_addr, SMI130_OFC2_ADDR, &v_data1_u8r, 1);
			*offset = (SMI130_S16)((((SMI130_S16)
				((signed char)v_data1_u8r))
			<< SMI130_SHIFT_4_POSITION) | (v_data2_u8r));
			break;
		case SMI130_Y_AXIS:
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_TRIM_GP0_ADDR_OFFSET_Y__REG, &v_data1_u8r, 1);
			v_data1_u8r = ((v_data1_u8r & SMI130_TRIM_GP0_ADDR_OFFSET_Y__MSK) >> SMI130_TRIM_GP0_ADDR_OFFSET_Y__POS);
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_OFC1_ADDR_OFFSET_Y__REG, &v_data2_u8r, 1);
			v_data2_u8r = ((v_data2_u8r & SMI130_OFC1_ADDR_OFFSET_Y__MSK) >> SMI130_OFC1_ADDR_OFFSET_Y__POS);
			v_data2_u8r = ((v_data2_u8r <<
			SMI130_SHIFT_1_POSITION) | v_data1_u8r);
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_OFC3_ADDR, &v_data1_u8r, 1);
			*offset = (SMI130_S16)((((SMI130_S16)
				((signed char)v_data1_u8r))
			<< SMI130_SHIFT_4_POSITION) | (v_data2_u8r));
			break;
		case SMI130_Z_AXIS:
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_TRIM_GP0_ADDR_OFFSET_Z__REG, &v_data1_u8r, 1);
			v_data1_u8r = ((v_data1_u8r & SMI130_TRIM_GP0_ADDR_OFFSET_Z__MSK) >> SMI130_TRIM_GP0_ADDR_OFFSET_Z__POS);
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_OFC1_ADDR_OFFSET_Z__REG, &v_data2_u8r, 1);
			v_data2_u8r = ((v_data2_u8r & SMI130_OFC1_ADDR_OFFSET_Z__MSK) >> SMI130_OFC1_ADDR_OFFSET_Z__POS);
			v_data2_u8r = ((v_data2_u8r << SMI130_SHIFT_1_POSITION)
				| v_data1_u8r);
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_OFC4_ADDR, &v_data1_u8r, 1);
			*offset = (SMI130_S16)((((SMI130_S16)
				((signed char)v_data1_u8r))
			<< SMI130_SHIFT_4_POSITION) | (v_data2_u8r));
			break;
		default:
			comres = E_SMI130_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/*****************************************************************************
 * Description: *//**\brief This API is used to set the status of offset
 *
 *  \param unsigned char axis,unsigned char offset
 *                         axis ->
 *                   SMI130_X_AXIS     ->      0
 *                   SMI130_Y_AXIS     ->      1
 *                   SMI130_Z_AXIS     ->      2
 *                   offset -> Any valid value
 *
 *
 *  \return
 *
 *
 *****************************************************************************/
int smi130_set_offset(
unsigned char axis, SMI130_S16 offset)
{
	int comres;
	unsigned char v_data1_u8r, v_data2_u8r;
	if (p_smi130 == SMI130_NULL) {
		comres = E_SMI130_NULL_PTR;
	} else {
		switch (axis) {
		case SMI130_X_AXIS:
			v_data1_u8r = ((signed char) (offset & 0x0FF0))
			>> SMI130_SHIFT_4_POSITION;
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_OFC2_ADDR, &v_data1_u8r, 1);

			v_data1_u8r = (unsigned char) (offset & 0x000C);
			v_data2_u8r = ((v_data2_u8r & ~SMI130_OFC1_ADDR_OFFSET_X__MSK) | ((v_data1_u8r << SMI130_OFC1_ADDR_OFFSET_X__POS) & SMI130_OFC1_ADDR_OFFSET_X__MSK));// RCG: Replacing SMI130_SET_BITSLICE
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_OFC1_ADDR_OFFSET_X__REG, &v_data2_u8r, 1);

			v_data1_u8r = (unsigned char) (offset & 0x0003);
			v_data2_u8r = ((v_data2_u8r & ~SMI130_TRIM_GP0_ADDR_OFFSET_X__MSK) | ((v_data1_u8r << SMI130_TRIM_GP0_ADDR_OFFSET_X__POS) & SMI130_TRIM_GP0_ADDR_OFFSET_X__MSK));// RCG: Replacing SMI130_SET_BITSLICE
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_TRIM_GP0_ADDR_OFFSET_X__REG, &v_data2_u8r, 1);
			break;
		case SMI130_Y_AXIS:
			v_data1_u8r = ((signed char) (offset & 0x0FF0)) >>
			SMI130_SHIFT_4_POSITION;
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_OFC3_ADDR, &v_data1_u8r, 1);

			v_data1_u8r = (unsigned char) (offset & 0x000E);
			v_data2_u8r = ((v_data2_u8r & ~SMI130_OFC1_ADDR_OFFSET_Y__MSK) | ((v_data1_u8r << SMI130_OFC1_ADDR_OFFSET_Y__POS) & SMI130_OFC1_ADDR_OFFSET_Y__MSK));// RCG: Replacing SMI130_SET_BITSLICE
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_OFC1_ADDR_OFFSET_Y__REG, &v_data2_u8r, 1);

			v_data1_u8r = (unsigned char) (offset & 0x0001);
			v_data2_u8r = ((v_data2_u8r & ~SMI130_TRIM_GP0_ADDR_OFFSET_Y__MSK) | ((v_data1_u8r << SMI130_TRIM_GP0_ADDR_OFFSET_Y__POS) & SMI130_TRIM_GP0_ADDR_OFFSET_Y__MSK));// RCG: Replacing SMI130_SET_BITSLICE
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_TRIM_GP0_ADDR_OFFSET_Y__REG, &v_data2_u8r, 1);
			break;
		case SMI130_Z_AXIS:
			v_data1_u8r = ((signed char) (offset & 0x0FF0)) >>
			SMI130_SHIFT_4_POSITION;
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_OFC4_ADDR, &v_data1_u8r, 1);

			v_data1_u8r = (unsigned char) (offset & 0x000E);
			v_data2_u8r = ((v_data2_u8r & ~SMI130_OFC1_ADDR_OFFSET_Z__MSK) | ((v_data1_u8r << SMI130_OFC1_ADDR_OFFSET_Z__POS) & SMI130_OFC1_ADDR_OFFSET_Z__MSK));// RCG: Replacing SMI130_SET_BITSLICE
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_OFC1_ADDR_OFFSET_Z__REG, &v_data2_u8r, 1);

			v_data1_u8r = (unsigned char) (offset & 0x0001);
			v_data2_u8r = ((v_data2_u8r & ~SMI130_TRIM_GP0_ADDR_OFFSET_Z__MSK) | ((v_data1_u8r << SMI130_TRIM_GP0_ADDR_OFFSET_Z__POS) & SMI130_TRIM_GP0_ADDR_OFFSET_Z__MSK));// RCG: Replacing SMI130_SET_BITSLICE
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
			SMI130_TRIM_GP0_ADDR_OFFSET_Z__REG, &v_data2_u8r, 1);
			break;
		default:
			comres = E_SMI130_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}

/*****************************************************************************
 * Description: *//**\brief This API is used to set data enable data
 *
 *
 *	\param unsigned char data_en:
 *			Value to be written passed as a parameter
 *			 0 --> Disable
 *			 1 --> Enable
 *
 *
 *	\return communication results
 *
 *
 *****************************************************************************/
int smi130_set_data_enable(unsigned char data_en)
{
	int comres;
	unsigned char v_data_u8r;
	if (p_smi130 == SMI130_NULL) {
		comres = E_SMI130_NULL_PTR;
	} else {
		comres = p_smi130->bus_read
			(p_smi130->dev_addr,
		SMI130_INT_ENABLE0_DATAEN__REG, &v_data_u8r, 1);
		v_data_u8r = ((v_data_u8r & ~SMI130_INT_ENABLE0_DATAEN__MSK) | ((data_en << SMI130_INT_ENABLE0_DATAEN__POS) & SMI130_INT_ENABLE0_DATAEN__MSK));// RCG: Replacing SMI130_SET_BITSLICE
		comres = p_smi130->bus_write
			(p_smi130->dev_addr,
		SMI130_INT_ENABLE0_DATAEN__REG, &v_data_u8r, 1);
	}
	return comres;
}

/*****************************************************************************
 * Description: *//**\brief This API is used to to do selftest to sensor
 * sensor
 *
 *
 *
 *
 *  \param unsigned char *result
 *
 *
 *
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
int smi130_selftest(unsigned char *result)
	{
	int comres = C_SMI130_Zero_U8X;
	unsigned char data1 = C_SMI130_Zero_U8X;
	unsigned char data2 = C_SMI130_Zero_U8X;

        // i2cget -f -y 0 0x68 0x3C
	comres = p_smi130->bus_read(p_smi130->dev_addr,
	SMI130_SELF_TEST_ADDR, &data1, C_SMI130_One_U8X);

        data2  = ((data1 & SMI130_SELF_TEST_ADDR_RATEOK__MSK) >> SMI130_SELF_TEST_ADDR_RATEOK__POS);
	data1  = ((data1 & ~SMI130_SELF_TEST_ADDR_TRIGBIST__MSK) | ((C_SMI130_One_U8X << SMI130_SELF_TEST_ADDR_TRIGBIST__POS) & SMI130_SELF_TEST_ADDR_TRIGBIST__MSK));// RCG: Replacing SMI130_SET_BITSLICE

        // i2cset -f -y 0 0x68 0x3C
        comres += p_smi130->bus_write(p_smi130->dev_addr,
	SMI130_SELF_TEST_ADDR_TRIGBIST__REG, &data1, C_SMI130_One_U8X);

	/* Waiting time to complete the selftest process */
	p_smi130->delay_msec(10);

	/* Reading Selftest result bir bist_failure */
        // i2cget -f -y 0 0x68 0x3C
	comres = p_smi130->bus_read(p_smi130->dev_addr,
	SMI130_SELF_TEST_ADDR_BISTFAIL__REG, &data1, C_SMI130_One_U8X);

        data1  = ((data1 & SMI130_SELF_TEST_ADDR_BISTFAIL__MSK) >> SMI130_SELF_TEST_ADDR_BISTFAIL__POS);
	if ((data1 == 0x00) && (data2 == 0x01))
		*result = C_SMI130_SUCCESS;
	else
		*result = C_SMI130_FAILURE;
	return comres;
}

/*****************************************************************************
 * Description: *//**\brief This API is used to to get watchdog status to sensor
 *
 *  \param unsigned char *wdt
 *                      Pointer to a variable passed as a parameter
 *  \return communication results
 *
 *****************************************************************************/
int smi130_get_wdt(unsigned char *wdt)
{
	int comres;
	unsigned char v_data_u8r;
	if (p_smi130 == SMI130_NULL) {
		comres = E_SMI130_NULL_PTR;
	} else {
		comres = p_smi130->bus_read
			(p_smi130->dev_addr, SMI130_BGW_SPI3_WDT_ADDR, &v_data_u8r, 1);
		 *wdt = !!(v_data_u8r & SMI130_BGW_SPI3_WDT_ADDR_I2C_WDT_EN__MSK);
	}
	return comres;
}

/*****************************************************************************
 * Description: *//**\brief This API is used to to set watchdog to sensor
 *
 *  \param unsigned char wdt,
 *              The wdt to be set passed as a parameter
 *
 *              0 disable
 *              1 enable
 *
 *              wdt_sel is fixed to '1' - timer period is always 50ms.
 *  \return communication results
 *****************************************************************************/
int smi130_set_wdt(unsigned char wdt)
{
	int comres;
	unsigned char v_data_u8r;
	if (p_smi130 == SMI130_NULL) {
		comres = E_SMI130_NULL_PTR;
	} else {
		switch (wdt) {
		case 0:
		case 1:
			comres = p_smi130->bus_read
				(p_smi130->dev_addr, SMI130_BGW_SPI3_WDT_ADDR, &v_data_u8r, 1);
			if(comres < 0) {
				return comres;
			}
			v_data_u8r &= ~(SMI130_BGW_SPI3_WDT_ADDR_I2C_WDT_EN__MSK
			              | SMI130_BGW_SPI3_WDT_ADDR_I2C_WDT_SEL__MSK);
			v_data_u8r |= (!!wdt << SMI130_BGW_SPI3_WDT_ADDR_I2C_WDT_EN__POS)
			              | SMI130_BGW_SPI3_WDT_ADDR_I2C_WDT_SEL__MSK;
			comres = p_smi130->bus_write
				(p_smi130->dev_addr,
				SMI130_BGW_SPI3_WDT_ADDR, &v_data_u8r, 1);
			break;
		default:
			comres = E_SMI130_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}

/*End of smi130 api*/
