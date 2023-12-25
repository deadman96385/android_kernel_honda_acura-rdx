/*
 * (C) Copyright 2013 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 *	SMI130 Linux IIO Driver
 */

#include "smi130_iio.h"

extern char smi_i2c_burst_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u16 len);

static irqreturn_t smi_buffer_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct smi_client_data *client_data = iio_priv(indio_dev);
	size_t bytes_per_datum = 0;
	s64 timestamp =  iio_get_time_ns();

	u8 buffer_data_out[SMI130_OUTPUT_DATA_SIZE];
	/*storge new axis data(or every frame)*/

	mutex_lock(&indio_dev->mlock);
	if (!(client_data->chip_config.gyro_fifo_enable))
		goto smi_buffer_handler_error;
	bytes_per_datum = 0;
	if (client_data->chip_config.gyro_fifo_enable)
		bytes_per_datum = SMI130_BYTES_PER_3AXIS_SENSOR;
/*
* There is new data to push to IIO ring buffer
* please give attentions to the data format
*/
	smi130_get_dataxyz(&client_data->value);

	buffer_data_out[0] = client_data->value.datax & 0xff;
	buffer_data_out[1] = (client_data->value.datax >> 8) & 0xff;
	buffer_data_out[2] = client_data->value.datay & 0xff;
	buffer_data_out[3] = (client_data->value.datay >> 8) & 0xff;
	buffer_data_out[4] = client_data->value.dataz & 0xff;
	buffer_data_out[5] = (client_data->value.dataz >> 8) & 0xff;
	/*for every frame, need 8 bytes to axis data storage*/
	iio_push_to_buffers_with_timestamp(indio_dev, buffer_data_out, timestamp);


smi_buffer_handler_error:
	mutex_unlock(&indio_dev->mlock);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;

}


int smi_allocate_ring(struct iio_dev *indio_dev)
{
	return iio_triggered_buffer_setup(indio_dev, &iio_pollfunc_store_time,
			&smi_buffer_handler, NULL);
}

void smi_deallocate_ring(struct iio_dev *indio_dev)
{
	iio_triggered_buffer_cleanup(indio_dev);
}
