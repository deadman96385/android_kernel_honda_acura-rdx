/*
 * (C) Copyright 2013 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * SMI130 Linux IIO Driver
 */

#include "smi130_iio.h"

#include <asm/irq.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>


static void smi_scan_query(struct iio_dev *indio_dev)
{
	 struct smi_client_data *client_data = iio_priv(indio_dev);

	 client_data->chip_config.gyro_fifo_enable =
		test_bit(SMI_SCAN_GYRO_X, indio_dev->active_scan_mask) ||
		test_bit(SMI_SCAN_GYRO_Y, indio_dev->active_scan_mask) ||
		test_bit(SMI_SCAN_GYRO_Z, indio_dev->active_scan_mask);
}

 /**
  *  smi_set_trig_ready_enable() - enable related functions such as new data mode.
  *  @indio_dev: Device driver instance.
  *  @enable: enable/disable
  */
static int smi_set_trig_ready_enable(struct iio_dev *indio_dev, bool enable)
{
	 struct smi_client_data *client_data = iio_priv(indio_dev);

	if (enable) {
		smi_scan_query(indio_dev);
		if (client_data->chip_config.gyro_fifo_enable)
			dev_notice(indio_dev->dev.parent, "smi scan query active enable.\n");
#if defined(SMI130_ENABLE_INT1) || defined(SMI130_ENABLE_INT2)
	smi130_set_data_enable(INT_ENABLE);
	dev_notice(indio_dev->dev.parent, "smi new data ready enable.\n");
#endif
	} else {
	smi130_set_reset_int(1);
#if defined(SMI130_ENABLE_INT1) || defined(SMI130_ENABLE_INT2)
	smi130_set_data_enable(INT_DISABLE);
	dev_notice(indio_dev->dev.parent, "smi new data ready disabled.\n");
#endif
	}
	 return 0;
}

 /**
  * smi_data_rdy_trigger_set_state() - set data ready state
  * @trig: Trigger instance
  * @state: Desired trigger state
  */
static int smi_data_rdy_trigger_set_state(struct iio_trigger *trig,
						 bool state)
{
	return smi_set_trig_ready_enable(iio_trigger_get_drvdata(trig), state);
}

static const struct iio_trigger_ops smi_trigger_ops = {
	 .owner = THIS_MODULE,
	 .set_trigger_state = &smi_data_rdy_trigger_set_state,
};

int smi_probe_trigger(struct iio_dev *indio_dev)
{
	int ret;
	struct smi_client_data *client_data = iio_priv(indio_dev);
	int irq_num = 0;
	client_data->trig = iio_trigger_alloc("%s-dev%d",
					 indio_dev->name,
					 indio_dev->id);
	if (client_data->trig == NULL) {
		ret = -ENOMEM;
		dev_err(&indio_dev->dev, "smi failed to allocate iio trigger.\n");
		goto error_alloc_trigger;
	}
	/*inital the gpio input direction*/
	gpio_direction_input(client_data->gpio_interrupt_pin);
	irq_num = gpio_to_irq(client_data->gpio_interrupt_pin);
	ret = devm_request_irq(&indio_dev->dev, irq_num, &iio_trigger_generic_data_rdy_poll, 0,
					"smi_iio_int", client_data->trig);
	if (ret) {
		dev_err(&client_data->client->dev,
				"smi could not request irq! err = %d\n", ret);
		goto error_irq_request;
	}
	/*gpio interrupt trig type*/
	irq_set_irq_type(irq_num, IRQ_TYPE_EDGE_RISING);
	iio_trigger_set_drvdata(client_data->trig, indio_dev);
	client_data->trig->dev.parent = &client_data->client->dev;
	client_data->trig->ops = &smi_trigger_ops;
	ret = iio_trigger_register(client_data->trig);
	 if (ret < 0) {
		dev_err(&indio_dev->dev, "smi iio trigger failed to register.\n");
		goto erro_iio_trigger_register;
	 }
	 indio_dev->trig = client_data->trig;

	 return 0;

erro_iio_trigger_register:
	free_irq(gpio_to_irq(client_data->client->irq), client_data->trig);
 error_irq_request:
	 iio_trigger_free(client_data->trig);
 error_alloc_trigger:
	 return ret;
}

void smi_remove_trigger(struct iio_dev *indio_dev)
{
	struct smi_client_data *client_data = iio_priv(indio_dev);
	iio_trigger_unregister(client_data->trig);
	free_irq(client_data->client->irq, client_data->trig);
	iio_trigger_free(client_data->trig);
}
