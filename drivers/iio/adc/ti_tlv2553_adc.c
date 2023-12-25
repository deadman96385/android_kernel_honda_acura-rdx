/*
 * TI TLV2553  SPI ADC driver
 *
 * Copyright 2015 Denso.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>

/*tlv2553 adc channel definitions*/
#define USB_OTG_VBUS     0
#define VBAT_MON         1
#define P6V_CAM_MON      2
#define VDD_IVA          3
#define P5V0_MON         4
#define VDD_GPU          5
#define VDD_CORE         6
#define VDD_MPU          7
#define VDDS18V          8
#define VDD_DDR          9
#define DVDDIO_3V3       10

#define VREF   3300

struct tlv2553 {
	struct spi_device   *spi;
	struct mutex        lock;
	unsigned long       vref_mv;
	u8                  buffer[3];
};

static int tlv2553_adc_conversion(struct tlv2553 *tlv, u8 channel)
{
        int ret1, ret2;

	/*adc mode: 16bit storage, 12bit output, msb out first, unipolar binary*/
        tlv->buffer[1] = 12;
        tlv->buffer[2] = channel << 4 | tlv->buffer[1];

        mutex_lock(&tlv->lock);

        ret1 = spi_write(tlv->spi, &tlv->buffer[2], 1);
        if (ret1 < 0){
            mutex_unlock(&tlv->lock);
        return ret1;
        }

        ret1 = spi_read(tlv->spi, &tlv->buffer, 2);

        ret1 = (tlv->buffer[0] <<4 | tlv->buffer[1]>> 4) & 0xfff;

        /*chan11 1/2 vref, chan12 vref-, chan13 verf+, chan14 analog, chan15 reserved */
        if ( channel >10) {
            mutex_unlock(&tlv->lock);
            return ret1;
        }
        else {
            udelay(1500);

            ret2 = spi_write(tlv->spi, &tlv->buffer[2], 1);
            if (ret2 < 0){
                 mutex_unlock(&tlv->lock);
                 return ret2;
            }

            ret2 = spi_read(tlv->spi, &tlv->buffer, 2);

            mutex_unlock(&tlv->lock);

            ret2 = (tlv->buffer[0] <<4 | tlv->buffer[1]>> 4) & 0xfff;

            return ret2;
        }
}

static int tlv2553_read_raw(struct iio_dev *indio_dev,
                           struct iio_chan_spec const *channel,
                           int *val,
                           int *val2,
                           long mask)
{
    struct tlv2553 * tlv = iio_priv(indio_dev);
    int ret;

    switch(mask) {
    case IIO_CHAN_INFO_RAW:

         ret = tlv2553_adc_conversion(tlv, channel->channel);
         if(ret < 0)
             return ret;

         *val = ret;
         return IIO_VAL_INT;

    case IIO_CHAN_INFO_SCALE:

        *val = tlv->vref_mv / 1000 ;
        *val2 = 12;
        return IIO_VAL_FRACTIONAL_LOG2;

    default:
        return -EINVAL;
    }
}

#define TLV2553_VOLTAGE_CHANNEL(num)                                    \
        {                                                               \
                .type = IIO_VOLTAGE,                                    \
                .indexed = 1,                                           \
                .channel = num,                                         \
                .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),           \
                .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),   \
                .scan_index = num,                                      \
                .scan_type = {                                          \
                    .sign = 'u',                                        \
                    .realbits = 12,                                     \
                    .storagebits = 16,                                  \
                    .endianness = IIO_BE,                               \
                 },                                                     \
}

static const struct iio_chan_spec tlv2553_channels[] = {
        TLV2553_VOLTAGE_CHANNEL(0),
        TLV2553_VOLTAGE_CHANNEL(1),
        TLV2553_VOLTAGE_CHANNEL(2),
        TLV2553_VOLTAGE_CHANNEL(3),
        TLV2553_VOLTAGE_CHANNEL(4),
        TLV2553_VOLTAGE_CHANNEL(5),
        TLV2553_VOLTAGE_CHANNEL(6),
        TLV2553_VOLTAGE_CHANNEL(7),
        TLV2553_VOLTAGE_CHANNEL(8),
        TLV2553_VOLTAGE_CHANNEL(9),
        TLV2553_VOLTAGE_CHANNEL(10),
        TLV2553_VOLTAGE_CHANNEL(11),   /*(vref- + vref+)/2  2048 0x800*/
        TLV2553_VOLTAGE_CHANNEL(12),   /*vref- 0  0x00*/
        TLV2553_VOLTAGE_CHANNEL(13),   /*vref+ 4095 0xff */
        TLV2553_VOLTAGE_CHANNEL(14),   /*sw powerdown*/
        TLV2553_VOLTAGE_CHANNEL(15),   /*reserved*/
};

static const struct iio_info tlv2553_info = {
       .driver_module = THIS_MODULE,
       .read_raw = tlv2553_read_raw,
};

static int tlv2553_probe(struct spi_device *spi)
{
       struct tlv2553 *tlv;
       struct iio_dev *indio_dev;
       int ret;

       indio_dev = devm_iio_device_alloc(&spi->dev, sizeof *tlv);
       if (indio_dev == NULL)
            ret = -ENOMEM;

        tlv = iio_priv(indio_dev);
        if (NULL == ret)
            ret = - EFAULT;

        tlv->vref_mv = VREF;

        /*spi device initialization*/
        tlv->spi = spi;
        spi_set_drvdata(spi, indio_dev);
        ret = spi_setup(spi);

        /*iio device initialization*/
        indio_dev->dev.parent = &spi->dev;
        indio_dev->name = spi_get_device_id(spi)->name;
        indio_dev->modes = INDIO_DIRECT_MODE;
        indio_dev->channels = tlv2553_channels;
        indio_dev->num_channels = ARRAY_SIZE(tlv2553_channels);
        indio_dev->info = &tlv2553_info;

        mutex_init(&tlv->lock);

        ret = iio_device_register(indio_dev);
        if (ret != 0)
            return -EFAULT;

        return 0;
}

static int tlv2553_remove(struct spi_device *spi)
{
       struct iio_dev *indio_dev = spi_get_drvdata(spi);
       struct tlv2553 *tlv = iio_priv(indio_dev);

       iio_device_unregister(indio_dev);
       iio_device_free(indio_dev);

       return 0;
}


static const struct spi_device_id tlv2553_id[] = {
       {"tlv2553", 0},
       {}
};

MODULE_DEVICE_TABLE(spi, tlv2553_id);

#ifdef CONFIG_OF
static const struct of_device_id tlv2553_of_match[] = {
        { .compatible = "tlv2553" },
        { }
};
MODULE_DEVICE_TABLE(of, tlv2553_of_match);
#endif

static struct spi_driver tlv2553_driver = {
        .driver = {
                .name   = "tlv2553",
                .owner  = THIS_MODULE,
                .of_match_table = of_match_ptr(tlv2553_of_match),
        },
        .probe = tlv2553_probe,
        .remove = tlv2553_remove,
        .id_table = tlv2553_id,

};
module_spi_driver(tlv2553_driver);

MODULE_DESCRIPTION("Texas Instruments TLV2553");
MODULE_LICENSE("GPL V2");

