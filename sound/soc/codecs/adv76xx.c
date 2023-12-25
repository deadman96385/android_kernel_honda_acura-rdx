/*
 * Copyright (C) 2014 Vodalys-labs
 * Author: Pablo Anton <pablo.anton@vodalys-labs.com>
 * Author: Jean-Michel Hautbois <jean-michel.hautbois@vodalys.com>
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <linux/device.h>
#include <media/adv7604.h>

#include "adv76xx.h"

static struct regmap * adv76xx_get_regmap(struct device *dev)
{
	struct adv76xx_state *state = dev->platform_data;

	if (!state) {
		dev_err(dev, "Regmap is not ready\n");
		return NULL;
	}

	dev_dbg(dev, "Regmap taken for HDMI sound device %s\n", dev_name(dev));

	return state->regmap[ADV76XX_PAGE_HDMI];
}

/*
 * Set the Master Clock register multiplier that control
 * the Master Clock output frequency
 */
static int adv76xx_set_mclk_fs_n(struct snd_soc_codec *codec,
				 unsigned int fs)
{
	struct adv76xx_state *state = codec->dev->platform_data;

	if (!state)
		return -EINVAL;

	return regmap_write(state->regmap[ADV76XX_PAGE_AFE],
			ADV76XX_MCLK_FS, fs);
}

static int adv76xx_codec_probe(struct snd_soc_codec * codec)
{
	struct adv76xx_state *state = codec->dev->platform_data;
	struct regmap *regmap;

	if (!state) {
		dev_err(codec->dev,
			"probe error, no platform data into codec device\n");
		return -EINVAL;
	}

	regmap = state->regmap[ADV76XX_PAGE_IO];

	if (!regmap) {
		dev_err(codec->dev, "probe error, no regmap for IO region\n");
		return -EINVAL;
	}

	return 0;
}


static struct snd_soc_codec_driver adv76xx_audio_codec = {
	.probe = adv76xx_codec_probe,
	.get_regmap = adv76xx_get_regmap,
};

static int adv76xx_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	int finalfmt;

	if (!codec)
		return -EINVAL;

	finalfmt = snd_soc_read(codec, ADV76XX_HDMI_REGISTER_03);
	if (finalfmt < 0) {
		dev_err(dai->dev, "Fail reading register HDMI %x\n",
				ADV76XX_HDMI_REGISTER_03);
		return -EINVAL;
	}

	/* Setting i2s data format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		finalfmt &= (~ADV76XX_I2SOUTMODE_MASK |
				ADV76XX_I2SOUTMODE_I2S);
		dev_dbg(dai->dev, "Configuring DAI as I2S\n");
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
	default:
		return -EINVAL;
	}

	snd_soc_write(codec, ADV76XX_HDMI_REGISTER_03, finalfmt);

	/*
	 * i2s clock and frame master setting.
	 * ONLY support:
	 *  - clock and frame master
	*/
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		/* Setting MCLK to 256fs */
		adv76xx_set_mclk_fs_n(codec, 0x01);
		dev_dbg(dai->dev, "Configuring I2S master\n");
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void log_pcm_sample_info(struct snd_soc_dai *dai)
{
	unsigned int reg, aux;
	struct adv76xx_state *state = dai->dev->platform_data;

	static const char * const w24_length[] = {
		"Not indicated",
		"20",
		"22",
		"reserved",
		"23",
		"24",
		"21",
	};

	static const char * const w20_length[] = {
		"Not indicated",
		"16",
		"18",
		"reserved",
		"19",
		"20",
		"21",
	};

	/* Check channel status data */
	if (state) {
		regmap_read(state->regmap[ADV76XX_PAGE_IO], 0x65, &reg);
		reg &= 0x80;
		dev_dbg(dai->dev, "CS_DATA is %s\n",
			reg ? "valid" : "not valid");
	}

	/* Check if there are PCM audio samples */
	reg = snd_soc_read(dai->codec, 0x18) & 0x01;
	dev_dbg(dai->codec->dev, "Received PCM package: %s\n",
		reg > 0 ? "yes":"no");

	reg = snd_soc_read(dai->codec, 0x36) & 0x02;
	dev_dbg(dai->dev, "Received PCM package(from CS_DATA): %s\n",
			reg ? "no" : "yes");

	if (!reg) {
		/* Get audio sampling frequency */
		reg = snd_soc_read(dai->codec, 0x39) & 0x0f;
		dev_dbg(dai->dev, "Sample Freq: %d kHz\n",
			reg > 0 ? cs_data_fs[reg] : 0);

		reg = snd_soc_read(dai->codec, 0x39) & 0x30;
		dev_dbg(dai->codec->dev, "Clock accuracy: %x\n",
			reg > 0 ? reg : -1);

		aux = snd_soc_read(dai->codec, 0x3a) & 0x01;
		dev_dbg(dai->dev, "Max audio sample word length: %d\n",
			aux ? 24 : 20);

		reg = (snd_soc_read(dai->codec, 0x3a) & 0x0e) >> 1;
		dev_dbg(dai->dev, "Audio sample word length: %s\n",
			aux ? w24_length[reg] : w20_length[reg]);
	}
}

static int adv76xx_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *hw_params,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct adv76xx_state *state = NULL;
	int reg, freq;
	int err = 0;

	if (!codec)
		return -EINVAL;

	state = codec->dev->platform_data;

	if (!state)
		return -EINVAL;

	/* Check if there are PCM audio samples */
	reg = snd_soc_read(dai->codec, ADV7611_PACKETS_DETECTED_2);
	if (reg < 0 || !(reg & ADV76XX_AUDIO_SAMPLE_PCKT_DET))
		dev_info(dai->dev,
			"Warning: It seems no PCM Audio available\n");

	/* Check if there Audio is muted */
	err = regmap_read(state->regmap[ADV76XX_PAGE_IO],
			ADV7611_HDMI_LVL_RAW_STATUS_2, &reg);
	if (!err && ((reg & 0x40) || (reg & 0x20)))
		dev_info(dai->dev,
			"Warning: AV Mute or Internal Audio is muted\n");

	/* Check if params and input format correspond*/
	freq = params_rate(hw_params);

	/* Get audio sampling frequency */
	reg = snd_soc_read(dai->codec, 0x39) & 0x0f;
	if (reg < 0 || (freq != cs_data_fs[reg]))
		dev_info(dai->dev,
		"Warning: Frequency selected %d may be wrong\n", freq);

	log_pcm_sample_info(dai);

	return 0;
}

static int adv76xx_mute(struct snd_soc_dai *dai, int mute)
{
	int reg;
	struct snd_soc_codec *codec = dai->codec;

	if (!codec)
		return -EINVAL;

	/* Set general MUTE_AUDIO depending on parameter mute */
	reg = snd_soc_read(codec, ADV76XX_MUTE_CTRL);
	if (reg < 0)
		return -1;

	if (snd_soc_write(codec, ADV76XX_MUTE_CTRL,
		      (reg & (mute ? 0xff : 0xfe))))
		return -1;

	return 0;
}

static struct snd_soc_dai_ops adv76xx_dai_ops = {
	.hw_params = adv76xx_hw_params,
	.set_fmt = adv76xx_set_dai_fmt,
	.digital_mute = adv76xx_mute,
};

#define ADV76XX_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE |\
			SNDRV_PCM_FMTBIT_S32_LE)


/* DAI (Digital Audio Interface) struct for the codec */
static struct snd_soc_dai_driver adv76xx_dais[] = {
	{
		.name = DAI_ADV7604_NAME,
		.capture = {
			.stream_name    = "HDMI-Capture",
			.channels_min   = 1,
			.channels_max   = 2,
			.rates		= SNDRV_PCM_RATE_8000_192000,
			.formats	= ADV76XX_FORMATS,
		},
		.ops = &adv76xx_dai_ops,
	},
	{
		.name = DAI_ADV7611_NAME,
		.capture = {
			.stream_name    = "HDMI-Capture",
			.channels_min   = 1,
			.channels_max   = 2,
			.rates		= SNDRV_PCM_RATE_8000_192000,
			.formats	= ADV76XX_FORMATS,
		},
		.ops = &adv76xx_dai_ops,
	},
};

static int adv76xx_codec_dev_probe(struct platform_device *pdev)
{
	struct adv76xx_state *state = pdev->dev.platform_data;
	struct regmap *regmap;
	int ret;

	/* Check platform data value */
	if (!state) {
		dev_err(&pdev->dev,
				"Platform probe error, bad platform data\n");
		return -EINVAL;
	}

	/* Check state values */
	regmap = state->regmap[ADV76XX_PAGE_IO];
	if (!regmap) {
		dev_err(&pdev->dev,
			"Platform probe error, no regmap for I2C IO region\n");
		return -EINVAL;
	}

	/* Register the codec on the platform device */
	ret = snd_soc_register_codec(&pdev->dev,
			&adv76xx_audio_codec,
			&adv76xx_dais[state->info->type],
			1);

	if (ret)
		return -ENODEV;

	return ret;
}

static int adv76xx_codec_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static const struct platform_device_id adv76xx_ids[] = {
	{"adv7611-asoc-codec", 0},
	{"adv7604-asoc-codec", 0},
	{},
};
MODULE_DEVICE_TABLE(platform, adv76xx_ids);

static struct platform_driver adv76xx_snd_driver = {
	.driver	= {
		.name   = PLATFORM_DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe 	= adv76xx_codec_dev_probe,
	.remove	= adv76xx_codec_dev_remove,
	.id_table = adv76xx_ids,
};

module_platform_driver(adv76xx_snd_driver);

MODULE_AUTHOR("Pablo Anton <pablo.antond@vodalys-labs.com>");
MODULE_AUTHOR("Jean-Michel Hautbois <jean-michel.hautbois@vodalys.com>");
MODULE_DESCRIPTION("ADV7611 Audio Codec Driver");
MODULE_LICENSE("GPL");
