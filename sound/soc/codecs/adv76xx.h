/*
 * Copyright (C) 2014 Vodalys-Labs
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

#ifndef ADV76XX_H_
#define ADV76XX_H_

#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define DAI_ADV7611_NAME	"adv7611-dai"
#define DAI_ADV7604_NAME	"adv7604-dai"
#define PLATFORM_DRIVER_NAME	"adv76xx-asoc-codec"

/* HDMI 0x39[3:0] - CS_DATA[27:24] 0 for reserved values*/
static const int cs_data_fs[] = {
	44100,
	0,
	48000,
	0,
	0,
	0,
	0,
	0,
	32000,
	88200,
	0,
	768000,
	96000,
	176000,
	192000,
	0,
};

#endif /* ADV7611_CODEC_H_ */
