/*
 * mlb150.h
 *
 * Copyright 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef _MLB150_H
#define _MLB150_H

/**
 * enum mlb_sync_ch_startup_mode - select the operation mode for sync channel
 *
 * The even values must be RX, the odd - TX.
 */
enum mlb_sync_ch_startup_mode {
	MLB_SYNC_MONO_RX   = 0, /* 16 bit mono */
	MLB_SYNC_MONO_TX   = 1,
	MLB_SYNC_STEREO_RX = 2, /* 16 bit stereo */
	MLB_SYNC_STEREO_TX = 3,
	MLB_SYNC_51_RX     = 4, /* 16 bit 5.1 */
	MLB_SYNC_51_TX     = 5,
	MLB_SYNC_51HQ_RX   = 6, /* 24 bit 5.1 */
	MLB_SYNC_51HQ_TX   = 7,
	MLB_SYNC_STEREOHQ_RX   = 8, /* 24 bit stereo */
	MLB_SYNC_STEREOHQ_TX   = 9,
};

/* define IOCTL command */
#define MLB_DBG_RUNTIME		_IO('S', 0x09)
#define MLB_SET_FPS		_IOW('S', 0x10, unsigned int)
#define MLB_GET_VER		_IOR('S', 0x11, unsigned long)
#define MLB_SET_DEVADDR		_IOR('S', 0x12, unsigned char)
/*!
 * set channel address for each logical channel
 * the MSB 16bits is for tx channel, the left LSB is for rx channel
 */
#define MLB_CHAN_SETADDR	_IOW('S', 0x13, unsigned int)
#define MLB_CHAN_STARTUP	_IO('S', 0x14)
#define MLB_CHAN_SHUTDOWN	_IO('S', 0x15)
#define MLB_CHAN_GETEVENT	_IOR('S', 0x16, unsigned long)

#define MLB_SET_ISOC_BLKSIZE_188 _IO('S', 0x17) /* deprecated! Use the sysFs method! */
#define MLB_SET_ISOC_BLKSIZE_196 _IO('S', 0x18) /* deprecated! Use the sysFs method! */
#define MLB_SET_SYNC_QUAD	_IOW('S', 0x19, unsigned int)

#define MLB_SYNC_CHAN_STARTUP	_IOW('S', 0x20, enum mlb_sync_ch_startup_mode)
#define MLB_GET_LOCK            _IO('S', 0x21)

#define MLB_GET_ISOC_BUFSIZE	_IOR('S', 0x22, unsigned int)

#define MLB_PAUSE_RX            _IO('S', 0x23)
#define MLB_RESUME_RX           _IO('S', 0x24)

#define MLB_USE_MFE /* enable multi-frame sub-buffering on synchronous channels */
#define MLB_USE_FCE /* enable flow control on isochronous channels */

#endif /* _MLB150_H */
