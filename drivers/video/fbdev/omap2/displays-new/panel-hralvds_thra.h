/*********************************************************
*
*  Copyright (c) 2014-2016 EAR Honda R&D Americas, Inc.
*  All rights reserved. You may not copy, distribute, publicly display,
*  create derivative works from or otherwise use or modify this
*  software without first obtaining a license from Honda R&D Americas, Inc.
*
*********************************************************/

#ifndef __HRALVDS_PANEL_THRA_H
#define __HRALVDS_PANEL_THRA_H

#include <hralvds-proto.h>

#define HRALVDS_THRA_WIDTH	1280
#define HRALVDS_THRA_MAX_X	(HRALVDS_THRA_WIDTH - 1)
#define HRALVDS_THRA_HEIGHT	720
#define HRALVDS_THRA_MAX_Y	(HRALVDS_THRA_HEIGHT - 1)

/*
 * Four pixels are removed from the bottom margin so that the display's
 * aspect ratio are in compliance with "7.1.1.2. Screen Aspect Ratio"
 * in the CDD (aspect ratio must be between 1.33 and 1.86).  The effective
 * screen dimension is now 1280x690.  As the panel tries to center the
 * display area, shift the vertical back porch down 4 pixels to compensate.
 */
#define HRALVDS_THRA_DEAD_ZONE_BOTTOM_FUDGE_FACTOR (4)

#define HRALVDS_THRA_DEAD_ZONE_LEFT     (20)
#define HRALVDS_THRA_DEAD_ZONE_RIGHT    (20)
#define HRALVDS_THRA_DEAD_ZONE_TOP      (17)
#define HRALVDS_THRA_DEAD_ZONE_BOTTOM \
	(17 - HRALVDS_THRA_DEAD_ZONE_BOTTOM_FUDGE_FACTOR)

/*
 * Display timings
 *
 * NOTE: The display area is shifted 20 pixels to the right and 17 pixels
 * down to apply the left and top margins of the "dead zone" per the
 * THRA HMI specifications.  The dead zone is meant to compensate for
 * viewing angle caused by the display's bezel and recessed LCD screen,
 * and manufacturing variances.
 *
 * However the panel is not able to render a 1240 display width properly.
 * So the right margin is compensated for in software using the Android
 * Window Manager's Overscan feature.  The margins are defined in
 * device/ti/gen1Honda/overlay/frameworks/base/core/res/res/values/dimens.xml
 *
 * Four pixels are removed from the bottom margin to make the display's
 * aspect ratio in compliance with "7.1.1.2. Screen Aspect Ratio"
 * in the CDD (aspect ratio must be between 1.33 and 1.86).  Those four pixels
 * will be handled by Android framework's overscan bottom margin.  The
 * effective screen dimension is now 1280x690, for an aspect ratio of 1.8550.
 * As the panel tries to center the display area, shift the vertical back
 * porch down 4 pixels to compensate.
 */

/* Horizontal Back Porch - the left margin */
#define HRALVDS_THRA_TIMING_HBP		(90 + HRALVDS_THRA_DEAD_ZONE_LEFT)
/* Horizontal Front Porch - the right margin */
#define HRALVDS_THRA_TIMING_HFP		(90 - HRALVDS_THRA_DEAD_ZONE_RIGHT)
/* Horizontal Synchronization Pulse Width */
#define HRALVDS_THRA_TIMING_HSW		(40)

/* Vertical Front Porch - the bottom margin */
#define HRALVDS_THRA_TIMING_VFP		(49 + HRALVDS_THRA_DEAD_ZONE_BOTTOM + \
	(HRALVDS_THRA_DEAD_ZONE_BOTTOM_FUDGE_FACTOR/2))
/* Vertical Back Porch - the top margin */
#define HRALVDS_THRA_TIMING_VBP		(51 + HRALVDS_THRA_DEAD_ZONE_TOP + \
	(HRALVDS_THRA_DEAD_ZONE_BOTTOM_FUDGE_FACTOR/2))
/* Vertical Synchronization Pulse Width */
#define HRALVDS_THRA_TIMING_VSW		(5)

#define HRALVDS_THRA_REFRESH_RATE	(60)


struct hralvds_thra_data {
	/* static array to track the current slot state */
	unsigned slot_state[HRALVDS_NUM_MT_SLOTS];
};

#endif
