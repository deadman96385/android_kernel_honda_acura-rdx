/*********************************************************
*
*  Copyright (c) 2014-2016 EAR Honda R&D Americas, Inc.
*  All rights reserved. You may not copy, distribute, publicly display,
*  create derivative works from or otherwise use or modify this
*  software without first obtaining a license from Honda R&D Americas, Inc.
*
*********************************************************/

#ifndef __HRALVDS_PANEL_TJBA_H
#define __HRALVDS_PANEL_TJBA_H

#include <hralvds-proto.h>

#define HRALVDS_TJBA_WIDTH	1920
#define HRALVDS_TJBA_MAX_X	(HRALVDS_TJBA_WIDTH - 1)
#define HRALVDS_TJBA_HEIGHT	720
#define HRALVDS_TJBA_MAX_Y	(HRALVDS_TJBA_HEIGHT - 1)

struct hralvds_tjba_data {
	int dummy;
};

#endif
