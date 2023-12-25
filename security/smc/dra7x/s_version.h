/*
 * Copyright (c) 2012 Trusted Logic S.A.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __S_VERSION_H__
#define __S_VERSION_H__

/*
 * Usage: define S_VERSION_BUILD on the compiler's command line.
 *
 * Then set:
 * - S_VERSION_OS
 * - S_VERSION_PLATFORM
 * - S_VERSION_MAIN
 * - S_VERSION_ENG is optional
 * - S_VERSION_PATCH is optional
 * - S_VERSION_BUILD = 0 if S_VERSION_BUILD not defined or empty
 */

#define S_VERSION_OS "A"          /* "A" for all Android */
#define S_VERSION_PLATFORM "O"    /* "O" for generic OMAP */

/*
 * This version number must be updated for each new release
 */
#define S_VERSION_MAIN  "01.03"

/*
* If this is a patch or engineering version use the following
* defines to set the version number. Else set these values to 0.
*/
#define S_VERSION_ENG 0
#define S_VERSION_PATCH 0

#define __STRINGIFY(X) #X
#define __STRINGIFY2(X) __STRINGIFY(X)

#if S_VERSION_ENG != 0
#define _S_VERSION_ENG "e" __STRINGIFY2(S_VERSION_ENG)
#else
#define _S_VERSION_ENG ""
#endif

#if S_VERSION_PATCH != 0
#define _S_VERSION_PATCH "p" __STRINGIFY2(S_VERSION_PATCH)
#else
#define _S_VERSION_PATCH ""
#endif

#if !defined(NDEBUG) || defined(_DEBUG)
#define S_VERSION_VARIANT "D   "
#else
#define S_VERSION_VARIANT "    "
#endif

#define S_VERSION_STRING \
	"SMC" \
	S_VERSION_OS \
	S_VERSION_PLATFORM \
	S_VERSION_MAIN \
	_S_VERSION_ENG \
	_S_VERSION_PATCH \
	S_VERSION_VARIANT

#endif /* __S_VERSION_H__ */
