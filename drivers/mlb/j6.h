/**
 * These are the common definitions for the configuration code for TI Jacinto6
 * MLB support, used both by the device tree and hard-coded configuration.
 */
#ifndef _MLB150_J6_H_
#define _MLB150_J6_H_

/*
 * On TI Jacinto6, the MLB DIM registers are offset by 0x400 from the
 * base address.
 */
#define MLB_DIM_REGISTER_OFFSET 0x400

extern int jacinto6_enableMLBmodule(void);
extern int jacinto6_6pinMLB(void);

#endif
