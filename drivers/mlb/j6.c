/*
 * Copyright (C) 2015 Cetitec, GmbH. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/**
 * This is the configuration code for TI Jacinto6 MLB support,
 * used both by the device tree and hard-coded configuration.
 */

#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/ioport.h>

#include "j6.h"

/* Table 18-25 */
#define J6_CONTROL_CORE_BASE_ADDR 0x4A002000
#define J6_CONTROL_CORE_SIZE      0x2000 /* 8kiB */

/* Tables 18-421, 18-423, 18-525, page 5468-5470 */
#define J6_OFFSET_CTRL_CORE_MLB_SIG_IO_CTRL  0x06C8
#define J6_OFFSET_CTRL_CORE_MLB_DAT_IO_CTRL  0x06CC
#define J6_OFFSET_CTRL_CORE_MLB_CLK_BG_CTRL  0x06D0

int jacinto6_6pinMLB(void)
{
	/* Map physical address range for pins into virtual address space */
	void __iomem *control_core = ioremap(J6_CONTROL_CORE_BASE_ADDR, J6_CONTROL_CORE_SIZE);

	if (!control_core)
		return -ENOMEM;

	/* 6-pin mode */

	/* MLB_MLBC0:MLBPEN is set by the IOCTL setFps. No need to set it here. */

	/* These settings were recommended by TI on 2014-04-10. */
	writel(0x00000008, control_core + J6_OFFSET_CTRL_CORE_MLB_SIG_IO_CTRL);
	writel(0x00000008, control_core + J6_OFFSET_CTRL_CORE_MLB_DAT_IO_CTRL);
	writel(0x00000080, control_core + J6_OFFSET_CTRL_CORE_MLB_CLK_BG_CTRL);

	iounmap(control_core);
	return 0;
}

/* Table 3-1059 */
#define J6_CM_L3INIT_BASE_ADDR   0x4A009300
#define J6_MLB_SS_CLKCTRL_OFFSET 0x0058

/* Table 3-1088 */
#define J6_CM_L4PER_CLKCTRL_OFFSET 0x400 /* absolute: 0x4A009700: calculated offset from J6_CM_L3INIT_BASE_ADDR */

/* Table 3-1214 */
#define J6_CM_L4PER2_CLKCTRL_OFFSET 0x5FC /* absolute: 0x4A0098FC: calculated offset from J6_CM_L3INIT_BASE_ADDR */

/* This function is based on TI sample code and is only executed on the EVM
 * platform */
int jacinto6_evm_enableMLBmodule(void)
{
	int ret = -ENOMEM;
	u32 l3Init_clkStatusCtrl;
	u32 l4Per2_clkStatusCtrl;
	u32 l4Per3_clkStatusCtrl;
	u32 l3Init_mlb_ss_ClkCtrl;
	u8 __iomem *baseAddr_l3init = ioremap(J6_CM_L3INIT_BASE_ADDR, 0x2000/* ? */);

	if (!baseAddr_l3init)
		goto fail;

	/* CM_L3INIT_CLKSTCTRL (0x4A009300) */
	l3Init_clkStatusCtrl = readl(baseAddr_l3init + 0x0);
	l3Init_clkStatusCtrl &= 0xFFFFFFC;
	l3Init_clkStatusCtrl |= 0x2;
	writel(l3Init_clkStatusCtrl, baseAddr_l3init + 0x0);

	l3Init_clkStatusCtrl = readl(baseAddr_l3init + 0x0);
	pr_debug("L3INIT SW wakeup done (CLKSTCTRL: %08X)\n", l3Init_clkStatusCtrl);

	/* CM_L4PER2_CLKSTCTRL (0x4A0098FC) */
	l4Per2_clkStatusCtrl = readl(baseAddr_l3init + J6_CM_L4PER2_CLKCTRL_OFFSET);
	l4Per2_clkStatusCtrl &= 0xFFFFFFC;
	l4Per2_clkStatusCtrl |= 0x2;
	writel(l4Per2_clkStatusCtrl, baseAddr_l3init + J6_CM_L4PER2_CLKCTRL_OFFSET);

	l4Per2_clkStatusCtrl = readl(baseAddr_l3init + J6_CM_L4PER2_CLKCTRL_OFFSET);
	pr_debug("L4PER2 SW wakeup done (CLKSTCTRL: %08X)\n", l4Per2_clkStatusCtrl);

	/* CM_L4PER3_CLKSTCTRL (0x4A009910, offset from 0x4A009300 is 0x610) */
	l4Per3_clkStatusCtrl = readl(baseAddr_l3init + 0x610);
	l4Per3_clkStatusCtrl &= 0xFFFFFFC;
	l4Per3_clkStatusCtrl |= 0x2;
	writel(l4Per3_clkStatusCtrl, baseAddr_l3init + 0x610);

	l4Per3_clkStatusCtrl = readl(baseAddr_l3init + 0x610);
	pr_debug("L4PER3 SW wakeup done (CLKSTCTRL: %08X)\n", l4Per3_clkStatusCtrl);

	/* CM_L3INIT_MLB_SS_CLKCTRL (0x4A009358) */
	l3Init_mlb_ss_ClkCtrl = readl(baseAddr_l3init + J6_MLB_SS_CLKCTRL_OFFSET);
	l3Init_mlb_ss_ClkCtrl &= 0xFFFFFFC;
	l3Init_mlb_ss_ClkCtrl |= 0x2;
	writel(l3Init_mlb_ss_ClkCtrl, baseAddr_l3init + J6_MLB_SS_CLKCTRL_OFFSET);

	l3Init_mlb_ss_ClkCtrl = readl(baseAddr_l3init + J6_MLB_SS_CLKCTRL_OFFSET);
	pr_debug("MLB module enable done (MLB_SS_CLKCTRL: %08X)\n", l3Init_mlb_ss_ClkCtrl);

	/* print status of MLB SS register again (why?!) */
	l3Init_mlb_ss_ClkCtrl = readl(baseAddr_l3init + J6_MLB_SS_CLKCTRL_OFFSET);
	pr_debug("MLB_SS_CLKCTRL %08X (2nd)\n", l3Init_mlb_ss_ClkCtrl);

	switch (l3Init_mlb_ss_ClkCtrl & 0x000F0002) {
		case 0x02:
			pr_debug("Jacinto6 MLB module is functional and out of standby.\n");
			ret = 0; /* success */
			break;
		case 0x40002: /* error */
			pr_debug("Jacinto6 MLB module is functional, but in standby.\n");
			ret = -ESHUTDOWN;
			break;
		case 0x70002: /* error */
			pr_debug("Jacinto6 MLB module is DISABLED && in standby.\n");
			ret = -ESHUTDOWN;
			break;
	}
	iounmap(baseAddr_l3init);
fail:
	return ret;
}

int jacinto6_enableMLBmodule(void)
{
#ifdef CONFIG_MACH_DENSOJ6REF
	return 0;
#else
	return jacinto6_evm_enableMLBmodule();
#endif
}

