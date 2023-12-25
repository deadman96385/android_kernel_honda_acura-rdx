/*
 * OMAP4 specific common source file.
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Author:
 *	Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 *
 * This program is free software,you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/platform_device.h>
#include <linux/memblock.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/export.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/irqchip/irq-crossbar.h>
#include <linux/of_address.h>
#include <linux/reboot.h>

#include <asm/hardware/cache-l2x0.h>
#include <asm/mach/map.h>
#include <asm/memblock.h>
#include <asm/smp_twd.h>

#include "omap-wakeupgen.h"
#include "soc.h"
#include "iomap.h"
#include "common.h"
#include "mmc.h"
#include "prminst44xx.h"
#include "prcm_mpu44xx.h"
#include "omap4-sar-layout.h"
#include "omap-secure.h"
#include "sram.h"

#ifdef CONFIG_CACHE_L2X0
static void __iomem *l2cache_base;
#endif

static void __iomem *sar_ram_base;
static void __iomem *gic_dist_base_addr;
static void __iomem *twd_base;

#define IRQ_LOCALTIMER		29

#ifdef CONFIG_OMAP_INTERCONNECT_BARRIER

/* Used to implement memory barrier on DRAM path */
#define OMAP4_DRAM_BARRIER_VA			0xfe600000

void __iomem *dram_sync __read_mostly;
static void __iomem *sram_sync;
static phys_addr_t dram_sync_paddr;
static u32 dram_sync_size;

#ifndef CONFIG_SOC_DRA7XX
/*
 * The OMAP4 bus structure contains asynchrnous bridges which can buffer
 * data writes from the MPU. These asynchronous bridges can be found on
 * paths between the MPU to EMIF, and the MPU to L3 interconnects.
 *
 * We need to be careful about re-ordering which can happen as a result
 * of different accesses being performed via different paths, and
 * therefore different asynchronous bridges.
 */

/*
 * OMAP4 interconnect barrier which is called for each mb() and wmb().
 * This is to ensure that normal paths to DRAM (normal memory, cacheable
 * accesses) are properly synchronised with writes to DMA coherent memory
 * (normal memory, uncacheable) and device writes.
 *
 * The mb() and wmb() barriers only operate only on the MPU->MA->EMIF
 * path, as we need to ensure that data is visible to other system
 * masters prior to writes to those system masters being seen.
 *
 * Note: the SRAM path is not synchronised via mb() and wmb().
 */
static void omap4_mb(void)
{
	if (dram_sync)
		writel_relaxed(0, dram_sync);
}

#else
/*
 * The DRA7 interconnect barrier ensure the posted writes to DRAM via
 * MPU->MPU_MA->EMIF to normal or device meory are observed by
 * L3 interconnect agents before any subsequent MPU access on the
 * L3 interconnect.
 * On memory that is not strongly-ordered, waiting for a dummy read
 * to complete on each EMIF would guarantee that any preceeding poted
 * writes are complete, but our dram_sync is mapped as strongly-ordered,
 * making the writes non-posted, so a dsb() after a such a write can
 * guarantee that all the preceeding accesses are complete for the given
 * EMIF.
 * Note than the second write might not be necessary if the MPU_MA complete
 * the writes on both EMIFS before the stronly-ordered write access completes
 * (this have to be double-checked with TI).
 */
static void dra7_mb(void)
{
	if (dram_sync) {
		writel_relaxed(0, dram_sync);
		writel_relaxed(0, dram_sync + (7 << 7));
		dsb();
	}
}
#endif

/*
 * OMAP4 Errata i688 - asynchronous bridge corruption when entering WFI.
 *
 * If a data is stalled inside asynchronous bridge because of back
 * pressure, it may be accepted multiple times, creating pointer
 * misalignment that will corrupt next transfers on that data path until
 * next reset of the system. No recovery procedure once the issue is hit,
 * the path remains consistently broken.
 *
 * Async bridges can be found on paths between MPU to EMIF and MPU to L3
 * interconnects.
 *
 * This situation can happen only when the idle is initiated by a Master
 * Request Disconnection (which is trigged by software when executing WFI
 * on the CPU).
 *
 * The work-around for this errata needs all the initiators connected
 * through an async bridge to ensure that data path is properly drained
 * before issuing WFI. This condition will be met if one Strongly ordered
 * access is performed to the target right before executing the WFI.
 *
 * In MPU case, L3 T2ASYNC FIFO and DDR T2ASYNC FIFO needs to be drained.
 * IO barrier ensure that there is no synchronisation loss on initiators
 * operating on both interconnect port simultaneously.
 *
 * This is a stronger version of the OMAP4 memory barrier below, and
 * operates on both the MPU->MA->EMIF path but also the MPU->OCP path
 * as well, and is necessary prior to executing a WFI.
 */
void omap_interconnect_sync(void)
{
	if (dram_sync && sram_sync) {
		writel_relaxed(readl_relaxed(dram_sync), dram_sync);
		writel_relaxed(readl_relaxed(sram_sync), sram_sync);
		isb();
	}
}

/* Steal one page physical memory for barrier implementation */
void __init omap_barrier_reserve_memblock(void)
{
	dram_sync_size = ALIGN(PAGE_SIZE, SZ_1M);
	dram_sync_paddr = arm_memblock_steal(dram_sync_size, SZ_1M);
}

void __init omap_barriers_init(void)
{
	struct map_desc dram_io_desc[1];

	dram_io_desc[0].virtual = OMAP4_DRAM_BARRIER_VA;
	dram_io_desc[0].pfn = __phys_to_pfn(dram_sync_paddr);
	dram_io_desc[0].length = dram_sync_size;
	dram_io_desc[0].type = MT_MEMORY_RW_SO;
	iotable_init(dram_io_desc, ARRAY_SIZE(dram_io_desc));
	dram_sync = (void __iomem *) dram_io_desc[0].virtual;
	sram_sync = (void __iomem *) DRA7XX_SRAM_VA;

	pr_info("OMAP4: Map %pa to %p for dram barrier\n",
		&dram_sync_paddr, dram_sync);


	/* Using soc_is_dra7xx() here is not effective */
#ifdef CONFIG_SOC_DRA7XX
	soc_mb = dra7_mb;
#else
	soc_mb = omap4_mb;
#endif
}

#endif

void __init gic_init_irq(void)
{
	void __iomem *omap_irq_base;

	/* Static mapping, never released */
	gic_dist_base_addr = ioremap(OMAP44XX_GIC_DIST_BASE, SZ_4K);
	BUG_ON(!gic_dist_base_addr);

	twd_base = ioremap(OMAP44XX_LOCAL_TWD_BASE, SZ_4K);
	BUG_ON(!twd_base);

	/* Static mapping, never released */
	omap_irq_base = ioremap(OMAP44XX_GIC_CPU_BASE, SZ_512);
	BUG_ON(!omap_irq_base);

	omap_wakeupgen_init();

	gic_init(0, 29, gic_dist_base_addr, omap_irq_base);
}

void gic_dist_disable(void)
{
	if (gic_dist_base_addr)
		__raw_writel(0x0, gic_dist_base_addr + GIC_DIST_CTRL);
}

void gic_dist_enable(void)
{
	if (gic_dist_base_addr)
		__raw_writel(0x1, gic_dist_base_addr + GIC_DIST_CTRL);
}

bool gic_dist_disabled(void)
{
	return !(__raw_readl(gic_dist_base_addr + GIC_DIST_CTRL) & 0x1);
}

void gic_timer_retrigger(void)
{
	u32 twd_int = __raw_readl(twd_base + TWD_TIMER_INTSTAT);
	u32 gic_int = __raw_readl(gic_dist_base_addr + GIC_DIST_PENDING_SET);
	u32 twd_ctrl = __raw_readl(twd_base + TWD_TIMER_CONTROL);

	if (twd_int && !(gic_int & BIT(IRQ_LOCALTIMER))) {
		/*
		 * The local timer interrupt got lost while the distributor was
		 * disabled.  Ack the pending interrupt, and retrigger it.
		 */
		pr_warn("%s: lost localtimer interrupt\n", __func__);
		__raw_writel(1, twd_base + TWD_TIMER_INTSTAT);
		if (!(twd_ctrl & TWD_TIMER_CONTROL_PERIODIC)) {
			__raw_writel(1, twd_base + TWD_TIMER_COUNTER);
			twd_ctrl |= TWD_TIMER_CONTROL_ENABLE;
			__raw_writel(twd_ctrl, twd_base + TWD_TIMER_CONTROL);
		}
	}
}

#ifdef CONFIG_CACHE_L2X0

void __iomem *omap4_get_l2cache_base(void)
{
	return l2cache_base;
}

static void omap4_l2x0_disable(void)
{
	outer_flush_all();
	/* Disable PL310 L2 Cache controller */
	omap_smc1(0x102, 0x0);
}

static void omap4_l2x0_set_debug(unsigned long val)
{
	/* Program PL310 L2 Cache controller debug register */
	omap_smc1(0x100, val);
}

static int __init omap_l2_cache_init(void)
{
	u32 aux_ctrl = 0;

	/*
	 * To avoid code running on other OMAPs in
	 * multi-omap builds
	 */
	if (!cpu_is_omap44xx() && !soc_is_am43xx())
		return -ENODEV;

	/* Static mapping, never released */
	l2cache_base = ioremap(OMAP44XX_L2CACHE_BASE, SZ_4K);
	if (WARN_ON(!l2cache_base))
		return -ENOMEM;

	/*
	 * 16-way associativity, parity disabled
	 * Way size - 32KB (es1.0)
	 * Way size - 64KB (es2.0 +)
	 * Way size - 16KB (am43xx)
	 */
	aux_ctrl = ((1 << L2X0_AUX_CTRL_ASSOCIATIVITY_SHIFT) |
			(0x1 << 25) |
			(0x1 << L2X0_AUX_CTRL_NS_LOCKDOWN_SHIFT) |
			(0x1 << L2X0_AUX_CTRL_NS_INT_CTRL_SHIFT));
	if (soc_is_am43xx()) {
		aux_ctrl |= ((0x1 << L2X0_AUX_CTRL_WAY_SIZE_SHIFT) |
			     (1 << L2X0_AUX_CTRL_DATA_PREFETCH_SHIFT) |
			     (1 << L2X0_AUX_CTRL_INSTR_PREFETCH_SHIFT) |
			     (1 << L2X0_AUX_CTRL_EARLY_BRESP_SHIFT));
	} else if (omap_rev() == OMAP4430_REV_ES1_0) {
		aux_ctrl |= 0x2 << L2X0_AUX_CTRL_WAY_SIZE_SHIFT;
	} else {
		aux_ctrl |= ((0x3 << L2X0_AUX_CTRL_WAY_SIZE_SHIFT) |
			(1 << L2X0_AUX_CTRL_SHARE_OVERRIDE_SHIFT) |
			(1 << L2X0_AUX_CTRL_DATA_PREFETCH_SHIFT) |
			(1 << L2X0_AUX_CTRL_INSTR_PREFETCH_SHIFT) |
			(1 << L2X0_AUX_CTRL_EARLY_BRESP_SHIFT));
	}
	if (soc_is_am43xx() || omap_rev() != OMAP4430_REV_ES1_0)
		omap_smc1(0x109, aux_ctrl);

	/* Enable PL310 L2 Cache controller */
	omap_smc1(0x102, 0x1);

	if (of_have_populated_dt())
		l2x0_of_init(aux_ctrl, L2X0_AUX_CTRL_MASK);
	else
		l2x0_init(l2cache_base, aux_ctrl, L2X0_AUX_CTRL_MASK);

	/*
	 * Override default outer_cache.disable with a OMAP4
	 * specific one
	*/
	outer_cache.disable = omap4_l2x0_disable;
	outer_cache.set_debug = omap4_l2x0_set_debug;

	return 0;
}
omap_early_initcall(omap_l2_cache_init);
#endif

void __iomem *omap4_get_sar_ram_base(void)
{
	return sar_ram_base;
}

/*
 * SAR RAM used to save and restore the HW
 * context in low power modes
 */
static int __init omap4_sar_ram_init(void)
{
	unsigned long sar_base;

	/*
	 * To avoid code running on other OMAPs in
	 * multi-omap builds
	 */
	if (cpu_is_omap44xx())
		sar_base = OMAP44XX_SAR_RAM_BASE;
	else if (soc_is_omap54xx())
		sar_base = OMAP54XX_SAR_RAM_BASE;
	else if (soc_is_dra7xx())
		sar_base = DRA7XX_SAR_RAM_BASE;
	else
		return -ENOMEM;

	/* Static mapping, never released */
	sar_ram_base = ioremap(sar_base, SZ_16K);
	if (WARN_ON(!sar_ram_base))
		return -ENOMEM;

	return 0;
}
omap_early_initcall(omap4_sar_ram_init);

void __init omap_gic_of_init(void)
{
	struct device_node *np;

	/* Extract GIC distributor and TWD bases for OMAP4460 ROM Errata WA */
	if (!cpu_is_omap446x())
		goto skip_errata_init;

	np = of_find_compatible_node(NULL, NULL, "arm,cortex-a9-gic");
	gic_dist_base_addr = of_iomap(np, 0);
	WARN_ON(!gic_dist_base_addr);

	np = of_find_compatible_node(NULL, NULL, "arm,cortex-a9-twd-timer");
	twd_base = of_iomap(np, 0);
	WARN_ON(!twd_base);

skip_errata_init:
	omap_wakeupgen_init();
#ifdef CONFIG_IRQ_CROSSBAR
	irqcrossbar_init();
#endif
	irqchip_init();
}
