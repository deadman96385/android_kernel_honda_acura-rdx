/*
 * Remote processor machine-specific module for OMAP4+ SoCs
 *
 * Copyright (C) 2011-2016 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)    "%s: " fmt, __func__

#include <linux/kernel.h>

#include <plat/dmtimer.h>

#include "omap_device.h"
#include "control.h"
#include "remoteproc.h"
#include "soc.h"

#define DSP1_EDMA_TPCC			0x40D10000
#define DSP2_EDMA_TPCC			0x41510000
#define DSP1_EDMA_TPTC0			0x40D05000
#define DSP2_EDMA_TPTC0			0x41505000
#define EDMA_TPCC_CCSTAT_OFFSET		0x640
#define EDMA_TPCC_EECR_OFFSET		0x1028
#define EDMA_TPCC_EECRH_OFFSET		0x102C
#define EDMA_TPCC_QEECR_OFFSET		0x1088
#define EDMA_DSP_TPTC1_OFFSET		0x1000
#define EDMA_TPTC_TCSTAT0_OFFSET	0x100
#define EDMA_TPTC_TCSTAT1_OFFSET	(EDMA_DSP_TPTC1_OFFSET + \
					EDMA_TPTC_TCSTAT0_OFFSET)

#define CTRL_CORE_DMA_DSP1_DREQ		0x4A002CF8
#define CTRL_CORE_DMA_DSP2_DREQ		0x4A002D20

void dra7_ctrl_write_dsp1_boot_addr(u32 bootaddr)
{
	dra7_ctrl_write_dsp_boot_addr(bootaddr, 0);
}

void dra7_ctrl_write_dsp2_boot_addr(u32 bootaddr)
{
	dra7_ctrl_write_dsp_boot_addr(bootaddr, 1);
}

static void dra7_wait_dsp_edma_compl(u32 inst)
{
	u32 dsp_edma_tpcc_base, dsp_edma_tptc_base;
	int timeout;
	void __iomem *tpcc_base, *tptc_base;

	if (!soc_is_dra7xx())
		return;

	dsp_edma_tpcc_base = inst ? DSP2_EDMA_TPCC : DSP1_EDMA_TPCC;
	dsp_edma_tptc_base = inst ? DSP2_EDMA_TPTC0 : DSP1_EDMA_TPTC0;

	tpcc_base = ioremap(dsp_edma_tpcc_base, SZ_16K);
	if (!tpcc_base) {
		pr_err("DSP EDMA TPCC ioremap failed\n");
		goto map_err1;
	}

	tptc_base = ioremap(dsp_edma_tptc_base, SZ_8K);
	if (!tptc_base) {
		pr_err("DSP EDMA TPTC ioremap failed\n");
		goto map_err2;
	}

	/* Disable all future EDMA and QDMA events to DSPx EDMA TPCC */
	writel_relaxed(0xFFFFFFFF, tpcc_base + EDMA_TPCC_EECR_OFFSET);
	writel_relaxed(0xFFFFFFFF, tpcc_base + EDMA_TPCC_EECRH_OFFSET);
	writel_relaxed(0xFFFFFFFF, tpcc_base + EDMA_TPCC_QEECR_OFFSET);

	/*
	 * Poll CCSTAT to ensure all actively serviced or queued events have
	 * been completed.
	 *
	 * The timeout is based on the duration which the EDMA CC queue will
	 * drain based on the slowest typical application.  This is chosen to
	 * be 1.0625ms, which assumes a full event queue with transfers for
	 * an 8kHz audio stream, plus one extra transfer for safe measure.
	 */
	timeout = 1063;
	pr_warn("waiting for DSP%d EDMA traffic on TPCC to complete\n",
		inst+1);
	while (((readl_relaxed(tpcc_base + EDMA_TPCC_CCSTAT_OFFSET))
		!= 0x0) && --timeout)
		udelay(1);
	if (timeout == 0)
		pr_warn("DSP%d EDMA transaction may be ongoing during shutdown! TPCC is active!\n",
			inst + 1);

	/*
	 * Check that PROGBUSY SRCACTV WSACTV, and DSTACTV bits of TCSTAT
	 * registers for DSP TPTC0 and TPTC1 are cleared prior to shutdown.
	 *
	 * The timeout is based on the duration of the EDMA transfer expected
	 * by the slowest typical application, which is chosen as 125us.  This
	 * would be the transfer request rate of an 8kHz audio stream, with one
	 * extra transfer for safe measure.
	 */
	timeout = 125;
	pr_warn("waiting for DSP%d EDMA traffic on TPTC0 to complete\n",
		inst+1);
	while (((readl_relaxed(tptc_base + EDMA_TPTC_TCSTAT0_OFFSET) & 0x77)
		!= 0x0) && --timeout)
		udelay(1);
	if (timeout == 0)
		pr_warn("DSP%d EDMA transaction may be ongoing during shutdown! TPTC0 is active!\n",
			inst + 1);

	timeout = 125;
	pr_warn("waiting for DSP%d EDMA traffic on TPTC1 to complete\n",
		inst+1);
	while (((readl_relaxed(tptc_base + EDMA_TPTC_TCSTAT1_OFFSET) & 0x77)
		!= 0x0) && --timeout)
		udelay(1);
	if (timeout == 0)
		pr_warn("DSP%d EDMA transaction may be ongoing during shutdown! TPTC1 is active!\n",
			inst + 1);
	iounmap(tptc_base);
map_err2:
	iounmap(tpcc_base);
map_err1:
	return;
}

static void dra7_clear_dsp_edma_xbar(u32 inst)
{
	u32 dsp_dreq_base;
	void __iomem *iomem_base;
	u8 offset;

	if (!soc_is_dra7xx())
		return;

	dsp_dreq_base = inst ? CTRL_CORE_DMA_DSP2_DREQ :
			CTRL_CORE_DMA_DSP1_DREQ;

	iomem_base = ioremap(dsp_dreq_base, SZ_64);
	if (!iomem_base) {
		pr_err("DSP EDMA ioremap failed\n");
		return;
	}

	pr_warn("Clearing all EDMA XBAR routings to DSP%d\n", inst+1);

	/*
	 * Clear all connections to DSPx EDMA crossbar, from
	 * CTRL_CORE_DMA_DSPx_DREQ_0_1 to CTRL_CORE_DMA_DSPx_DREQ_18_19.
	 */
	for (offset = 0x0; offset <= 0x24; offset += 0x4)
		writel_relaxed(0x0, iomem_base + offset);

	iounmap(iomem_base);
}

void dra7_dsp1_pre_shutdown(void)
{
	dra7_clear_dsp_edma_xbar(0);
	dra7_wait_dsp_edma_compl(0);
}

void dra7_dsp2_pre_shutdown(void)
{
	dra7_clear_dsp_edma_xbar(1);
	dra7_wait_dsp_edma_compl(1);
}

/**
 * omap_rproc_device_enable - enable the remoteproc device
 * @pdev: the rproc platform device
 *
 * This function performs the necessary low-level functions to enable
 * a remoteproc device to start executing. This typically includes
 * releasing the reset lines, and enabling the clocks for the device.
 * We do not usually expect this function to fail.
 *
 * Return: 0 on success, or the return code from the failed function
 */
int omap_rproc_device_enable(struct platform_device *pdev)
{
	int ret = -EINVAL;

	/*
	 * This reset management follows a device name check to differentiate
	 * DSP and IPU processor subsystems. This check is weak and is ok for
	 * now because of the dependencies against the pdata-quirks, where
	 * the devices are given specific device names that satisfy the
	 * criteria for the check. It can easily be replaced with a stronger
	 * check like device node compatibility check, if needed.
	 */
	if (strstr(dev_name(&pdev->dev), "dsp")) {
		ret = omap_device_deassert_hardreset(pdev, "dsp");
		if (ret)
			goto out;
	} else if (strstr(dev_name(&pdev->dev), "ipu")) {
		ret = omap_device_deassert_hardreset(pdev, "cpu0");
		if (ret)
			goto out;

		ret = omap_device_deassert_hardreset(pdev, "cpu1");
		if (ret)
			goto out;
	} else {
		pr_err("unsupported remoteproc\n");
		goto out;
	}

	ret = omap_device_enable(pdev);

out:
	if (ret)
		pr_err("failed for proc %s\n", dev_name(&pdev->dev));
	return ret;
}

/**
 * omap_rproc_device_shutdown - shutdown the remoteproc device
 * @pdev: the rproc platform device
 *
 * This function performs the necessary low-level functions to shutdown
 * a remoteproc device. This typically includes disabling the clocks
 * for the device and asserting the associated reset lines. We do not
 * usually expect this function to fail.
 *
 * Return: 0 on success, or the return code from the failed function
 */
int omap_rproc_device_shutdown(struct platform_device *pdev)
{
	int ret = -EINVAL;

	ret = omap_device_idle(pdev);
	if (ret)
		goto out;

	/*
	 * This reset management follows a device name check to differentiate
	 * DSP and IPU processor subsystems. This check is weak and is ok for
	 * now because of the dependencies against the pdata-quirks, where
	 * the devices are given specific device names that satisfy the
	 * criteria for the check. It can easily be replaced with a stronger
	 * check like device node compatibility check, if needed.
	 */
	if (strstr(dev_name(&pdev->dev), "dsp")) {
		ret = omap_device_assert_hardreset(pdev, "dsp");
	} else if (strstr(dev_name(&pdev->dev), "ipu")) {
		ret = omap_device_assert_hardreset(pdev, "cpu1");
		if (ret)
			goto out;

		ret = omap_device_assert_hardreset(pdev, "cpu0");
		if (ret)
			goto out;
	} else {
		pr_err("unsupported remoteproc\n");
	}

out:
	if (ret)
		pr_err("failed for proc %s\n", dev_name(&pdev->dev));
	return ret;
}

/**
 * omap_rproc_request_timer - request a timer for a remoteproc
 * @np - device node pointer to the desired timer
 *
 * This function is used primarily to request a timer associated with
 * a remoteproc. The remoteproc driver core needs to store the returned
 * handle to invoke other timer specific ops (like starting a timer either
 * during device initialization or during a resume operation, or for
 * stopping/freeing a timer).
 *
 * Returns an OMAP timer handle on success, otherwise an equivalent ERR_PTR
 */
struct omap_dm_timer *omap_rproc_request_timer(struct device_node *np)
{
	struct omap_dm_timer *timer;
	int ret = 0;

	timer = omap_dm_timer_request_by_node(np);
	if (!timer) {
		pr_err("request for timer node %p failed\n", np);
		return ERR_PTR(-EBUSY);
	}

	ret = omap_dm_timer_set_source(timer, OMAP_TIMER_SRC_SYS_CLK);
	if (ret) {
		pr_err("error setting OMAP_TIMER_SRC_SYS_CLK as source for timer node %p\n",
		       np);
		omap_dm_timer_free(timer);
		return ERR_PTR(ret);
	}

	/* clean counter, remoteproc code will set the value */
	omap_dm_timer_set_load(timer, 0, 0);

	return timer;
}

/**
 * omap_rproc_start_timer - start a timer for a remoteproc
 * @timer - handle to a OMAP timer
 *
 * This function is used to start a timer associated with a remoteproc,
 * obtained using the request_timer ops. The function needs to be invoked
 * by the remoteproc driver core to start the timer (during device
 * initialization) or to just resume the timer.
 *
 * Returns 0 on success, otherwise a failure as returned by DMTimer API
 */
int omap_rproc_start_timer(struct omap_dm_timer *timer)
{
	return omap_dm_timer_start(timer);
}

/**
 * omap_rproc_stop_timer - stop a timer for a remoteproc
 * @timer - handle to a struct omap_dm_timer
 *
 * This function is used to disable a timer associated with a remoteproc,
 * and needs to be called either during a device shutdown or suspend
 * operation. The separate function allows the remoteproc driver core to
 * just stop a timer without having to release the timer during a suspend
 * operation.
 *
 * Returns 0 on success, otherwise a failure as returned by DMTimer API
 */
int omap_rproc_stop_timer(struct omap_dm_timer *timer)
{
	return omap_dm_timer_stop(timer);
}

/**
 * omap_rproc_release_timer - release a timer for a remoteproc
 * @timer - handle to a struct omap_dm_timer
 *
 * This function is used primarily to release a timer associated with
 * a remoteproc. The dmtimer will be available for other clients to use
 * once released.
 *
 * Returns 0 on success, otherwise a failure as returned by DMTimer API
 */
int omap_rproc_release_timer(struct omap_dm_timer *timer)
{
	return omap_dm_timer_free(timer);
}

/**
 * omap_rproc_get_timer_irq - get the irq for a timer
 * @timer - handle to a OMAP timer
 *
 * This function is used to get the irq associated with a timer, obtained
 * using the request_timer ops. The function is called by the OMAP remoteproc
 * driver to register a interrupt handler to handle watchdog events on the
 * remote processor.
 *
 * Returns the irq id on success, otherwise a failure as returned by DMTimer API
 */
int omap_rproc_get_timer_irq(struct omap_dm_timer *timer)
{
	return omap_dm_timer_get_irq(timer);
}

/**
 * omap_rproc_ack_timer_irq - acknowledge a timer irq
 * @timer - handle to a OMAP timer
 *
 * This function is used to clear the irq associated with a timer, obtained
 * using the request_timer ops. The function is called by the OMAP remoteproc
 * driver upon a watchdog event on the remote processor to clear the interrupt
 * status of the watchdog timer.
 *
 * Returns the irq id on success, otherwise a failure as returned by DMTimer API
 */
void omap_rproc_ack_timer_irq(struct omap_dm_timer *timer)
{
	omap_dm_timer_write_status(timer, OMAP_TIMER_INT_OVERFLOW);
}
