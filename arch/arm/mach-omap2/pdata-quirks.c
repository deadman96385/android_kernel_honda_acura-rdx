/*
 * Legacy platform_data quirks
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/*********************************************************
 *
 *  This file modified by Honda R&D Americas, Inc. between
 *  February 20, 2015 and March 6, 2015
 *
 *  All modifications made by Honda R&D Americas, Inc.
 *  are Copyright (c) 2015 Honda R&D Americas, Inc.
 *
 *  Honda R&D Americas, Inc. hereby licenses those modifications
 *  under the terms set forth in the file HONDA-NOTICE
 *  located in the root of the directory /vendor/honda.
 *
 *********************************************************/

#include <linux/clk.h>
#include <linux/davinci_emac.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of_platform.h>
#include <linux/wl12xx.h>

#include <linux/platform_data/pinctrl-single.h>
#include <linux/platform_data/iommu-omap.h>
#include <linux/platform_data/wkup_m3.h>
#include <linux/platform_data/sgx-omap.h>
#include <linux/platform_data/pci-dra7xx.h>
#include <linux/platform_data/remoteproc-omap.h>
#include <linux/platform_data/remoteproc-pruss.h>

#include "am35xx.h"
#include "common.h"
#include "common-board-devices.h"
#include "dss-common.h"
#include "control.h"
#include "omap-secure.h"
#include "soc.h"
#include "omap_device.h"
#include "remoteproc.h"
//#include "clock7xx.h"


#define PMIC_I2C_ADDR          0x58
#define PMIC_BACKUP0           0x18
#define PMIC_BACKUP1           0x19
#define PMIC_BACKUP2           0x1a
#define PMIC_BACKUP3           0x1b
#define PMIC_BACKUP4           0x1c

/* Reset reasons to write the PMIC BACKUP0 register */
#define DRA7XX_PMIC_WM_RST_REASON_THERMAL_IVA    0xF
#define DRA7XX_PMIC_WM_RST_REASON_THERMAL_DSPEV  0xE
#define DRA7XX_PMIC_WM_RST_REASON_THERMAL_CORE   0xD
#define DRA7XX_PMIC_WM_RST_REASON_THERMAL_MM     0xC
#define DRA7XX_PMIC_WM_RST_REASON_THERMAL_MPU    0xB
#define DRA7XX_PMIC_WM_RST_REASON_CLEAR          0x0 /* no reason or unintended reset */
#define DRA7XX_PMIC_WM_RST_REASON_NORMAL         0x1 /* "reboot" user intended reset */
#define DRA7XX_PMIC_WM_RST_REASON_BOOTLOADER     0x2 /* "reboot bootloader" */
#define DRA7XX_PMIC_WM_RST_REASON_RECOVERY       0x3 /* "reboot recovery" */
#define DRA7XX_PMIC_WM_RST_REASON_DEVELUPDATE    0x4 /* "reboot develupdate" */
#define DRA7XX_PMIC_WM_RST_REASON_AUTHFAILED     0x5 /* "reboot authfailed" */
#define DRA7XX_PMIC_WM_RST_REASON_WAITMOST       0x6 /* "reboot waitmost" */
#define DRA7XX_PMIC_WM_RST_REASON_FACTORYRESET   0x7 /* "reboot factoryreset" */

extern void __init dra7xx_clk_init(void);

struct pdata_init {
	const char *compatible;
	void (*fn)(void);
};

struct of_dev_auxdata omap_auxdata_lookup[];
static struct twl4030_gpio_platform_data twl_gpio_auxdata;

#if IS_ENABLED(CONFIG_OMAP_IOMMU)
int omap_iommu_set_pwrdm_constraint(struct platform_device *pdev, bool request,
				    u8 *pwrst);
#else
static inline int omap_iommu_set_pwrdm_constraint(struct platform_device *pdev,
						  bool request, u8 *pwrst)
{
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_WL12XX)

static struct wl12xx_platform_data wl12xx __initdata;

static void __init __used legacy_init_wl12xx(unsigned ref_clock,
					     unsigned tcxo_clock,
					     int gpio)
{
	int res;

	wl12xx.board_ref_clock = ref_clock;
	wl12xx.board_tcxo_clock = tcxo_clock;
	wl12xx.irq = gpio_to_irq(gpio);

	res = wl12xx_set_platform_data(&wl12xx);
	if (res) {
		pr_err("error setting wl12xx data: %d\n", res);
		return;
	}
}
#else
static inline void legacy_init_wl12xx(unsigned ref_clock,
				      unsigned tcxo_clock,
				      int gpio)
{
}
#endif

#if defined(CONFIG_SOC_AM33XX) || defined(CONFIG_SOC_AM43XX)
static struct gfx_sgx_platform_data sgx_pdata = {
	.reset_name = "gfx",
	.deassert_reset = omap_device_deassert_hardreset,
};
#endif

#ifdef CONFIG_MACH_NOKIA_N8X0
static void __init omap2420_n8x0_legacy_init(void)
{
	omap_auxdata_lookup[0].platform_data = n8x0_legacy_init();
}
#else
#define omap2420_n8x0_legacy_init	NULL
#endif

#ifdef CONFIG_ARCH_OMAP3
static void __init hsmmc2_internal_input_clk(void)
{
	u32 reg;

	reg = omap_ctrl_readl(OMAP343X_CONTROL_DEVCONF1);
	reg |= OMAP2_MMCSDIO2ADPCLKISEL;
	omap_ctrl_writel(reg, OMAP343X_CONTROL_DEVCONF1);
}

static struct iommu_platform_data omap3_iommu_pdata = {
	.reset_name = "mmu",
	.assert_reset = omap_device_assert_hardreset,
	.deassert_reset = omap_device_deassert_hardreset,
	.device_enable = omap_device_enable,
	.device_idle = omap_device_idle,
};

static struct iommu_platform_data omap3_iommu_isp_pdata = {
	.device_enable = omap_device_enable,
	.device_idle = omap_device_idle,
};

static int omap3_sbc_t3730_twl_callback(struct device *dev,
					   unsigned gpio,
					   unsigned ngpio)
{
	int res;

	res = gpio_request_one(gpio + 2, GPIOF_OUT_INIT_HIGH,
			       "wlan rst");
	if (res)
		return res;

	gpio_export(gpio, 0);

	return 0;
}

static void __init omap3_sbc_t3730_twl_init(void)
{
	twl_gpio_auxdata.setup = omap3_sbc_t3730_twl_callback;
}

static void __init omap3_sbc_t3730_legacy_init(void)
{
	legacy_init_wl12xx(WL12XX_REFCLOCK_38, 0, 136);
	omap_ads7846_init(1, 57, 0, NULL);
}

static void __init omap3_igep0020_legacy_init(void)
{
}

static void __init omap3_evm_legacy_init(void)
{
	legacy_init_wl12xx(WL12XX_REFCLOCK_38, 0, 149);
}

static void __init omap3_zoom_legacy_init(void)
{
	legacy_init_wl12xx(WL12XX_REFCLOCK_26, 0, 162);
}

static void am35xx_enable_emac_int(void)
{
	u32 v;

	v = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	v |= (AM35XX_CPGMAC_C0_RX_PULSE_CLR | AM35XX_CPGMAC_C0_TX_PULSE_CLR |
	      AM35XX_CPGMAC_C0_MISC_PULSE_CLR | AM35XX_CPGMAC_C0_RX_THRESH_CLR);
	omap_ctrl_writel(v, AM35XX_CONTROL_LVL_INTR_CLEAR);
	omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR); /* OCP barrier */
}

static void am35xx_disable_emac_int(void)
{
	u32 v;

	v = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	v |= (AM35XX_CPGMAC_C0_RX_PULSE_CLR | AM35XX_CPGMAC_C0_TX_PULSE_CLR);
	omap_ctrl_writel(v, AM35XX_CONTROL_LVL_INTR_CLEAR);
	omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR); /* OCP barrier */
}

static struct emac_platform_data am35xx_emac_pdata = {
	.interrupt_enable	= am35xx_enable_emac_int,
	.interrupt_disable	= am35xx_disable_emac_int,
};

static void __init am3517_evm_legacy_init(void)
{
	u32 v;

	v = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
	v &= ~AM35XX_CPGMACSS_SW_RST;
	omap_ctrl_writel(v, AM35XX_CONTROL_IP_SW_RESET);
	omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET); /* OCP barrier */
}

static void __init nokia_n900_legacy_init(void)
{
	hsmmc2_internal_input_clk();

	if (omap_type() == OMAP2_DEVICE_TYPE_SEC) {
		if (IS_ENABLED(CONFIG_ARM_ERRATA_430973)) {
			pr_info("RX-51: Enabling ARM errata 430973 workaround\n");
			/* set IBE to 1 */
			rx51_secure_update_aux_cr(BIT(6), 0);
		} else {
			pr_warning("RX-51: Not enabling ARM errata 430973 workaround\n");
			pr_warning("Thumb binaries may crash randomly without this workaround\n");
		}
	}
}
#endif /* CONFIG_ARCH_OMAP3 */

#if defined(CONFIG_SOC_AM33XX) || defined(CONFIG_SOC_AM43XX)
static struct wkup_m3_platform_data wkup_m3_data = {
	.reset_name = "wkup_m3",
	.assert_reset = omap_device_assert_hardreset,
	.deassert_reset = omap_device_deassert_hardreset,
};
static struct pruss_platform_data pruss_pdata = {
	.reset_name = "pruss",
	.assert_reset = omap_device_assert_hardreset,
	.deassert_reset = omap_device_deassert_hardreset,
};
#endif

#ifdef CONFIG_ARCH_OMAP4
static void __init omap4_sdp_legacy_init(void)
{
	legacy_init_wl12xx(WL12XX_REFCLOCK_26,
			   WL12XX_TCXOCLOCK_26, 53);
}

static void __init omap4_panda_legacy_init(void)
{
	legacy_init_wl12xx(WL12XX_REFCLOCK_38, 0, 53);
}
#endif

#if defined(CONFIG_ARCH_OMAP4) || defined(CONFIG_SOC_OMAP5) || \
	defined(CONFIG_SOC_DRA7XX)
static struct omap_rproc_timer_ops omap_rproc_dmtimer_ops = {
	.request_timer = omap_rproc_request_timer,
	.release_timer = omap_rproc_release_timer,
	.start_timer = omap_rproc_start_timer,
	.stop_timer = omap_rproc_stop_timer,
	.get_timer_irq = omap_rproc_get_timer_irq,
	.ack_timer_irq = omap_rproc_ack_timer_irq,
};

static struct iommu_platform_data omap4_iommu_pdata = {
	.reset_name = "mmu_cache",
	.assert_reset = omap_device_assert_hardreset,
	.deassert_reset = omap_device_deassert_hardreset,
	.device_enable = omap_device_enable,
	.device_idle = omap_device_idle,
};

static struct omap_rproc_pdata omap4_ipu_pdata = {
	.device_enable = omap_rproc_device_enable,
	.device_shutdown = omap_rproc_device_shutdown,
	.timer_ops = &omap_rproc_dmtimer_ops,
};
#endif

#if defined(CONFIG_ARCH_OMAP4) || defined(CONFIG_SOC_OMAP5)
static struct omap_rproc_pdata omap4_dsp_pdata = {
	.device_enable = omap_rproc_device_enable,
	.device_shutdown = omap_rproc_device_shutdown,
	.set_bootaddr = omap_ctrl_write_dsp_boot_addr,
	.timer_ops = &omap_rproc_dmtimer_ops,
};
#endif

#ifdef CONFIG_SOC_DRA7XX
static struct iommu_platform_data dra7_dsp_mmu_edma_pdata = {
	.device_enable = omap_device_enable,
	.device_idle = omap_device_idle,
};

static struct iommu_platform_data dra7_ipu1_dsp_iommu_pdata = {
	.reset_name = "mmu_cache",
	.assert_reset = omap_device_assert_hardreset,
	.deassert_reset = omap_device_deassert_hardreset,
	.device_enable = omap_device_enable,
	.device_idle = omap_device_idle,
	.set_pwrdm_constraint = omap_iommu_set_pwrdm_constraint,
};

static struct omap_rproc_pdata dra7_dsp1_pdata = {
	.device_enable = omap_rproc_device_enable,
	.device_shutdown = omap_rproc_device_shutdown,
	.set_bootaddr = dra7_ctrl_write_dsp1_boot_addr,
	.timer_ops = &omap_rproc_dmtimer_ops,
	.pre_shutdown = dra7_dsp1_pre_shutdown,
};

static struct omap_rproc_pdata dra7_dsp2_pdata = {
	.device_enable = omap_rproc_device_enable,
	.device_shutdown = omap_rproc_device_shutdown,
	.set_bootaddr = dra7_ctrl_write_dsp2_boot_addr,
	.timer_ops = &omap_rproc_dmtimer_ops,
	.pre_shutdown = dra7_dsp2_pre_shutdown,
};
#endif

#ifdef CONFIG_SOC_AM33XX
static void __init am335x_evmsk_legacy_init(void)
{
	legacy_init_wl12xx(WL12XX_REFCLOCK_38, 0, 31);
}
#endif

#ifdef CONFIG_SOC_OMAP5
static void __init omap5_uevm_legacy_init(void)
{
}
#endif

#ifdef CONFIG_SOC_AM43XX
static void __init am4_evm_legacy_init_lcd_hdmi_switch(int gpio_num)
{
	if (gpio_request_one(gpio_num, GPIOF_DIR_OUT, "LCD/HDMI switch"))
		return;

	if (of_find_compatible_node(NULL, NULL, "sil,sii9022"))
		gpio_set_value(gpio_num, 0);
	else
		gpio_set_value(gpio_num, 1);
}

#define AM43x_GP_EVM_EMMCNAND_GPIO 121	/* gpio3_25 set SEL_eMMCorNANDn */
static void __init am437x_gp_evm_legacy_init(void)
{
	struct device_node *node;
	int ret, flags;
	const int lcd_hdmi_switch_gpio = 104;

	am4_evm_legacy_init_lcd_hdmi_switch(lcd_hdmi_switch_gpio);

	flags = GPIOF_OUT_INIT_HIGH;	/* Enable eMMC */
	node = of_find_compatible_node(NULL, NULL, "ti,am3352-gpmc");

	if (node && of_device_is_available(node))
		flags = GPIOF_OUT_INIT_LOW;	/* Enable NAND */

	ret = gpio_request_one(AM43x_GP_EVM_EMMCNAND_GPIO,
			       flags, "NAND/eMMC switch");
	if (ret)
		pr_err("%s: Failed to get gpio %d\n",
		       __func__, AM43x_GP_EVM_EMMCNAND_GPIO);
}

static void __init am43x_epos_evm_legacy_init(void)
{
	const int lcd_hdmi_switch_gpio = 65;

	am4_evm_legacy_init_lcd_hdmi_switch(lcd_hdmi_switch_gpio);
}
#endif

#ifdef CONFIG_SOC_DRA7XX
static struct pci_dra7xx_platform_data dra7xx_pci_pdata = {
	.reset_name = "pcie",
	.assert_reset = omap_device_assert_hardreset,
	.deassert_reset = omap_device_deassert_hardreset,
};
#endif

static struct pcs_pdata pcs_pdata;

void omap_pcs_legacy_init(int irq, void (*rearm)(void))
{
	pcs_pdata.irq = irq;
	pcs_pdata.rearm = rearm;
}

/*
 * GPIOs for TWL are initialized by the I2C bus and need custom
 * handing until DSS has device tree bindings.
 */
void omap_auxdata_legacy_init(struct device *dev)
{
	if (dev->platform_data)
		return;

	if (strcmp("twl4030-gpio", dev_name(dev)))
		return;

	dev->platform_data = &twl_gpio_auxdata;
}

/*
 * Few boards still need auxdata populated before we populate
 * the dev entries in of_platform_populate().
 */
static struct pdata_init auxdata_quirks[] __initdata = {
#ifdef CONFIG_SOC_OMAP2420
	{ "nokia,n800", omap2420_n8x0_legacy_init, },
	{ "nokia,n810", omap2420_n8x0_legacy_init, },
	{ "nokia,n810-wimax", omap2420_n8x0_legacy_init, },
#endif
#ifdef CONFIG_ARCH_OMAP3
	{ "compulab,omap3-sbc-t3730", omap3_sbc_t3730_twl_init, },
#endif
	{ /* sentinel */ },
};

struct of_dev_auxdata omap_auxdata_lookup[] __initdata = {
#ifdef CONFIG_MACH_NOKIA_N8X0
	OF_DEV_AUXDATA("ti,omap2420-mmc", 0x4809c000, "mmci-omap.0", NULL),
#endif
#ifdef CONFIG_ARCH_OMAP3
	OF_DEV_AUXDATA("ti,omap3-padconf", 0x48002030, "48002030.pinmux", &pcs_pdata),
	OF_DEV_AUXDATA("ti,omap3-padconf", 0x480025a0, "480025a0.pinmux", &pcs_pdata),
	OF_DEV_AUXDATA("ti,omap3-padconf", 0x48002a00, "48002a00.pinmux", &pcs_pdata),
	OF_DEV_AUXDATA("ti,omap2-iommu", 0x5d000000, "5d000000.mmu",
		       &omap3_iommu_pdata),
	OF_DEV_AUXDATA("ti,omap2-iommu", 0x480bd400, "480bd400.mmu",
		       &omap3_iommu_isp_pdata),
	/* Only on am3517 */
	OF_DEV_AUXDATA("ti,davinci_mdio", 0x5c030000, "davinci_mdio.0", NULL),
	OF_DEV_AUXDATA("ti,am3517-emac", 0x5c000000, "davinci_emac.0",
		       &am35xx_emac_pdata),
#endif
#ifdef CONFIG_SOC_AM33XX
	OF_DEV_AUXDATA("ti,am3353-wkup-m3", 0x44d00000, "44d00000.wkup_m3",
		       &wkup_m3_data),
	OF_DEV_AUXDATA("ti,am335x-pruss", 0x4a300000, "4a300000.pruss",
		       &pruss_pdata),
#endif
#ifdef CONFIG_SOC_AM43XX
	OF_DEV_AUXDATA("ti,am4372-wkup-m3", 0x44d00000, "44d00000.wkup_m3",
		       &wkup_m3_data),
	OF_DEV_AUXDATA("ti,am437-padconf", 0x44e10800, "44e10800.pinmux", &pcs_pdata),
	OF_DEV_AUXDATA("ti,am4372-pruss", 0x54400000, "54400000.pruss",
		       &pruss_pdata),
#endif
#if defined(CONFIG_SOC_AM33XX) || defined(CONFIG_SOC_AM43XX)
	OF_DEV_AUXDATA("ti,sgx", 0x56000000, "56000000.sgx",
		       &sgx_pdata),
#endif
#ifdef CONFIG_ARCH_OMAP4
	OF_DEV_AUXDATA("ti,omap4-padconf", 0x4a100040, "4a100040.pinmux", &pcs_pdata),
	OF_DEV_AUXDATA("ti,omap4-padconf", 0x4a31e040, "4a31e040.pinmux", &pcs_pdata),
	OF_DEV_AUXDATA("ti,omap4-rproc-dsp", 0, "dsp", &omap4_dsp_pdata),
	OF_DEV_AUXDATA("ti,omap4-rproc-ipu", 0, "ipu", &omap4_ipu_pdata),
#endif
#ifdef CONFIG_SOC_OMAP5
	OF_DEV_AUXDATA("ti,omap5-padconf", 0x4a002840, "4a002840.pinmux", &pcs_pdata),
	OF_DEV_AUXDATA("ti,omap5-padconf", 0x4ae0c840, "4ae0c840.pinmux", &pcs_pdata),
	OF_DEV_AUXDATA("ti,omap5-rproc-dsp", 0, "dsp", &omap4_dsp_pdata),
	OF_DEV_AUXDATA("ti,omap5-rproc-ipu", 0, "ipu", &omap4_ipu_pdata),
#endif
#if defined(CONFIG_ARCH_OMAP4) || defined(CONFIG_SOC_OMAP5)
	OF_DEV_AUXDATA("ti,omap4-iommu", 0x4a066000, "4a066000.mmu",
		       &omap4_iommu_pdata),
	OF_DEV_AUXDATA("ti,omap4-iommu", 0x55082000, "55082000.mmu",
		       &omap4_iommu_pdata),
#endif
#ifdef CONFIG_SOC_DRA7XX
	OF_DEV_AUXDATA("ti,dra7-pcie", 0x51000000, "51000000.pcie",
		       &dra7xx_pci_pdata),
	OF_DEV_AUXDATA("ti,dra7-pcie", 0x51800000, "51800000.pcie",
		       &dra7xx_pci_pdata),
	OF_DEV_AUXDATA("ti,dra7-padconf", 0x4a003400, "4a003400.pinmux", &pcs_pdata),
	OF_DEV_AUXDATA("ti,dra7-iommu", 0x40d01000, "40d01000.mmu",
		       &dra7_ipu1_dsp_iommu_pdata),
	OF_DEV_AUXDATA("ti,dra7-iommu", 0x40d02000, "40d02000.mmu",
		       &dra7_dsp_mmu_edma_pdata),
	OF_DEV_AUXDATA("ti,dra7-iommu", 0x41501000, "41501000.mmu",
		       &dra7_ipu1_dsp_iommu_pdata),
	OF_DEV_AUXDATA("ti,dra7-iommu", 0x41502000, "41502000.mmu",
		       &dra7_dsp_mmu_edma_pdata),
	OF_DEV_AUXDATA("ti,dra7-iommu", 0x55082000, "55082000.mmu",
		       &omap4_iommu_pdata),
	OF_DEV_AUXDATA("ti,dra7-iommu", 0x58882000, "58882000.mmu",
		       &dra7_ipu1_dsp_iommu_pdata),
	OF_DEV_AUXDATA("ti,dra7-rproc-ipu", 0x55020000, "55020000.ipu",
		       &omap4_ipu_pdata),
	OF_DEV_AUXDATA("ti,dra7-rproc-ipu", 0x58820000, "58820000.ipu",
		       &omap4_ipu_pdata),
	OF_DEV_AUXDATA("ti,dra7-rproc-dsp", 0x40800000, "40800000.dsp",
		       &dra7_dsp1_pdata),
	OF_DEV_AUXDATA("ti,dra7-rproc-dsp", 0x41000000, "41000000.dsp",
		       &dra7_dsp2_pdata),
#endif
	{ /* sentinel */ },
};


#ifdef CONFIG_DENSO_ERRATA_I862
/* Implements best effort writing warm reset reason to a PMIC scratch register
 * which the system can read at boot time.  If i2c or other failures occur here
 * they are ignored as this is a best effort, and any failure will simply tell
 * the softwware reading the reboot reason that is was a cold reset */
int denso_errata_i862_workaround(struct notifer_block *nb, unsigned long action,
								 void *d)
{
	const char *cmd = (char*)d;
	int i = 0;
	unsigned char pmicreason = 0;
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	static const char reason_normal[] = "normal";
	static const char reason_bootloader[] = "bootloader";
	static const char reason_recovery[] = "recovery";
	static const char reason_develupdate[] = "develupdate";
	static const char reason_factoryreset[] = "factoryreset";
	static const char reason_waitmost[] = "waitmost";

	if (soc_is_dra7xx() && (cmd != NULL))
	{
		/* write reason to pmic register, but let warm reset still occur */
		if (!strncmp(reason_normal, cmd, sizeof(reason_normal)-1))
			pmicreason = DRA7XX_PMIC_WM_RST_REASON_NORMAL;
		else if (!strncmp(reason_bootloader, cmd, sizeof(reason_bootloader)-1))
			pmicreason = DRA7XX_PMIC_WM_RST_REASON_BOOTLOADER;
		else if (!strncmp(reason_recovery, cmd, sizeof(reason_recovery)-1))
			pmicreason = DRA7XX_PMIC_WM_RST_REASON_RECOVERY;
		else if (!strncmp(reason_develupdate, cmd, sizeof(reason_develupdate)-1))
			pmicreason = DRA7XX_PMIC_WM_RST_REASON_DEVELUPDATE;
		else if (!strncmp(reason_factoryreset, cmd, sizeof(reason_factoryreset)-1))
			pmicreason = DRA7XX_PMIC_WM_RST_REASON_FACTORYRESET;
		else if (!strncmp(reason_waitmost, cmd, sizeof(reason_waitmost)-1))
			pmicreason = DRA7XX_PMIC_WM_RST_REASON_WAITMOST;
		else
			pmicreason = DRA7XX_PMIC_WM_RST_REASON_NORMAL;

		/* PMIC is on i2c1 --> bus 0 */
		adap = i2c_get_adapter(0);
		if (!adap)
			err = -1;
		else {
			msg->addr = PMIC_I2C_ADDR;    /* I2C address of chip */
			msg->flags = 0;
			msg->len = ARRAY_SIZE(data);
			msg->buf = data;
			data[0] = PMIC_BACKUP0;       /* register num */
			data[1] = pmicreason;         /* register data */
			err = i2c_transfer(adap, msg, ARRAY_SIZE(msg));
			i2c_put_adapter(adap);        /* no i2c errata retries in this case */
		}

		if(err < 0)
			pr_emerg("%s: error writing reset cmd to PMIC: %s, err: %d",
					__func__, cmd, err);
		else
			pr_emerg("%s: wrote %d to PMIC\n", __func__, pmicreason);
	}

	return NOTIFY_OK;
}

static struct notifier_block denso_i862_notifier = {
	.notifier_call = denso_errata_i862_workaround
};

#endif


#ifdef CONFIG_MACH_DENSOJ6REF
void DensoJ6REF_dra7xx_init1 (void)
{
	/* gpio3_29 => GPIO93 => EXP_ETH0_RSTn = Logic 1 */
	gpio_request(93, "EXP_ETH0_RSTn");
	gpio_direction_output(93, 1);
	gpio_set_value(93, 1);
	gpio_export(93, true);

	/* gpio4_8 => GPIO104 => Turn ON PWR_ON_BTH_WIFI = Logic 1 */
	gpio_request(104, "PWR_ON_BTH_WIFI");
	gpio_direction_output(104, 1);
	gpio_set_value(104, 1);
	gpio_export(104, true);

	/* gpio4_6 => GPIO102 => Release BT_Resetn => Logic 1 */
	gpio_request(102, "BT_Resetn");
	gpio_direction_output(102, 1);
	gpio_set_value(102, 1);
	gpio_export(102, true);

	/* gpio4_7 => GPIO103 => Release Wifi_Resetn => Logic 1 */
	gpio_set_value(103, 1);

	/* gpio2_1 => GPIO33 => CAM_PWR_6V_EN => Logic 1 */
	gpio_request(33, "CAM_PWR_6V_EN");
	gpio_direction_output(33, 1);
	gpio_set_value(33, 1);
	gpio_export(33, true);
	printk("\n GPIO33 Requested/Set \n");

	/* gpio4_3 => GPIO99 => PS_USB_CD0 => Logic 1 passthrough */
	gpio_request(99, "PS_USB_CD0");
	gpio_direction_output(99, 1);
	gpio_set_value(99, 1);
	gpio_export(99, false);
	printk("\n GPIO99 Requested/Cleared \n");

	/* gpio4_10 => GPIO106 => PS_USB_CD1 => Logic 0 passthrough */
	gpio_request(106, "PS_USB_CD1");
	gpio_direction_output(106, 0);
	gpio_set_value(106, 0);
	gpio_export(106, false);
	printk("\n GPIO106 Requested/Cleared \n");

	/* gpio6_12 => GPIO172 => USB1_DRV_bus => Logic 1 turn on USB_OTG_POWER */
	gpio_request(172, "USB1_DRV_bus");
	gpio_direction_output(172, 1);
	gpio_set_value(172, 1);
	gpio_export(172, true);
	printk("\n GPIO172 Requested/Set \n");

	/* gpio1_23 => GPIO23 => INT_3DGYRO => Logic 1 turn on USB_OTG_POWER */
	gpio_request(23, "INT_3DGYRO");
	gpio_direction_output(23, 0);
	gpio_set_value(23, 0);
	gpio_export(23, false);
	printk("\n GPIO23 Requested/Set \n");
}

static void __init denso_omap_quirks(void)
{
	/*struct device_node *np; */

#ifdef CONFIG_DENSO_ERRATA_I862
	register_reboot_notifier(&denso_i862_notifier);
#endif

	DensoJ6REF_dra7xx_init1();   /* Init DensoJ6REF GPIOs  */
	/* create clock aliases based on 'clock_alias' nodes */
/* Doesn't exist in the init function anymore/Not denso specific*/
/*
	for_each_node_by_name(np, "clock_alias") {
		omap_create_clk_alias(np);
		of_node_put(np);
	}
*/
}
#endif

/*
 * Few boards still need to initialize some legacy devices with
 * platform data until the drivers support device tree.
 */
static struct pdata_init pdata_quirks[] __initdata = {
#ifdef CONFIG_ARCH_OMAP3
	{ "compulab,omap3-sbc-t3730", omap3_sbc_t3730_legacy_init, },
	{ "nokia,omap3-n900", nokia_n900_legacy_init, },
	{ "nokia,omap3-n9", hsmmc2_internal_input_clk, },
	{ "nokia,omap3-n950", hsmmc2_internal_input_clk, },
	{ "isee,omap3-igep0020", omap3_igep0020_legacy_init, },
	{ "ti,omap3-evm-37xx", omap3_evm_legacy_init, },
	{ "ti,omap3-zoom3", omap3_zoom_legacy_init, },
	{ "ti,am3517-evm", am3517_evm_legacy_init, },
#endif
#ifdef CONFIG_ARCH_OMAP4
	{ "ti,omap4-sdp", omap4_sdp_legacy_init, },
	{ "ti,omap4-panda", omap4_panda_legacy_init, },
#endif
#ifdef CONFIG_SOC_AM33XX
	{ "ti,am335x-evmsk", am335x_evmsk_legacy_init, },
#endif
#ifdef CONFIG_SOC_OMAP5
	{ "ti,omap5-uevm", omap5_uevm_legacy_init, },
#endif
#ifdef CONFIG_SOC_AM43XX
	{ "ti,am437x-gp-evm", am437x_gp_evm_legacy_init, },
	{ "ti,am43x-epos-evm", am43x_epos_evm_legacy_init, },
#endif
#ifdef CONFIG_MACH_DENSOJ6REF
	{ "denso,dra75xref", denso_omap_quirks, },
	{ "denso,dra74xmid", denso_omap_quirks, },
	{ "denso,dra74xlow", denso_omap_quirks, },
	{ "denso,dra72xlow", denso_omap_quirks, },
#endif
	{ /* sentinel */ },
};

static void pdata_quirks_check(struct pdata_init *quirks)
{
	while (quirks->compatible) {
		if (of_machine_is_compatible(quirks->compatible)) {
			if (quirks->fn)
				quirks->fn();
			break;
		}
		quirks++;
	}
}

void __init pdata_quirks_init(struct of_device_id *omap_dt_match_table)
{
	omap_sdrc_init(NULL, NULL);
	pdata_quirks_check(auxdata_quirks);
	of_platform_populate(NULL, omap_dt_match_table,
			     omap_auxdata_lookup, NULL);
	pdata_quirks_check(pdata_quirks);
}
