/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2014-2015 Cetitec, GmbH. All Rights Reserved.
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
 * This is the configuration code specific to TI Jacinto6
 * MLB support.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include "mlb150.h"
#include "mlb150_ext.h"
#include "mlb150_int.h"
#include "j6.h"

/* Interrupt configuration on TI Jacinto6 */
#define J6_MLB_IRQ_MAX_NR    159
#define J6_MLB_IRQ_VIRQ_OFFS 32 /* origin unknown, to be used with AHB and MLB interrupts */

#define J6_MLB_IRQ_MISSING1  132
#define J6_MLB_IRQ_MISSING2  133

/* The default irq number chosen for the kernel in TI 6AK.1.1 software release */
static unsigned int j6_mlb_irq_mlb  = 116;
static unsigned int j6_mlb_irq_ahb0 = 117;
static unsigned int j6_mlb_irq_ahb1 = 118;
static unsigned int gic_only;

module_param_named(gic, gic_only, uint, S_IRUGO);
MODULE_PARM_DESC(gic, "set to 1 to map the interrupts using the irq domain of GIC");
module_param_named(irq_mlb,  j6_mlb_irq_mlb,  uint, S_IRUGO);
module_param_named(irq_ahb0, j6_mlb_irq_ahb0, uint, S_IRUGO);
module_param_named(irq_ahb1, j6_mlb_irq_ahb1, uint, S_IRUGO);

/* origin unknwon */
#define J6_MLB_BASE_ADDR   0x4842C000
#define J6_MLB_BASE_SIZE   0x4000

/* Table 18-25 */
#define J6_CROSSBAR_BASE_ADDR    0x4A002000
#define J6_CROSSBAR_SIZE         0x2000 /* 8 kiB */

/*
 * Device IRQ crossbar input lines for Jacinto6 SoC. See Table 17-7 and
 * Figure 24-205.
 * Note: MPU IRQs are listed in section 17.3.1, Table 17-2
 */
#define J6_MLB_IRQ_SYS_INT0 228 /* IRQs for logical channels  0 to 31 */
#define J6_MLB_IRQ_SYS_INT1 229 /* IRQs for logical channels 32 to 63 */
#define J6_MLB_IRQ_SYS      397 /* IRQs for errors                    */


const struct of_device_id mlb150_of_device_ids[] = {
	{.compatible = "smsc,mlb150"},
	{},
};


static int reliable_irq_mapping;

static struct {
	irq_hw_number_t hwirq;
	unsigned int virq;
} dumb_irq_map[3];

static unsigned int dumb_map_irq_to_virq(struct device_node *node, irq_hw_number_t irq)
{
	/*
	 * FIXME: this must use the correct irq domain instead of just an offset
	 * The problem is that I cannot find the irq domain if there is no
	 * device tree node. There seem to be no default domain controller
	 * (how?!) -- ari
	 */
	struct irq_data *irqd;
	unsigned int virq;
	for (virq = 0; virq < ARRAY_SIZE(dumb_irq_map); ++virq)
		if (dumb_irq_map[virq].hwirq == irq) {
			virq = dumb_irq_map[virq].virq;
			goto mapped;
		}
	virq = !node * J6_MLB_IRQ_VIRQ_OFFS + irq;
mapped:
	irqd = irq_get_irq_data(virq);

	if (!irqd || irqd_to_hwirq(irqd) != irq)
		pr_err("mlb150: failed to reliably map hw irq %lu to virq\n", irq);
	else
		reliable_irq_mapping = 1;

	return virq;
}

static irq_hw_number_t unmap_virq(unsigned int virq)
{
	struct irq_data *irqd = irq_get_irq_data(virq);
	return reliable_irq_mapping && irqd ? irqd_to_hwirq(irqd) : virq - J6_MLB_IRQ_VIRQ_OFFS;
}


int mlb150_get_mlb_io(struct mlb_data *drvdata)
{
	u32 baseaddr;
	u64 basesize;
	void __iomem *base;
	const struct of_device_id *match;

	match = of_match_device(mlb150_of_device_ids, drvdata->dev);

	if (!match) {
		dev_err(drvdata->dev, "mlb device not found in the device tree\n");
		/*
		 * TODO: add an mlb definition to the device tree
		 * return -ENOENT;
		 */
	}
	if (match) {
		drvdata->irq_mlb  = irq_of_parse_and_map(drvdata->dev->of_node, 0);
		drvdata->irq_ahb0 = irq_of_parse_and_map(drvdata->dev->of_node, 1);
		drvdata->irq_ahb1 = irq_of_parse_and_map(drvdata->dev->of_node, 2);
	} else {
		drvdata->irq_mlb  = dumb_map_irq_to_virq(drvdata->dev->of_node, j6_mlb_irq_mlb);
		drvdata->irq_ahb0 = dumb_map_irq_to_virq(drvdata->dev->of_node, j6_mlb_irq_ahb0);
		drvdata->irq_ahb1 = dumb_map_irq_to_virq(drvdata->dev->of_node, j6_mlb_irq_ahb1);
	}
	pr_debug("virq: mlb %u, ahb0 %u, ahb1 %u%s\n",
		 drvdata->irq_mlb, drvdata->irq_ahb0, drvdata->irq_ahb1,
		 match ? " (from device tree)" : "");
	if (!drvdata->irq_mlb || !drvdata->irq_ahb0 || !drvdata->irq_ahb1)
		return -ENOSPC;

	/* MLB base address */
	baseaddr = J6_MLB_BASE_ADDR;
	basesize = J6_MLB_BASE_SIZE;
	if (match) {
		if (of_can_translate_address(drvdata->dev->of_node)) {
			unsigned int flags = 0;
			const __be32 *ptr;
			basesize = 0;
			ptr = of_get_address(drvdata->dev->of_node, 0, &basesize, &flags);
			baseaddr = be32_to_cpu(*ptr);
			pr_debug("of_get_address: ptr: %08X, size: %llu, flags: %08X\n",
				 baseaddr, basesize, flags);
		} else
			return -ENOENT;
	}

	base = ioremap(baseaddr, basesize);
	dev_info(drvdata->dev, "MLB DIM phys addr 0x%08X mapped at 0x%p (length: %llu)\n",
		 baseaddr, base, basesize);

	if (unlikely(base == NULL)) {
		dev_err(drvdata->dev, "failed to do ioremap with mlb150 base\n");
		return -ENOMEM;
	}

	drvdata->membase = base;
#ifdef MLB_DIM_REGISTER_OFFSET
	base += MLB_DIM_REGISTER_OFFSET;
	dev_info(drvdata->dev, "adjusted mlb register base: 0x%p\n", base);
#endif
	drvdata->mlbregs = base;
	return 0;
}

#ifdef DEBUG
static void printCrossbarConfig4CPU(void __iomem *crossbar)
{
	const u8 __iomem *addr = crossbar + 0x0A48;
	unsigned i;

	u32 map = readl_relaxed(addr);
	pr_debug("%08x:   4: %4u    5: %4u\n", (u32)addr, map & 0xffff, map >> 16);
	for (i = 8; i < 132; i += 2) {
		addr = crossbar + 0xA4C + ((i-8) * 4 / 2);
		map = readl_relaxed(addr);
		pr_debug("%08x: %3u: %4u  %3u: %4u\n", (unsigned)addr, i, map & 0xffff, i+1, map >> 16);
	}

	for (i = 134; i < 160; i += 2) {
		addr = crossbar + 0xB44 + ((i-134) * 4 / 2);
		map = readl_relaxed(addr);
		pr_debug("%08x: %3u: %4u  %3u: %4u\n", (unsigned)addr, i, map & 0xffff, i+1, map >> 16);
	}
}
#else
static void printCrossbarConfig4CPU(void __iomem *crossbar) {}
#endif


static int jacinto6_setupIRQs(const struct mlb_data *drvdata)
{
	static const u16 crossbar_ints[] = {
		J6_MLB_IRQ_SYS, J6_MLB_IRQ_SYS_INT0, J6_MLB_IRQ_SYS_INT1
	};

	unsigned int i;
	void __iomem *crossbar;
	u32 hw_irq_numbers[3];

	/* crossbar configuration needs hw irq numbers */
	hw_irq_numbers[0] = unmap_virq(drvdata->irq_mlb);
	hw_irq_numbers[1] = unmap_virq(drvdata->irq_ahb0);
	hw_irq_numbers[2] = unmap_virq(drvdata->irq_ahb1);

	pr_debug("hw irq: mlb %u, ahb0 %u, ahb1 %u\n",
		 hw_irq_numbers[0], hw_irq_numbers[1], hw_irq_numbers[2]);

	for (i = 0; i < 3; ++i) {
		const u32 irq_nr = hw_irq_numbers[i];
		/* check range / forbidden numbers */
		if (irq_nr > J6_MLB_IRQ_MAX_NR ||
		    irq_nr == J6_MLB_IRQ_MISSING1 ||
		    irq_nr == J6_MLB_IRQ_MISSING2) {
			pr_debug("invalid irq %s: %u\n",
				 i == 0 ? "mlb"  :
				 i == 1 ? "ahb0" :
				 i == 2 ? "ahb1" : "?",
				 irq_nr);
			return -ENOENT; /* error */
		}
	}

	/* Map physical address range for IRQ crossbar into virtual address space */
	crossbar = ioremap(J6_CROSSBAR_BASE_ADDR, J6_CROSSBAR_SIZE);

	if (!crossbar)
		return -EPERM;

	pr_debug("J6 crossbar accessible from CPU at 0x%p\n", crossbar);
	printCrossbarConfig4CPU(crossbar);

	for (i = 0; i < 3; i++) {
		unsigned int offset;
		void __iomem *reg = crossbar;
		const u32 irq_nr = hw_irq_numbers[i];

		/* Address calculation for IRQ crossbar register: See Table 18-26.
		 * Each two IRQs use a 32-bit register.
		 * IRQ 4/5 register is special at offset 0xA48; Then the others
		 * follow at offset 0xA4C (8/9) up to 130/131(0xB40) and from
		 * 0xB44 (134/135) up to 0xB74 (158/159).
		 */
		if (irq_nr < 8)
			offset = 0xA48;
		else {
			offset = 0xA4C;
			/* two IRQs per 4 Byte register */
			offset += (irq_nr - 8) * 4 / 2;

			/* compensate the "missing" 132/133. */
			if (irq_nr > J6_MLB_IRQ_MISSING1)
				offset -= 4;
		}

		reg += offset;

		pr_debug("%s: irq nr = %u, crossbar offs 0x%x (%u->%u)\n",
			 i == 0 ? "mlb " :
			 i == 1 ? "ahb0" :
			 i == 2 ? "ahb1" : "?",
			 irq_nr, offset / 2, readw(reg), crossbar_ints[i]);

		/*
		 * Section 18.4.6.4:
		 *   Write to bits [8:0] or [24:16] of a 32-bit-word,
		 *   i.e. always bits [8:0] of a 16-bit value
		 */
		writew(crossbar_ints[i] & 0x01FFu, reg);
	}
	printCrossbarConfig4CPU(crossbar);
	/* unmap the temporary mapping */
	iounmap(crossbar);
	return 0;
}

int mlb150_init_mlb_io(struct mlb_data *drvdata)
{
	int ret;

	ret = jacinto6_enableMLBmodule();
	if (ret) {
		dev_err(drvdata->dev, "failed enabling MLB module (%d)", ret);
		goto fail;
	}
	if (!drvdata->dev->of_node)
		ret = jacinto6_setupIRQs(drvdata);
	if (ret) {
		dev_err(drvdata->dev, "failed setting the J6 interrupts (%d)", ret);
		goto fail;
	}

	ret = jacinto6_6pinMLB();
	if (ret) {
		dev_err(drvdata->dev, "failed initializing MLB IO (%d)", ret);
		goto fail;
	}
fail:
	return ret;
}

void mlb150_free_mlb_io(struct mlb_data *drvdata)
{
	if (drvdata->membase) {
		iounmap(drvdata->membase);
		drvdata->membase = NULL;
	}
	if (drvdata->irq_ahb0) {
		free_irq(drvdata->irq_ahb0, drvdata);
		drvdata->irq_ahb0 = 0;
	}
	if (drvdata->irq_ahb1) {
		free_irq(drvdata->irq_ahb1, drvdata);
		drvdata->irq_ahb1 = 0;
	}
	if (drvdata->irq_mlb) {
		free_irq(drvdata->irq_mlb, drvdata);
		drvdata->irq_mlb = 0;
	}
}

static struct platform_device *plat_dev;

static void setup_gic_irq_numbers(void)
{
	int virq;
	struct irq_desc *desc;
	for_each_irq_desc(virq, desc) {
		const char *irqname;
		irq_hw_number_t hwirq;
		struct irq_data *irqd = &desc->irq_data;

		if (!irqd || !irqd->chip || !irqd->chip->name ||
		    memcmp(irqd->chip->name, "GIC", 4) != 0)
			continue;

		hwirq = irqd_to_hwirq(irqd);
		if (hwirq >= J6_MLB_IRQ_MAX_NR ||
		    hwirq == J6_MLB_IRQ_MISSING1 ||
		    hwirq == J6_MLB_IRQ_MISSING2) {
			pr_debug("%4d: hwirq %3lu %s unavailble\n",
				 virq, hwirq,
				 irqd->chip ? irqd->chip->name : "NULL");
			continue;
		}
		irqname = NULL;
		if (hwirq == j6_mlb_irq_mlb) {
			dumb_irq_map[0].hwirq = hwirq;
			dumb_irq_map[0].virq = irqd->irq;
			irqname = "mlb";
		} else if (hwirq == j6_mlb_irq_ahb0) {
			dumb_irq_map[1].hwirq = hwirq;
			dumb_irq_map[1].virq = irqd->irq;
			irqname = "ahb0";
		} else if (hwirq == j6_mlb_irq_ahb1) {
			dumb_irq_map[2].hwirq = hwirq;
			dumb_irq_map[2].virq = irqd->irq;
			irqname = "ahb1";
		}
		if (irqname)
			pr_info("mlb150: %4d: hwirq %3lu %s -> %s\n",
				virq, hwirq,
				irqd->chip ? irqd->chip->name : "NULL", irqname);
		else
			pr_debug("%4d: hwirq %3lu %s\n",
				 virq, hwirq,
				 irqd->chip ? irqd->chip->name : "NULL");
	}
}

static int irq_test;
module_param_named(irq, irq_test,  int, S_IRUGO);
static irqreturn_t irq_test_isr(int irq, void *dev_id)
{
	pr_info_ratelimited("irq %d %p taken\n", irq, dev_id);
	return IRQ_NONE;
}

int __init mlb150_init_driver(struct platform_driver *drv, int (*probe)(struct platform_device *))
{
	if (gic_only)
		setup_gic_irq_numbers();

	if (irq_test > 0) {
		int ret = request_irq(irq_test, irq_test_isr, IRQF_TRIGGER_HIGH, "mlb150", NULL);
		if (ret < 0)
			pr_err("request_irq: failed %d\n", ret);
		return ret;
	}
	// TODO Jacinto6 with DeviceTree/OF support will have to use platorm_driver_probe
	// Otherwise the probe callback is executed twice: once for the dummy
	// device created by platform_create_bundle, and once for the one in
	// the device tree.
	plat_dev = platform_create_bundle(drv, probe, NULL, 0, NULL, 0);

	if (IS_ERR(plat_dev))
		return PTR_ERR(plat_dev);

	return 0;
}

void __exit mlb150_exit_driver(struct platform_driver *drv)
{
	if (irq_test > 0) {
		free_irq(irq_test, NULL);
		pr_debug("MLB driver removed\n");
		return;
	}
	if (plat_dev)
		platform_device_unregister(plat_dev);

	platform_driver_unregister(drv);
}

