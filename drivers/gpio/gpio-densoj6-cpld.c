/*
 * Released under the GPL v2.
 * Copyright (C) 2012-2016 DENSO
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, see <http://www.gnu.org/licenses>.
 */


#include <linux/init.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/io.h>
#include <asm/mach/irq.h>
#include <linux/irqdomain.h>
#include <linux/slab.h>

#define MAX_NR_GPIO_BANKS	8
#define MAX_NR_GPIO		32
#define DRIVER_NAME		"densoj6-cpld-gpio"

#define INT_MASK	0
#define INT_POL		1
#define	DATA_DIR	2
#define DATA_OUT	3
#define DATA_IN		4

struct densoj6_cpld_gpio {
	struct gpio_chip	gc;
	struct irq_chip		ic;
	int			irq; /* GPIO Expander -> iMX6Q*/
	u32			base;
	spinlock_t		lock;
	struct platform_device	*pdev;
	struct irq_domain	*domain;
	int			irq_base;
	struct lock_class_key	irq_nested_lock_class;
};

static inline u32 get_bank_base(u32 chip_base, unsigned offset)
{
	return (u32) (chip_base
		+ (((offset & 0x0000001F) >> 2) << 3 ));
}

static inline u8 gpio_to_mask(unsigned offset)
{
	return (u8) (1  << (offset & 0x00000003));
}

static u32 get_irq_status(struct densoj6_cpld_gpio *chip)
{
	u32		bank_base	= 0;
	void __iomem	*dir_reg	= 0;
	void __iomem	*intmask_reg	= 0;
	void __iomem	*datain_reg	= 0;
	void __iomem	*intpol_reg	= 0;
	u8		dir		= 0;
	u8		intmask		= 0;
	u8		datain		= 0;
	u8		intpol		= 0;
	int		i		= 0;
	u32		status		= 0;
	unsigned long	flags		= 0;


	for (i = 0; i < chip->gc.ngpio; i += 4) {
		bank_base	= get_bank_base(chip->base, i);
		dir_reg		= (void __iomem *) (bank_base + DATA_DIR);
		intmask_reg	= (void __iomem *) (bank_base + INT_MASK);
		datain_reg	= (void __iomem *) (bank_base + DATA_IN);
		intpol_reg	= (void __iomem *) (bank_base + INT_POL);

		spin_lock_irqsave(&chip->lock, flags);
		dir		= readb(dir_reg);
		intmask		= readb(intmask_reg);
		datain		= readb(datain_reg);
		intpol		= readb(intpol_reg);
		spin_unlock_irqrestore(&chip->lock, flags);

		/* TODO: no status register o find out source of interrupt
		* the logic to find interrupt source implemented below is
		* if direction is set as in, interrupt is not masked, data read
		* is same as the interrupt polarity set.
		* Note: This logic may be incorrect, if the interrupt line
		* asserted for a small duration & de-asserted by the time
		* the pin is read.
		* The status[31:0] = bank0[3:0]
				| bank1[3:0] << 4
				| bank2[3:0] << 8
				| bank3[3:0] << 12
				| bank4[3:0] << 16
				| bank5[3:0] << 20
				| bank6[3:0] << 24
				| bank7[3:0] << 28
		*/
		status |= (((u32) (~dir & ~intmask & ~(intpol ^ datain))
					& 0x0000000f) << i);
	}

	return status;
}

static void densoj6_cpld_gpio_irq_handler(u32 irq, struct irq_desc *desc)
{
	struct densoj6_cpld_gpio   *chip	= irq_get_handler_data(irq);
	struct irq_chip		*ic	= irq_get_chip(irq);
	u32			status	= 0;
	unsigned int		i = 0;

	status = get_irq_status(chip);

	chained_irq_enter(ic, desc);
	for (i = 0; i < chip->gc.ngpio; i++) {
		if (status & (1 << i))
			generic_handle_irq(irq_find_mapping(chip->domain, i));
	}
	chained_irq_exit(ic, desc);
}

static void densoj6_cpld_gpio_irq_mask(struct irq_data *data)
{
	struct densoj6_cpld_gpio	*chip
		= (struct densoj6_cpld_gpio *) irq_data_get_irq_chip_data(data);
	unsigned offset = data->hwirq;
	void __iomem *reg = (void __iomem *) (get_bank_base(chip->base, offset)
				+ INT_MASK);
	u8	val = 0;
	unsigned long		flags = 0;
	u8 mask = gpio_to_mask(offset);

	spin_lock_irqsave(&chip->lock, flags);
	val = readb(reg);
	val |= mask;
	writeb(val, reg);
	spin_unlock_irqrestore(&chip->lock, flags);

	return;
}

static void densoj6_cpld_gpio_irq_unmask(struct irq_data *data)
{
	struct densoj6_cpld_gpio	*chip
		= (struct densoj6_cpld_gpio *) irq_data_get_irq_chip_data(data);
	unsigned offset = data->hwirq;
	void __iomem *reg = (void __iomem *) (get_bank_base(chip->base, offset)
				+ INT_MASK);
	u8	val = 0;
	unsigned long		flags = 0;
	u8 mask = gpio_to_mask(offset);

	spin_lock_irqsave(&chip->lock, flags);
	val = readb(reg);
	val &= ~mask;
	writeb(val, reg);
	spin_unlock_irqrestore(&chip->lock, flags);

	return;
}

static int densoj6_cpld_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct densoj6_cpld_gpio	*chip
		= (struct densoj6_cpld_gpio *) irq_data_get_irq_chip_data(data);
	unsigned offset = data->hwirq;
	void __iomem *reg = (void __iomem *) (get_bank_base(chip->base, offset)
				+ INT_POL);
	u8			val = 0;
	unsigned long		flags = 0;
	u8 mask = gpio_to_mask(offset);

	spin_lock_irqsave(&chip->lock, flags);
	val = readb(reg);

	switch (type) {
	case IRQ_TYPE_LEVEL_LOW:
		val &= ~mask;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		val |= mask;
		break;
	default:
		spin_unlock_irqrestore(&chip->lock, flags);
		return -EINVAL;
	}

	writeb(val, reg);
	spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

static int densoj6_cpld_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
	struct densoj6_cpld_gpio	*chip = container_of(gc, struct densoj6_cpld_gpio,
							gc);
	unsigned long	flags = 0;
	u8		val = 0;
	void __iomem	*reg = (void __iomem *)
				(get_bank_base(chip->base, offset) + DATA_DIR);

	u8 mask = gpio_to_mask(offset);

	spin_lock_irqsave(&chip->lock, flags);
	
	val = readb(reg);
	val &= ~mask;

	writeb(val, reg);
	spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

static int densoj6_cpld_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct densoj6_cpld_gpio	*chip = container_of(gc, struct densoj6_cpld_gpio,
							gc);
	unsigned long	flags = 0;
	u8		val = 0;
	void __iomem	*reg = (void __iomem *)
				(get_bank_base(chip->base, offset) + DATA_IN);

	u8 mask = gpio_to_mask(offset);

	spin_lock_irqsave(&chip->lock, flags);
	
	val = readb(reg);
	
	spin_unlock_irqrestore(&chip->lock, flags);

	val &= mask;
	val >>= (offset & 0x00000003);

	return (int) val;
}

static int densoj6_cpld_gpio_direction_output(struct gpio_chip *gc,
						unsigned offset, int value)
{
	struct densoj6_cpld_gpio	*chip = container_of(gc, struct densoj6_cpld_gpio,
							gc);
	unsigned long	flags = 0;
	u8		val = 0;
	void __iomem	*reg = (void __iomem *)
				(get_bank_base(chip->base, offset) + DATA_DIR);

	u8 mask = gpio_to_mask(offset);

	spin_lock_irqsave(&chip->lock, flags);
	
	val = readb(reg);
	val |= mask;
	
	writeb(val, reg);
	spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

static void densoj6_cpld_gpio_set(struct gpio_chip *gc,
				unsigned offset, int value)
{
	struct densoj6_cpld_gpio	*chip = container_of(gc, struct densoj6_cpld_gpio,
							gc);
	unsigned long	flags = 0;
	u8		val = 0;
	void __iomem	*reg = (void __iomem *)
				(get_bank_base(chip->base, offset) + DATA_OUT);

	u8 mask = gpio_to_mask(offset);

	spin_lock_irqsave(&chip->lock, flags);
	
	val = readb(reg);
	if (value)
		val |= mask;
	else
		val &= ~mask;

	writeb(val, reg);
	spin_unlock_irqrestore(&chip->lock, flags);

	return;
}

static int densoj6_cpld_gpio_to_irq(struct gpio_chip *gc,
				unsigned offset)
{
	struct densoj6_cpld_gpio	*chip =
				container_of(gc, struct densoj6_cpld_gpio, gc);

	return irq_find_mapping(chip->domain, offset);
}

static void densoj6_cpld_gpio_init(struct densoj6_cpld_gpio *chip)
{
	u32		bank_base	= 0;
	void __iomem	*dir_reg	= 0;
	void __iomem	*intmask_reg	= 0;
	int		i		= 0;
	unsigned long	flags		= 0;


	for (i = 0; i < chip->gc.ngpio; i += 4) {
		bank_base	= get_bank_base(chip->base, i);
		dir_reg		= (void __iomem *) (bank_base + DATA_DIR);
		intmask_reg	= (void __iomem *) (bank_base + INT_MASK);

		spin_lock_irqsave(&chip->lock, flags);
		/* mask the interrupts */
		writeb(0xff, intmask_reg);
		/* tristate the pins */
		writeb(0x00, dir_reg);
		spin_unlock_irqrestore(&chip->lock, flags);

	}
}

/*
* Expected device tree entry
* --------------------------
densoj6-cpld-gpio@0c000000 {
	compatible = "denso,densoj6-cpld-gpio";
	reg = <0x0c000000 0x4000>;
	host_irq-gpio = <&gpio1 29 0>;
	interrupts = <0 29 0x04>;
	interrupt-parent = <&gpio1>;
	gpio-controller;
	#gpio-cells = <2>;
	interrupt-controller;
	#interrupt-cells = <2>;
	status = "okay";
};
*/


/*static int __devinit gpio_densoj6_cpld_probe(struct platform_device *pdev) */
static int __init gpio_densoj6_cpld_probe(struct platform_device *pdev)
{
	struct densoj6_cpld_gpio	*chip = NULL;
	int			ret   = 0;
	unsigned int		i = 0;
	unsigned int		irq;
	struct resource		*iores = NULL;
	struct device_node	*np     = pdev->dev.of_node;
	int			host_int_gpio = 0;
	/* tmp is just used for removing warnings */
	int			tmp = 0;

    printk("Initializing DENSO_CPLD_GPIO\n");

	chip = kzalloc(sizeof(struct densoj6_cpld_gpio), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "failed to allocate densoj6_cpld_gpio\n");
		goto err_alloc_chip;
	}

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!iores) {
		ret = -ENODEV;
		goto err_get_res;
	}

	if (!request_mem_region(iores->start,
				resource_size(iores), pdev->name)) {
		ret = -EBUSY;
		goto err_req_mem_reg;
	}

	chip->base = (u32) ioremap(iores->start, resource_size(iores));
	if (!chip->base) {
		ret = -ENOMEM;
		goto err_ioremap;
	}

	/*
	* TODO: check wheather the device tree should pass the irq directly
	* or GPIO1_29. If GPIO1_29 then the corresponding irq can be derived
	*/
#if 0
	chip->irq = platform_get_irq(pdev, 0);
#else
	host_int_gpio = of_get_named_gpio(np, "host_irq-gpio", 0);
	if (!gpio_is_valid(host_int_gpio)) {
		ret = -EIO;
		goto err_get_irq;
	}

	chip->irq = gpio_to_irq(host_int_gpio);
#endif
	if (chip->irq < 0) {
		ret = -EINVAL;
		goto err_get_irq;
	}
	densoj6_cpld_gpio_init(chip);

	spin_lock_init(&chip->lock);

	chip->gc.label			= dev_name(&pdev->dev);
	chip->gc.owner			= THIS_MODULE;
	chip->gc.direction_input	= densoj6_cpld_gpio_direction_input;
	chip->gc.direction_output	= densoj6_cpld_gpio_direction_output;
	chip->gc.get			= densoj6_cpld_gpio_get;
	chip->gc.set			= densoj6_cpld_gpio_set;
	chip->gc.to_irq			= densoj6_cpld_gpio_to_irq;
	chip->gc.base			= -1;
	chip->gc.ngpio			= MAX_NR_GPIO;

	chip->ic.name			= dev_name(&pdev->dev);
	chip->ic.irq_mask		= densoj6_cpld_gpio_irq_mask;
	chip->ic.irq_unmask		= densoj6_cpld_gpio_irq_unmask;
	chip->ic.irq_set_type		= densoj6_cpld_gpio_irq_set_type;

	chip->pdev			= pdev;

	ret = gpiochip_add(&chip->gc);

	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add gpio chip");
		goto err_gpiochip_add;
	}
#if 0 //interrupt part commented temporarily
	irq_set_chained_handler(chip->irq, densoj6_cpld_gpio_irq_handler);
	irq_set_handler_data(chip->irq, chip);
	irq_set_irq_type(chip->irq, IRQ_TYPE_EDGE_FALLING);

	chip->irq_base = irq_alloc_descs(-1, 0, 32, numa_node_id());

	if (chip->irq_base < 0) {
		ret = chip->irq_base;
		goto err_irq_alloc_desc;
	}

	chip->domain = irq_domain_add_legacy(np, 32, chip->irq_base, 0,
					&irq_domain_simple_ops, NULL);

	if (!chip->domain) {
		ret = -ENODEV;
		goto err_irq_domain_add;
	}

	for (i = 0; i < chip->gc.ngpio; ++i) {
		irq = chip->irq_base + i;
		irq_set_lockdep_class(irq, &chip->irq_nested_lock_class);
		irq_set_chip_and_handler(irq, &chip->ic, handle_level_irq);
		irq_set_chip_data(irq, chip);
		irq_modify_status(irq, IRQ_NOREQUEST, 0);
	}
#endif
	platform_set_drvdata(pdev, chip);

	return 0;

#if 0
err_set_pdrvdat:
	for (i = 0; i < chip->gc.ngpio; ++i) {
		irq = chip->irq_base + i;
		irq_set_handler(irq, NULL);
		irq_set_chip(irq, &no_irq_chip);
		irq_set_chip_data(irq, NULL);
		irq_modify_status(irq, 0, 0);
	}
#endif

err_irq_domain_add:
	irq_free_descs(chip->irq_base, 32);
err_irq_alloc_desc:
	tmp = gpiochip_remove(&chip->gc);
err_gpiochip_add:
err_get_irq:
	iounmap((void __iomem *) chip->base);
err_ioremap:
	release_mem_region(iores->start, resource_size(iores));
err_req_mem_reg:
err_get_res:
	kfree(chip);
err_alloc_chip:

	return ret;
}

/*static int __devexit gpio_densoj6_cpld_remove(struct platform_device *pdev) */
static int __exit gpio_densoj6_cpld_remove(struct platform_device *pdev)
{
	struct densoj6_cpld_gpio	*chip = (struct densoj6_cpld_gpio *)
					platform_get_drvdata(pdev);
	unsigned int		i = 0;
	unsigned int		irq = 0;
	struct resource		*iores = platform_get_resource(pdev,
							IORESOURCE_MEM, 0);
	/* tmp is just used for removing warnings */
	int			tmp = 0;
#if 0 //interrupt part commented temporarily
	for (i = 0; i < chip->gc.ngpio; ++i) {
		irq = chip->irq_base + i;
		irq_set_handler(irq, NULL);
		irq_set_chip(irq, &no_irq_chip);
		irq_set_chip_data(irq, NULL);
		irq_modify_status(irq, 0, 0);
	}

	irq_free_descs(chip->irq_base, 32);
#endif
	tmp = gpiochip_remove(&chip->gc);
	iounmap((void __iomem *) chip->base);
	release_mem_region(iores->start, resource_size(iores));
	kfree(chip);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id densoj6_cpld_gpio_match[] = {
	{ .compatible = "denso,densoj6-cpld-gpio",},
	{},
};
MODULE_DEVICE_TABLE(of, densoj6_cpld_gpio_match);
#else
# define densoj6_cpld_gpio_match NULL
#endif

static struct platform_driver densoj6_cpld_gpio_driver = {
	.probe = gpio_densoj6_cpld_probe,
	.remove = gpio_densoj6_cpld_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = densoj6_cpld_gpio_match,
	},
};

module_platform_driver(densoj6_cpld_gpio_driver);

MODULE_AUTHOR("DENSO");
MODULE_DESCRIPTION("CPLD GPIO Expander Driver for Denso Board");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
