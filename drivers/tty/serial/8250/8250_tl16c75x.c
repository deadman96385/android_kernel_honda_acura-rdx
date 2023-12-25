/*
 * Copyright (C) 2016 DENSO International America
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/irq.h>

/* devicetree headers */
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

/* serial headers */
#include <linux/serial_8250.h>
#include "8250.h"

struct tl16c75x_priv {
	int line;
	struct serial_rs485 rs485;
};

static void
tl16c75x_set_rs485(struct uart_port *port, struct serial_rs485 *rs485)
{
	struct tl16c75x_priv *priv = (struct tl16c75x_priv*)port->private_data;
	unsigned long irqflags;
	unsigned char lcr, efr, afr, mcr;
	spin_lock_irqsave(&port->lock, irqflags);
	lcr = port->serial_in(port, UART_LCR);
	/* enable enhanced features ( required ) */
	port->serial_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	efr = port->serial_in(port, UART_EFR);
	port->serial_out(port, UART_EFR, efr | UART_EFR_ECB);
	/* set afr functions */
	port->serial_out(port, UART_LCR, UART_LCR_CONF_MODE_C);
	afr = 0;
	if (priv->rs485.flags & SER_RS485_ENABLED) {
		afr |= UART_AFR_485EN;
	} else {
		afr &= ~UART_AFR_485EN;
	}
	if (priv->rs485.flags & SER_RS485_RX_DURING_TX) {
		afr |= UART_AFR_RCVEN;
	} else {
		afr &= ~UART_AFR_RCVEN;
	}
	port->serial_out(port, UART_AFR, afr);
	port->serial_out(port, UART_LCR, lcr);
	/* if the modem was put into RS485 mode
	 *   enable it.
	 */
	mcr = port->serial_in(port, UART_MCR);
	port->serial_out(port, UART_MCR, 0);
	if (afr & UART_AFR_485EN) {
		port->serial_out(port, UART_MCR, mcr | UART_MCR_DTR);
	} else {
		port->serial_out(port, UART_MCR, mcr & ~UART_MCR_DTR);
	}

	spin_unlock_irqrestore(&port->lock, irqflags);
}

static void
tl16c75x_sync_rs485(struct uart_port *port)
{
	struct tl16c75x_priv *priv = (struct tl16c75x_priv*)port->private_data;
	unsigned long irqflags;
	unsigned char lcr, efr, afr;
	spin_lock_irqsave(&port->lock, irqflags);
	lcr = port->serial_in(port, UART_LCR);
	/* enable enhanced features ( required )*/
	port->serial_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	efr = port->serial_in(port, UART_EFR);
	port->serial_out(port, UART_EFR, efr | UART_EFR_ECB);
	/* get afr functions */
	port->serial_out(port, UART_LCR, UART_LCR_CONF_MODE_C);
	afr = port->serial_in(port, UART_AFR);
	priv->rs485.flags = 0;
	if (afr & UART_AFR_485EN) {
		priv->rs485.flags |= SER_RS485_ENABLED;
	}
	if (afr & UART_AFR_RCVEN) {
		priv->rs485.flags |= SER_RS485_RX_DURING_TX;
	}
	port->serial_out(port, UART_LCR, lcr);
	spin_unlock_irqrestore(&port->lock, irqflags);
}

#if 1
static void
print_regs(struct uart_port *port)
{
	unsigned long irqflags;
	unsigned char mcr, msr, lcr, lsr, iir, ier, efr, afr, dll, dlh;

	spin_lock_irqsave(&port->lock, irqflags);
	lcr = port->serial_in(port, UART_LCR);

	port->serial_out(port, UART_LCR, 0x00);
	ier = port->serial_in(port, UART_IER);
	mcr = port->serial_in(port, UART_MCR);
	lsr = port->serial_in(port, UART_LSR);
	msr = port->serial_in(port, UART_MSR);
	iir = port->serial_in(port, UART_IIR);

	port->serial_out(port, UART_LCR, UART_LCR_CONF_MODE_A);
	dll = port->serial_in(port, UART_DLL);
	dlh = port->serial_in(port, UART_DLM);

	port->serial_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	efr = port->serial_in(port, UART_EFR);

	port->serial_out(port, UART_LCR, UART_LCR_CONF_MODE_C);
	afr = port->serial_in(port, UART_AFR);

	port->serial_out(port, UART_LCR, lcr);
	spin_unlock_irqrestore(&port->lock, irqflags);

	printk("### IIR: %04x  IER: %04x\n", iir, ier);
	printk("### MCR: %04x  LCR: %04x\n", mcr, lcr);
	printk("### MSR: %04x  LSR: %04x\n", msr, lsr);
	printk("### EFR: %04x  AFR: %04x\n", efr, afr);
	printk("### DLH: %04x  DLL: %04x\n", dlh, dll);
}
#endif

static int
tl16c75x_ioctl(struct uart_port *port,
                unsigned int cmd,
                unsigned long arg)
{
	struct tl16c75x_priv *priv;
	struct serial_rs485 rs485user;
	priv = (struct tl16c75x_priv*)port->private_data;

	memset(&rs485user, 0x00, sizeof(rs485user));
	switch (cmd) {
	case TIOCSRS485:
		if (copy_from_user(&rs485user, (struct serial_rs485 *) arg,
				sizeof(rs485user))) {
			return -EFAULT;
		} else {
			memcpy(&(priv->rs485), &rs485user, sizeof(rs485user));
			tl16c75x_set_rs485(port, &priv->rs485);
		}
	break;
	case TIOCGRS485:
		tl16c75x_sync_rs485(port);
		if (copy_to_user((struct serial_rs485 *) arg,
				&(priv->rs485),
				sizeof(priv->rs485))) {
			return -EFAULT;
		}
	break;
#if 1
	case 0x9044:
		print_regs(port);
		break;
#endif
	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

static int
tl16c75x_probe(struct platform_device *pdev)
{
	struct resource *regs;
	struct resource *irq;
	struct tl16c75x_priv *priv;
	struct uart_8250_port up;
	int ret;

	/* Check for DEV node */
	if (!pdev->dev.of_node)
		return -ENODEV;

	/* Clear UART Structure */
	memset(&up, 0, sizeof(up));

	/* Setup Platform DEV */
	up.port.dev = &pdev->dev;

	/* Setup Memory Resource */
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "missing registers\n");
		return -EINVAL;
	}
	up.port.mapbase = regs->start;

	/* Setup IRQ Resource*/
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "missing interrupts\n");
		return -EINVAL;
	}
	up.port.irq = irq->start;
	/** Set IRQ Edge Trigger - important or it masks out the interrupt */
	up.port.irqflags = IRQF_TRIGGER_RISING;

	/* Setup UART Clock */
	of_property_read_u32(pdev->dev.of_node,
                            "clock-frequency", &up.port.uartclk);
	if (!up.port.uartclk) {
		dev_err(&pdev->dev, "No clock speed specified\n");
		return -EFAULT;
	}

	/* Setup UART Line # */
	/* This doesn't actually work... ask TI? */
	up.port.line = of_alias_get_id(pdev->dev.of_node, "serial");
	if (up.port.line < 0) {
		dev_err(&pdev->dev, "failed to get alias id, errno %d\n",
							up.port.line);
		return -ENODEV;
	}

	/* Setup Port Features */
	up.port.type = PORT_TI_TL16C75X;
	up.port.iotype = UPIO_MEM;
	up.port.regshift = 0;
	up.port.flags = UPF_FIXED_TYPE | UPF_IOREMAP;

	/* setup private data */
	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	up.port.private_data = priv;

	/* Set Custom ioctl */
	up.port.ioctl = tl16c75x_ioctl;

	/* Register the Port */
	ret = serial8250_register_8250_port(&up);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register 8250 port\n");
		return -EFAULT;
	}
	priv->line = ret;
	return 0;
}

static int
tl16c75x_remove(struct platform_device *pdev)
{
	struct tl16c75x_priv *priv = platform_get_drvdata(pdev);
	serial8250_unregister_port(priv->line);
	return 0;
}

static const struct of_device_id tl16c75x_dt_ids[] = {
	{ .compatible = "ti,tl16c752-uart" },
	{ .compatible = "ti,tl16c754-uart" },
	{ /* sentinal */ },
};
MODULE_DEVICE_TABLE(of, tl16c75x_dt_ids);

static struct platform_driver tl16c75x_platform_driver = {
	.driver = {
		.name			= "TI TL16C75X",
		.of_match_table = tl16c75x_dt_ids,
		.owner		= THIS_MODULE,
	},
	.probe			= tl16c75x_probe,
	.remove			= tl16c75x_remove,
};
module_platform_driver(tl16c75x_platform_driver);

MODULE_AUTHOR("Kyle Davison");
MODULE_DESCRIPTION("8250 serial module for TL16C75X devices");
MODULE_LICENSE("GPL");
