/**
 * drivers/extcon/extcon-usb-dummy.c - USB dummy extcon driver
 *
 * Copyright (C) 2017 Honda R&D Americas, Inc.
 *
 * Based on extcon-usb-gpio.c:
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com
 * Author: Roger Quadros <rogerq@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/extcon.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <linux/uaccess.h>
#include <linux/debugfs.h>

struct usb_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;
	struct dentry *root;
};

/* List of detectable cables */
enum {
	EXTCON_CABLE_USB = 0,
	EXTCON_CABLE_USB_HOST,
	EXTCON_CABLE_END,
};

static const char *usb_extcon_cable[] = {
	[EXTCON_CABLE_USB] = "USB",
	[EXTCON_CABLE_USB_HOST] = "USB-HOST",
	NULL,
};

static int extcon_open_show(struct seq_file *s, void *unused)
{
	return 0;
}

static int extcon_vbus_open(struct inode *inode, struct file *file)
{
	return single_open(file, extcon_open_show, inode->i_private);
}

static ssize_t extcon_vbus_write(struct file *file,
		const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct seq_file		*s = file->private_data;
	struct usb_extcon_info		*info = s->private;
	unsigned int		val;
	char			buf[32];

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	buf[sizeof(buf) - 1] = '\0';
	sscanf(buf, "%d\n",&val);

	if (val == 0 || val == 1) {
		extcon_set_cable_state(info->edev,
			usb_extcon_cable[EXTCON_CABLE_USB], val);
	}
	return count;
}

static const struct file_operations extcon_vbus_fops = {
	.open			= extcon_vbus_open,
	.write			= extcon_vbus_write,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static int extcon_id_open(struct inode *inode, struct file *file)
{
	return single_open(file, extcon_open_show, inode->i_private);
}

static ssize_t extcon_id_write(struct file *file,
		const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct seq_file		*s = file->private_data;
	struct usb_extcon_info		*info = s->private;
	unsigned int		val;
	char			buf[32];

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	buf[sizeof(buf) - 1] = '\0';
	sscanf(buf, "%d\n",&val);

	if (val == 0 || val == 1) {
		extcon_set_cable_state(info->edev,
			usb_extcon_cable[EXTCON_CABLE_USB_HOST], !val);
	}
	return count;
}

static const struct file_operations extcon_id_fops = {
	.open			= extcon_id_open,
	.write			= extcon_id_write,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static int usb_extcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct usb_extcon_info *info;
	int ret, err = 0;
	struct dentry *file;

	if (!np)
		return -EINVAL;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;

	info->edev = devm_extcon_dev_allocate(dev, usb_extcon_cable);
	if (IS_ERR(info->edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
		err = -ENOMEM;
		goto err0;
	}

	ret = devm_extcon_dev_register(dev, info->edev);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon device\n");
		err = ret;
		goto err1;
	}

	platform_set_drvdata(pdev, info);
	device_init_wakeup(dev, 1);

	info->root = debugfs_create_dir(dev_name(dev), NULL);
	if (!info->root) {
		dev_err(dev, "failed to create directory for debugfs.\n");
		ret = -ENOMEM;
		goto err2;
	}

	file = debugfs_create_file("id", S_IRUGO | S_IWUSR, info->root,
				info, &extcon_id_fops);
	if (!file) {
		dev_err(dev, "failed to create debugfs file for usb-id.\n");
		ret = -ENOMEM;
		goto err3;
	}

	file = debugfs_create_file("vbus", S_IRUGO | S_IWUSR, info->root,
				info, &extcon_vbus_fops);
	if (!file) {
		dev_err(dev, "failed to create debugfs file for usb-vbus.\n");
		ret = -ENOMEM;
		goto err3;
	}

	return 0;
err3:
	debugfs_remove_recursive(info->root);
err2:
	devm_extcon_dev_unregister(dev, info->edev);
err1:
	devm_extcon_dev_free(dev, info->edev);
err0:
	kfree(info);
	return ret;
}

static int usb_extcon_remove(struct platform_device *pdev)
{
	struct usb_extcon_info *info = platform_get_drvdata(pdev);

	debugfs_remove_recursive(info->root);
	devm_extcon_dev_unregister(info->dev, info->edev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int usb_extcon_suspend(struct device *dev)
{
	return 0;
}

static int usb_extcon_resume(struct device *dev)
{
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(usb_extcon_pm_ops,
			usb_extcon_suspend, usb_extcon_resume);

static struct of_device_id usb_extcon_dt_match[] = {
	{ .compatible = "linux,extcon-usb-dummy", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, usb_extcon_dt_match);

static struct platform_driver usb_extcon_driver = {
	.probe		= usb_extcon_probe,
	.remove		= usb_extcon_remove,
	.driver		= {
		.name	= "extcon-usb-dummy",
		.pm	= &usb_extcon_pm_ops,
		.of_match_table = usb_extcon_dt_match,
	},
};

module_platform_driver(usb_extcon_driver);

MODULE_AUTHOR("Honda R&D Americas, Inc.");
MODULE_DESCRIPTION("USB dummy extcon driver");
MODULE_LICENSE("GPL v2");
