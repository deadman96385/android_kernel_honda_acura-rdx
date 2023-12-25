/*
 * mlb150_int.h
 *
 * Copyright 2012-2013 CETITEC GmbH. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef _MLB150_INT_H
#define _MLB150_INT_H

#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/kthread.h>
#include <linux/of_device.h>

struct mlb150_ext;

/* struct members sorted by "hot-ness": frequently used parameters first */
struct mlb_data {
	void __iomem *membase;		/* mlb module base address */
	void __iomem *mlbregs;		/* start of mlb register io */
	struct mlb_dev_info *devinfo;	/* the array of channels */
	u32 actl;			/* cached ACTL value, used by AHB ISR */
	struct device *dev;

	/* cold members */
	struct class class;	/* device class */
	unsigned int irq_ahb0;	/* vIRQ numbers */
	unsigned int irq_ahb1;
	unsigned int irq_mlb;
	dev_t firstdev;
	struct cdev cdev;

	int minor_ctrl0;
	int minor_opt30;
	int minor_async0;
	int minor_isoc0;
	int minor_sync0;

	struct kthread_worker isr_wrk;
	struct task_struct *isr_task;

	atomic_t isoc_sync_on;
	struct task_struct *isoc_sync_task;
};

extern u32 number_sync_channels;
extern unsigned int number_isoc_channels;

struct mlb150_dmabuf;

struct mlb150_io_buffers;
#define MLB150_IO_BUFFERS_INIT(bufs) {  \
	.in = LIST_HEAD_INIT(bufs.in),  \
	.out = LIST_HEAD_INIT(bufs.out) \
}

int __must_check mlb150_get_tx_buffers(struct mlb_data *, int minor, struct mlb150_io_buffers *);
int __must_check mlb150_put_tx_buffers(struct mlb_data *, int minor, struct mlb150_io_buffers *);
void mlb150_free_dmabuf(struct mlb_data *, int minor, struct mlb150_dmabuf *);

int __must_check mlb150_get_tx_buffer_but_last(struct mlb_data *, int minor, struct mlb150_io_buffers *);

void mlb150_chan_setaddr(struct mlb_data *, int minor, unsigned int caddr);
int __must_check mlb150_chan_startup(struct mlb_data *, int minor, enum channelmode);
int __must_check mlb150_chan_allocate_dmabufs(struct mlb_data *, int minor,
					      enum channelmode,
					      unsigned int bytes_per_frame,
					      unsigned int sync_buf_size);
int mlb150_chan_shutdown(struct mlb_data *, int minor, enum channelmode);
int __must_check mlb150_do_open(struct mlb_data *, int minor, enum channelmode);
int __must_check mlb150_do_release(struct mlb_data *, int minor, enum channelmode);

void ext_set_drvdata(struct mlb_data *);

struct mlb150_io_buffers;

void ext_start_rx(unsigned int minor, enum mlb150_channel_type, struct mlb150_io_buffers *);
void ext_start_tx(unsigned int minor, enum mlb150_channel_type);
void ext_cleanup(unsigned int minor, enum mlb150_channel_type);

static inline
struct mlb150_dmabuf *dmabuf_of(const struct list_head *entry)
{
	return list_entry((struct list_head *)entry, struct mlb150_dmabuf, head);
}

static inline
unsigned dmabuf_count(const struct list_head *list)
{
	unsigned c = 0;
	const struct list_head *pos;

	list_for_each(pos, list)
		++c;
	return c;
}

/*
 * Target platform specific part of the driver implementation
 */
extern const struct of_device_id mlb150_of_device_ids[];
int mlb150_init_driver(struct platform_driver *, int (*probe)(struct platform_device *));
void mlb150_exit_driver(struct platform_driver *);

extern int mlb150_get_mlb_io(struct mlb_data *);
extern int mlb150_init_mlb_io(struct mlb_data *);
extern void mlb150_free_mlb_io(struct mlb_data *);

#endif /* _MLB150_INT_H */

