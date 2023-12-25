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
/*
 *  This file is modified by Honda R&D Americas, Inc. between March 15, 2016
 *  and May 10, 2016.
 *
 *  All modifications made by Honda R&D Americas, Inc.
 *  are Copyright (c) 2016 Honda R&D Americas, Inc.
 *
 *  Honda R&D Americas, Inc. hereby licenses those modifications
 *  under the terms set forth in the file HONDA-NOTICE
 *  located in the root of the directory /vendor/honda.
 */

#include <linux/version.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/crc16.h>
#include <linux/ktime.h>

#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
#include <linux/sched/rt.h>
#endif

#include "mlb150.h"
#include "mlb150_ext.h"
#include "mlb150_int.h"

#define DRIVER_NAME "mlb150"
#define MLB_DRIVER_VERSION "2.0.1"

#ifdef CONFIG_MLB150_DEBUG_POISON
# define DEBUG_POISON 1
#else
# define DEBUG_POISON 0
#endif
#ifndef DEBUG_ADT
#define DEBUG_ADT 0
#endif
#ifndef DEBUG_CTR
#define	DEBUG_CTR 0
#endif
#ifndef DEBUG_REGOP
#define DEBUG_REGOP 0
#endif
#ifndef DEBUG_REGDUMP
#define DEBUG_REGDUMP 0
#endif
#ifndef DEBUG_CTRDUMP
#define DEBUG_CTRDUMP 0
#endif
#ifndef DEBUG_AHB_ISR
#define DEBUG_AHB_ISR 0
#endif
#ifndef DEBUG_MLBLOCK
#define DEBUG_MLBLOCK 0
#endif

#ifndef DMABUF_QUEUE_DEPTH_STATS
#define DMABUF_QUEUE_DEPTH_STATS 0
#endif

#ifndef MLB_RX_TX_STATISTICS
#define MLB_RX_TX_STATISTICS 1
#endif

#ifndef MLB_RX_TX_DUMP
#define MLB_RX_TX_DUMP 1
#endif

#ifndef MLB_PMP_DUMP
#define MLB_PMP_DUMP 0
#endif
#ifndef MLB_WATCHDOG_DUMP
#define MLB_WATCHDOG_DUMP 0 /* trace out watchdog exchange */
#endif

#ifndef MLB_DEBUG_FTRACE
#define MLB_DEBUG_FTRACE 0
#endif
#if MLB_DEBUG_FTRACE
#undef pr_debug
#define pr_debug trace_printk
#define diag_printk(fmt, ...) \
	trace_printk(fmt, ## __VA_ARGS__)
#else
#define diag_printk(fmt, ...) \
	printk(KERN_DEBUG fmt, ## __VA_ARGS__)
#endif

#define MLB_SYSTEM_CHANNEL	0
#define MLB_J6S4I_CHANNEL	63 /* used by workaround of no-isoc-irq on Jacinto 6 */
#define MLB_FIRST_CHANNEL	1
#define MLB_LAST_CHANNEL	63
/*!
 * MLB module memory map registers define
 */
#define REG_MLBC0		0x0
#define MLBC0_MLBEN		(0x1)
#define MLBC0_MLBCLK_MASK	(0x7 << 2)
#define MLBC0_MLBCLK_SHIFT	(2)
#define MLBC0_MLBPEN		(0x1 << 5)
#define MLBC0_MLBLK		(0x1 << 7)
#define MLBC0_ASYRETRY		(0x1 << 12)
#define MLBC0_CTLRETRY		(0x1 << 12)
#define MLBC0_FCNT_MASK		(0x7 << 15)
#define MLBC0_FCNT_SHIFT	(15)

#define REG_MLBPC0		0x8
#define MLBPC0_MCLKHYS		(0x1 << 11)

#define REG_MS0			0xC
#define REG_MS1			0x14

#define REG_MSS			0x20
#define MSS_RSTSYSCMD		(0x1)
#define MSS_LKSYSCMD		(0x1 << 1)
#define MSS_ULKSYSCMD		(0x1 << 2)
#define MSS_CSSYSCMD		(0x1 << 3)
#define MSS_SWSYSCMD		(0x1 << 4)
#define MSS_SERVREQ		(0x1 << 5)

#define REG_MSD			0x24

#define REG_MIEN		0x2C
#define MIEN_ISOC_PE		(0x1)
#define MIEN_ISOC_BUFO		(0x1 << 1)
#define MIEN_SYNC_PE		(0x1 << 16)
#define MIEN_ARX_DONE		(0x1 << 17)
#define MIEN_ARX_PE		(0x1 << 18)
#define MIEN_ARX_BREAK		(0x1 << 19)
#define MIEN_ATX_DONE		(0x1 << 20)
#define MIEN_ATX_PE		(0x1 << 21)
#define MIEN_ATX_BREAK		(0x1 << 22)
#define MIEN_CRX_DONE		(0x1 << 24)
#define MIEN_CRX_PE		(0x1 << 25)
#define MIEN_CRX_BREAK		(0x1 << 26)
#define MIEN_CTX_DONE		(0x1 << 27)
#define MIEN_CTX_PE		(0x1 << 28)
#define MIEN_CTX_BREAK		(0x1 << 29)

#define REG_MLBPC2		0x34
#define REG_MLBPC1		0x38
#define MLBPC1_VAL		(0x00000888)

#define REG_MLBC1		0x3C
#define MLBC1_LOCK		(0x1 << 6)
#define MLBC1_CLKM		(0x1 << 7)
#define MLBC1_NDA_MASK		(0xFF << 8)
#define MLBC1_NDA_SHIFT		(8)

#define REG_HCTL		0x80
#define HCTL_RST0		(0x1)
#define HCTL_RST1		(0x1 << 1)
#define HCTL_EN			(0x1 << 15)

#define REG_HCMR0		0x88
#define REG_HCMR1		0x8C
#define REG_HCER0		0x90
#define REG_HCER1		0x94
#define REG_HCBR0		0x98
#define REG_HCBR1		0x9C

#define REG_MDAT0		0xC0
#define REG_MDAT1		0xC4
#define REG_MDAT2		0xC8
#define REG_MDAT3		0xCC

#define REG_MDWE0		0xD0
#define REG_MDWE1		0xD4
#define REG_MDWE2		0xD8
#define REG_MDWE3		0xDC

#define REG_MCTL		0xE0
#define MCTL_XCMP		(0x1)

#define REG_MADR		0xE4
#define MADR_WNR		(0x1 << 31)
#define MADR_TB			(0x1 << 30) /* DS62420AP2 (March 2011) */
#define MADR_ADDR_MASK		(0x7f << 8)
#define MADR_ADDR_SHIFT		(0)

#define REG_ACTL		0x3C0
#define ACTL_MPB		(0x1 << 4)
#define ACTL_DMAMODE		(0x1 << 2)
#define ACTL_SMX		(0x1 << 1)
#define ACTL_SCE		(0x1)

#define REG_ACSR0		0x3D0
#define REG_ACSR1		0x3D4
#define REG_ACMR0		0x3D8
#define REG_ACMR1		0x3DC

#define LOGIC_CH_NUM		(64)
#define BUF_CDT_OFFSET		(0x0)
#define BUF_ADT_OFFSET		(0x40)
#define BUF_CAT_MLB_OFFSET	(0x80)
#define BUF_CAT_HBI_OFFSET	(0x88)
#define BUF_CTR_END_OFFSET	(0x8F)

#define CAT_MODE_RX		(0x1 << 0)
#define CAT_MODE_TX		(0x1 << 1)
#define CAT_MODE_INBOUND_DMA	(0x1 << 8)
#define CAT_MODE_OUTBOUND_DMA	(0x1 << 9)

/*!
 * DBR address assignment macros
 */
#define CH_CTRL_BUF_DEP		(64)
#define CH_ASYNC_BUF_DEP	(1536) /* MOST150 and MEP */
#define CH_ISOC_BLK_SIZE_MIN	(188)
#define CH_ISOC_BLK_SIZE_MAX	(196)
#define CH_ISOC_BLK_SIZE_DEFAULT CH_ISOC_BLK_SIZE_MIN

/*
 * DS62420AP2: "Isochronous buffers must be large enough to hold at least 3
 * blocks (packets) of data"
 */
#define CH_ISOC_BLK_NUM_MIN     (3)
#define CH_ISOC_BLK_NUM_DEFAULT	(8)

static unsigned sysfs_isoc_blk_size = CH_ISOC_BLK_SIZE_DEFAULT;
static unsigned sysfs_isoc_blk_num  = CH_ISOC_BLK_NUM_DEFAULT;

/* calculated and set in alloc_minor_devices() */
static unsigned dbr_size_per_isoc; /* minimum: CH_ISOC_BLK_NUM_MIN * CH_ISOC_BLK_SIZE_MAX */



#define CH_CTRL_DBR_BUF_OFFSET	(0x0)

/*
 * OS62420 Data Sheet (DS62420AP2, page 45)
 *
 * 6.1 Data Buffer RAM
 * The OS62420 requires an external Data Buffer RAM (DBR) that is 8-bit * 16k
 * entries deep.
 */
#define DBR_CAPACITY		(16*1024)

#define DBR_BUF_START 0x00000

#define CAT_CL_SHIFT		(0x0)
#define CAT_CT_SHIFT		(8)
#define CAT_CT_SYNC		(0)
#define CAT_CT_CTRL		(1)
#define CAT_CT_ASYNC		(2)
#define CAT_CT_ISOC		(3)
#define CAT_CT_RSRV4		(4)
#define CAT_CT_RSRV5		(5)
#define CAT_CT_RSRV6		(6)
#define CAT_CT_RSRV7		(7)
#define CAT_CE			(0x1 << 11)
#define CAT_RNW			(0x1 << 12)
#define CAT_MT			(0x1 << 13)
#define CAT_FCE			(0x1 << 14)
#define CAT_MFE			(0x1 << 14)

#define CDT_WSBC_SHIFT		(14)
#define CDT_WPC_SHIFT		(11)
#define CDT_RSBC_SHIFT		(30)
#define CDT_RPC_SHIFT		(27)
#define CDT_WPC_1_SHIFT		(12)
#define CDT_RPC_1_SHIFT		(28)
#define CDT_WPTR_SHIFT		(0)
#define CDT_SYNC_WSTS_MASK	(0x0000f000)
#define CDT_SYNC_WSTS_SHIFT	(12)
#define CDT_CTRL_ASYNC_WSTS_MASK	(0x0000f000)
#define CDT_CTRL_ASYNC_WSTS_IDLE_MASK	(0x0000b000) /* only value 0 is valid */
#define CDT_CTRL_ASYNC_WSTS_SHIFT	(12)
#define CDT_ISOC_WSTS_MASK	(0x0000e000)
#define CDT_ISOC_WSTS_SHIFT	(13)
#define CDT_RPTR_SHIFT		(16)
#define CDT_SYNC_RSTS_MASK	(0xf0000000)
#define CDT_SYNC_RSTS_SHIFT	(28)
#define CDT_CTRL_ASYNC_RSTS_MASK	(0xf0000000)
#define CDT_CTRL_ASYNC_RSTS_IDLE_MASK	(0xb0000000) /* only value 0 is valid */
#define CDT_CTRL_ASYNC_RSTS_SHIFT	(28)
#define CDT_ISOC_RSTS_MASK	(0xe0000000)
#define CDT_ISOC_RSTS_SHIFT	(29)
#define CDT_CTRL_ASYNC_WSTS_1	(0x1 << 14)
#define CDT_CTRL_ASYNC_RSTS_1	(0x1 << 15)
#define CDT_BD_SHIFT		(0)
#define CDT_BA_SHIFT		(16)
#define CDT_BS_SHIFT		(0)
#define CDT_BF_SHIFT		(31)

#define ADT_PG			(0x1 << 13)
#define ADT_LE			(0x1 << 14)
#define ADT_CE			(0x1 << 15)
#define ADT_BD1_SHIFT		(0)
#define ADT_ERR1		(0x1 << 13)
#define ADT_DNE1		(0x1 << 14)
#define ADT_RDY1		(0x1 << 15)
#define ADT_BD2_SHIFT		(16)
#define ADT_ERR2		(0x1 << 29)
#define ADT_DNE2		(0x1 << 30)
#define ADT_RDY2		(0x1 << 31)
#define ADT_BA1_SHIFT		(0x0)
#define ADT_BA2_SHIFT		(0x0)
#define ADT_PS1			(0x1 << 12)
#define ADT_PS2			(0x1 << 28)
#define ADT_MEP1		(0x1 << 11)
#define ADT_MEP2		(0x1 << 27)

#define  CTR_ZERO ((u32 [4]) {0,0,0,0})

#define MLB_CONTROL_DEV_NAME	"ctrl"
#define MLB_ASYNC_DEV_NAME	"async"
#define MLB_SYNC_DEV_NAME	"sync"
#define MLB_ISOC_DEV_NAME	"isoc"

#define TX_CHANNEL		0
#define RX_CHANNEL		1
/* max package data size */
#define TRANS_RING_NODES	BUF_RING_NODES
#define RX_BUF_COUNT            (TRANS_RING_NODES)
#define TX_BUF_COUNT            (10)

#define TX_FLUSH_TIMEOUT_SYNC	6 /* ms, derived from 5333 us */
#define TX_FLUSH_TIMEOUT	10 /* ms */

/* The interval of polls for DNE bits in ADT */
#define DNE_POLL_START_INT usecs_to_jiffies(500)
#define DNE_POLL_INT       usecs_to_jiffies(100)
#define DNE_POLL_TIMEOUT   (150000) /* us */

#define ISOC_SYNC_DEV(drv) ((drv)->devinfo + used_minor_devices - 1)

static const char *ctype_names[4] = {
	[MLB150_CTYPE_SYNC]  = MLB_SYNC_DEV_NAME,
	[MLB150_CTYPE_CTRL]  = MLB_CONTROL_DEV_NAME,
	[MLB150_CTYPE_ASYNC] = MLB_ASYNC_DEV_NAME,
	[MLB150_CTYPE_ISOC]  = MLB_ISOC_DEV_NAME
};

#define DMAB_RX_POISON 0xab
#define DMAB_TX_POISON 0xcd
#if DEBUG_POISON
static void wait_expire_invalid_pmp(struct mlb_dev_info *);
#else
#define poison_dmab(dmab, poison, size) do {} while(0)
#define wait_expire_invalid_pmp(pdevinfo) do { } while (0)
#endif

struct mlb_channel_info {
	/* the DMA buffers queue */
	spinlock_t dmabuf_slock; /* for `hw` and `q` */
	struct list_head hw;     /* the buffers currently used by MLB hardware */
	struct list_head q;      /* the queue of ready (rx) or empty (tx) buffers */
	struct list_head drops;  /* the list of drop space buffers */
	int pingpong;
	int closing_tx; /* requires taking free_slock */

	spinlock_t free_slock; /* for `free` */
	struct list_head free; /* the free for use buffers */

	spinlock_t usr_slock;
	struct list_head usr; /* the queue of ready buffers for the connected userspace */

#if DEBUG_POISON
	/* the buffers to check for PMP errors */
	struct list_head pmp;
	spinlock_t pmp_lock;
#endif
	/* channel address */
	u32 address;
	/* DBR reservation */
	u32 dbr_buf_head;
	unsigned int dbr_size;
	uint dbr_avail, rpc, wpc;
	int dim2_async_bug;
	/* channel DMA buffer size */
	unsigned int buf_size;

	atomic_t polling;
	struct timespec poll_start;
	struct kthread_work work;
	struct timer_list isr_timer;
	unsigned poll_cnt;
#if DMABUF_QUEUE_DEPTH_STATS
	atomic_t free_cnt; /* FIXME remove free list accounting */
#endif
};

#define MLB_DEV_NAME_SIZE 20
struct mlb_dev_info {
	/* device node name */
	char dev_name[MLB_DEV_NAME_SIZE];
	/* channel type */
	enum mlb150_channel_type channel_type;
	unsigned bytes_per_frame;
	unsigned isoc_blk_size, isoc_blk_num;
	unsigned sync_buf_size;
	/* channel info for tx/rx */
	struct mlb_channel_info channels[2];

	/* exception event */
	unsigned long ex_event;
	/* channel started up or not */
	atomic_t on;
	/* device open count */
	atomic_t opencnt;
	/* wait queue head for channel */
	wait_queue_head_t rx_wq;
	wait_queue_head_t tx_wq;
	/* spinlock for event access */
	spinlock_t event_lock;

	/*
	 * The dma_bufs[0] points to allocated DMA memory.
	 * The there is some space set aside between rx and tx buffers
	 * for dropping packets.
	 */
	size_t                dma_size;
	void		     *dma_cpu;
	dma_addr_t	      dma_handle;
	struct mlb150_dmabuf  dma_bufs[RX_BUF_COUNT + TX_BUF_COUNT];
	dma_addr_t            dma_bufs_rxtx_boundary;
	/* 4 dmabufs to serve ping-pong buffers for rx and tx */
	struct mlb150_dmabuf  dma_drop[4];
	atomic_t pauseRx;

	/* only used by mlb_channel_{enable,disable} */
	unsigned is_reading:1;
	unsigned is_writing:1;

	struct mlb_data *drvdata;

#if MLB_RX_TX_STATISTICS
	/* Statistics */
	rwlock_t stat_lock;
	long long tx_bytes, rx_bytes, rx_pkts, tx_pkts;
	long long rx_drops, tx_drops;
	struct device_attribute attr;
#endif /* MLB_RX_TX_STATISTICS */
	struct device_attribute bufsize_attr;
	struct device_attribute dumpattr;
};

static __always_inline bool dev_is_opt3(const struct mlb_dev_info *dev)
{
	/* No other device starts with 'o' */
	return (dev->dev_name[0] == 'o');
}


/* Protects all (non-statistic) sysfs members:
 * - sysfs_isoc_blk_size
 * - sysfs_isoc_blk_num
 * Only use the access functions below, never reference the variables directly!
 */
static DEFINE_SPINLOCK(sysfs_lock);

static unsigned __get_current_sysfs_val_unsigned(const unsigned *var)
{
	unsigned long flags;
	unsigned retval;

	spin_lock_irqsave(&sysfs_lock, flags);
	retval = *var;
	spin_unlock_irqrestore(&sysfs_lock, flags);

	return retval;
}

static __always_inline unsigned get_current_isoc_blk_sz(void)
{
	return __get_current_sysfs_val_unsigned(&sysfs_isoc_blk_size);
}

static __always_inline unsigned get_current_isoc_blk_num(void)
{
	return __get_current_sysfs_val_unsigned(&sysfs_isoc_blk_num);
}

/* Helper function for reading out both settings at once */
static void get_current_isoc_settings(unsigned *size, unsigned *num)
{
	unsigned long flags;

	spin_lock_irqsave(&sysfs_lock, flags);
	*size = sysfs_isoc_blk_size;
	*num  = sysfs_isoc_blk_num;
	spin_unlock_irqrestore(&sysfs_lock, flags);
}

static void set_current_isoc_blk_sz(unsigned new_value)
{
	unsigned long flags;

	spin_lock_irqsave(&sysfs_lock, flags);
	sysfs_isoc_blk_size = new_value;
	spin_unlock_irqrestore(&sysfs_lock, flags);
}

static void set_current_isoc_blk_num(unsigned new_value)
{
	unsigned long flags;

	spin_lock_irqsave(&sysfs_lock, flags);
	sysfs_isoc_blk_num = new_value;
	spin_unlock_irqrestore(&sysfs_lock, flags);
}

/* helper function */
static ssize_t sysfs_write_attribute_unsigned(char *buf, unsigned value)
{
	unsigned pos;
	buf[0] = '\0'; /* just to be sure */
	/* Writing the sysfs to buffer provided by kernel (size: PAGE_SIZE). */
	pos = scnprintf(buf, PAGE_SIZE-1, "%u ", value);

	/* Append a '\n': buf[pos] holds the zero termination printed by scnprintf(). */
	buf[  pos] = '\n';
	buf[++pos] = '\0';
	return pos;
}

static ssize_t isoc_blk_sz_show(struct device_driver *driver, char *buf)
{
	return sysfs_write_attribute_unsigned(buf, get_current_isoc_blk_sz());
}

static ssize_t isoc_blk_num_show(struct device_driver *driver, char *buf)
{
	return sysfs_write_attribute_unsigned(buf, get_current_isoc_blk_num());
}

static bool sysfs_read_attribute_unsigned(const char *buf, unsigned *val)
{
	return sscanf(buf, "%u", val) == 1;
}

static ssize_t isoc_blk_sz_store(struct device_driver *driver, const char *buf, size_t count)
{
	unsigned value_read = 0;

	if(!sysfs_read_attribute_unsigned(buf, &value_read))
		return -EINVAL;
	
	/* Check range */
	if (value_read < CH_ISOC_BLK_SIZE_MIN ||
	    value_read > CH_ISOC_BLK_SIZE_MAX) {
		pr_warn("isoc_blk_sz_store: Invalid block "
			"size of %u!\n", value_read);
		return -ERANGE;
	}

	set_current_isoc_blk_sz(value_read);
	return count;
}

static ssize_t isoc_blk_num_store(struct device_driver *driver, const char *buf, size_t count)
{
	unsigned value_read = 0;

	if(!sysfs_read_attribute_unsigned(buf, &value_read))
		return -EINVAL;

	/* Validate: Upper limit: available DBR size
	 * Lower limit: hardware limit
	 */
	if (value_read < CH_ISOC_BLK_NUM_MIN ||
	    /* safe compare as value_read is non-trusted from userspace */
	    value_read > dbr_size_per_isoc ||
	    value_read * CH_ISOC_BLK_SIZE_MAX > dbr_size_per_isoc) {
		pr_warn("isoc_blk_num_store: Invalid number of blocks "
			"(%u). Max: %u\n", value_read,
			dbr_size_per_isoc/CH_ISOC_BLK_SIZE_MAX);
		return -ERANGE;
	}

	set_current_isoc_blk_num(value_read);
	return count;
}

static DRIVER_ATTR_RW(isoc_blk_sz);
static DRIVER_ATTR_RW(isoc_blk_num);

static void update_isoc_buf_size(struct mlb_dev_info *dev, int minor)
{
	/* update channel values with current ones from sysfs variables */
	get_current_isoc_settings(&dev->isoc_blk_size,
				  &dev->isoc_blk_num);

	/* recalculate buffer sizes using new values */
	dev->channels[TX_CHANNEL].buf_size = dev->isoc_blk_size * dev->isoc_blk_num;
	dev->channels[RX_CHANNEL].buf_size = dev->isoc_blk_size * dev->isoc_blk_num;

	/* inform extensions, too. */
	mlb150_ext_set_isoc_params(minor, dev->isoc_blk_size,
					  dev->isoc_blk_num);
}


static inline bool is_drop_buf(const struct mlb_dev_info *pdevinfo, const struct mlb150_dmabuf *dmab)
{
	return pdevinfo->dma_drop->phys == dmab->phys;
}

/* returns 0, 1, or 2, the latter of course meaning "2+": current depth of hw */
static inline unsigned hwbuf_count(const struct list_head *head)
{
	return !list_empty(head) + (head->next->next != head);
}

static inline bool has_free_hw_buf(const struct mlb_channel_info *ci)
{
	return hwbuf_count(&ci->hw) < 2;
}

#if DMABUF_QUEUE_DEPTH_STATS
static void free_list_size_inc(struct mlb_channel_info *ci)
{
	atomic_inc(&ci->free_cnt);
}

static void free_list_size_dec(struct mlb_channel_info *ci)
{
	atomic_dec(&ci->free_cnt);
}

static void free_list_size_set(struct mlb_channel_info *ci)
	__must_hold(&ci->free_slock)
{
	atomic_set(&ci->free_cnt, dmabuf_count(&ci->free));
}
/* dmabuf_slock must be held because of accessing hw and q */
static void trace_free_dmabufs(const char *func, struct mlb_dev_info *pdevinfo,
			       int rxtx, const struct list_head *detached)
	__must_hold(&pdevinfo->channels[rxtx].dmabuf_slock)
{
	struct mlb_channel_info *ci = &pdevinfo->channels[rxtx];
	unsigned hwcnt = hwbuf_count(&ci->hw);
	char detcnt[24];

	if (detached)
		snprintf(detcnt, sizeof(detcnt), "+%u", dmabuf_count(detached));
	else
		*detcnt = '\0';

	pr_debug("%s: hw %lx %lx, q %2u, usr %2u, free %2d%s\n",
		 func,
		 hwcnt > 0 ? (long)dmabuf_of(ci->hw.next)->phys : 0,
		 hwcnt > 1 ? (long)dmabuf_of(ci->hw.next->next)->phys : 0,
		 dmabuf_count(&ci->q),
		 dmabuf_count(&ci->usr),
		 atomic_read(&ci->free_cnt),
		 detcnt);
}
#else
static void free_list_size_inc(struct mlb_channel_info *ci) {}
static void free_list_size_dec(struct mlb_channel_info *ci) {}
static void free_list_size_set(struct mlb_channel_info *ci) {}
static void trace_free_dmabufs(const char *func, struct mlb_dev_info *pdevinfo,
			       int rxtx, const struct list_head *detached) {}
#endif

static void update_stats(struct mlb_dev_info *pdevinfo, int rxtx, unsigned pkt_size);
static void update_pmp_stats(struct mlb_dev_info *pdevinfo, int rxtx, const void *pmh);
static inline void update_decoded_stats(struct mlb_dev_info *devinfo, int rxtx, const void *pkt)
{
	if ((devinfo->channel_type == MLB150_CTYPE_CTRL ||
	     devinfo->channel_type == MLB150_CTYPE_ASYNC) && pkt)
		update_pmp_stats(devinfo, rxtx, pkt);
	else
		update_stats(devinfo, rxtx, devinfo->channels[rxtx].buf_size);
}
static void update_drop_stats(struct mlb_dev_info *pdevinfo, int rxtx);

static void init_mlb_channel_info_dma(struct mlb_channel_info *ci)
{
	INIT_LIST_HEAD(&ci->free);
	free_list_size_set(ci);
	INIT_LIST_HEAD(&ci->hw);
	INIT_LIST_HEAD(&ci->q);
	INIT_LIST_HEAD(&ci->drops);
	INIT_LIST_HEAD(&ci->usr);
	ci->pingpong = 0;
	ci->closing_tx = 0;
}

static void __iomem *mlb_base;		/* mlb module base address */

static unsigned int used_minor_devices; /* not defined until probe */

static u32 read_mlb_reg(u32 mlb_reg, const char *name)
{
	u32 reg = readl(mlb_base + mlb_reg);
#if DEBUG_REGOP
	pr_debug("MLB R: %s 0x%08x => %08x\n", name, mlb_reg, reg);
#endif
	return reg;
}

#define READ_MLB_REG(mlb_reg) \
	read_mlb_reg(mlb_reg, #mlb_reg)

static void write_mlb_reg(u32 value, u32 mlb_reg, const char *valname, const char *regname)
{
#if DEBUG_REGOP
	pr_debug("MLB W: %s 0x%08x <= %s %08x\n", regname, mlb_reg, valname, value);
#endif
	writel(value, mlb_base + mlb_reg);
}

#define WRITE_MLB_REG(value, mlb_reg) \
	write_mlb_reg(value, mlb_reg, #value, #mlb_reg)

static int __must_check wait_mlb_mctl(unsigned timeout)
{
	unsigned t = timeout;

	while ((!(readl(mlb_base + REG_MCTL) & MCTL_XCMP))
		&& --t)
		;
#if DEBUG_REGOP
	pr_debug("MLB wait %u: MCTL XCMP is set in %u\n", timeout, timeout - t);
#endif
	return t ? 0 : -ETIME;
}

/* default number of sync channels which is used
   if module is loaded without parameters. */
u32 number_sync_channels = 7;
module_param(number_sync_channels, int, 0444);

/* number of isochronous channels to provide by default */
unsigned int number_isoc_channels = 1;
module_param_named(isoc_channels, number_isoc_channels, uint, 0444);

static unsigned int opt3_flag = 1;
module_param_named(opt3, opt3_flag, uint, 0444);
/*
 * Used to work around the problem with no isoc AHB interrupts after disabling
 * a sync channel
 */
#define DEFAULT_ISOC_SYNC_INTERVAL 10 /* ms */
static int isoc_sync_quirk = 1;
module_param(isoc_sync_quirk, int, 0444);
MODULE_PARM_DESC(isoc_sync_quirk,
	"Set to 1 to start triggering isoc interrupts by sending a sync packet");

#define DEBUG_TRACE_CTRL  (1 << 0)
#define DEBUG_TRACE_ASYNC (1 << 1)
#define DEBUG_TRACE_SYNC  (1 << 2)
#define DEBUG_TRACE_ISOC  (1 << 3)
#define DEBUG_TRACE_FTRACE (1 << 4)
static unsigned int debug_mask;
module_param_named(debug, debug_mask, uint, 0644);

static unsigned int underrun_mode = 1;
module_param_named(underrun, underrun_mode, uint, 0644);
MODULE_PARM_DESC(underrun,
	"0: no underrun handling on sync channels, 1: generate silence stream");

static unsigned int irq_cpu = CONFIG_MLB150_CPU_AFFINITY;
module_param(irq_cpu, int, 0644);
MODULE_PARM_DESC(irq_cpu,
	"The CPU used for handling the interrupt and worker thread. -1 for all CPUs.");

static DEFINE_SPINLOCK(ctr_lock);

#define debug_channel(pdevinfo) \
	((pdevinfo->channel_type == MLB150_CTYPE_ISOC && \
	  (debug_mask & DEBUG_TRACE_ISOC)) || \
	 (pdevinfo->channel_type == MLB150_CTYPE_CTRL && \
	  (debug_mask & DEBUG_TRACE_CTRL)) || \
	 (pdevinfo->channel_type == MLB150_CTYPE_ASYNC && \
	  (debug_mask & DEBUG_TRACE_ASYNC)) || \
	 (pdevinfo->channel_type == MLB150_CTYPE_SYNC && \
	  (debug_mask & DEBUG_TRACE_SYNC)))

#define DEBUG_LLD_DUMP (DEBUG_TRACE_ISOC|DEBUG_TRACE_SYNC|DEBUG_TRACE_ASYNC|DEBUG_TRACE_CTRL)
#define lld_dump(channel, buf, count, fmt, ...) do { \
	if (debug_mask & DEBUG_LLD_DUMP) \
		lld_dump_(channel, buf, count, fmt, ##__VA_ARGS__); \
	} while (0)
#define pr_lld_dump(fmt, ...) do { \
	if (debug_mask & DEBUG_TRACE_FTRACE) \
		trace_printk(fmt, ##__VA_ARGS__); \
	else \
		pr_debug(fmt, ##__VA_ARGS__); \
	} while (0)

#define DIAG_DBR    (1 << 0)
#define DIAG_CTR    (1 << 1)
#define DIAG_RX_DMABUF (1 << 2)
#define DIAG_TX_DMABUF (1 << 3)
#define DIAG_DMABUF (DIAG_RX_DMABUF | DIAG_TX_DMABUF)
static void dump_mlb_diag(const struct mlb_dev_info *pdevinfo, unsigned flags);

static void lld_dump_(const struct mlb_dev_info *channel, const void *buf, size_t count, const char *fmt, ...)
{
#if MLB_RX_TX_DUMP
	char msg[80], msg2[80] = "";
	char xline[3 * 32 + 1];
	unsigned length = min_t(size_t, sizeof(xline) / 3, count);
	va_list args;

	va_start(args, fmt);
	vsnprintf(msg, sizeof(msg), fmt, args);
	va_end(args);

	*xline = '\0';

	if (channel->channel_type == MLB150_CTYPE_ISOC && (debug_mask & DEBUG_TRACE_ISOC)) {
		unsigned int o;

		for (o = 0; o + channel->isoc_blk_size < count; o += channel->isoc_blk_size) {
			length = sizeof(xline) / 3;
			if (length > count - o)
				length = count - o;
			hex_dump_to_buffer(buf + o, length, sizeof(xline) / 3, 1, xline, sizeof(xline), false);
			pr_lld_dump("%s %s:[%zd] %s%s\n", msg, channel->dev_name,
				    count, xline, length < count ? "..." : "");
		}

		length = sizeof(xline) / 3;

		if (length > count - o)
			length = count - o;

		hex_dump_to_buffer(buf + o, length, sizeof(xline) / 3, 1, xline, sizeof(xline), false);

	} else if ((channel->channel_type == MLB150_CTYPE_CTRL && (debug_mask & DEBUG_TRACE_CTRL)) ||
		   (channel->channel_type == MLB150_CTYPE_ASYNC && (debug_mask & DEBUG_TRACE_ASYNC))) {

		const __be16 *PMH = buf; /* dma buffers should be aligned enough */
		unsigned pml = be16_to_cpup(PMH);

		if (pml < 3) {
			pr_lld_dump("%s: invalid PMP packet: PML %u\n", channel->dev_name, pml);
			dump_mlb_diag(channel, 0);
		} else if (pml < count - 2) {
			pml += 2;
			count = pml;
			length = min(length, pml);
			if (length < count)
				sprintf(msg2, " crc %04x", crc16(0, buf, count));
		}
		hex_dump_to_buffer(buf, length, sizeof(xline) / 3, 1, xline, sizeof(xline), false);

	} else if (channel->channel_type == MLB150_CTYPE_SYNC && (debug_mask & DEBUG_TRACE_SYNC)) {
		sprintf(msg2, " crc %04x", crc16(0, buf, count));
		length = count;
		if (length > 16)
			length = 16;
		hex_dump_to_buffer(buf, length, 16, 1, xline, sizeof(xline), false);
	} else
		goto noprint;
	pr_lld_dump("%s %s:[%zd] %s%s%s\n", msg, channel->dev_name,
		    count, xline, length < count ? "..." : "", msg2);
noprint: ; /* continue to PMP/Watchdog decoding, if enabled */
#endif
#if MLB_PMP_DUMP || MLB_WATCHDOG_DUMP
	if ((channel->channel_type == MLB150_CTYPE_CTRL ||
	     channel->channel_type == MLB150_CTYPE_ASYNC) &&
	    count >= 6 /* shortest Watchdog.Start */) {
		unsigned pml = be16_to_cpup(buf);
		const char *m = "";
		char fktbuf[8] = ".?";
		char op[4] = ".?";
		const char *fkt = fktbuf;
		/*
		 * 00 06 - PML
		 * 01    - PMHL (header length)
		 * 14    - PMHB (header body: FIFONo 010 - ICM FIFO, FIFOMT 10 - Data)
		 * PMB: Control Message
		 * 30 9x - FktID 0x309
		 * x0    - OpType 0 (Start)
		 * 0x    - TelID 0 (as required for ICM)
		 * x0 00 - Length (0)
		 */
		if (pml < 6)
			m = NULL;
		else {
			const u8 *PMH = buf + 2;
			unsigned pmhl = PMH[0];

			if (pmhl <= pml - 1 && /* must be inside the PMP message */
			    pmhl <= count - 3 /* must be inside the passed buffer */) {
				switch (PMH[1 /* PMHB */] & 0x38) { /* FIFONo */
				case 0:
					m = "MCM";
					break;
				case 1 << 3:
					m = "MDP";
					break;
				case 2 << 3:
					m = "ICM";
					break;
				case 3 << 3:
					m = "ALL";
					break;
				case 4 << 3:
					m = "MEP";
					break;
				default:
					m = NULL;
				}
#if !MLB_PMP_DUMP
				m = NULL;
#endif
				if ((PMH[1 /* PMHB */] & 0x3E) == 0x14 /* ICM Data */) {
					/* This is called FktIdOp in SMSC/Microchip documentation */
					unsigned fnidop = (PMH[1 + pmhl] << 8) | PMH[1 + pmhl + 1];

					switch (fnidop & 0xfff0) {
					case 0x3090:
						m = "ICM";
						fkt = ".Watchdog";
						break;
					default:
						sprintf(fktbuf, ".%03x", (fnidop & 0xfff0) >> 4);
					}
					sprintf(op, ".%x", fnidop & 0xf);
				}
			}
		}
		if (m)
			pr_lld_dump("%s %s: %s%s%s\n", msg, channel->dev_name, m, fkt, op);
	}
#endif
}

static struct {
	const char name[7], nl;
	uint offs; /* to mlb_base */
} mlb_registers[] = {
#define DEF_REG(n) { .name = #n, .nl = __LINE__&1, .offs = REG_##n}
	DEF_REG(MLBC0), DEF_REG(MLBPC0),
	DEF_REG(MS0), DEF_REG(MS1),
	DEF_REG(MSS), DEF_REG(MSD),
	DEF_REG(MIEN),
	DEF_REG(MLBPC2), DEF_REG(MLBPC1), DEF_REG(MLBC1),
	DEF_REG(HCTL), DEF_REG(HCMR0), DEF_REG(HCMR1),
	DEF_REG(HCER0), DEF_REG(HCER1), DEF_REG(HCBR0), DEF_REG(HCBR1),
	DEF_REG(MCTL), DEF_REG(MADR),
	DEF_REG(MDAT0), DEF_REG(MDAT1), DEF_REG(MDAT2), DEF_REG(MDAT3),
	DEF_REG(MDWE0), DEF_REG(MDWE1), DEF_REG(MDWE2), DEF_REG(MDWE3),
	DEF_REG(ACTL), // DEF_REG(ACSR0), DEF_REG(ACSR1),
	DEF_REG(ACMR0), DEF_REG(ACMR1),
#undef  DEF_REG
};
/* this does not require ctr_lock on purpose: so that it can be used anywhere */
static void collect_registers(u32 *val)
{
	uint i;

	for (i = 0; i < ARRAY_SIZE(mlb_registers); ++i)
		val[i] = readl(mlb_base + mlb_registers[i].offs);
}

#if DEBUG_REGDUMP

#define DUMP_REG(reg) pr_debug(#reg": 0x%08x\n", READ_MLB_REG(reg))

static void dump_registers(void)
{
	u32 regs[ARRAY_SIZE(mlb_registers)];
	uint i;

	collect_registers(regs);
	pr_debug("Dump registers:\n");
	for (i = 0; i < ARRAY_SIZE(mlb_registers); ++i)
		pr_debug("REG_%s: 0x%08x\n", mlb_registers[i].name, regs[i]);
}
#else
static void dump_registers(void) {}
#endif

/*
 * format the lower 16 bits of `v` in hex and return the end of the result in
 * `buf`
 */
static inline char *fmt_u16(char *buf, unsigned v)
{
	static const char hex[16] = "0123456789abcdef";
	*buf++ = hex[v >> 12 & 0xf];
	*buf++ = hex[v >>  8 & 0xf];
	*buf++ = hex[v >>  4 & 0xf];
	*buf++ = hex[v       & 0xf];
	return buf;
}
/*
 * Format an ADT entry value.
 * Requires 38 bytes in `buf` for full value.
 */
#define ADT_STS_DNE_FMT(adt) \
	adt_sts_1_fmt((char [10]){0}, (adt)[1])
#define ADT_STS_FMT(adt) \
	adt_sts_fmt((char [20]){0}, (adt))
#define ADT_FMT(adt, ba1, ba2) \
	adt_fmt((char [40]){0}, (adt), (ba1), (ba2))

static inline char *adt_sts_1_fmt(char *buf, const u32 adt1)
{
	char *s = buf;

	if (adt1) {
		buf = fmt_u16(buf, adt1);
		*buf++ = ' ';
		buf = fmt_u16(buf, adt1 >> 16);
	} else {
		*buf++ = '-';
		*buf++ = '-';
	}
	*buf++ = '\0';
	return s;
}

static inline char *adt_sts_fmt(char *buf, const u32 *adt)
{
	char *s = buf;

	buf = fmt_u16(buf, adt[0]);
	if (!(adt[0] & 0xfff))
		buf[-1] = buf[-2] = buf[-3] = '-';
	*buf++ = ' ';
	if (adt[0] >> 16)
		buf = fmt_u16(buf, adt[0] >> 16);
	else
		*buf++ = '-';
	*buf++ = ' ';
	adt_sts_1_fmt(buf, adt[1]);
	return s;
}

static inline char *adt_fmt(char *buf, const u32 *adt, u32 ba1, u32 ba2)
{
	char *s = adt_sts_fmt(buf, adt);

	buf += strlen(s);
	*buf++ = ' ';

	buf = fmt_u16(buf, adt[2] >> 16);
	buf = fmt_u16(buf, adt[2]);
	*buf++ = ba1 && adt[2] == ba1 ? '*' : ' ';

	buf = fmt_u16(buf, adt[3] >> 16);
	buf = fmt_u16(buf, adt[3]);
	if (ba2 && adt[3] == ba2)
		*buf++ = '*';
	*buf = '\0';
	return s;
}

#define MDWE_DISABLE_ALL ((u32 [4]) {0, 0, 0, 0})
#define MDWE_ENABLE_ALL  ((u32 [4]) {0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff})

static inline void set_ctr_write_mask(const u32 *enable)
	__must_hold(&ctr_lock)
{
	WRITE_MLB_REG(enable[0], REG_MDWE0);
	WRITE_MLB_REG(enable[1], REG_MDWE1);
	WRITE_MLB_REG(enable[2], REG_MDWE2);
	WRITE_MLB_REG(enable[3], REG_MDWE3);
}

/* ctr_lock MUST be held when calling this */
static int __must_check read_ctr_nolock(u32 ctr_offset, u32 *ctr_val)
	__must_hold(&ctr_lock)
{
	WRITE_MLB_REG(0, REG_MCTL);

	WRITE_MLB_REG(ctr_offset, REG_MADR);

	if (unlikely(wait_mlb_mctl(1000))) {
		pr_warn(DRIVER_NAME ": Read CTR@0x%x timeout\n", ctr_offset);
		return -ETIME;
	}

	ctr_val[0] = READ_MLB_REG(REG_MDAT0);
	ctr_val[1] = READ_MLB_REG(REG_MDAT1);
	ctr_val[2] = READ_MLB_REG(REG_MDAT2);
	ctr_val[3] = READ_MLB_REG(REG_MDAT3);


	return 0;
}

static int __must_check read_ctr(u32 ctr_offset, u32 *ctr_val)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&ctr_lock, flags);

	ret = read_ctr_nolock(ctr_offset, ctr_val);

	spin_unlock_irqrestore(&ctr_lock, flags);

	return ret;
}

static u32 read_cdt_rpc_nolock(uint address)
	__must_hold(&ctr_lock)
{
	u32 cdt[2];

	WRITE_MLB_REG(0, REG_MCTL);
	WRITE_MLB_REG(BUF_CDT_OFFSET + address, REG_MADR);
	if (unlikely(wait_mlb_mctl(1000))) {
		pr_warn(DRIVER_NAME ": read CDT 0x%x timeout\n", address);
		return ~(u32)0;
	}
	cdt[0] = READ_MLB_REG(REG_MDAT0);
	cdt[1] = READ_MLB_REG(REG_MDAT1);
	return ((cdt[0] >> CDT_RPC_SHIFT) & 0x1f) |
	       ((cdt[1] >> CDT_RPC_1_SHIFT) & 0x7) << 5;
}

static u32 read_cdt_rpc(uint address)
{
	u32 rpc;
	unsigned long flags;

	spin_lock_irqsave(&ctr_lock, flags);
	rpc = read_cdt_rpc_nolock(address);
	spin_unlock_irqrestore(&ctr_lock, flags);
	return rpc;
}

static int dim2_async_bug(struct mlb_channel_info *ci)
	__must_hold(&ci->dmabuf_slock)
{
	u32 rpc = read_cdt_rpc(ci->address);

	while (rpc != (ci->rpc & 0xff)) {
		ci->dbr_avail += ci->buf_size;
		ci->rpc++;
	}
	if (ci->dbr_avail < ci->buf_size)
		goto bug;
	if (ci->wpc < ci->rpc || ci->wpc - ci->rpc == 0xff) {
		pr_debug("unexplained rpc: %u (%u), wpc: %u\n",
			 ci->rpc, rpc, ci->wpc);
		goto bug;
	}
	ci->dbr_avail -= ci->buf_size;
	ci->wpc++;
	return 0;
bug:
	ci->dim2_async_bug = 1;
	return -EAGAIN;
}

/* ctr_lock MUST be held when calling this */
static int __must_check write_ctr_nolock(u32 ctr_offset, const u32 ctr_val[4]) __must_hold(&ctr_lock)
{
	WRITE_MLB_REG(0, REG_MCTL);

	WRITE_MLB_REG(ctr_val[0], REG_MDAT0);
	WRITE_MLB_REG(ctr_val[1], REG_MDAT1);
	WRITE_MLB_REG(ctr_val[2], REG_MDAT2);
	WRITE_MLB_REG(ctr_val[3], REG_MDAT3);

	WRITE_MLB_REG(MADR_WNR | ctr_offset, REG_MADR);

	if (unlikely(wait_mlb_mctl(1000))) {
		pr_warn(DRIVER_NAME": Write CTR@0x%x timeout\n", ctr_offset);
		return -ETIME;
	}
	return 0;
}

/* this function will erase the given part of the DBR */
static int reset_dbr(u32 dbr_offset, u32 dbr_size)
{
	u32 i;
	unsigned long flags;

	pr_debug("DBR reset offs 0x%08x size %u\n", dbr_offset, dbr_size);

	spin_lock_irqsave(&ctr_lock, flags);
	for (i = 0; i < dbr_size; i++) {
		WRITE_MLB_REG(0, REG_MCTL);
		WRITE_MLB_REG(0, REG_MDAT0);
		WRITE_MLB_REG(MADR_WNR | MADR_TB |
			(dbr_offset + i), REG_MADR);

		/* mask is not used for DBR access */

		if (unlikely(wait_mlb_mctl(1000))) {
			pr_warn(DRIVER_NAME": Write CTR timeout\n");
			spin_unlock_irqrestore(&ctr_lock, flags);
			return -ETIME;
		}
		/*
		 * This is a long operation, so give others (and IRQ!) a
		 * chance.
		 */
		if (i % 500 == 0) {
			spin_unlock_irqrestore(&ctr_lock, flags);
			spin_lock_irqsave(&ctr_lock, flags);
		}
	}
	spin_unlock_irqrestore(&ctr_lock, flags);

	return 0;
}

static int __must_check write_ctr(u32 ctr_offset, const u32 ctr_val[4])
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&ctr_lock, flags);

	ret = write_ctr_nolock(ctr_offset, ctr_val);

#if DEBUG_CTR
	if (0 == ret) {
		u32 ctr_rd[4] = { 0 };

		if (!read_ctr_nolock(ctr_offset, ctr_rd)) {
			if (!(ctr_val[0] == ctr_rd[0] &&
			      ctr_val[1] == ctr_rd[1] &&
			      ctr_val[2] == ctr_rd[2] &&
			      ctr_val[3] == ctr_rd[3])) {
				pr_debug("ctr write failed\n");
				ret = -EBADE;
			}
		} else {
			pr_debug("ctr read failed\n");
			ret = -EBADE;
		}
	}
#endif
	spin_unlock_irqrestore(&ctr_lock, flags);

	return ret;
}

#if DEBUG_CTR
/* ctr_lock MUST be held when calling this */
static int __must_check read_cat_nolock(u32 ctr_offset, u32 ch, u16 *cat_val)
	__must_hold(&ctr_lock)
{
	u16 ctr_val[8] __aligned(4) = { 0 };

	if (read_ctr_nolock(ctr_offset, (u32 *)ctr_val))
		return -ETIME;

	/*
	 * Use u16 array to get u32 array value,
	 * need to convert
	 * */
	*cat_val = ctr_val[ch % 8];

	 return 0;
}
#endif

static int __must_check write_cat_nolock(u32 ctr_offset, u32 ch, const u16 cat_val)
	__must_hold(&ctr_lock)
{
	u16 ctr_val[8] __aligned(4) = { 0 };

	if (unlikely(read_ctr_nolock(ctr_offset, (u32 *)ctr_val)))
		return -ETIME;

	ctr_val[ch % 8] = cat_val;
	if (unlikely(write_ctr_nolock(ctr_offset, (u32 *)ctr_val)))
		return -ETIME;

	return 0;
}

static int get_adt_status(u32 ch, u32 reg[4], const unsigned int mask)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ctr_lock, flags);
	WRITE_MLB_REG(0, REG_MCTL);
	WRITE_MLB_REG(BUF_ADT_OFFSET + ch, REG_MADR);
	if (unlikely(wait_mlb_mctl(1000))) {
		ret = -ETIME;
		goto unlock;
	}
	ret = 0;
	if (mask & BIT(0))
		reg[0] = READ_MLB_REG(REG_MDAT0);
	if (mask & BIT(1))
		reg[1] = READ_MLB_REG(REG_MDAT1);
	if (mask & BIT(2))
		reg[2] = READ_MLB_REG(REG_MDAT2);
	if (mask & BIT(3))
		reg[3] = READ_MLB_REG(REG_MDAT3);
unlock:
	spin_unlock_irqrestore(&ctr_lock, flags);
#if DEBUG_ADT
	pr_debug("Get ch %d adt sts: 0x%08x\n", ch, reg);
#endif
	return ret;
}

static void dump_ctr(u32 ch_start, u32 ch_end)
{
	u32 i;
	u32 ctr_val[4] = { 0 };

	printk(KERN_DEBUG "CDT Table 0x%02x-0x%02x\n",
	       BUF_CDT_OFFSET + ch_start,
	       BUF_CDT_OFFSET + ch_end - 1);
	for (i = BUF_CDT_OFFSET + ch_start;
	     i < BUF_CDT_OFFSET + ch_end; ++i) {
		if (read_ctr_nolock(i, ctr_val))
			break;
		printk(KERN_DEBUG "CTR 0x%02x: %08x %08x %08x %08x\n",
		       i, ctr_val[3], ctr_val[2], ctr_val[1], ctr_val[0]);
	}

	printk(KERN_DEBUG "ADT Table 0x%02x-0x%02x\n",
	       BUF_ADT_OFFSET + ch_start,
	       BUF_ADT_OFFSET + ch_end - 1);
	for (i = BUF_ADT_OFFSET + ch_start;
	     i < BUF_ADT_OFFSET + ch_end; ++i) {
		if (read_ctr_nolock(i, ctr_val))
			break;
		printk(KERN_DEBUG "CTR 0x%02x: %08x %08x %08x %08x\n",
		       i, ctr_val[3], ctr_val[2], ctr_val[1], ctr_val[0]);
	}

	printk(KERN_DEBUG "CAT MLB Table 0x%02x-0x%02x\n",
	       BUF_CAT_MLB_OFFSET + (ch_start >> 3),
	       BUF_CAT_MLB_OFFSET + (ch_end >> 3));
	for (i = BUF_CAT_MLB_OFFSET + (ch_start >> 3);
	     i <= BUF_CAT_MLB_OFFSET + (ch_end >> 3); ++i) {
		if (read_ctr_nolock(i, ctr_val))
			break;
		printk(KERN_DEBUG "CTR 0x%02x: %08x %08x %08x %08x\n",
		       i, ctr_val[3], ctr_val[2], ctr_val[1], ctr_val[0]);
	}

	printk(KERN_DEBUG "CAT HBI Table 0x%02x-0x%02x\n",
	       BUF_CAT_HBI_OFFSET + (ch_start >> 3),
	       BUF_CAT_HBI_OFFSET + (ch_end >> 3));
	for (i = BUF_CAT_HBI_OFFSET + (ch_start >> 3);
	     i <= BUF_CAT_HBI_OFFSET + (ch_end >> 3); ++i) {
		if (read_ctr_nolock(i, ctr_val))
			break;
		printk(KERN_DEBUG "CTR 0x%02x: %08x %08x %08x %08x\n",
		       i, ctr_val[3], ctr_val[2], ctr_val[1], ctr_val[0]);
	}
}

static void dump_dbr(u32 offset, unsigned int length)
{
	char bytes[64];
	char prefix[16];
	unsigned used = 0;

	for (; length--; offset++) {
		u32 timeout = 1000;

		WRITE_MLB_REG(MADR_TB | offset, REG_MADR);
		while ((!(READ_MLB_REG(REG_MCTL) & MCTL_XCMP))
			&& --timeout)
			;
		if (!timeout) {
			pr_warn(DRIVER_NAME": Read DBR timeout\n");
			return;
		}
		bytes[used++] = READ_MLB_REG(REG_MDAT0) & 0xff;
		if (32 == used) {
			sprintf(prefix, "DBR 0x%04x: ", offset - used + 1);
			print_hex_dump(KERN_DEBUG, prefix, DUMP_PREFIX_NONE,
				       32, 1, bytes, used, false);
			used = 0;
		}
	}
	if (!used)
		return;
	sprintf(prefix, "DBR 0x%04x:_", offset - used + 1);
	print_hex_dump(KERN_DEBUG, prefix, DUMP_PREFIX_NONE, 32, 1,
		       bytes, used, false);
}

static int dump_channel_mlb_diag(const struct mlb_dev_info *pdevinfo,
				 const char *comment)
{
	uint dbr_last = 0;
	int active = atomic_read(&pdevinfo->on);
	const struct mlb_channel_info *r = &pdevinfo->channels[RX_CHANNEL];
	const struct mlb_channel_info *t = &pdevinfo->channels[TX_CHANNEL];
	const char *ann;
	char args[32];

	dbr_last = max(dbr_last, t->dbr_buf_head + t->buf_size);
	dbr_last = max(dbr_last, r->dbr_buf_head + r->buf_size);
	if (MLB150_CTYPE_ISOC == pdevinfo->channel_type) {
		ann = " blocks: ";
		snprintf(args, sizeof(args), "%u/%u",
			 pdevinfo->isoc_blk_num, pdevinfo->isoc_blk_size);
	} else if (MLB150_CTYPE_SYNC == pdevinfo->channel_type) {
		ann = " bpf: ";
		snprintf(args, sizeof(args), "%u", pdevinfo->bytes_per_frame);
	} else {
		ann = "";
		*args = '\0';
	}
	diag_printk("MLB %6s tx 0x%x 0x%04x/%u; rx 0x%x 0x%04x/%u;%s%s%s%s\n",
		    pdevinfo->dev_name,
		    t->address, t->dbr_buf_head, t->buf_size,
		    r->address, r->dbr_buf_head, r->buf_size,
		    ann, args,
		    active ? "" : " (inactive)",
		    comment);
	return active;
}

static void dump_channel_dmabuf_diag(const struct mlb_dev_info *pdevinfo,
				     const struct mlb_channel_info *ci,
				     const char *comment)
{
	const struct mlb150_dmabuf *dmab;

	diag_printk("  %s: hw %d q %d usr %d free %d\n", comment,
		    dmabuf_count(&ci->hw),
		    dmabuf_count(&ci->q),
		    dmabuf_count(&ci->usr),
		    dmabuf_count(&ci->free));
	list_for_each_entry(dmab, &ci->hw, head)
		diag_printk("  %s: hw %lx\n", comment, (ulong)dmab->phys);
	list_for_each_entry(dmab, &ci->q, head)
		diag_printk("  %s:  q %lx\n", comment, (ulong)dmab->phys);
}
/*
 * This one intentionally takes no additional locks: it is
 * usually an emergency situation and locks may be not reliable.
 */
static void dump_mlb_diag(const struct mlb_dev_info *pdevinfo, unsigned flags)
{
	uint i, dbr_last = 0;
	u32 ctr[4], regs[ARRAY_SIZE(mlb_registers)];
	struct mlb_data *drvdata = pdevinfo->drvdata;

	collect_registers(regs);
	diag_printk("MLB DIAGNOSTICS (%s)\n", pdevinfo->dev_name);
	/* MLB registers */
	for (i = 0; i < ARRAY_SIZE(mlb_registers);) {
		char line[80];
		int nl = mlb_registers[i].nl, pos = 0;

		for (; mlb_registers[i].nl == nl &&
		     i < ARRAY_SIZE(mlb_registers) && pos < sizeof(line); ++i)
			pos += snprintf(line + pos, sizeof(line) - pos,
					" %6s=%08x",
					mlb_registers[i].name, regs[i]);
		diag_printk("%s\n", line);
	}
	/* Channels information */
	diag_printk("# name tx-channel dbr-offs dbr-size rx-channel dbr offs dbr size bpf/blocks active\n");
	for (i = 0; i < used_minor_devices; ++i) {
		struct mlb_dev_info *d = &drvdata->devinfo[i];
		struct mlb_channel_info *r = &d->channels[RX_CHANNEL];
		struct mlb_channel_info *t = &d->channels[TX_CHANNEL];
		int active = dump_channel_mlb_diag(d,
			d == pdevinfo ? " (trigger)": "");

		if (d == pdevinfo && (flags & DIAG_DMABUF)) {
			if (flags & DIAG_RX_DMABUF)
				dump_channel_dmabuf_diag(d, r, "rx");
			if (flags & DIAG_TX_DMABUF)
				dump_channel_dmabuf_diag(d, t, "tx");
		}
		if (!active)
			continue;
		if (read_ctr_nolock(BUF_CDT_OFFSET + r->address, ctr) == 0)
			diag_printk("CDT rx: ctr ofs 0x%02x: %08x %08x %08x %08x\n",
				    BUF_CDT_OFFSET + r->address,
				    ctr[3], ctr[2], ctr[1], ctr[0]);
		if (read_ctr_nolock(BUF_CDT_OFFSET + t->address, ctr) == 0)
			diag_printk("CDT TX: ctr ofs 0x%02x: %08x %08x %08x %08x\n",
				    BUF_CDT_OFFSET + t->address,
				    ctr[3], ctr[2], ctr[1], ctr[0]);
		if (read_ctr_nolock(BUF_ADT_OFFSET + r->address, ctr) == 0)
			diag_printk("ADT rx: ctr ofs 0x%02x: %08x %08x %08x %08x\n",
				    BUF_ADT_OFFSET + r->address,
				    ctr[3], ctr[2], ctr[1], ctr[0]);
		if (read_ctr_nolock(BUF_ADT_OFFSET + t->address, ctr) == 0)
			diag_printk("ADT TX: ctr ofs 0x%02x: %08x %08x %08x %08x\n",
				    BUF_ADT_OFFSET + t->address,
				    ctr[3], ctr[2], ctr[1], ctr[0]);
	}
	if (flags & DIAG_CTR)
		dump_ctr(0, 63);
	diag_printk("DBR use: %u of %d%s\n", dbr_last, DBR_CAPACITY,
		    dbr_last > DBR_CAPACITY ? " OVERRUN!" : "");
	/* Dump DBR for the failed channel */
	diag_printk("DBR TX ch %u offs 0x%04x, %u bytes\n",
	       pdevinfo->channels[TX_CHANNEL].address,
	       pdevinfo->channels[TX_CHANNEL].dbr_buf_head,
	       pdevinfo->channels[TX_CHANNEL].buf_size);
	if (flags & DIAG_DBR)
		dump_dbr(pdevinfo->channels[TX_CHANNEL].dbr_buf_head,
			 pdevinfo->channels[TX_CHANNEL].buf_size);
	diag_printk("DBR RX ch %u offs 0x%04x, %u bytes\n",
		    pdevinfo->channels[RX_CHANNEL].address,
		    pdevinfo->channels[RX_CHANNEL].dbr_buf_head,
		    pdevinfo->channels[RX_CHANNEL].buf_size);
	if (flags & DIAG_DBR)
		dump_dbr(pdevinfo->channels[RX_CHANNEL].dbr_buf_head,
			 pdevinfo->channels[RX_CHANNEL].buf_size);
	diag_printk("END OF MLB DIAGNOSTICS (%s)\n", pdevinfo->dev_name);
}

static void dump_driver_diag(struct mlb_dev_info *pdevinfo,
			     const char *text, size_t len)
{
	struct mlb_dev_info *d;
	uint cnt,  i;
	char line[80];

	diag_printk(DRIVER_NAME": DIAGNOSTICS (%s)%.*s\n", pdevinfo->dev_name, len, text);
	*line = '\0';
	if (!pdevinfo)
		goto end_of_diag;
	if (pdevinfo->channel_type == MLB150_CTYPE_CTRL) {
		d = pdevinfo->drvdata->devinfo;
		cnt = used_minor_devices;
	} else {
		d = pdevinfo;
		cnt = 1;
	}
	for (i = 0; i < cnt; ++d, ++i) {
		ulong dmabuf_flags, free_flags;
		struct mlb_channel_info *ci;
		int active = atomic_read(&d->on);

		if (!active) {
			if (strlen(line) + strlen(d->dev_name) < sizeof(line) - 1)
				strcat(strcat(line, " "), d->dev_name);
			else {
				diag_printk("  inactive%s\n", line);
				*line = '\0';
			}
			continue;
		}
		diag_printk("  %u/%u %s\n", i, cnt, d->dev_name);
		ci = &d->channels[RX_CHANNEL];
		spin_lock_irqsave(&ci->dmabuf_slock, dmabuf_flags);
		spin_lock_irqsave(&ci->free_slock, free_flags);
		dump_channel_dmabuf_diag(d, ci, "rx");
		spin_unlock_irqrestore(&ci->free_slock, free_flags);
		spin_unlock_irqrestore(&ci->dmabuf_slock, dmabuf_flags);
		ci = &d->channels[TX_CHANNEL];
		spin_lock_irqsave(&ci->dmabuf_slock, dmabuf_flags);
		spin_lock_irqsave(&ci->free_slock, free_flags);
		dump_channel_dmabuf_diag(d, ci, "tx");
		spin_unlock_irqrestore(&ci->free_slock, free_flags);
		spin_unlock_irqrestore(&ci->dmabuf_slock, dmabuf_flags);
	}
end_of_diag:
	if (*line)
		diag_printk("  inactive%s\n", line);
	diag_printk(DRIVER_NAME": END OF DIAGNOSTICS (%s)\n", pdevinfo->dev_name);
}

/*!
 * Initial the MLB module device
 */
static inline void disable_dma_irq(void)
{
	WRITE_MLB_REG(0, REG_ACMR0);
	WRITE_MLB_REG(0, REG_ACMR1);
}

/* channel specific irq enable */
static inline void enable_dma_irq_ch(u32 ch, bool enable)
	 __must_hold(&ctr_lock)
{
	u32 acmr_val;
	u32 acmr_adr = ch < 32 ? REG_ACMR0 : REG_ACMR1;

	acmr_val = READ_MLB_REG(acmr_adr);

	if (enable)
		acmr_val |= (u32)1 << (ch % 32);
	else
		acmr_val &= ~((u32)1 << (ch % 32));

	WRITE_MLB_REG(acmr_val, acmr_adr);

	pr_debug("ACMR%d: 0x%08x: %sabled %u\n",
		 acmr_adr == REG_ACMR1, acmr_val,
		 enable ? "en": "dis", ch);
}

static void init_ahb_irq(struct mlb_data *drvdata)
{
	u32 reg;

	/* Step 1. Program the ACMRn registers to enable interrupts from all
	 * active DMA channels */
	/* Well. Don't do this here but in mlb_channel_enable for each channel */

	/* Step 2. Select the status clear method:
	 * ACTL.SCE = 0, hardware clears on read
	 * We only support DMA MODE 1 */

	reg = READ_MLB_REG(REG_ACTL);
	reg &= ~ACTL_SCE;     /* Software clear enable: hardware clears on read */
	reg |= ACTL_DMAMODE;  /* DMA Mode: DMA Mode 1, full AMBA compatibility */
	reg &= ~ACTL_MPB;     /* packet buffering mode: single-packet */

	/* Step 3. Select 1 or 2 interrupt signals:
	 * ACTL.SMX = 0: one interrupt for channels 0 - 31 on ahb_init[0]
	 *      and another interrupt for channels 32 - 63 on ahb_init[1]
	 * ACTL.SMX = 1: single interrupt all channels on ahb_init[0]
	 */
	reg &= ~ACTL_SMX; /* AHB interrupt mux enable: disabled, ACSR0->ahb_int[0], ACSR1->ahb_int[1] */

	WRITE_MLB_REG(reg, REG_ACTL);
	drvdata->actl = reg;
}

static inline void enable_mlb_irq(u32 enable)
{
	/* Step 1, Select the MSn to be cleared by software,
	 * writing a '0' to the appropriate bits */
	WRITE_MLB_REG(0, REG_MS0);
	WRITE_MLB_REG(0, REG_MS1);

	/*
	 * Program MIEN to enable error interrupts for all active MLB channels
	 * and for transfer completion on the asynchronous channel (needed to
	 * work around a bug in DIM2 where it sometimes corrupts sync tx
	 * during concurrent async tx).
	 */
	if (enable)
		WRITE_MLB_REG(MIEN_CTX_PE | MIEN_CRX_PE |
			      MIEN_ATX_PE | MIEN_ARX_PE | MIEN_ATX_DONE |
			      MIEN_SYNC_PE | MIEN_ISOC_PE | MIEN_ISOC_BUFO,
			      REG_MIEN);
	else
		WRITE_MLB_REG(0, REG_MIEN);
}

/* complete CDT reset */
static void reset_cdt(void)
{
	int i;

	for (i = 0; i < LOGIC_CH_NUM; ++i)
		if (write_ctr(BUF_CDT_OFFSET + i, CTR_ZERO))
			break;
}

static int init_channel_cdt(const struct mlb_dev_info *pdevinfo, int direction)
	 __must_hold(&ctr_lock)
{
	const enum mlb150_channel_type ctype = pdevinfo->channel_type;
	const u32 ch = pdevinfo->channels[direction].address;
	const u32 dbr_buf_head = pdevinfo->channels[direction].dbr_buf_head;

	u32 cdt_val[4] = { 0 };

	pr_debug("channel %s %u %s, DBR@0x%08x\n", ctype_names[ctype], ch,
		 direction == RX_CHANNEL ? "rx": "tx", dbr_buf_head);

	/* a. Set the 14-bit base address (BA) */
	cdt_val[3] = dbr_buf_head << CDT_BA_SHIFT;

	/* b. Set the 12-bit or 13-bit buffer depth (BD)
	 * BD = buffer depth in bytes - 1 */

	switch (ctype) {
	case MLB150_CTYPE_SYNC:
		/* For synchronous channels: (BD + 1) = 4 * m * bpf (12bit) */
		cdt_val[3] |= (SYNC_BUFFER_DEP(pdevinfo->bytes_per_frame) - 1)
			<< CDT_BD_SHIFT;
		break;
	case MLB150_CTYPE_CTRL:
		/* For control channels: (BD + 1) >= max packet length (64) */
		/* BD */
		cdt_val[3] |= ((pdevinfo->channels[direction].dbr_size - 1) << CDT_BD_SHIFT);
		break;
	case MLB150_CTYPE_ASYNC:
		/* For asynchronous channels: (BD + 1) >= max packet length
		 * 1024 (MOST50) or 1536 (MOST150) for a MOST Data packet (MDP)
		 * 1536 for a MOST Ethernet Packet (MEP) */
		cdt_val[3] |= ((pdevinfo->channels[direction].dbr_size - 1) << CDT_BD_SHIFT);
		break;
	case MLB150_CTYPE_ISOC:
		/* For isochronous channels: Blocksize must be at least 3.
		 * Buffer depth must be an integer multiple of blocksize.
		 */
		BUG_ON(pdevinfo->isoc_blk_num < 3);

		/* BS */
		cdt_val[1] |= (pdevinfo->isoc_blk_size - 1);
		/* BD */
		cdt_val[3] |= (pdevinfo->isoc_blk_num * pdevinfo->isoc_blk_size - 1)
			<< CDT_BD_SHIFT;
		break;
	}

	pr_debug("Set CDT val of channel %u %s: 0x%08x 0x%08x 0x%08x 0x%08x\n",
		 ch, ctype_names[ctype], cdt_val[3], cdt_val[2], cdt_val[1], cdt_val[0]);

	if (unlikely(write_ctr_nolock(BUF_CDT_OFFSET + ch, cdt_val)))
		return -ETIME;

#if DEBUG_CTR
	{
		u32 cdt_rd[4] = { 0 };

		if (likely(!read_ctr_nolock(BUF_CDT_OFFSET + ch, cdt_rd))) {
			pr_debug("CDT val of channel %u %s: 0x%08x 0x%08x 0x%08x 0x%08x\n",
				 ch, ctype_names[ctype], cdt_rd[3], cdt_rd[2], cdt_rd[1],
				 cdt_rd[0]);
			if (cdt_rd[3] == cdt_val[3] &&
			    cdt_rd[2] == cdt_val[2] &&
			    cdt_rd[1] == cdt_val[1] &&
			    cdt_rd[0] == cdt_val[0]) {
				pr_debug("set cdt succeed!\n");
				return 0;
			}
			pr_debug("set cdt failed!\n");
			return -EBADE;
		}
		pr_debug("Read CDT val of channel %u failed\n", ch);
		return -EBADE;
	}
#endif
	return 0;
}

/* ctr_lock MUST be held when calling this */
static int init_channel_cat(u32 ch, u32 cat_mode, enum mlb150_channel_type ctype)
{
	static const u32 ctype_cat[] = {
		[MLB150_CTYPE_SYNC]  = CAT_CT_SYNC,
		[MLB150_CTYPE_CTRL]  = CAT_CT_CTRL,
		[MLB150_CTYPE_ASYNC] = CAT_CT_ASYNC,
		[MLB150_CTYPE_ISOC]  = CAT_CT_ISOC,
	};

	u16 cat_val;
#if DEBUG_CTR
	u16 cat_rd;
#endif

	cat_val = CAT_CE | (ctype_cat[ctype] << CAT_CT_SHIFT) | ch;

	if (cat_mode & CAT_MODE_OUTBOUND_DMA)
		cat_val |= CAT_RNW;

	if (MLB150_CTYPE_SYNC == ctype) {
		cat_val |= CAT_MT;
#ifdef MLB_USE_MFE
		/* enable multi-frame sub-buffering */
		cat_val |= CAT_MFE;
#endif
	}

#ifdef MLB_USE_FCE
	if (MLB150_CTYPE_ISOC == ctype) {
		cat_val |= CAT_FCE;
		pr_debug("enabling FCE on channel %u %s\n", ch, ctype_names[ctype]);
	}
#endif

	switch (cat_mode) {
	case CAT_MODE_RX | CAT_MODE_INBOUND_DMA:
	case CAT_MODE_TX | CAT_MODE_OUTBOUND_DMA:
		pr_debug("set CAT val of channel %d %s: 0x%04x\n",
			 ch, ctype_names[ctype], cat_val);

		if (unlikely(write_cat_nolock(BUF_CAT_MLB_OFFSET + (ch >> 3), ch, cat_val)))
			return -ETIME;
#if DEBUG_CTR
		if (likely(!read_cat_nolock(BUF_CAT_MLB_OFFSET + (ch >> 3), ch, &cat_rd)))
			pr_debug("CAT val of mlb channel %d: 0x%04x\n",
				 ch, cat_rd);
		else {
			pr_debug("Read CAT of mlb channel %d failed\n", ch);
			return -EBADE;
		}
#endif
		break;
	case CAT_MODE_TX | CAT_MODE_INBOUND_DMA:
	case CAT_MODE_RX | CAT_MODE_OUTBOUND_DMA:
		pr_debug("set HBI CAT val of channel %d %s: 0x%04x\n",
			 ch, ctype_names[ctype], cat_val);
		if (unlikely(write_cat_nolock(BUF_CAT_HBI_OFFSET + (ch >> 3), ch, cat_val)))
			return -ETIME;
#if DEBUG_CTR
		if (likely(!read_cat_nolock(BUF_CAT_HBI_OFFSET + (ch >> 3), ch, &cat_rd)))
			pr_debug("CAT val of hbi channel %d: 0x%04x\n", ch, cat_rd);
		else {
			pr_debug("Read CAT of hbi channel %d failed\n", ch);
			return -EBADE;
		}
#endif
		break;
	default:
		return -EBADRQC;
	}

#if DEBUG_CTR
	if (cat_val == cat_rd)
		pr_debug("set cat succeed!\n");
	else {
		pr_debug("set cat failed!\n");
		return -EBADE;
	}
#endif
	return 0;
}

/* complete CAT reset */
static void reset_cat(void)
{
	int i;

	for (i = 0; i < (LOGIC_CH_NUM >> 3); ++i) {
		if (write_ctr(BUF_CAT_MLB_OFFSET + i, CTR_ZERO) ||
		    write_ctr(BUF_CAT_HBI_OFFSET + i, CTR_ZERO))
			break;
	}
}

/* channel specific ctr reset */
/* ctr_lock MUST be held when calling this; */
/* ctr write must be unmasked */
static inline int reset_channel_ctr(u32 ch)
	 __must_hold(&ctr_lock)
{
	u16 ctr_val_16 = 0;

	if (write_cat_nolock(BUF_CAT_MLB_OFFSET + (ch >> 3), ch,
			     ctr_val_16))
		goto timeout;
	if (write_cat_nolock(BUF_CAT_HBI_OFFSET + (ch >> 3), ch,
			     ctr_val_16))
		goto timeout;
	/* no need to reset cdt */

	/* amba */
	if (write_ctr_nolock(BUF_ADT_OFFSET + ch, CTR_ZERO))
		goto timeout;

	return 0;
timeout:
	return -ETIME;
}

/* CTR write must be unmasked accordingly */
static void init_rfb(const struct mlb_dev_info *pdevinfo, int direction)
	 __must_hold(&ctr_lock)
{
	const enum mlb150_channel_type ctype = pdevinfo->channel_type;
	const u32 ch            = pdevinfo->channels[direction].address;
	const unsigned cat_mode = (direction == RX_CHANNEL ? CAT_MODE_RX :
							     CAT_MODE_TX);

	/* Step 1, Initialize all bits of CAT to '0' */
	/* not necessary here; done on probe/channel disable */
	/* Step 2, Initialize logical channel */
	/* Step 3, Program the CDT for channel N */
	init_channel_cdt(pdevinfo, direction);
	/* Step 4, Program the CAT for the inbound DMA */
	init_channel_cat(ch, cat_mode | CAT_MODE_INBOUND_DMA, ctype);
	/* Step 5, Program the CAT for the outbound DMA */
	init_channel_cat(ch, cat_mode | CAT_MODE_OUTBOUND_DMA, ctype);
}

/* complete ADT reset */
static void reset_adt(void)
{
	int i;

	for (i = 0; i < LOGIC_CH_NUM; ++i)
		if (write_ctr(BUF_ADT_OFFSET + i, CTR_ZERO))
			break;
}

/* complete CTR reset; only useful for init */
static void reset_ctr(void)
{
	set_ctr_write_mask(MDWE_ENABLE_ALL);
	reset_cdt();
	reset_adt();
	reset_cat();
}

static void reset_all_registers(void)
{
	uint i;

	for (i = 0; i < ARRAY_SIZE(mlb_registers); ++i)
		WRITE_MLB_REG(0, mlb_registers[i].offs);
}

static int init_channel_adt(struct mlb_channel_info *chinfo, enum mlb150_channel_type ctype)
{
#if DEBUG_ADT || DEBUG_CTR
	char adtext[40];
#endif
	u32 ctr_val[4] = { 0 };

	/* a. Set the 32-bit base address (BA1) */
	/* no need to do this; RDY is not set either */

	ctr_val[1] = (chinfo->buf_size - 1) << ADT_BD1_SHIFT;
	ctr_val[1] |= (chinfo->buf_size - 1) << ADT_BD2_SHIFT;

	/*
	 * Single-packet buffering mode: set PS bit for every ctrl/async
	 * buffer
	 */
	if (MLB150_CTYPE_ASYNC == ctype || MLB150_CTYPE_CTRL == ctype)
		ctr_val[1] |= ADT_PS1 | ADT_PS2;

	ctr_val[0] |= ADT_LE | ADT_CE;

#if DEBUG_ADT
	pr_debug("set ADT val of channel %d %s: %s\n",
		 chinfo->address, ctype_names[ctype],
		 adt_fmt(adtext, ctr_val, 0));
#endif

	if (unlikely(write_ctr_nolock(BUF_ADT_OFFSET + chinfo->address, ctr_val)))
		return -ETIME;

#if DEBUG_CTR
	{
		u32 ctr_rd[4] = { 0 };

		if (likely(!read_ctr_nolock(BUF_ADT_OFFSET + chinfo->address, ctr_rd))) {
			pr_debug("init ADT val of channel %d %s: %s\n",
				 chinfo->address, ctype_names[ctype],
				 adt_fmt(adtext, ctr_rd, 0));
			if (ctr_rd[3] == ctr_val[3] &&
			    ctr_rd[2] == ctr_val[2] &&
			    ctr_rd[1] == ctr_val[1] &&
			    ctr_rd[0] == ctr_val[0]) {
				pr_debug("init adt succeeded!\n");
				return 0;
			}
			pr_debug("init adt failed!\n");
			return -EBADE;
		}
		pr_debug("Read ADT val of channel %d failed\n", chinfo->address);
		return -EBADE;
	}
#endif

	return 0;
}

static void mlb150_dev_exit(void)
{
	disable_dma_irq();
	enable_mlb_irq(0);

	WRITE_MLB_REG(0, REG_HCTL);
	WRITE_MLB_REG(0, REG_MLBC0);

	WRITE_MLB_REG(0x0, REG_HCMR0);
	WRITE_MLB_REG(0x0, REG_HCMR1);
}

static void mlb150_dev_init(struct mlb_data *drvdata)
{
	u32 c0_val = 0;

	/* reset the CTR (here!!) */
	reset_ctr();

	/* reset all registers */
	reset_all_registers();

	/* Step 1, Configure the MediaLB interface */

	c0_val |= MLBC0_MLBEN;

#ifdef MLB_USE_MFE
	/* set FCNT to 64 frames per sub-buffer (b110) */
	c0_val |= FCNT_VALUE << MLBC0_FCNT_SHIFT;
#endif

	WRITE_MLB_REG(c0_val, REG_MLBC0);

	/* Step 2, Configure the HBI interface */
	WRITE_MLB_REG(0xffffffff, REG_HCMR0);
	WRITE_MLB_REG(0xffffffff, REG_HCMR1);
	WRITE_MLB_REG(HCTL_EN, REG_HCTL);

	/* setup ahb channel interrupts (but don't enable yet) */
	init_ahb_irq(drvdata);

	/* enable protocol error interrupts for all channels */
	enable_mlb_irq(1);
}

/* ctr_lock MUST be held when calling this */
static int unmute_sync_channel(u32 rx_ch, u32 tx_ch, enum channelmode mode) __must_hold(&ctr_lock)
{
	u32 timeout = 10000;
	u32 mfe = 0;

	/* Check that MediaLB clock is running (MLBC1.CLKM = 0)
	 * If MLBC1.CLKM = 1, clear the register bit, wait one
	 * APB or I/O clock cycle and repeat the check */
	while ((readl(mlb_base + REG_MLBC1) & MLBC1_CLKM)
	       && --timeout)
		WRITE_MLB_REG(~MLBC1_CLKM, REG_MLBC1);

	if (unlikely(0 == timeout))
		goto timeout;

	timeout = 10000;
	/* Poll for MLB lock (MLBC0.MLBLK = 1) */
	while (!(readl(mlb_base + REG_MLBC0) & MLBC0_MLBLK)
	       && --timeout)
		;

	if (unlikely(0 == timeout))
		goto timeout;

#ifdef MLB_USE_MFE
	/* enable multi-frame sub-buffering */
	mfe |= CAT_MFE;
#endif

	/* Unmute synchronous channel(s) */
	if (is_reading(mode)) {
		if (write_cat_nolock(BUF_CAT_MLB_OFFSET + (rx_ch >> 3),
				     rx_ch, mfe | CAT_CE | rx_ch))
			goto timeout;
		if (write_cat_nolock(BUF_CAT_HBI_OFFSET + (rx_ch >> 3),
				     rx_ch, mfe | CAT_CE | rx_ch | CAT_RNW))
			goto timeout;
	}
	if (is_writing(mode)) {
		if (write_cat_nolock(BUF_CAT_MLB_OFFSET + (tx_ch >> 3),
				     tx_ch, mfe | CAT_CE | tx_ch | CAT_RNW))
			goto timeout;
		if (write_cat_nolock(BUF_CAT_HBI_OFFSET + (tx_ch >> 3),
				     tx_ch, mfe | CAT_CE | tx_ch))
			goto timeout;
	}

	return 0;
timeout:
	return -ETIME;
}

static void free_channel_dmabuf(struct mlb_channel_info *ci, struct mlb150_dmabuf *dmab)
{
	unsigned long flags;

	spin_lock_irqsave(&ci->free_slock, flags);
	dmab->flags = DMABUF_FREE;
	list_add(&dmab->head, &ci->free);
	free_list_size_inc(ci);
	spin_unlock_irqrestore(&ci->free_slock, flags);
}

static struct mlb_channel_info *get_dmabuf_channel_info(struct mlb_dev_info *pdevinfo, struct mlb150_dmabuf *dmab)
{
	return dmab->phys < pdevinfo->dma_bufs_rxtx_boundary ?
		&pdevinfo->channels[RX_CHANNEL] : &pdevinfo->channels[TX_CHANNEL];
}

static void free_dmabuf(struct mlb_dev_info *pdevinfo, struct mlb150_dmabuf *dmab)
{
	free_channel_dmabuf(get_dmabuf_channel_info(pdevinfo, dmab), dmab);
}

void mlb150_free_dmabuf(struct mlb_data *drvdata, int minor, struct mlb150_dmabuf *dmab)
{
	free_dmabuf(&drvdata->devinfo[minor], dmab);
}

/*!
 * MLB receive start function
 */
static int mlb_start_channel_rx(struct mlb_dev_info *pdevinfo, u32 dne_sts,
				dma_addr_t ba1, dma_addr_t ba2)
{
#if DEBUG_ADT
	int rret;
	char adtext[40];
	u32 adt1[4] = { 0 };
#endif
	const bool ctrl_async =
		pdevinfo->channel_type == MLB150_CTYPE_CTRL ||
		pdevinfo->channel_type == MLB150_CTYPE_ASYNC;
	int ret;
	unsigned long flags;
	u32 adt[4] = { 0 };
	u32 adt_mask[4] = { 0 };
	struct mlb_channel_info *pchinfo = &pdevinfo->channels[RX_CHANNEL];

	/* Setup ADT channel entry for RX */

	if (dne_sts & ADT_DNE1) {
		adt_mask[1] |= ADT_RDY1 | ADT_DNE1 | ADT_ERR1;
		adt_mask[2] = 0xffffffff;
		adt[1] |= ADT_RDY1;
		if (ctrl_async) {
			adt_mask[1] |= ADT_PS1 | ADT_MEP1;
			adt[1] |= ADT_PS1;
		}
		adt[2] = ba1; /* BA1 */
	}
	if (dne_sts & ADT_DNE2) {
		adt_mask[1] |= ADT_RDY2 | ADT_DNE2 | ADT_ERR2;
		adt_mask[3] = 0xffffffff;
		adt[1] |= ADT_RDY2;
		if (ctrl_async) {
			adt_mask[1] |= ADT_PS2 | ADT_MEP2;
			adt[1] |= ADT_PS2;
		}
		adt[3] = ba2; /* BA2 */
	}
	if (unlikely((dne_sts & (ADT_DNE1 | ADT_DNE2)) == 0)) {
		pr_debug(DRIVER_NAME": ADT %d %s: no buffer\n",
			 pchinfo->address, pdevinfo->dev_name);
		return -EBUSY;
	}

#if DEBUG_ADT
	pr_debug("set ADT channel %d %s: %s\n",
		 pchinfo->address, pdevinfo->dev_name,
		 adt_fmt(adtext, adt, ba1, ba2));
	pr_debug("set MDWE        %d %s: %s\n",
		 pchinfo->address, pdevinfo->dev_name,
		 adt_fmt(adtext, adt_mask, 0xffffffff, 0xffffffff));
#endif

	spin_lock_irqsave(&ctr_lock, flags);
	set_ctr_write_mask(adt_mask);
	ret = write_ctr_nolock(BUF_ADT_OFFSET + pchinfo->address, adt);
#if DEBUG_ADT
	// Test if the read back value is consistent with the written
	rret = read_ctr_nolock(BUF_ADT_OFFSET + pchinfo->address, adt1);
#endif
	spin_unlock_irqrestore(&ctr_lock, flags);
#if DEBUG_ADT
	if (rret == 0 &&
	    ((adt1[0] & adt_mask[0]) != (adt[0] & adt_mask[0]) ||
	     (adt1[1] & adt_mask[1]) != (adt[1] & adt_mask[1]) ||
	     (adt1[2] & adt_mask[2]) != (adt[2] & adt_mask[2]) ||
	     (adt1[3] & adt_mask[3]) != (adt[3] & adt_mask[3]))) {
		pr_debug("%s/%u: ADT set : %s/%s\n",
			 pdevinfo->dev_name, pchinfo->address,
			 adt_fmt(adtext, adt, ba1, ba2),
			 ADT_FMT(adt_mask, 0xffffffff, 0xffffffff));
		pr_debug("%s/%u: ADT read: %s\n",
			 pdevinfo->dev_name, pchinfo->address,
			 adt_fmt(adtext, adt1, ba1, ba2));
	}
#endif

	return ret;
}

/*!
 * MLB transmit start function
 */
static int __must_check mlb_start_channel_tx(struct mlb_dev_info *pdevinfo, dma_addr_t buf_addr, u32 adt_sts)
{
#if DEBUG_ADT
	char adtext[40];
#endif
	struct mlb_channel_info *tx_ci = &pdevinfo->channels[TX_CHANNEL];
	const u32 ps_mask =
		pdevinfo->channel_type == MLB150_CTYPE_CTRL ||
		pdevinfo->channel_type == MLB150_CTYPE_ASYNC ?
		ADT_PS1 | ADT_PS2 : 0;
	unsigned long flags;
	u32 adt_mask[4] = { 0 };
	u32 adt[4] = { 0 };
	int ret;

	spin_lock_irqsave(&ctr_lock, flags);

#if DEBUG_ADT
	ret = read_ctr_nolock(BUF_ADT_OFFSET + tx_ci->address, adt);
	if (ret)
		goto skip;

	adt_sts = adt[1];
	if (likely(adt_sts & (ADT_DNE1 | ADT_DNE2)))
		pr_debug("%d %s: TX buffer done, adt %s\n",
			 tx_ci->address, pdevinfo->dev_name,
			 adt_fmt(adtext, adt, 0xffffffff));
	else if (adt_sts & (ADT_RDY1|ADT_RDY2)) {
		pr_debug("%d %s: TX busy\n",
			 tx_ci->address, pdevinfo->dev_name);
		ret = -EBUSY;
		goto unlock;
	}
	if (adt[0] & ADT_PG) {
		/* pong (2) buffer page is active */
		if ((adt_sts & ADT_RDY2) && !(adt_sts & ADT_DNE2))
			pr_warn(DRIVER_NAME": %d %s: active pong tx buffer busy\n",
				tx_ci->address, pdevinfo->dev_name);
	} else {
		/* ping (1) buffer page is active */
		if ((adt_sts & ADT_RDY1) && !(adt_sts & ADT_DNE1))
			pr_warn(DRIVER_NAME": %d %s: active ping buffer busy\n",
				tx_ci->address, pdevinfo->dev_name);
	}
skip:
#endif
	/*
	 * Setup ADT for TX: clear DNEx and ERRx, set PSx/MEPx for ctrl and
	 * async. Use the ping buffer (RDY1) if there is no ready or done
	 * buffers.
	 */
	adt[1] = ADT_RDY1 | ADT_RDY2 | ps_mask;

	if (tx_ci->pingpong == 1) {
		adt_mask[1] |= ADT_RDY2 | ADT_DNE2 | ADT_ERR2 | (ps_mask & ADT_PS2);
		adt_mask[3] = 0xffffffff;
		adt[3] = buf_addr; /* BA2 */
	} else {
		adt_mask[1] |= ADT_RDY1 | ADT_DNE1 | ADT_ERR1 | (ps_mask & ADT_PS1);
		adt_mask[2] = 0xffffffff;
		adt[2] = buf_addr; /* BA1 */
	}
	tx_ci->pingpong = !tx_ci->pingpong;
#if DEBUG_ADT
	pr_debug("%d %s: set ADT channel: %s\n",
		 tx_ci->address, pdevinfo->dev_name,
		 adt_fmt(adtext, adt, buf_addr));
	pr_debug("%d %s: set MDWE       : %s\n",
		 tx_ci->address, pdevinfo->dev_name,
		 adt_fmt(adtext, adt_mask, 0xffffffff));
#endif
	set_ctr_write_mask(adt_mask);
	ret = write_ctr_nolock(BUF_ADT_OFFSET + tx_ci->address, adt);
#if DEBUG_ADT
	if (read_ctr_nolock(BUF_ADT_OFFSET + tx_ci->address, adt) == 0) {
		pr_debug("%d %s: ack ADT channel: %s\n",
			 tx_ci->address, pdevinfo->dev_name,
			 adt_fmt(adtext, adt, buf_addr));
	}
unlock:
#endif
	spin_unlock_irqrestore(&ctr_lock, flags);

	return ret;
}

static void get_free_buffer(struct mlb_channel_info *ci, struct list_head *out)
	__must_hold(&ci->free_slock)
{
	list_move(ci->free.next, out);
	free_list_size_dec(ci);
}

static int __must_check get_tx_buffers(struct mlb_dev_info *pdevinfo, struct mlb150_io_buffers *bufs)
{
	struct mlb_channel_info *ci = &pdevinfo->channels[TX_CHANNEL];
	unsigned long flags;
	bool oob, closing;

	spin_lock_irqsave(&ci->free_slock, flags);
	oob = (closing = ci->closing_tx) || list_empty(&ci->free);
	if (!oob) {
		list_splice_init(&ci->free, &bufs->in);
		free_list_size_set(ci);
	}
	spin_unlock_irqrestore(&ci->free_slock, flags);

	return oob ? (closing ? -ESHUTDOWN : -EAGAIN) : 0;
}

static int __must_check get_tx_buffer(struct mlb_dev_info *pdevinfo, struct mlb150_io_buffers *bufs)
{
	struct mlb_channel_info *ci = &pdevinfo->channels[TX_CHANNEL];
	unsigned long flags;
	bool oob, closing;

	spin_lock_irqsave(&ci->free_slock, flags);
	oob = (closing = ci->closing_tx) || list_empty(&ci->free);
	if (!oob)
		get_free_buffer(ci, &bufs->in);
	spin_unlock_irqrestore(&ci->free_slock, flags);

	return oob ? (closing ? -ESHUTDOWN : -EAGAIN) : 0;
}

/* never takes the last free buffer */
int mlb150_get_tx_buffer_but_last(struct mlb_data *drvdata, int minor, struct mlb150_io_buffers *bufs)
{
	struct mlb_dev_info *pdevinfo = &drvdata->devinfo[minor];
	struct mlb_channel_info *ci = &pdevinfo->channels[TX_CHANNEL];
	unsigned long flags;
	bool oob, closing;

	spin_lock_irqsave(&ci->free_slock, flags);
	oob = (closing = ci->closing_tx) || ci->free.next->next == &ci->free;
	if (!oob)
		get_free_buffer(ci, &bufs->in);
	spin_unlock_irqrestore(&ci->free_slock, flags);

	return oob ? (closing ? -ESHUTDOWN : -EAGAIN) : 0;
}

static void put_tx_buffers(struct mlb_dev_info *pdevinfo, struct mlb150_io_buffers *bufs)
{
	unsigned long flags;
	struct mlb_channel_info *ci = &pdevinfo->channels[TX_CHANNEL];

	spin_lock_irqsave(&ci->dmabuf_slock, flags);
	list_splice_tail(&bufs->out, &ci->q);

	while (!list_empty(&ci->q) && has_free_hw_buf(ci)) {
		int ret;
		struct mlb150_dmabuf *dmab = dmabuf_of(ci->q.next);

		if (pdevinfo->channel_type == MLB150_CTYPE_ASYNC &&
		    dim2_async_bug(ci) == -EAGAIN)
			break;
		list_move_tail(&dmab->head, &ci->hw);

		trace_free_dmabufs(__func__, pdevinfo, TX_CHANNEL, &bufs->in);
		lld_dump(pdevinfo, dmab->virt, ci->buf_size, "T%d %lx",
			 hwbuf_count(&ci->hw), dmab->phys);

		if (ci->pingpong)
			dmab->flags |=  DMABUF_PINGPONG;
		else
			dmab->flags &= ~DMABUF_PINGPONG;
		ret = mlb_start_channel_tx(pdevinfo, dmab->phys, 0);
		if (unlikely(ret)) {
			list_move(&dmab->head, &bufs->in);
			pr_err(DRIVER_NAME": transmission failed (%d), packet dropped\n", ret);
			// TODO notify the originator of the packet
			break;
		}
	}
	spin_unlock_irqrestore(&ci->dmabuf_slock, flags);

	spin_lock_irqsave(&ci->free_slock, flags);
	list_splice(&bufs->in, &ci->free);
	free_list_size_set(ci);
	spin_unlock_irqrestore(&ci->free_slock, flags);
	mlb150_io_buffers_init(bufs);
}

int mlb150_get_tx_buffers(struct mlb_data *drvdata, int minor, struct mlb150_io_buffers *bufs)
{
	struct mlb_dev_info *pdevinfo = &drvdata->devinfo[minor];

	return get_tx_buffers(pdevinfo, bufs);
}

int mlb150_put_tx_buffers(struct mlb_data *drvdata, int minor, struct mlb150_io_buffers *bufs)
{
	struct mlb_dev_info *pdevinfo = &drvdata->devinfo[minor];

	if (unlikely(!atomic_read(&pdevinfo->on)))
		return -ESHUTDOWN;

	put_tx_buffers(pdevinfo, bufs);
	return 0;
}

static void tx_work(struct kthread_work *);
static void rx_work(struct kthread_work *);
static void rx_poll(struct mlb_dev_info *);
static void tx_poll(struct mlb_dev_info *);

static void start_rx_based_on_done_bits(struct mlb_dev_info *pdevinfo,
					unsigned dnecnt, unsigned adt_sts);

static void mlb_channel_enable(struct mlb_dev_info *pdevinfo,
			       enum channelmode mode)
{
	struct mlb_channel_info *tx_ci = &pdevinfo->channels[TX_CHANNEL];
	struct mlb_channel_info *rx_ci = &pdevinfo->channels[RX_CHANNEL];
	u32 tx_ch = tx_ci->address;
	u32 rx_ch = rx_ci->address;
	unsigned long flags;

	atomic_set(&pdevinfo->on, 1);

	pdevinfo->is_reading = is_reading(mode);
	pdevinfo->is_writing = is_writing(mode);
	/*!
	 * setup the direction, enable, channel type,
	 * mode select, channel address and mask buf start
	 */
	pr_debug("%s: enabling: rx %d/%u, tx %d/%u, mode %s bpf %u\n",
		 pdevinfo->dev_name,
		 rx_ch, rx_ci->buf_size,
		 tx_ch, tx_ci->buf_size,
		 MLB_RDONLY == mode ? "ro" :
		 MLB_WRONLY == mode ? "wr" :
		 MLB_RDWR   == mode ? "rw" : "??",
		 pdevinfo->bytes_per_frame);

	/*
	 * Make sure the workers are initialized before anything
	 * happens: they're used by the AHB ISR
	 */
	init_kthread_work(&rx_ci->work, rx_work);
	init_kthread_work(&tx_ci->work, tx_work);

	/* Reset DBR to avoid receive/transmit trash on start */

	if (pdevinfo->is_reading)
		reset_dbr(rx_ci->dbr_buf_head, rx_ci->dbr_size);

	if (pdevinfo->is_writing)
		reset_dbr(tx_ci->dbr_buf_head, tx_ci->dbr_size);

	spin_lock_irqsave(&ctr_lock, flags);

	/*
	 * Note: the driver is running with SCE=0 (hardware clears irq
	 * status on read), so the ACSR reset is omited.
	 */

	set_ctr_write_mask(MDWE_ENABLE_ALL);
	rx_ci->pingpong = 0;
	tx_ci->pingpong = 0;
	tx_ci->dim2_async_bug = 0;
	tx_ci->dbr_avail = tx_ci->dbr_size;
	tx_ci->rpc = read_cdt_rpc_nolock(tx_ci->address);
	tx_ci->wpc = tx_ci->rpc;

	if (pdevinfo->is_reading) {
		init_rfb(pdevinfo, RX_CHANNEL);
		init_channel_adt(rx_ci, pdevinfo->channel_type);
	}

	if (pdevinfo->is_writing) {
		init_rfb(pdevinfo, TX_CHANNEL);
		init_channel_adt(tx_ci, pdevinfo->channel_type);
	}

	/* Synchronize and unmute synchronous channel */
	if (MLB150_CTYPE_SYNC == pdevinfo->channel_type)
		unmute_sync_channel(rx_ch, tx_ch, mode);

	set_ctr_write_mask(MDWE_DISABLE_ALL);
	spin_unlock_irqrestore(&ctr_lock, flags);

	/*
	 * This needs a lock: concurrently running mlb_channel_enable can
	 * race, like when applications reconnect after MOST critical
	 * unlock.
	 */
	spin_lock_irqsave(&ctr_lock, flags);
	if (pdevinfo->is_reading)
		enable_dma_irq_ch(rx_ch, true);
	if (pdevinfo->is_writing)
		enable_dma_irq_ch(tx_ch, true);
	spin_unlock_irqrestore(&ctr_lock, flags);

	dump_registers();
#if DEBUG_CTRDUMP
	dump_ctr(0, tx_ci->address + 1);
#endif

	if (pdevinfo->is_reading) {
		atomic_set(&pdevinfo->pauseRx, 0);
		wmb();
		spin_lock_irqsave(&rx_ci->dmabuf_slock, flags);
		start_rx_based_on_done_bits(pdevinfo, 2, ADT_DNE1 | ADT_DNE2);
		spin_unlock_irqrestore(&rx_ci->dmabuf_slock, flags);
	}
}

static void mlb_channel_disable(struct mlb_dev_info *pdevinfo)
{
	struct mlb_channel_info *tx_ci = &pdevinfo->channels[TX_CHANNEL];
	struct mlb_channel_info *rx_ci = &pdevinfo->channels[RX_CHANNEL];
	u32 tx_ch = tx_ci->address;
	u32 rx_ch = rx_ci->address;
	unsigned long flags;

	/*
	 * TODO: notify the user space/extension of shutdown
	 * Otherwise they might still hold some dmabufs and get no chance to
	 * release them.
	 */

	/*
	 * Take all DMA buffers off the queues and free lists. The `hw`
	 * dmabuf is freed after disabling interrupts and DMA.
	 * Put everything on the free list before leaving.
	 */
	LIST_HEAD(txbufs);
	LIST_HEAD(rxbufs);

#if MLB_RX_TX_STATISTICS
	pr_debug("%s: disabling: tx%s %lld, rx%s %lld pkts\n",
		 pdevinfo->dev_name,
		 pdevinfo->is_writing ? "" : " (inactive)",
		 pdevinfo->tx_pkts,
		 pdevinfo->is_reading ? "" : " (inactive)",
		 pdevinfo->rx_pkts);
#else
	pr_debug("%s: disabling:%s%s\n",
		 pdevinfo->dev_name,
		 pdevinfo->is_writing ? " tx" : "",
		 pdevinfo->is_reading ? " rx" : "");
#endif /* MLB_RX_TX_STATISTICS */

	atomic_set(&tx_ci->polling, 0);
	atomic_set(&rx_ci->polling, 0);
	if (pdevinfo->is_writing)
		del_timer_sync(&tx_ci->isr_timer);
	if (pdevinfo->is_reading)
		del_timer_sync(&rx_ci->isr_timer);

	/* reset/disable this channel */
	spin_lock_irqsave(&ctr_lock, flags);
	if (pdevinfo->is_writing)
		enable_dma_irq_ch(tx_ch, false);
	if (pdevinfo->is_reading)
		enable_dma_irq_ch(rx_ch, false);

	set_ctr_write_mask(MDWE_ENABLE_ALL);

	if (pdevinfo->is_writing)
		reset_channel_ctr(tx_ch);
	if (pdevinfo->is_reading)
		reset_channel_ctr(rx_ch);

	set_ctr_write_mask(MDWE_DISABLE_ALL);
	spin_unlock_irqrestore(&ctr_lock, flags);

	/* block the user process until all PMP failures expired */
	wait_expire_invalid_pmp(pdevinfo);
	/* wait until the extension tasks complete */
	if (pdevinfo->is_writing)
		flush_kthread_work(&tx_ci->work);
	if (pdevinfo->is_reading)
		flush_kthread_work(&rx_ci->work);
	/* get back the dmabufs borrowed by the extension modules */
	ext_cleanup(pdevinfo - pdevinfo->drvdata->devinfo,
		    pdevinfo->channel_type);
	local_irq_save(flags);

	spin_lock(&tx_ci->free_slock);
	list_splice_init(&tx_ci->free, &txbufs);
	free_list_size_set(tx_ci);
	spin_unlock(&tx_ci->free_slock);

	spin_lock(&rx_ci->free_slock);
	list_splice_init(&rx_ci->free, &rxbufs);
	free_list_size_set(rx_ci);
	spin_unlock(&rx_ci->free_slock);

	spin_lock(&tx_ci->dmabuf_slock);
	list_splice_init(&tx_ci->q, &txbufs);
	spin_unlock(&tx_ci->dmabuf_slock);
	spin_lock(&tx_ci->usr_slock);
	list_splice_init(&tx_ci->usr, &txbufs);
	spin_unlock(&tx_ci->usr_slock);

	spin_lock(&rx_ci->dmabuf_slock);
	list_splice_init(&rx_ci->q, &rxbufs);
	spin_unlock(&rx_ci->dmabuf_slock);
	spin_lock(&rx_ci->usr_slock);
	list_splice_init(&rx_ci->usr, &rxbufs);
	spin_unlock(&rx_ci->usr_slock);

	spin_lock(&tx_ci->dmabuf_slock);
	list_splice_init(&tx_ci->hw, &txbufs);
	spin_unlock(&tx_ci->dmabuf_slock);

	/*
	 * TX does not use drop buffers, but RX does
	 * and they must be sorted into the right lists.
	 */
	spin_lock(&rx_ci->dmabuf_slock);
	while (!list_empty(&rx_ci->hw)) {
		if (is_drop_buf(pdevinfo, dmabuf_of(rx_ci->hw.next)))
			list_move(rx_ci->hw.next, &rx_ci->drops);
		else
			list_move(rx_ci->hw.next, &rxbufs);
	}
	spin_unlock(&rx_ci->dmabuf_slock);

	local_irq_restore(flags);

	rx_ci->address = 0;
	tx_ci->address = 0;
	wake_up_interruptible(&pdevinfo->tx_wq);

	spin_lock_irqsave(&tx_ci->free_slock, flags);
	tx_ci->closing_tx = 0;
	list_splice(&txbufs, &tx_ci->free);
	free_list_size_set(tx_ci);
	spin_unlock_irqrestore(&tx_ci->free_slock, flags);

	spin_lock_irqsave(&rx_ci->free_slock, flags);
	list_splice(&rxbufs, &rx_ci->free);
	free_list_size_set(rx_ci);
	spin_unlock_irqrestore(&rx_ci->free_slock, flags);

	atomic_set(&pdevinfo->on, 0);
}

static s64 poll_time(const struct mlb_channel_info *ci)
{
	struct timespec now;

	ktime_get_ts(&now);
	return ktime_us_delta(timespec_to_ktime(now),
			      timespec_to_ktime(ci->poll_start));
}

inline u32 dmabuf_dne_flag(const struct mlb150_dmabuf *dmab)
{
	return dmab->flags & DMABUF_PINGPONG ? ADT_DNE2 : ADT_DNE1;
}

/*
 * This is the implementation of the data which are sent to MLB/MOST in case
 * of underrung condition. One can conceivably imagine using an easily
 * distinguisheable tone instead of constant buffer, which is likely to
 * produce silence.
 */
static void fill_underrun_buf(void *p, uint buf_size, uint bpf __maybe_unused)
{
	memset(p, 0, buf_size);
}

/*
 * The synchronous channels are special: the TX interrupt comes
 * periodically, and something *must* provide the data.
 * If the something cannot provide the data, the DIM will just use the
 * current contents of DBR, producing a characteristic whistle on
 * MOST. Use special data (i.e. silence) to mitigate that.
 */
static void mlb_tx_sync_underrun(struct mlb_dev_info *pdevinfo,
				 struct mlb_channel_info *ci)
	__must_hold(&ci->dmabuf_slock)
{
	bool fill = false;

	if (likely(!list_empty(&ci->drops)))
		list_move(ci->drops.next, &ci->q);
	else {
		unsigned long flags;

		pr_debug("%s/%u: out of drop buffers (tx underrun)\n",
			 pdevinfo->dev_name, ci->address);
		spin_lock_irqsave(&ci->free_slock, flags);
		if (!list_empty(&ci->free)) {
			get_free_buffer(ci, &ci->q);
			fill = true;
		}
		spin_unlock_irqrestore(&ci->free_slock, flags);
	}
	if (!list_empty(&ci->q)) {
		struct mlb150_dmabuf *dmab;

		dmab = list_first_entry(&ci->q, struct mlb150_dmabuf, head);
		if (fill)
			fill_underrun_buf(dmab->virt, ci->buf_size,
					  pdevinfo->bytes_per_frame);
		dmab->flags |= DMABUF_UNDERRUN;
	}
}

static int mlb_tx_handle(struct mlb_dev_info *, struct mlb_channel_info *,
			 u32 ctr_adt[4], bool in_isr);

#define pr_debug_chan(pdevinfo, ci, fmt, ...) do { \
	if (debug_channel(pdevinfo)) \
		pr_debug("%s/%u: cpu%d: " fmt, \
			 pdevinfo->dev_name, ci->address, \
			 raw_smp_processor_id(), ## __VA_ARGS__); \
} while(0)

static void tx_poll(struct mlb_dev_info *pdevinfo)
{
	struct mlb_channel_info *ci = &pdevinfo->channels[TX_CHANNEL];
	unsigned long flags;
	u32 ctr_adt[4] = {0};

	spin_lock_irqsave(&ci->dmabuf_slock, flags);
	if (atomic_read(&ci->polling) == 0)
		goto unlock;
	++ci->poll_cnt;
	if (get_adt_status(ci->address, ctr_adt, BIT(0) | BIT(1))) {
		pr_debug("%s/%u: failure reading ADT\n",
			 pdevinfo->dev_name, ci->address);
		goto poll;
	}
	if (!(ctr_adt[1] & (ADT_DNE1 | ADT_DNE2)))
		goto poll;
	if (mlb_tx_handle(pdevinfo, ci, ctr_adt, false) == -EAGAIN)
		goto unlock;
	pr_debug_chan(pdevinfo, ci,
		      "TX poll (%d) -> IRQ: %4lldus (%u polls), ADT %s\n",
		      hweight32(ctr_adt[1] & (ADT_DNE1 | ADT_DNE2)),
		      poll_time(ci), ci->poll_cnt,
		      ADT_STS_FMT(ctr_adt));
	goto unlock;
poll:
	mod_timer(&ci->isr_timer, jiffies + DNE_POLL_INT);
unlock:
	spin_unlock_irqrestore(&ci->dmabuf_slock, flags);
}

static void tx_isr_defer_poll(unsigned long data)
{
	tx_poll((struct mlb_dev_info *)data);
}

// FIXME: handle ERRx bits (DMABUF_ERR)
static noinline void tx_poll_first(struct mlb_dev_info *pdevinfo)
	__must_hold(&pdevinfo->channels[TX_CHANNEL].dmabuf_slock)
{
	struct mlb_channel_info *ci = &pdevinfo->channels[TX_CHANNEL];
	u32 ctr_adt[4] = {0};
	bool poll_int = false;

	if (atomic_read(&ci->polling)) {
		poll_int = true;
		atomic_set(&ci->polling, 0);
		del_timer(&ci->isr_timer);
	}
	if (get_adt_status(ci->address, ctr_adt, BIT(0) | BIT(1))) {
		pr_debug("%s/%u: failure reading ADT\n",
			 pdevinfo->dev_name, ci->address);
		goto poll;
	}
	if (!(ctr_adt[1] & (ADT_DNE1 | ADT_DNE2)))
		goto poll;
	if (mlb_tx_handle(pdevinfo, ci, ctr_adt, true) == -EAGAIN)
		return;
	if (poll_int) {
		ci->poll_cnt++;
		pr_debug_chan(pdevinfo, ci,
			      "TX poll (i%d) -> IRQ: %lldus (%u polls) ADT %s\n",
			      hweight32(ctr_adt[1] & (ADT_DNE1 | ADT_DNE2)),
			      poll_time(ci), ci->poll_cnt,
			      ADT_STS_FMT(ctr_adt));
	}
	return;
poll:
	pr_debug_chan(pdevinfo, ci, "TX IRQ (%u) -> poll: ADT %s\n",
		      hweight32(ctr_adt[1] & (ADT_DNE1 | ADT_DNE2)),
		      ADT_STS_FMT(ctr_adt));
	ktime_get_ts(&ci->poll_start);
	atomic_set(&ci->polling, 1);
	ci->poll_cnt = 1;
	mod_timer(&ci->isr_timer, jiffies + DNE_POLL_START_INT);
}

static void mlb_tx_isr(struct mlb_dev_info *pdevinfo, int minor)
{
	struct mlb_channel_info *ci = &pdevinfo->channels[TX_CHANNEL];
	unsigned long flags;

	spin_lock_irqsave(&ci->dmabuf_slock, flags);
	tx_poll_first(pdevinfo);
	spin_unlock_irqrestore(&ci->dmabuf_slock, flags);
}

static void tx_trace_sent(struct mlb_dev_info *pdevinfo,
			  struct mlb_channel_info *ci,
			  struct mlb150_dmabuf *dmab)
{
	int nbuf = hwbuf_count(&ci->hw);

	if (nbuf)
		pr_debug("t%d %lx %s: sent, %lx in hw%s\n",
			 nbuf, (long)dmab->phys,
			 pdevinfo->dev_name,
			 (long)dmabuf_of(ci->hw.next)->phys,
			 list_is_last(ci->hw.next, &ci->hw) ?
			 "" : "*UNTERMINATED*");
	else
		pr_debug("t%d %lx %s: sent\n", nbuf,
			 (long)dmab->phys, pdevinfo->dev_name);
}

static void tx_process_done_dmabuf(struct mlb_dev_info *pdevinfo,
				   struct mlb_channel_info *ci,
				   struct mlb150_dmabuf *dmab,
				   u32 ctr_adt[4])
{
	__list_del_entry(&dmab->head);
	if (debug_channel(pdevinfo))
		tx_trace_sent(pdevinfo, ci, dmab);
	update_decoded_stats(pdevinfo, TX_CHANNEL, dmab->virt);
	dmab->flags = DMABUF_FREE;
	if (is_drop_buf(pdevinfo, dmab))
		list_add(&dmab->head, &ci->drops);
	else
		free_channel_dmabuf(ci, dmab);
	/*
	 * TODO: notify the user space/extension of success
	 * The underrun buffers shouldn't be counted,
	 * they cannot have someone expecting them.
	 */
}

static noinline int mlb_tx_handle(struct mlb_dev_info *pdevinfo,
				 struct mlb_channel_info *ci,
				 u32 ctr_adt[4],
				 bool in_isr)
{
	int ret;
	struct mlb150_dmabuf *dmab, *next;
	bool underrun = false;
	unsigned dnecnt = 0;
	u32 adt_sts = ctr_adt[1] & (ADT_DNE1 | ADT_DNE2);

	switch (hwbuf_count(&ci->hw)) {
	case 1:
		dmab = dmabuf_of(ci->hw.next);
		if (dmabuf_dne_flag(dmab) & adt_sts) {
			++dnecnt;
			underrun = dmab->flags & DMABUF_UNDERRUN;
			tx_process_done_dmabuf(pdevinfo, ci, dmab, ctr_adt);
		}
		break;
	case 2:
		dnecnt = 2;
		dmab = dmabuf_of(ci->hw.next);
		if ((bool)(ctr_adt[0] & ADT_PG) != (bool)(dmab->flags & DMABUF_PINGPONG))
			dmab = dmabuf_of(ci->hw.next->next);
		WARN_ON(&dmab->head == &ci->hw);
		if (dmabuf_dne_flag(dmab) & adt_sts) {
			++dnecnt;
			underrun = dmab->flags & DMABUF_UNDERRUN;
			tx_process_done_dmabuf(pdevinfo, ci, dmab, ctr_adt);
		}
		dmab = dmabuf_of(ci->hw.next);
		WARN_ON(&dmab->head == &ci->hw);
		if (dmabuf_dne_flag(dmab) & adt_sts) {
			++dnecnt;
			underrun = underrun || (dmab->flags & DMABUF_UNDERRUN);
			tx_process_done_dmabuf(pdevinfo, ci, dmab, ctr_adt);
		}
		break;
	}
	if (unlikely(!in_isr &&
		     pdevinfo->channel_type == MLB150_CTYPE_SYNC &&
		     list_empty(&ci->q))) {
		/*
		 * If the last dmabuf in hw queue was an underrun mitigation
		 * dmabuf, skip the warning, it was already logged
		 */
		if (!underrun && !ci->closing_tx)
			pr_warn(DRIVER_NAME": %s/%u: underrun!\n",
				pdevinfo->dev_name, ci->address);
		if (underrun_mode)
			mlb_tx_sync_underrun(pdevinfo, ci);
	}
	/*
	 * Check ADT.PG if both buffers were done and ADT.PG does not match
	 * the first buffer on hw queue: swap the first queued hardware
	 * buffers
	 */
	if (dnecnt == 2) {
		bool pdma = ci->pingpong;
		bool pg = ctr_adt[0] & ADT_PG;

		if (pg != pdma) {
			pr_debug("tx %lx %s: PG p%cng, flags p%cng, ADT %s\n",
				 (long)dmabuf_of(ci->q.next)->phys, pdevinfo->dev_name,
				 "io"[pg], "io"[pdma],
				 ADT_STS_FMT(ctr_adt));
			// XXX For not yet clear reasons, this
			// XXX situation never happened during
			// XXX 14 hours of stress testing.

			// XXX Buffer swapping commented out until the
			// XXX situation is reproduced and tested.
			// list_move(&dmab->head, dmab->head.next);
		}
	}
	ret = 0;
	list_for_each_entry_safe(dmab, next, &ci->q, head) {
		if (pdevinfo->channel_type == MLB150_CTYPE_ASYNC &&
		    dim2_async_bug(ci) == -EAGAIN) {
			ret = -EAGAIN;
			break;
		}
		list_move_tail(&dmab->head, &ci->hw);

		trace_free_dmabufs(__func__, pdevinfo, TX_CHANNEL, NULL);
		lld_dump(pdevinfo, dmab->virt, ci->buf_size, "t%d %lx",
			 hwbuf_count(&ci->hw), dmab->phys);

		if (ci->pingpong)
			dmab->flags |=  DMABUF_PINGPONG;
		else
			dmab->flags &= ~DMABUF_PINGPONG;
		ret = mlb_start_channel_tx(pdevinfo, dmab->phys, 0); // XXX toggles ci->pingpong
		if (unlikely(ret)) {
			__list_del_entry(&dmab->head);
			free_channel_dmabuf(ci, dmab);
			pr_err(DRIVER_NAME": channel %d %s: transmission failed (%d), packet dropped\n",
			       ci->address, pdevinfo->dev_name, ret);
			/* TODO: Notify the user space/extension of the error */
			update_drop_stats(pdevinfo, TX_CHANNEL);
			ret = 0;
			break;
		}
		if (!has_free_hw_buf(ci))
			break;
	}
	atomic_set(&ci->polling, 0);
	if (dnecnt)
		queue_kthread_work(&pdevinfo->drvdata->isr_wrk, &ci->work);
	return ret;
}

static void tx_work(struct kthread_work *work)
{
	struct mlb_channel_info *ci = container_of(work, struct mlb_channel_info, work);
	struct mlb_dev_info *pdevinfo = container_of(ci, struct mlb_dev_info, channels[TX_CHANNEL]);
	int minor = pdevinfo - pdevinfo->drvdata->devinfo;

	ext_start_tx(minor, pdevinfo->channel_type);
	wake_up_interruptible(&pdevinfo->tx_wq);
}

#if DEBUG_POISON
static DECLARE_WAIT_QUEUE_HEAD(invalid_pmp_wait);
static struct task_struct *invalid_pmp_task;

static inline s32 get_invalid_pml(const void *p, unsigned buf_size)
{
	u16 pml = be16_to_cpup(p);

	if (unlikely((DEBUG_POISON &&
		      (pml == (DMAB_RX_POISON | (DMAB_RX_POISON << 8)))) ||
		     2u + pml > buf_size || pml < 3u))
		return pml;
	return -1;
}

struct invalid_pmp
{
	struct list_head head;
	struct mlb150_dmabuf *dmab;
	struct timespec ts;
};

static int invalid_pmp_fn(void *_drvdata)
{
	struct mlb_data *drvdata = _drvdata;
	unsigned i;
	unsigned long flags;
	struct list_head bufs;
	struct mlb_dev_info *pdevinfo;
	struct mlb_channel_info *rx_ci;
	struct list_head *devpmp;
	struct invalid_pmp *pmp;
	struct mlb150_dmabuf *dmab;
	DECLARE_WAITQUEUE(wait, current);

	devpmp = kmalloc_array(used_minor_devices, sizeof(*devpmp), GFP_KERNEL);
	for (i = 0; i < used_minor_devices; i++)
		INIT_LIST_HEAD(&devpmp[i]);

	add_wait_queue(&invalid_pmp_wait, &wait);
	set_current_state(TASK_INTERRUPTIBLE);
	for (;;) {
		struct timespec now;
		signed long remain = schedule_timeout(msecs_to_jiffies(1));

		if (kthread_should_stop())
			break;
		set_current_state(TASK_INTERRUPTIBLE);
		if (remain == 0)
			; /* timeout */

		ktime_get_ts(&now);

		for (i = 0; i < used_minor_devices; i++) {
			INIT_LIST_HEAD(&bufs);
			pdevinfo = &drvdata->devinfo[i];
			rx_ci = &pdevinfo->channels[RX_CHANNEL];
			spin_lock_irqsave(&rx_ci->pmp_lock, flags);
			if (!list_empty(&rx_ci->pmp))
				list_splice_init(&rx_ci->pmp, &bufs);
			spin_unlock_irqrestore(&rx_ci->pmp_lock, flags);
			list_for_each_entry(dmab, &bufs, head) {
				pmp = kmalloc(sizeof(*pmp), GFP_KERNEL);
				pmp->dmab = dmab;
				pmp->ts = now;
				list_add(&pmp->head, &devpmp[i]);
			}
		}
		for (i = 0; i < used_minor_devices; i++) {
			struct invalid_pmp *next;

			pdevinfo = &drvdata->devinfo[i];
			rx_ci = &pdevinfo->channels[RX_CHANNEL];
			INIT_LIST_HEAD(&bufs);
			list_for_each_entry_safe(pmp, next, &devpmp[i], head) {
				struct timespec sub = timespec_sub(now, pmp->ts);
				if (get_invalid_pml(pmp->dmab->virt, rx_ci->buf_size) == -1) {
					pr_warn("%s: INVALID PMP %lx became valid after %ld.%06ld\n",
						pdevinfo->dev_name, (long)pmp->dmab->phys,
						sub.tv_sec, sub.tv_nsec / 1000);
					lld_dump(pdevinfo, pmp->dmab->virt, rx_ci->buf_size, "INVALID PMP %lx",
						(long)pmp->dmab->phys);
					goto expire;
				}
				if (sub.tv_sec < 5)
					continue;
				pr_warn("%s: INVALID PMP %lx expired after %ld.%06ld\n",
					pdevinfo->dev_name, (long)pmp->dmab->phys,
					sub.tv_sec, sub.tv_nsec / 1000);
			expire:
				list_move(&pmp->dmab->head, &bufs);
				list_del(&pmp->head);
				kfree(pmp);
			}
			if (!list_empty(&bufs)) {
				spin_lock_irqsave(&rx_ci->free_slock, flags);
				list_splice_init(&bufs, &rx_ci->free);
				spin_unlock_irqrestore(&rx_ci->free_slock, flags);
			}
		}
	}
	remove_wait_queue(&invalid_pmp_wait, &wait);
	set_current_state(TASK_RUNNING);
	for (i = 0; i < used_minor_devices; i++) {
		struct invalid_pmp *next;

		pdevinfo = &drvdata->devinfo[i];
		rx_ci = &pdevinfo->channels[RX_CHANNEL];
		INIT_LIST_HEAD(&bufs);

		list_for_each_entry_safe(pmp, next, &devpmp[i], head) {
			list_move(&pmp->dmab->head, &bufs);
			kfree(pmp);
		}
		spin_lock_irqsave(&rx_ci->pmp_lock, flags);
		if (!list_empty(&rx_ci->pmp))
			list_splice_init(&rx_ci->pmp, &bufs);
		spin_unlock_irqrestore(&rx_ci->pmp_lock, flags);
		if (!list_empty(&bufs)) {
			spin_lock_irqsave(&rx_ci->free_slock, flags);
			list_splice_init(&bufs, &rx_ci->free);
			spin_unlock_irqrestore(&rx_ci->free_slock, flags);
			pr_info("%s: rx channel has unexpired invalid PMP\n",
				pdevinfo->dev_name);
		}
	}
	kfree(devpmp);
	return 0;
}

static int rx_validate_pmp(struct mlb_dev_info *pdevinfo, struct mlb150_dmabuf *dmab, const u32 *ctr_val)
{
	struct mlb_channel_info *ci = &pdevinfo->channels[RX_CHANNEL];
	s32 pml = get_invalid_pml(dmab->virt, ci->buf_size);
	struct mlb150_dmabuf *other_hw_buf;
	unsigned long flags;
	u32 dne_cnt;

	if (pml == -1)
		return 0;

	dne_cnt = hweight32(ctr_val[1] & (ADT_DNE1 | ADT_DNE2));

	if (dmab == dmabuf_of(ci->hw.next))
		other_hw_buf = dmabuf_of(ci->hw.next->next);
	else
		other_hw_buf = dmabuf_of(ci->hw.next);

	pr_warn("R(%s): INVALID PMP %lx: PML 0x%04x, dne_cnt: %u, adt_sts: %08X\n",
		pdevinfo->dev_name, (long)dmab->phys, pml, dne_cnt, ctr_val[1]);
	pr_warn("   ctr[0]: %08X\n", ctr_val[0]);
	pr_warn("   ctr[1]: %08X\n", ctr_val[1]);
	pr_warn("   ctr[2]: %08X\n", ctr_val[2]);
	pr_warn("   ctr[3]: %08X\n", ctr_val[3]);
	pr_warn("   paddr       %lx matches ctr[%u] (p%cng buffer)\n", (long)dmab->phys,
		((long)dmab->phys == ctr_val[2]) ? 2   : (((long)dmab->phys == ctr_val[3]) ? 3   : 0),
		((long)dmab->phys == ctr_val[2]) ? 'i' : (((long)dmab->phys == ctr_val[3]) ? 'o' : 'X') );

	if (other_hw_buf != dmab) {
		pr_warn("   other_paddr %lx matches ctr[%u] (p%cng buffer)\n", (long)other_hw_buf->phys,
			((long)other_hw_buf->phys == ctr_val[2]) ? 2   : (((long)other_hw_buf->phys == ctr_val[3]) ? 3   : 0),
			((long)other_hw_buf->phys == ctr_val[2]) ? 'i' : (((long)other_hw_buf->phys == ctr_val[3]) ? 'o' : 'X') );
		lld_dump(pdevinfo, other_hw_buf->virt, ci->buf_size, "   other_hw_buf %lx:",
			 (long)other_hw_buf->phys);
	} else
		pr_warn("Only one buffer!!! ???\n");

	spin_lock_irqsave(&ci->pmp_lock, flags);
	list_add_tail(&dmab->head, &ci->pmp);
	spin_unlock_irqrestore(&ci->pmp_lock, flags);
	wake_up_interruptible(&invalid_pmp_wait);
	return -1;
}

static void wait_expire_invalid_pmp(struct mlb_dev_info *pdevinfo)
{
	for (;;) {
		bool empty;
		unsigned long flags;
		struct mlb_channel_info *rx_ci = &pdevinfo->channels[RX_CHANNEL];

		spin_lock_irqsave(&rx_ci->pmp_lock, flags);
		empty = list_empty(&rx_ci->pmp);
		spin_unlock_irqrestore(&rx_ci->pmp_lock, flags);
		if (empty)
			break;
		msleep_interruptible(10);
	}
}
#else
static inline int rx_validate_pmp(struct mlb_dev_info *pdevinfo, struct mlb150_dmabuf *dmab, const u32 *ctr_val)
{
	return 0;
}
#endif /* DEBUG_POISON */

static inline void drop_rx(struct mlb_dev_info *pdevinfo, dma_addr_t phys, const char *why)
{
	/*
	 * Allow at most 3 messages to be emitted every 30 seconds to prevent
	 * a kernel log spam.  Excessive logging to the serial console can
	 * cause enough scheduling delays to cause further buffer overflows,
	 * which can culminate into a feedback loop.
	 */
	static DEFINE_RATELIMIT_STATE(drop_rx_ratelimit_state, 30 * HZ, 3);

	update_drop_stats(pdevinfo, RX_CHANNEL);

	if (!__ratelimit(&drop_rx_ratelimit_state))
		return;

	pr_warn("R(%s): no free DMA buffers, dropping %lx%s\n",
		pdevinfo->dev_name, (long)phys, why ? : "");
}

static inline void valid_dmab_or_bug(const struct mlb_dev_info *pdevinfo, const struct mlb150_dmabuf *dmab)
{
	if ((pdevinfo->dma_bufs <= dmab && dmab < pdevinfo->dma_bufs + ARRAY_SIZE(pdevinfo->dma_bufs)) ||
		(pdevinfo->dma_drop <= dmab && dmab < pdevinfo->dma_drop + ARRAY_SIZE(pdevinfo->dma_drop)))
		return;
	BUG();
}

static unsigned mlb_rx_handle(struct mlb_dev_info *pdevinfo, bool pauseRx, u32 ctr_val[4],
			      unsigned long flags);

// FIXME: handle ERRx bits (DMABUF_ERR)
static noinline void rx_get_flags(struct mlb_dev_info *pdevinfo, bool *pauseRx, u32 ctr_val[4])
{
	struct mlb_channel_info *ci = &pdevinfo->channels[RX_CHANNEL];
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ctr_lock, flags);
	/* Read pause under the ctr_lock held */
	*pauseRx = atomic_read(&pdevinfo->pauseRx);
	ret = read_ctr_nolock(BUF_ADT_OFFSET + ci->address, ctr_val);

	/* Check whether to pause the channel */
	if (dev_is_opt3(pdevinfo) && *pauseRx && ret == 0) {
		/* We must not unconditionally clear all done and ready bits here
		 * because this is racy with currently ongoing Rx if we get an IRQ
		 * with only one done bit set. We must only clear the done and ready
		 * bits for the buffers actually done. Thus, it might take two IRQs
		 * (with each one done bit set) until the channel is paused.
		 */
		u32 mask[4] = {0, 0, 0, 0};
		mask[1] = (ctr_val[1] & (ADT_DNE1 | ADT_DNE2));
		/* Set ready bit(s) in the mask only for position(s) where the done bit(s)
		 * were set. For both ping- and pong-buffers, the ready bits are located
		 * at one higher bit position than the done bits.
		 */
		mask[1] |= (mask[1] << 1);

		set_ctr_write_mask(mask);
		/* write only zeroes: Reuse the "mask" array. */
		mask[1] = 0;
		if (write_ctr_nolock(BUF_ADT_OFFSET + ci->address, mask))
			pr_debug("%s/%u: unable to write to CTR for opt3!\n",
				 pdevinfo->dev_name, ci->address);
	}
	spin_unlock_irqrestore(&ctr_lock, flags);

	if (unlikely(ret != 0)) {
		pr_debug("%s/%u: unable to read adt\n",
			 pdevinfo->dev_name, ci->address);
		ctr_val = CTR_ZERO;
	}
}

static void rx_poll(struct mlb_dev_info *pdevinfo)
{
	struct mlb_channel_info *ci = &pdevinfo->channels[RX_CHANNEL];
	unsigned long flags;
	u32 ctr_val[4];
	bool pauseRx;
	unsigned int u;

	spin_lock_irqsave(&ci->dmabuf_slock, flags);
	if (atomic_read(&ci->polling) == 0)
		goto unlock;
	++ci->poll_cnt;
	rx_get_flags(pdevinfo, &pauseRx, ctr_val);
	if (!(ctr_val[1] & (ADT_DNE1 | ADT_DNE2)))
		goto requeue;
	u = mlb_rx_handle(pdevinfo, pauseRx, ctr_val, flags);
	if (!u) {
		s64 us = poll_time(ci);

		if (us < DNE_POLL_TIMEOUT)
			goto requeue;
		pr_debug_chan(pdevinfo, ci,
			      "RX poll (timeout) -> IRQ: %lldus (%u polls), ADT %s\n",
			      us, ci->poll_cnt, ADT_STS_FMT(ctr_val));
		goto unlock;
	}
	if (u) {
		pr_debug_chan(pdevinfo, ci,
			      "RX poll (%u) -> IRQ: %lldus (%u polls), ADT %s\n",
			      u, poll_time(ci), ci->poll_cnt,
			      ADT_STS_FMT(ctr_val));
		goto unlock;
	}
requeue:
	mod_timer(&ci->isr_timer, jiffies + DNE_POLL_INT);
unlock:
	spin_unlock_irqrestore(&ci->dmabuf_slock, flags);
}

static void rx_isr_defer_poll(unsigned long data)
{
	rx_poll((struct mlb_dev_info *)data);
}

static noinline void mlb_rx_isr(struct mlb_dev_info *pdevinfo, int minor)
{
	struct mlb_channel_info *ci = &pdevinfo->channels[RX_CHANNEL];
	unsigned long flags;
	u32 ctr_val[4];
	bool pauseRx, poll_int = false;

	spin_lock_irqsave(&ci->dmabuf_slock, flags);
	if (atomic_read(&ci->polling)) {
		atomic_set(&ci->polling, 0);
		del_timer(&ci->isr_timer);
		poll_int = true;
	}
	rx_get_flags(pdevinfo, &pauseRx, ctr_val);
	if (!(ctr_val[1] & (ADT_DNE1 | ADT_DNE2)))
		goto queue;
	if (mlb_rx_handle(pdevinfo, pauseRx, ctr_val, flags)) {
		if (poll_int) {
			ci->poll_cnt++;
			pr_debug_chan(pdevinfo, ci,
				      "RX poll (i%d) -> IRQ: %lldus (%u polls), ADT %s\n",
				      hweight32(ctr_val[1] & (ADT_DNE1 | ADT_DNE2)),
				      poll_time(ci),
				      ci->poll_cnt, ADT_STS_FMT(ctr_val));
		}
		goto unlock;
	}
queue:
	pr_debug_chan(pdevinfo, ci, "RX IRQ (%u) -> poll: ADT %s\n",
		      hweight32(ctr_val[1] & (ADT_DNE1 | ADT_DNE2)),
		      ADT_STS_FMT(ctr_val));
	ktime_get_ts(&ci->poll_start);
	atomic_set(&ci->polling, 1);
	ci->poll_cnt = 1;
	mod_timer(&ci->isr_timer, jiffies + DNE_POLL_START_INT);
unlock:
	spin_unlock_irqrestore(&ci->dmabuf_slock, flags);
}

/* Detach completed buffers from hw queue: move to "in" (or drop) list */
static void process_done_dmabuf(struct mlb_dev_info *pdevinfo,
				struct mlb_channel_info *ci,
				struct mlb150_dmabuf *dmab,
				struct list_head *in,
				u32 ctr_adt[4])
	__must_hold(&ci->dmabuf_slock)
{
	__list_del_entry(&dmab->head);
	if (is_drop_buf(pdevinfo, dmab))
		list_add(&dmab->head, &ci->drops);
	else if ((pdevinfo->channel_type == MLB150_CTYPE_CTRL ||
		  pdevinfo->channel_type == MLB150_CTYPE_ASYNC) &&
		 ci == &pdevinfo->channels[RX_CHANNEL] &&
		 rx_validate_pmp(pdevinfo, dmab, ctr_adt) < 0)
		;
	else {
		list_add_tail(&dmab->head, in);
		update_decoded_stats(pdevinfo,
				     ci == &pdevinfo->channels[RX_CHANNEL] ?
				     RX_CHANNEL : TX_CHANNEL,
				     dmab->virt);
		lld_dump(pdevinfo, dmab->virt, ci->buf_size, "r%d %lx",
			 hweight32(ctr_adt[1] & (ADT_DNE1 | ADT_DNE2)),
			 dmab->phys);
	}
}

static noinline
unsigned mlb_rx_handle(struct mlb_dev_info *pdevinfo, bool pauseRx, u32 ctr_val[4],
		       unsigned long flags)
{
	bool wakeup_q;
	LIST_HEAD(in);
	LIST_HEAD(free);
	struct mlb150_dmabuf *next, *dmab;
	struct mlb_channel_info *ci = &pdevinfo->channels[RX_CHANNEL];
	u32 adt_sts = ctr_val[1] & (ADT_DNE1 | ADT_DNE2);
	unsigned done = 0;

	switch (adt_sts) {
	case ADT_DNE1:
	case ADT_DNE2:
		done = 1;
		dmab = dmabuf_of(ci->hw.next);
		if (!(dmabuf_dne_flag(dmab) & adt_sts))
			dmab = dmabuf_of(ci->hw.next->next);
		WARN_ON(!(dmabuf_dne_flag(dmab) & adt_sts));
		process_done_dmabuf(pdevinfo, ci, dmab, &in, ctr_val);
		break;
	case ADT_DNE1 | ADT_DNE2:
		done = 2;
		dmab = dmabuf_of(ci->hw.next);
		if ((bool)(ctr_val[0] & ADT_PG) != (bool)(dmab->flags & DMABUF_PINGPONG))
			dmab = dmabuf_of(ci->hw.next->next);
		WARN_ON((bool)(ctr_val[0] & ADT_PG) != (bool)(dmab->flags & DMABUF_PINGPONG));
		process_done_dmabuf(pdevinfo, ci, dmab, &in, ctr_val);
		dmab = dmabuf_of(ci->hw.next);
		process_done_dmabuf(pdevinfo, ci, dmab, &in, ctr_val);
		break;
	}
	if (!list_empty(&in))
		list_splice_tail(&in, &ci->q);
	wakeup_q = !list_empty(&ci->q);
	list_for_each_entry_safe(dmab, next, &free, head) {
		__list_del_entry(&dmab->head);
		if (is_drop_buf(pdevinfo, dmab))
			list_add(&dmab->head, &ci->drops);
		else {
			update_drop_stats(pdevinfo, RX_CHANNEL);
			free_channel_dmabuf(ci, dmab);
		}
	}
	if (done && !pauseRx)
		start_rx_based_on_done_bits(pdevinfo, done, adt_sts);
	// Maybe call the extensions for synchronous (latency-sensitive)
	// channels immediately?
	if (wakeup_q)
		queue_kthread_work(&pdevinfo->drvdata->isr_wrk, &ci->work);
	return done;
}


static noinline
void start_rx_based_on_done_bits(struct mlb_dev_info *pdevinfo,
				 unsigned need_free, unsigned adt_sts)
	__must_hold(&pdevinfo->channels[RX_CHANNEL].dmabuf_slock)
{
	unsigned long flags;
	struct mlb_channel_info *ci = &pdevinfo->channels[RX_CHANNEL];
	dma_addr_t ba1, ba2;
	LIST_HEAD(free);
	struct mlb150_dmabuf *dmab;

	/*
	 * Obtain free buffers, trying free list, user list, and finally,
	 * extension queued list.
	 */

	/* take a buffer off the free buffers */
	spin_lock_irqsave(&ci->free_slock, flags);
	while (need_free && !list_empty(&ci->free)) {
		get_free_buffer(ci, &free);
		--need_free;
	}
	spin_unlock_irqrestore(&ci->free_slock, flags);

	if (unlikely(need_free)) {
		/*
		 * Take a buffer off the userspace buffers.
		 * Caused by a slow reader of the cdev.
		 */
		spin_lock_irqsave(&ci->usr_slock, flags);
		while (need_free && !list_empty(&ci->usr)) {
			list_move(ci->usr.next, &free);
			--need_free;
			drop_rx(pdevinfo, dmabuf_of(free.next)->phys, " from file queue");
		}
		spin_unlock_irqrestore(&ci->usr_slock, flags);
	}

	if (unlikely(need_free)) {
		/*
		 * Take a buffer off the queued buffers.
		 * Probably caused by a slow extension code.
		 */
		while (need_free && !list_empty(&ci->q)) {
			list_move(ci->q.next, &free);
			--need_free;
			drop_rx(pdevinfo, dmabuf_of(free.next)->phys, " from pending queue");
		}
		/*
		 * Direct dma to the packet drop space.
		 * Probably caused by a slow user of an extension which
		 * detaches buffers (syncsound).
		 */
		while (need_free && !list_empty(&ci->drops)) {
			list_move(ci->drops.next, &free);
			--need_free;
			drop_rx(pdevinfo, dmabuf_of(free.next)->phys, " (drop space)");
		}
	}
	ba1 = ba2 = 0;
	if (ADT_DNE1 & adt_sts) {
		dmab = dmabuf_of(free.next);
		BUG_ON(unlikely(list_empty(&free)));
		valid_dmab_or_bug(pdevinfo, dmab);
		ba1 = dmab->phys;
		dmab->flags &= ~DMABUF_PINGPONG;
		poison_dmab(dmab, DMAB_RX_POISON, ci->buf_size);
		list_move_tail(&dmab->head, &ci->hw);
	}
	if (ADT_DNE2 & adt_sts) {
		dmab = dmabuf_of(free.next);
		BUG_ON(unlikely(list_empty(&free)));
		valid_dmab_or_bug(pdevinfo, dmab);
		ba2 = dmab->phys;
		dmab->flags |= DMABUF_PINGPONG;
		poison_dmab(dmab, DMAB_RX_POISON, ci->buf_size);
		list_move_tail(&dmab->head, &ci->hw);
	}
	mlb_start_channel_rx(pdevinfo, adt_sts, ba1, ba2);
	atomic_set(&ci->polling, 0);
}

static void rx_trace_recv(struct mlb_dev_info *pdevinfo, const struct list_head *in)
{
	unsigned i = 0;
	struct mlb150_dmabuf *dmab;

	list_for_each_entry(dmab, in, head)
		if (i++)
			pr_debug("R(%s) %lx recv (%u)\n", pdevinfo->dev_name, (long)dmab->phys, i);
		else
			; // pr_debug("R(%s) %lx recv\n", pdevinfo->dev_name, (long)dmab->phys);
}

static void rx_work(struct kthread_work *work)
{
	bool empty_usr;
	struct mlb_channel_info *ci = container_of(work, struct mlb_channel_info, work);
	struct mlb_dev_info *pdevinfo = container_of(ci, struct mlb_dev_info, channels[RX_CHANNEL]);
	int minor = pdevinfo - pdevinfo->drvdata->devinfo;
	struct mlb150_io_buffers bufs = MLB150_IO_BUFFERS_INIT(bufs);
	unsigned long flags;

	spin_lock_irqsave(&ci->dmabuf_slock, flags);
	list_splice_init(&ci->q, &bufs.in);
	spin_unlock_irqrestore(&ci->dmabuf_slock, flags);

	if (list_empty(&bufs.in))
		return;

	if (debug_channel(pdevinfo))
		rx_trace_recv(pdevinfo, &bufs.in);

	ext_start_rx(minor, pdevinfo->channel_type, &bufs);

	spin_lock_irqsave(&ci->usr_slock, flags);
	list_splice_tail(&bufs.out, &ci->usr);
	list_splice_tail(&bufs.in, &ci->usr);
	empty_usr = list_empty(&ci->usr);
	spin_unlock_irqrestore(&ci->usr_slock, flags);

	if (!empty_usr)
		wake_up_interruptible(&pdevinfo->rx_wq);
}

static irqreturn_t mlb_ahb_isr(int irq, void *dev_id)
{
	struct mlb_data *drvdata = dev_id;
	u32 acsr0, hcer0;
	u32 acsr1, hcer1;
	int minor, isrs = 0, errs = 0;

	/* Step 5, Read the ACSRn registers to determine which channel or
	 * channels are causing the interrupt */
	acsr0 = READ_MLB_REG(REG_ACSR0);
	acsr1 = READ_MLB_REG(REG_ACSR1);

	hcer0 = READ_MLB_REG(REG_HCER0);
	hcer1 = READ_MLB_REG(REG_HCER1);

#if DEBUG_AHB_ISR
	pr_debug("irq=%i, ACSR0=0x%08X, ACSR1=0x%08X, HCER0=0x%08X, HCER1=0x%08X\n",
		 irq, acsr0, acsr1, hcer0, hcer1);
#endif
	for (minor = 0; minor < used_minor_devices; minor++) {
		struct mlb_dev_info *pdevinfo = &drvdata->devinfo[minor];
		unsigned int i;

		for (i = TX_CHANNEL; i <= RX_CHANNEL; ++i) {
			struct mlb_channel_info *ci = &pdevinfo->channels[i];
			u32 int_sts = ci->address < 32 ?  acsr0 :  acsr1;
			u32 ch_mask = 1 << (ci->address % 32);

			/* test the channel interrupt status */
			if (int_sts & ch_mask) {
				int on;
				u32 *err = ci->address < 32 ? &hcer0 : &hcer1;
				/* In case of an error, clear the error bit and print a warning. */
				if (*err & ch_mask) {
					++errs;
					*err &= ~ch_mask;
					dump_mlb_diag(pdevinfo, 0);
					pr_warn("%s: %cx channel %d encountered an AHB error!\n",
						pdevinfo->dev_name, i == RX_CHANNEL ? 'r' : 't', ci->address);
				}
				on = atomic_read(&pdevinfo->on);
#if DEBUG_AHB_ISR
				pr_debug("%c(%s) isr%s\n", "RT"[i == TX_CHANNEL],
					 pdevinfo->dev_name, on ? "" : " (inactive)");
#endif
				if (unlikely(!on))
					pr_err_ratelimited(DRIVER_NAME ": AHB interrupt for inactive device %s\n",
							   pdevinfo->dev_name);
				else {
					if (i == RX_CHANNEL)
						mlb_rx_isr(pdevinfo, minor);
					else
						mlb_tx_isr(pdevinfo, minor);
				}
				++isrs;
			}
		}
	}

	if (unlikely(!isrs)) {
		pr_warn_ratelimited(DRIVER_NAME ": unexpected AHB interrupt. Sharing IRQ?\n");
		return IRQ_NONE;
	}

	/* clear the handled error bits in the register */
	if (errs) {
		WRITE_MLB_REG(hcer0, REG_HCER0);
		WRITE_MLB_REG(hcer1, REG_HCER1);
	}
	/* Step 6, If ACTL.SCE = 1, write the result of step 5 back to ACSR0
	 * and ACSR1 to clear the interrupt
	 */
	if (ACTL_SCE & drvdata->actl) {
		WRITE_MLB_REG(acsr0, REG_ACSR0);
		WRITE_MLB_REG(acsr1, REG_ACSR1);
	}

	return IRQ_HANDLED;
}

static int mlb_clear_channel_error(enum mlb150_channel_type ctype, uint ch,
				   const bool is_rx, u32 *cis, u32 cdt[4])
{
	*cis = 0;
	if (read_ctr(BUF_CDT_OFFSET + ch, cdt))
		return -ETIMEDOUT;
	switch (ctype) {
		u32 mask, shift;

	case MLB150_CTYPE_SYNC:
		mask = is_rx ? CDT_SYNC_WSTS_MASK : CDT_SYNC_RSTS_MASK;
		shift = is_rx ? CDT_SYNC_WSTS_SHIFT : CDT_SYNC_RSTS_SHIFT;
		*cis = (cdt[2] & mask) >> shift;
		cdt[2] &= ~(0x8 << shift);
		break;
	case MLB150_CTYPE_CTRL:
	case MLB150_CTYPE_ASYNC:
		mask = is_rx ? CDT_CTRL_ASYNC_WSTS_MASK : CDT_CTRL_ASYNC_RSTS_MASK;
		shift = is_rx ? CDT_CTRL_ASYNC_WSTS_SHIFT : CDT_CTRL_ASYNC_RSTS_SHIFT;
		/* RSTS: ReceiverProtocolError, WSTS: command protocol */
		*cis = ((cdt[2] & mask) >> shift) & BIT(2);
		cdt[2] &= ~(0x4 << shift);
		mask = is_rx ? CDT_CTRL_ASYNC_WSTS_1 : CDT_CTRL_ASYNC_RSTS_1;
		/* RSTS: ReceiverBreak, WSTS: Async/Control Break command */
		*cis |= cdt[3] & mask ? 0x1 << 4 : 0;
		cdt[3] &= ~mask;
		break;
	case MLB150_CTYPE_ISOC:
		mask = is_rx ? CDT_ISOC_WSTS_MASK : CDT_ISOC_RSTS_MASK;
		shift = is_rx ? CDT_ISOC_WSTS_SHIFT : CDT_ISOC_RSTS_SHIFT;
		*cis = (cdt[2] & mask) >> shift;
		cdt[2] &= ~(0x6 << shift);
		break;
	}
	if (!*cis)
		return 0;
	pr_info_ratelimited(DRIVER_NAME": %cX_CH: %u, CDT[127:0]: %08x %08x %08x %08x\n",
			    is_rx ? 'R' : 'T', ch,
			    cdt[3], cdt[2], cdt[1], cdt[0]);
	/* Clear status errors to resume channel operation */
	if (write_ctr(BUF_CDT_OFFSET + ch, cdt))
		pr_err(DRIVER_NAME"failed to reset CDT of %u\n", ch);
	return 0;
}

static void mlb_async_tx_done(struct mlb_dev_info *pdevinfo)
{
	struct mlb_channel_info *ci = &pdevinfo->channels[TX_CHANNEL];
	unsigned long flags;

	spin_lock_irqsave(&ci->dmabuf_slock, flags);
	if (ci->dim2_async_bug)
		tx_poll_first(pdevinfo);
	spin_unlock_irqrestore(&ci->dmabuf_slock, flags);
}

/* error interrupt */
static irqreturn_t mlb_isr(int irq, void *dev_id)
{
	struct mlb_data *drvdata = dev_id;
	unsigned long flags;
	int minor;
	int cnt = 0;

	/* Step 4, Read the MSn register to determine which channel(s)
	 * are causing the interrupt */
	const u32 ms0 = READ_MLB_REG(REG_MS0);
	const u32 ms1 = READ_MLB_REG(REG_MS1);

	WRITE_MLB_REG(0, REG_MS0);
	WRITE_MLB_REG(0, REG_MS1);

	pr_debug("irq=%i, MS0=0x%08X, MS1=0x%08X\n", irq, ms0, ms1);

	for (minor = 0; minor < used_minor_devices; minor++) {
		struct mlb_dev_info *pdevinfo = &drvdata->devinfo[minor];
		const u32 rx_mlb_ch = pdevinfo->channels[RX_CHANNEL].address;
		const u32 tx_mlb_ch = pdevinfo->channels[TX_CHANNEL].address;
		const u32 rx_int_sts = rx_mlb_ch < 32 ? ms0 : ms1;
		const u32 tx_int_sts = tx_mlb_ch < 32 ? ms0 : ms1;
		u32 tx_cis = 0, rx_cis = 0;
		u32 cdt[4];

		/* Get tx channel interrupt status */
		if (tx_int_sts & (1 << (tx_mlb_ch % 32))) {
			++cnt;
			if (mlb_clear_channel_error(pdevinfo->channel_type,
						    tx_mlb_ch, false,
						    &tx_cis, cdt))
				goto timeout;
			if (MLB150_CTYPE_ASYNC == pdevinfo->channel_type &&
			    (cdt[2] & CDT_CTRL_ASYNC_RSTS_IDLE_MASK) == 0)
				mlb_async_tx_done(pdevinfo);
		}
		/* Get rx channel interrupt status */
		if (rx_int_sts & (1 << (rx_mlb_ch % 32))) {
			++cnt;
			mlb_clear_channel_error(pdevinfo->channel_type,
						rx_mlb_ch, true, &rx_cis, cdt);
		}

timeout: /* we might manage to re-read the status later */

		if (!tx_cis && !rx_cis)
			continue;

		/* fill exception event */
		spin_lock_irqsave(&pdevinfo->event_lock, flags);
		pdevinfo->ex_event |= (rx_cis << 16) | tx_cis;
		spin_unlock_irqrestore(&pdevinfo->event_lock, flags);
	}

	return cnt ? IRQ_HANDLED : IRQ_NONE;
}

// TODO: show the amount of free DBR available
static int alloc_dma_bufs(struct device *dev, struct mlb_dev_info *pdevinfo, enum channelmode mode)
{
	int i;
	int ret;
	size_t offs;
	struct mlb150_dmabuf *dmab;
	struct mlb_channel_info *tx_ci = &pdevinfo->channels[TX_CHANNEL];
	struct mlb_channel_info *rx_ci = &pdevinfo->channels[RX_CHANNEL];
	const size_t rx_buf_size = rx_ci->buf_size;
	const size_t tx_buf_size = tx_ci->buf_size;
	size_t reserved = 0, rx_size = 0, tx_size = 0, dma_size;

	if (is_reading(mode))
		rx_size = rx_buf_size * RX_BUF_COUNT;
	reserved = max(rx_buf_size, tx_buf_size);
	if (is_writing(mode))
		tx_size = tx_buf_size * TX_BUF_COUNT;

	ret = dma_set_mask(dev, DMA_BIT_MASK(32));
	if (ret) {
		ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_err(dev, "no usable DMA configuration for %s (%d)\n",
				pdevinfo->dev_name, ret);
			return -ENOMEM;
		}
	}

	dma_size = round_up(rx_size + reserved, 8) + tx_size;
	pdevinfo->dma_cpu = dma_zalloc_coherent(dev, dma_size,
						&pdevinfo->dma_handle,
						GFP_KERNEL);

	if (unlikely(!pdevinfo->dma_cpu)) {
		dev_err(dev, "cannot allocate DMA buffers for %s\n",
			pdevinfo->dev_name);
		return -ENOMEM;
	}

	pdevinfo->dma_size = dma_size;
	offs = 0;
	dmab = pdevinfo->dma_bufs;

	for (i = RX_BUF_COUNT * !!rx_size; i--; ++dmab, offs += rx_buf_size) {
		dmab->virt = pdevinfo->dma_cpu + offs;
		dmab->phys = pdevinfo->dma_handle + offs;
		list_add_tail(&dmab->head, &rx_ci->free);
	}
	free_list_size_set(rx_ci);

	pdevinfo->dma_bufs_rxtx_boundary = pdevinfo->dma_handle + offs;

	/* start of the tx dma area */
	offs = pdevinfo->dma_size - tx_size;

	for (i = TX_BUF_COUNT * !!tx_size; i--; ++dmab, offs += tx_buf_size) {
		dmab->virt = pdevinfo->dma_cpu + offs;
		dmab->phys = pdevinfo->dma_handle + offs;
		list_add_tail(&dmab->head, &tx_ci->free);
	}
	free_list_size_set(tx_ci);

	pdevinfo->dma_drop[0].virt = pdevinfo->dma_cpu + rx_size;
	pdevinfo->dma_drop[0].phys = pdevinfo->dma_handle + rx_size;
	pdevinfo->dma_drop[1] = pdevinfo->dma_drop[0];
	pdevinfo->dma_drop[2] = pdevinfo->dma_drop[0];
	pdevinfo->dma_drop[3] = pdevinfo->dma_drop[0];
	list_add(&pdevinfo->dma_drop[0].head, &rx_ci->drops);
	list_add(&pdevinfo->dma_drop[1].head, &rx_ci->drops);
	list_add(&pdevinfo->dma_drop[2].head, &tx_ci->drops);
	list_add(&pdevinfo->dma_drop[3].head, &tx_ci->drops);
	if (pdevinfo->channel_type == MLB150_CTYPE_SYNC && tx_size) {
		fill_underrun_buf(pdevinfo->dma_drop[0].virt, reserved,
				  pdevinfo->bytes_per_frame);
		pr_debug("%s: pre-initialized drop-buffer (%lx/%u): %02x %02x %02x ...\n",
			 pdevinfo->dev_name,
			 (ulong)pdevinfo->dma_drop[0].phys, reserved,
			 ((char *)(pdevinfo->dma_drop[0].virt))[0],
			 ((char *)(pdevinfo->dma_drop[0].virt))[1],
			 ((char *)(pdevinfo->dma_drop[0].virt))[2]);
	}

	pr_debug("%s, %s DMA: "
		 "RX %dx 0x%p (phy 0x%lx), TX %dx 0x%p (phy 0x%lx), "
		 "buffers %lu (rx) and %lu (tx) bytes, %lu reserved, %lu total\n",
		 pdevinfo->dev_name,
		 mode == MLB_RDWR ? "rw" :
		 mode == MLB_RDONLY ? "ro" :
		 mode == MLB_WRONLY ? "wo" : "??",
		 RX_BUF_COUNT,
		 list_empty(&rx_ci->free) ? NULL : dmabuf_of(rx_ci->free.next)->virt,
		 list_empty(&rx_ci->free) ? 0l   : (long)dmabuf_of(rx_ci->free.next)->phys,
		 TX_BUF_COUNT,
		 list_empty(&tx_ci->free) ? NULL : dmabuf_of(tx_ci->free.next)->virt,
		 list_empty(&tx_ci->free) ? 0l   : (long)dmabuf_of(tx_ci->free.next)->phys,
		 (ulong)rx_ci->buf_size, (ulong)tx_ci->buf_size,
		 (ulong)reserved, (ulong)pdevinfo->dma_size);
	list_for_each_entry(dmab, &rx_ci->free, head)
		pr_debug(" DMA RX virt %p phys %lx\n", dmab->virt, (long)dmab->phys);
	list_for_each_entry(dmab, &tx_ci->free, head)
		pr_debug(" DMA TX virt %p phys %lx\n", dmab->virt, (long)dmab->phys);
	return 0;
}

static void free_dma_bufs(struct device *dev, struct mlb_dev_info *pdevinfo)
{
	unsigned long flags, free_flags;
	struct mlb_channel_info *tx_ci = &pdevinfo->channels[TX_CHANNEL];
	struct mlb_channel_info *rx_ci = &pdevinfo->channels[RX_CHANNEL];

	if (!pdevinfo->dma_size)
		return;

	spin_lock_irqsave(&rx_ci->dmabuf_slock, flags);
	spin_lock_irqsave(&rx_ci->free_slock, free_flags);
	init_mlb_channel_info_dma(rx_ci);
	spin_unlock_irqrestore(&rx_ci->free_slock, free_flags);
	spin_unlock_irqrestore(&rx_ci->dmabuf_slock, flags);
	spin_lock_irqsave(&tx_ci->dmabuf_slock, flags);
	spin_lock_irqsave(&tx_ci->free_slock, free_flags);
	init_mlb_channel_info_dma(tx_ci);
	spin_unlock_irqrestore(&tx_ci->free_slock, free_flags);
	spin_unlock_irqrestore(&tx_ci->dmabuf_slock, flags);

	dma_free_coherent(dev, pdevinfo->dma_size,
			  pdevinfo->dma_cpu, pdevinfo->dma_handle);
	pdevinfo->dma_size = 0;
}

static void allocate_dmabufs_common_checks(struct mlb_dev_info *pdevinfo,
					   enum channelmode mode)
{
	struct mlb_channel_info *tx_ci = &pdevinfo->channels[TX_CHANNEL];
	struct mlb_channel_info *rx_ci = &pdevinfo->channels[RX_CHANNEL];

	if (MLB_RDWR == mode && tx_ci->dbr_buf_head == rx_ci->dbr_buf_head)
		dev_err(pdevinfo->drvdata->dev,
			"%s does not support simultaneous read and write (DBR reserved only for one direction)\n",
			pdevinfo->dev_name);
}

static int mlb_chan_allocate_dmabufs(struct mlb_data *drvdata,
				     struct mlb_dev_info *pdevinfo,
				     enum channelmode mode,
				     unsigned int bytes_per_frame)
{
	int ret = 0; /* already allocated or no reallocation needed */

	allocate_dmabufs_common_checks(pdevinfo, mode);
	if (pdevinfo->channel_type == MLB150_CTYPE_SYNC)
		return -ENOTBLK;
	if (bytes_per_frame) {
		char comm[TASK_COMM_LEN];
		pr_err(DRIVER_NAME": %s[%ld]: number of bytes per frame must be 0 for %s\n",
		       get_task_comm(comm, current),
		       (long)current->pid, pdevinfo->dev_name);
		ret = -EINVAL;
	}
	/*
	 * For Isoc, we're always freeing and allocating again. This
	 * ensures correct buffer sizes, but might not be necessary.
	 * TODO: Implement: only when size changes!
	 */
	if (pdevinfo->channel_type == MLB150_CTYPE_ISOC) {
		free_dma_bufs(pdevinfo->drvdata->dev, pdevinfo);
		pdevinfo->dma_size = 0;
	}
	if (!pdevinfo->dma_size)
		ret = alloc_dma_bufs(drvdata->dev, pdevinfo, mode);
	return ret;
}

static int mlb_sync_allocate_dmabufs(struct mlb_data *drvdata,
				     struct mlb_dev_info *pdevinfo,
				     enum channelmode mode,
				     unsigned int bytes_per_frame,
				     unsigned int sync_buf_size)
{
	struct mlb_channel_info *tx_ci = &pdevinfo->channels[TX_CHANNEL];
	struct mlb_channel_info *rx_ci = &pdevinfo->channels[RX_CHANNEL];

	allocate_dmabufs_common_checks(pdevinfo, mode);
	if (pdevinfo->channel_type != MLB150_CTYPE_SYNC)
		return -EINVAL;
	if (!valid_sync_buf_size(sync_buf_size, bytes_per_frame)) {
		dev_err(pdevinfo->drvdata->dev,
			"%s: invalid buffer size %u bpf %u\n",
			pdevinfo->dev_name, sync_buf_size, bytes_per_frame);
		return -EINVAL;
	}
	if (pdevinfo->dma_size &&
	    pdevinfo->bytes_per_frame == bytes_per_frame &&
	    rx_ci->buf_size == sync_buf_size &&
	    tx_ci->buf_size == sync_buf_size)
		/* already allocated or no reallocation needed */
		return 0;
	free_dma_bufs(pdevinfo->drvdata->dev, pdevinfo);
	pdevinfo->bytes_per_frame = bytes_per_frame;
	rx_ci->buf_size = sync_buf_size;
	tx_ci->buf_size = sync_buf_size;
	return alloc_dma_bufs(pdevinfo->drvdata->dev, pdevinfo, mode);
}

int mlb150_chan_allocate_dmabufs(struct mlb_data *drvdata, int minor,
				 enum channelmode mode,
				 unsigned int bytes_per_frame,
				 unsigned int sync_buf_size)
{
	struct mlb_dev_info *pdevinfo = &drvdata->devinfo[minor];

	if (pdevinfo->channel_type == MLB150_CTYPE_SYNC)
		return mlb_sync_allocate_dmabufs(drvdata, pdevinfo, mode,
						 bytes_per_frame,
						 sync_buf_size);

	return mlb_chan_allocate_dmabufs(drvdata, pdevinfo, mode,
					 bytes_per_frame);
}

int mlb150_do_open(struct mlb_data *drvdata, int minor, enum channelmode mode)
{
	struct mlb_dev_info *pdevinfo = &drvdata->devinfo[minor];

	if (platform_get_drvdata(container_of(drvdata->dev, struct platform_device, dev)) != drvdata)
		/* initialization not completed yet */
		return -ENODEV;
	if (minor < 0 || minor >= used_minor_devices) {
		dev_dbg(drvdata->dev, "no device\n");
		return -ENODEV;
	}
	if (atomic_cmpxchg(&pdevinfo->opencnt, 0, 1) != 0) {
		dev_dbg(drvdata->dev, "%s busy\n", pdevinfo->dev_name);
		return -EBUSY;
	}
	pdevinfo->ex_event = 0;

	if (pdevinfo->channel_type == MLB150_CTYPE_ISOC)
		update_isoc_buf_size(pdevinfo, minor);

	return 0;
}

static int mlb150_open(struct inode *inode, struct file *filp)
{
	enum channelmode mode;
	int minor = MINOR(inode->i_rdev);
	struct mlb_data *drvdata =
		container_of(inode->i_cdev, struct mlb_data, cdev);
	char comm[TASK_COMM_LEN];

	pr_debug("%s[%ld]: open %s %s (%sblocking)\n",
		 get_task_comm(comm, current), (long)current->pid,
		 drvdata->devinfo[minor].dev_name,
		 (filp->f_flags & O_ACCMODE) == O_RDWR ? "rw" :
		 (filp->f_flags & O_ACCMODE) == O_RDONLY ? "ro" :
		 (filp->f_flags & O_ACCMODE) == O_WRONLY ? "wo" : "??",
		 (filp->f_flags & O_NONBLOCK) ? "non-" : "");

	nonseekable_open(inode, filp);
	filp->private_data = drvdata;

	switch (filp->f_flags & O_ACCMODE) {
	case O_RDWR:
		/* Sync channels must only be opened for read or write, never both */
		if (drvdata->devinfo[minor].channel_type == MLB150_CTYPE_SYNC)
			return -EINVAL;
		mode = MLB_RDWR;
		break;
	case O_RDONLY:
		mode = MLB_RDONLY;
		break;
	case O_WRONLY:
		mode = MLB_WRONLY;
		break;
	default:
		return -EINVAL;
	}

	return mlb150_do_open(drvdata, minor, mode);
}

static int __must_check stop_tx(struct mlb_dev_info *pdevinfo)
{
	unsigned long flags;
	struct mlb_channel_info *tx_ci = &pdevinfo->channels[TX_CHANNEL];
	const unsigned tmo = MLB150_CTYPE_SYNC == pdevinfo->channel_type ?
		msecs_to_jiffies(TX_FLUSH_TIMEOUT_SYNC) :
		msecs_to_jiffies(TX_FLUSH_TIMEOUT);
	unsigned nbuf;
	int ret = 1;

	pr_debug("waiting tx of %s\n", pdevinfo->dev_name);

	spin_lock_irqsave(&tx_ci->free_slock, flags);
	tx_ci->closing_tx = 1;
	wake_up_interruptible(&pdevinfo->tx_wq);
	spin_unlock_irqrestore(&tx_ci->free_slock, flags);

	spin_lock_irqsave(&tx_ci->dmabuf_slock, flags);
	nbuf = dmabuf_count(&tx_ci->q) + hwbuf_count(&tx_ci->hw);
	if (nbuf)
		ret = wait_event_interruptible_lock_irq_timeout(pdevinfo->tx_wq,
			list_empty(&tx_ci->q) && list_empty(&tx_ci->hw),
			tx_ci->dmabuf_slock, tmo * (nbuf + 1));
	spin_unlock_irqrestore(&tx_ci->dmabuf_slock, flags);

	if (0 == ret)
		ret = -ETIMEDOUT;
	if (ret > 0)
		ret = 0;
	return ret;
}

static void isoc_sync_stop(struct mlb_dev_info *pdevinfo);

int mlb150_do_release(struct mlb_data *drvdata, int minor,
		      enum channelmode mode)
{
	int ret;
	struct mlb_dev_info *pdevinfo = &drvdata->devinfo[minor];

	pr_debug("release %s\n", pdevinfo->dev_name);

	/* wait for completion of the buffers scheduled for transmission */
	ret = stop_tx(pdevinfo);

	if (-ETIMEDOUT == ret) {
		char comm[TASK_COMM_LEN];
		pr_warn(DRIVER_NAME ": %s[%ld]: %s: waiting for the tx channel failed, data lost\n",
			get_task_comm(comm, current), (long)current->pid, pdevinfo->dev_name);
	}

	dump_registers();
#if DEBUG_CTRDUMP
	dump_ctr(0, pdevinfo->channels[TX_CHANNEL].address + 1);
#endif

	/* clear channel settings and info */
	mlb_channel_disable(pdevinfo);
	free_dma_bufs(drvdata->dev, pdevinfo);

	/* decrease the open count */
	atomic_set(&pdevinfo->opencnt, 0);

	if (isoc_sync_quirk)
		isoc_sync_stop(pdevinfo);

	return 0;
}

static int mlb150_release(struct inode *inode, struct file *filp)
{
	int minor = MINOR(inode->i_rdev);
	struct mlb_data *drvdata = filp->private_data;

	pr_debug("release %s (syscall)\n", drvdata->devinfo[minor].dev_name);

	return mlb150_do_release(drvdata, minor, MLB_RDWR);
}

/* caddr: tx/rx channel address: 0xttttrrrr */
void mlb150_chan_setaddr(struct mlb_data *drvdata, int minor, unsigned caddr)
{
	struct mlb_dev_info *pdevinfo = &drvdata->devinfo[minor];
	struct mlb_channel_info *tx_ci = &pdevinfo->channels[TX_CHANNEL];
	struct mlb_channel_info *rx_ci = &pdevinfo->channels[RX_CHANNEL];

	if (unlikely(atomic_read(&pdevinfo->on))) {
		pr_debug("channel already started up\n");
		return;
	}

	tx_ci->address = (caddr >> 16) & 0xFFFF;
	rx_ci->address = caddr & 0xFFFF;

	pr_debug("set ch addr on minor %d (%s), channel type: %s, tx: %d 0x%x, rx: %d 0x%x\n",
		 minor, pdevinfo->dev_name,
		 ctype_names[pdevinfo->channel_type],
		 tx_ci->address, tx_ci->address,
		 rx_ci->address, rx_ci->address);
}

static int isoc_sync_fn(void *arg)
{
	static int start_quiet;
	struct mlb_data *drv = arg;
	struct mlb_dev_info *dev = ISOC_SYNC_DEV(drv);
	struct mlb_channel_info *ci = &dev->channels[TX_CHANNEL];
	struct mlb150_io_buffers bufs = MLB150_IO_BUFFERS_INIT(bufs);
	unsigned long flags;
	int is_on, test_free;
	int ret;

	if (!start_quiet) {
		dev_info(drv->dev, "started isoc interrupt trigger task\n");
		start_quiet = 1;
	}
repeat:
	set_current_state(TASK_INTERRUPTIBLE);
	if (kthread_should_stop()) {
		__set_current_state(TASK_RUNNING);
		goto finish;
	}
	set_current_state(TASK_RUNNING);
	test_free = 0;
	ret = get_tx_buffer(dev, &bufs);
	if (ret == 0) {
		mlb150_ext_put_dmabuf(&bufs, mlb150_ext_get_dmabuf(&bufs));
		put_tx_buffers(dev, &bufs);
		goto repeat;
	} else if (ret == -ESHUTDOWN)
		goto finish;
	else
		test_free = 1;
	is_on = 1;
	spin_lock_irqsave(&ci->free_slock, flags);
	ret = wait_event_interruptible_lock_irq(dev->tx_wq,
		ci->closing_tx ||
		!(is_on = atomic_read(&dev->on)) ||
		(test_free && !list_empty(&ci->free)),
		ci->free_slock);
	if (ci->closing_tx || !is_on)
		ret = -ESHUTDOWN;
	spin_unlock_irqrestore(&ci->free_slock, flags);
	if (ret == -ESHUTDOWN)
		goto finish;
	goto repeat;
finish:
	return 0;
}

static int start_isoc_sync_channel(struct mlb_dev_info *pdevinfo)
{
	int ret;
	struct mlb_data *drv = pdevinfo->drvdata;
	struct mlb_channel_info *tx_ci = &pdevinfo->channels[TX_CHANNEL];
	struct mlb_channel_info *rx_ci = &pdevinfo->channels[RX_CHANNEL];

	tx_ci->address = MLB_J6S4I_CHANNEL;
	rx_ci->address = 0;
	ret = mlb_sync_allocate_dmabufs(drv, pdevinfo, MLB_WRONLY,
					pdevinfo->bytes_per_frame,
					pdevinfo->sync_buf_size);
	if (ret != 0)
		goto fail;
	mlb_channel_enable(pdevinfo, MLB_WRONLY);
	drv->isoc_sync_task = kthread_create(isoc_sync_fn, drv, DRIVER_NAME "_isoc_sync");
	if (IS_ERR(drv->isoc_sync_task)) {
		ret = PTR_ERR(drv->isoc_sync_task);
		goto fail_disable;
	}
	wake_up_process(drv->isoc_sync_task);
	return 0;
fail_disable:
	mlb_channel_disable(pdevinfo);
	free_dma_bufs(drv->dev, pdevinfo);
fail:
	atomic_set(&drv->isoc_sync_on, 0);
	return ret;
}

static int isoc_sync_init(struct mlb_dev_info *pdevinfo, enum channelmode mode)
{
	struct mlb_data *drv = pdevinfo->drvdata;
	struct mlb_channel_info *tx_ci = &pdevinfo->channels[TX_CHANNEL];
	struct mlb_channel_info *rx_ci = &pdevinfo->channels[RX_CHANNEL];

	if ((is_reading(mode) && rx_ci->address == MLB_J6S4I_CHANNEL) ||
	    (is_writing(mode) && tx_ci->address == MLB_J6S4I_CHANNEL)) {
		char comm[TASK_COMM_LEN];

		pr_info(DRIVER_NAME ": %s[%ld] tries to use reserved channel %d through %s:%s%s\n",
			get_task_comm(comm, current), (long)current->pid,
			MLB_J6S4I_CHANNEL,
			pdevinfo->dev_name,
			rx_ci->address == MLB_J6S4I_CHANNEL ? " RX": "",
			tx_ci->address == MLB_J6S4I_CHANNEL ? " TX": "");
		return -EEXIST;
	}
	if (pdevinfo->channel_type != MLB150_CTYPE_ISOC)
		return 0;
	if (atomic_cmpxchg(&drv->isoc_sync_on, 0, 1) != 0)
		return 0;
	return start_isoc_sync_channel(ISOC_SYNC_DEV(drv));
}

static void isoc_sync_stop(struct mlb_dev_info *pdevinfo)
{
	uint i;
	ulong flags;
	struct mlb_data *drv = pdevinfo->drvdata;
	struct mlb_dev_info *dev = ISOC_SYNC_DEV(drv);
	struct mlb_channel_info *tx_ci = &dev->channels[TX_CHANNEL];

	/* see if there are active isoc channels */
	for (i = 0; i < used_minor_devices; ++i)
		if (drv->devinfo[i].channel_type == MLB150_CTYPE_ISOC &&
		    atomic_read(&drv->devinfo[i].on))
			return;
	if (atomic_cmpxchg(&drv->isoc_sync_on, 1, 0) != 1)
		return;
	spin_lock_irqsave(&tx_ci->free_slock, flags);
	tx_ci->closing_tx = 1;
	wake_up_interruptible(&dev->tx_wq);
	spin_unlock_irqrestore(&tx_ci->free_slock, flags);
	kthread_stop(drv->isoc_sync_task);
	mlb_channel_disable(dev);
	free_dma_bufs(drv->dev, dev);
}

static int mlb_chan_startup(struct mlb_dev_info *pdevinfo,
			    enum channelmode mode)
{
	unsigned int i;
	struct mlb_channel_info *tx_ci = &pdevinfo->channels[TX_CHANNEL];
	struct mlb_channel_info *rx_ci = &pdevinfo->channels[RX_CHANNEL];
	char comm[TASK_COMM_LEN];

	/*
	 * As the sync and isoc channels impose their own protocol they
	 * certainly cannot be used to talk over the reserved system
	 * (INIC administrative) channel.
	 */
	if ((pdevinfo->channel_type == MLB150_CTYPE_SYNC ||
	     pdevinfo->channel_type == MLB150_CTYPE_ISOC) &&
	    ((rx_ci->address == MLB_SYSTEM_CHANNEL && is_reading(mode)) ||
	     (tx_ci->address == MLB_SYSTEM_CHANNEL && is_writing(mode)))) {
		pr_info(DRIVER_NAME ": %s[%ld] tries to use reserved channel through %s:%s%s \n",
			get_task_comm(comm, current), (long)current->pid,
			pdevinfo->dev_name,
			rx_ci->address == MLB_SYSTEM_CHANNEL ? " RX": "",
			tx_ci->address == MLB_SYSTEM_CHANNEL ? " TX": "");
		return -EEXIST;
	}
	for (i = 0; i < used_minor_devices; ++i) {
		const struct mlb_dev_info *devinfo = &pdevinfo->drvdata->devinfo[i];

		if (devinfo == pdevinfo || !atomic_read(&devinfo->on))
			continue;

		if (is_writing(mode) &&
		    (devinfo->channels[TX_CHANNEL].address == tx_ci->address ||
		     devinfo->channels[RX_CHANNEL].address == tx_ci->address)) {
			pr_debug("%s uses MLB channel %u, needed for TX by %s\n",
				 devinfo->dev_name, tx_ci->address, pdevinfo->dev_name);
			return -ETXTBSY;
		}
		if (is_reading(mode) &&
		    (devinfo->channels[TX_CHANNEL].address == rx_ci->address ||
		     devinfo->channels[RX_CHANNEL].address == rx_ci->address)) {
			pr_debug("%s uses MLB channel %u, needed for RX by %s\n",
				 devinfo->dev_name, rx_ci->address, pdevinfo->dev_name);
			return -EBUSY;
		}
	}
	if (isoc_sync_quirk) {
		int ret = isoc_sync_init(pdevinfo, mode);
		if (ret)
			return ret;
	}
	pr_debug("start channel on %s, MLB addrs tx: %d, rx: %d\n",
		 pdevinfo->dev_name,
		 tx_ci->address,
		 rx_ci->address);
	mlb_channel_enable(pdevinfo, mode);
	return 0;
}

int mlb150_chan_startup(struct mlb_data *drvdata, int minor,
			enum channelmode mode)
{
	struct mlb_dev_info *pdevinfo = &drvdata->devinfo[minor];

	if (unlikely(atomic_read(&pdevinfo->on))) {
		pr_debug("channel already started up\n");
		return -EBUSY;
	}

	return mlb_chan_startup(pdevinfo, mode);
}

int mlb150_chan_shutdown(struct mlb_data *drvdata, int minor,
			 enum channelmode mode)
{
	struct mlb_dev_info *pdevinfo = &drvdata->devinfo[minor];

	if (unlikely(atomic_read(&pdevinfo->on) == 0)) {
		pr_debug("channel already shut down\n");
		return 0;
	}

	pr_debug("%ld[%s]: shutdown channel tx: %d, rx: %d\n",
		 (long)current->pid, pdevinfo->dev_name,
		 pdevinfo->channels[TX_CHANNEL].address, pdevinfo->channels[RX_CHANNEL].address);
	mlb_channel_disable(pdevinfo);
	return 0;
}

static int mlb_ioctl_clk_speed(struct mlb_dev_info *pdevinfo, void __user *argp)
{
	unsigned int fs;
	u32 c0_val, mlb_fs;

	/* get fs from user space */
	if (unlikely(copy_from_user(&fs, argp, sizeof(fs)))) {
		pr_debug("copy from user failed\n");
		return -EFAULT;
	}
	if (fs > 2048) {
		char comm[TASK_COMM_LEN];

		pr_err("MLB_SET_FPS: %s[%ld]: 6 pin MLB speed %u fs is above 2048; not supported\n",
		       get_task_comm(comm, current), (long)current->pid, fs);
		return -EINVAL;
	}

	/* check fs value */
	switch (fs) {
	case 256:
	case 512:
	case 1024:
		mlb_fs = fs >> 9;
		break;
	case 2048:
	case 3072:
	case 4096:
		mlb_fs = (fs >> 10) + 1;
		break;
	case 6144:
		mlb_fs = fs >> 10;
		break;
	case 8192:
		mlb_fs = (fs >> 10) - 1;
		break;
	default:
		pr_debug("invalid fs argument: %u\n", fs);
		return -EINVAL;
	}

	c0_val = READ_MLB_REG(REG_MLBC0);
	c0_val &= ~MLBC0_MLBCLK_MASK;
	c0_val |= mlb_fs << MLBC0_MLBCLK_SHIFT;

	/* Enable or disable 6-pin mode depending on fs set */
	if (fs <= 1024)
		c0_val &= ~MLBC0_MLBPEN;
	else
		c0_val |= MLBC0_MLBPEN;

	pr_debug("c0_val: 0x%x (before write reg)\n", c0_val);
	WRITE_MLB_REG(c0_val, REG_MLBC0);

	c0_val = READ_MLB_REG(REG_MLBC0);
	printk(KERN_DEBUG DRIVER_NAME": MLB_SET_FPS %s: fs %d, MLBC0: 0x%08x (%s lock)\n",
	       pdevinfo->dev_name, fs, c0_val,
	       c0_val & MLBC0_MLBLK ? "" : "no" );

	WRITE_MLB_REG(READ_MLB_REG(REG_MLBC0) | MLBC0_MLBEN, REG_MLBC0);
	WRITE_MLB_REG(READ_MLB_REG(REG_MLBC1) & 0xFFFFFF0F, REG_MLBC1);
	dump_registers();
	return 0;
}

static int __must_check valid_caddr(struct mlb_data *drvdata, struct mlb_dev_info *pdevinfo, unsigned int caddr)
{
	unsigned int tx, rx;
	if (!caddr)
		return -EINVAL;
	tx = (caddr >> 16) & 0xffff;
	rx = caddr & 0xffff;
	/* This allows selection of the reserved System Channel (logical 0) */
	if (tx > MLB_LAST_CHANNEL || rx > MLB_LAST_CHANNEL)
		return -ERANGE;
	return 0;
}

static long mlb150_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct inode *inode = file_inode(filp);
	struct mlb_data *drvdata = filp->private_data;
	struct mlb_dev_info *pdevinfo = NULL;
	void __user *argp = (void __user *)arg;
	unsigned long flags, event;
	int minor;

	minor = MINOR(inode->i_rdev);
	pdevinfo = &drvdata->devinfo[minor];

	/*
	 * Trace out ioctls on debug-enabled channels and all
	 * channel-independent (not using pdevinfo) ioctls
	 */
	if (debug_channel(pdevinfo) ||
	    MLB_SET_FPS      == cmd ||
	    MLB_GET_VER      == cmd ||
	    MLB_SET_DEVADDR  == cmd ||
	    MLB_GET_LOCK     == cmd ||
	    MLB_GET_ISOC_BUFSIZE == cmd)
		pr_debug("ioctl %s %s\n",
			 MLB_CHAN_SETADDR         == cmd ? "CHAN_SETADDR" :
			 MLB_CHAN_STARTUP         == cmd ? "CHAN_STARTUP" :
			 MLB_SET_FPS              == cmd ? "SET_FPS" :
			 MLB_GET_VER              == cmd ? "GET_VER" :
			 MLB_SET_DEVADDR          == cmd ? "SET_DEVADDR" :
			 MLB_CHAN_SHUTDOWN        == cmd ? "CHAN_SHUTDOWN" :
			 MLB_CHAN_GETEVENT        == cmd ? "CHAN_GETEVENT" :
			 MLB_SET_SYNC_QUAD        == cmd ? "SET_SYNC_QUAD" :
			 MLB_SYNC_CHAN_STARTUP    == cmd ? "SYNC_CHAN_STARTUP" :
			 MLB_GET_LOCK             == cmd ? "GET_LOCK":
			 MLB_GET_ISOC_BUFSIZE     == cmd ? "GET_ISOC_BUFSIZE" :
			 MLB_SET_ISOC_BLKSIZE_188 == cmd ? "SET_ISOC_BLKSIZE_188" :
			 MLB_SET_ISOC_BLKSIZE_196 == cmd ? "SET_ISOC_BLKSIZE_196" :
			 MLB_PAUSE_RX             == cmd ? "MLB_PAUSE_RX" :
			 MLB_RESUME_RX            == cmd ? "MLB_RESUME_RX" :
			 "unknown", pdevinfo->dev_name);

	switch (cmd) {
		enum channelmode direction;
		int ret;
	case MLB_CHAN_SETADDR:
		{
			unsigned int caddr;
			/* get channel address from user space */
			if (copy_from_user(&caddr, argp, sizeof(caddr))) {
				pr_debug("copy from user failed\n");
				return -EFAULT;
			}
			ret = valid_caddr(drvdata, pdevinfo, caddr);
			if (ret)
				return ret;
			if (unlikely(atomic_read(&pdevinfo->on))) {
				pr_debug("channel already started up\n");
				return -EBUSY;
			}
			mlb150_chan_setaddr(drvdata, minor, caddr);
			break;
		}
	case MLB_CHAN_STARTUP:
		if (pdevinfo->channel_type == MLB150_CTYPE_SYNC)
			return -EINVAL;
		if (unlikely(atomic_read(&pdevinfo->on))) {
			pr_debug("channel already started up\n");
			return -EBUSY;
		}
		switch (filp->f_flags & O_ACCMODE) {
		default:
		case O_RDWR:
			direction = MLB_RDWR;
			break;
		case O_RDONLY:
			direction = MLB_RDONLY;
			break;
		case O_WRONLY:
			direction = MLB_WRONLY;
			break;
		}
		ret = mlb_chan_allocate_dmabufs(drvdata, pdevinfo, direction, 0);
		return ret ? ret : mlb_chan_startup(pdevinfo, direction);

	case MLB_SYNC_CHAN_STARTUP:
		if (unlikely(atomic_read(&pdevinfo->on))) {
			pr_debug("channel already started up\n");
			return -EBUSY;
		}
		if (pdevinfo->channel_type != MLB150_CTYPE_SYNC)
			return -EINVAL;
		else {
			enum mlb_sync_ch_startup_mode mode;
			unsigned int bytes_per_frame;
			const int syncDevNr = minor - drvdata->minor_sync0;
			const unsigned dev_max_bytes_supported =
				mlb150_max_sync_dev_constraints[syncDevNr].bytes_per_ch *
				mlb150_max_sync_dev_constraints[syncDevNr].num_ch;

			/* get channel mode from user space */
			if (copy_from_user(&mode, argp, sizeof(mode))) {
				pr_debug("copy from user failed\n");
				return -EFAULT;
			}
			direction = (int)mode & 1 ? MLB_WRONLY : MLB_RDONLY;
			switch (mode) {
			case MLB_SYNC_MONO_RX:
			case MLB_SYNC_MONO_TX:
				bytes_per_frame = 1 * 2;
				break;
			case MLB_SYNC_STEREO_RX:
			case MLB_SYNC_STEREO_TX:
				bytes_per_frame = 2 * 2;
				break;
			case MLB_SYNC_51_RX:
			case MLB_SYNC_51_TX:
				bytes_per_frame = 6 * 2;
				break;
			case MLB_SYNC_51HQ_RX:
			case MLB_SYNC_51HQ_TX:
				bytes_per_frame = 6 * 3;
				break;
			case MLB_SYNC_STEREOHQ_RX:
			case MLB_SYNC_STEREOHQ_TX:
				bytes_per_frame = 2 * 3;
				break;
			default:
				return -EINVAL;
			}

			if (bytes_per_frame > dev_max_bytes_supported)
				return -EINVAL;

			ret = mlb_sync_allocate_dmabufs(drvdata,
				pdevinfo, direction, bytes_per_frame,
				max(pdevinfo->sync_buf_size,
				    SYNC_BUFFER_DEP(bytes_per_frame)));
			return ret ? ret : mlb_chan_startup(pdevinfo, direction);
		}
		break;
	case MLB_CHAN_SHUTDOWN:
		if (unlikely(atomic_read(&pdevinfo->on) == 0)) {
			pr_debug("channel is not started\n");
			return -EBADF;
		}
		ret = stop_tx(pdevinfo);
		mlb_channel_disable(pdevinfo);
		pr_debug("%s shut down (ret %d)\n", pdevinfo->dev_name, ret);
		return ret;
	case MLB_CHAN_GETEVENT:
		if (unlikely(atomic_read(&pdevinfo->on) == 0)) {
			pr_debug("channel is not started\n");
			return -EBADF;
		}
		/* get and clear the ex_event */
		spin_lock_irqsave(&pdevinfo->event_lock, flags);
		event = pdevinfo->ex_event;
		pdevinfo->ex_event = 0;
		spin_unlock_irqrestore(&pdevinfo->event_lock, flags);

		if (event) {
			pr_debug("%s: exception event 0x%lx\n", pdevinfo->dev_name, event);
			if (copy_to_user(argp, &event, sizeof(event))) {
				pr_debug("copy to user failed\n");
				return -EFAULT;
			}
		}
		return -EAGAIN;

	case MLB_SET_FPS:
		if (atomic_read(&pdevinfo->on)) {
			pr_debug("%s: channel already started\n", pdevinfo->dev_name);
			return -EBUSY;
		}
		return mlb_ioctl_clk_speed(pdevinfo, argp);

	case MLB_GET_LOCK:
		{
			u32 reg = READ_MLB_REG(REG_MLBC0);
			pr_debug("MLB_GET_LOCK: MLBC0: 0x%x\n", reg);
			reg &= MLBC0_MLBLK;
			if (unlikely(copy_to_user(argp, &reg, sizeof(reg))))
				return -EFAULT;
		}
		break;

	case MLB_GET_VER:
		{
			u32 version;

			/* get MLB device module version */
			version = 0x03030003;

#if DEBUG_MLBLOCK
			version |=
			    READ_MLB_REG(REG_MLBC0) & MLBC0_MLBLK;
#endif

			pr_debug("get version: 0x%08x\n", version);

			if (copy_to_user(argp, &version, sizeof(version))) {
				pr_err(DRIVER_NAME": copy to user failed\n");
				return -EFAULT;
			}
			break;
		}

	case MLB_SET_DEVADDR:
		{
			u32 c1_val;
			u8 devaddr;

			/* get MLB device address from user space */
			if (unlikely(copy_from_user
				     (&devaddr, argp, sizeof(unsigned char)))) {
				pr_err(DRIVER_NAME": copy from user failed\n");
				return -EFAULT;
			}

			c1_val = READ_MLB_REG(REG_MLBC1);
			c1_val &= ~MLBC1_NDA_MASK;
			c1_val |= devaddr << MLBC1_NDA_SHIFT;
			WRITE_MLB_REG(c1_val, REG_MLBC1);
			pr_debug("MLB_SET_DEVADDR: addr %u 0x%x, MLBC1: 0x%08x\n",
				 devaddr, devaddr,
				 READ_MLB_REG(REG_MLBC1));

			break;
		}

	case MLB_GET_ISOC_BUFSIZE:
		{
			/* return the size of this channel (recalculated in do_open()) */
			const unsigned int size = pdevinfo->isoc_blk_size * pdevinfo->isoc_blk_num;

			if (unlikely(copy_to_user(argp, &size, sizeof(size))))
				return -EFAULT;
		}
		break;
	case MLB_PAUSE_RX:
		{
			if (!dev_is_opt3(pdevinfo))
				return -EFAULT;
			pr_debug("opt3: Pausing channel!\n");
			atomic_set(&pdevinfo->pauseRx, 1);
		}
		break;
	case MLB_RESUME_RX:
		{
			struct mlb_channel_info *ci = &pdevinfo->channels[RX_CHANNEL];
			bool free_bufs_available;
			int read_ctr_ret;
			u32 ctr_val[4];
			u32 done_bits;
			unsigned long flags;

			if (!atomic_read(&pdevinfo->pauseRx))
				return -EFAULT;

			spin_lock_irqsave(&ci->free_slock, flags);
			free_bufs_available = (dmabuf_count(&ci->free) >= 2);
			spin_unlock_irqrestore(&ci->free_slock, flags);

			if (!free_bufs_available) {
				pr_err("MLB_RESUME_RX: No free buffers to start again!\n");
				return -EAGAIN;
			}

			spin_lock_irqsave(&ctr_lock, flags);
			read_ctr_ret = read_ctr_nolock(BUF_ADT_OFFSET + ci->address, ctr_val);
			/* Reset the flag under the lock! */
			atomic_set(&pdevinfo->pauseRx, 0);
			spin_unlock_irqrestore(&ctr_lock, flags);

			if (read_ctr_ret)
				return -EFAULT;
			/* Start ping- and/or pong if the corresponding ready bit is not set.
			 * Done bits are located one position to the right relative to ready.
			 */
			done_bits = ((ctr_val[1] & (ADT_RDY1 | ADT_RDY2)) >> 1) ^
				    (ADT_DNE1 | ADT_DNE2);
			if (done_bits) {
				spin_lock_irqsave(&ci->dmabuf_slock, flags);
				start_rx_based_on_done_bits(pdevinfo, hweight32(done_bits),
							    done_bits);
				spin_unlock_irqrestore(&ci->dmabuf_slock, flags);
			}
			else
				pr_debug("opt3: Starting up again (cleared flag)!\n");
		}
		break;
	default:
		pr_debug("invalid ioctl command\n");
		return -EINVAL;
	}

	return 0;
}

static ssize_t do_read(struct mlb_dev_info *pdevinfo,
		       unsigned int f_flags, char __user *buf, size_t count)
{
	int ret;
	unsigned long flags;
	struct mlb150_dmabuf *dmab = NULL;
	struct mlb_channel_info *ci = &pdevinfo->channels[RX_CHANNEL];
	unsigned int size = ci->buf_size;

	if (unlikely(size > count)) {
		pr_warn(DRIVER_NAME": RX user buffer (%zdB) is too small (%uB needed).\n",
			count, size);
		return -EINVAL;
	}

	do {
		ret = -EIO;
		spin_lock_irqsave(&ci->usr_slock, flags);
		if (list_empty(&ci->usr)) {

			if (f_flags & O_NONBLOCK) {
				ret = -EAGAIN;
				goto unlock;
			}
			if (debug_channel(pdevinfo))
				pr_debug("%ld: %s: read: waiting...\n",
					 (long)current->pid,
					 pdevinfo->dev_name);
			ret = wait_event_interruptible_lock_irq(pdevinfo->rx_wq,
								!list_empty(&ci->usr),
								ci->usr_slock);

			if (ret)
				goto unlock;
		}
		dmab = dmabuf_of(ci->usr.next);
		__list_del_entry(&dmab->head);
unlock:
		spin_unlock_irqrestore(&ci->usr_slock, flags);

		if (dmab) {
			ret = copy_to_user(buf, dmab->virt, size);
			free_channel_dmabuf(ci, dmab);

			if (ret)
				ret = -EFAULT;
		}
		if (ret) {
			pr_debug("%ld: %s: aborted read %d\n", (long)current->pid, pdevinfo->dev_name, ret);
			return ret;
		}
	} while (!dmab);

	if (debug_channel(pdevinfo))
		pr_debug("%ld: %s: read returning %d\n", (long)current->pid, pdevinfo->dev_name, size);
	return (ssize_t)size;
}

static ssize_t mlb150_read(struct file *filp, char __user *buf,
			   size_t count, loff_t *f_pos)
{
	ssize_t ret;
	struct mlb_data *drvdata = filp->private_data;
	int minor = MINOR(file_inode(filp)->i_rdev);
	struct mlb_dev_info *pdevinfo = &drvdata->devinfo[minor];

	if (debug_channel(pdevinfo))
		pr_debug("read %s count %zd\n", pdevinfo->dev_name, count);

	if (unlikely(!(READ_MLB_REG(REG_MLBC0) & MLBC0_MLBLK))) {
		pr_err("read %s count %zd without MLB Lock!\n", pdevinfo->dev_name, count);
		printk(KERN_DEBUG "read %s count %zd without MLB Lock!\n", pdevinfo->dev_name, count);
	}

	ret = do_read(pdevinfo, filp->f_flags, buf, count);

	if (ret >= 0)
		*f_pos = 0;

	return ret;
}

/* TODO this is not protected against disabled/shutdown channel */
static ssize_t do_write(struct mlb_dev_info *pdevinfo,
			const char __user *buf, size_t count,
			int nonblock)
{
	int ret;
	struct mlb_channel_info *ci = &pdevinfo->channels[TX_CHANNEL];
	struct mlb150_io_buffers bufs = MLB150_IO_BUFFERS_INIT(bufs);
	struct mlb150_dmabuf *dmab = NULL;

	if (debug_channel(pdevinfo))
		pr_debug("%ld: %s: write, count %zd%s\n",
			 (long)current->pid, pdevinfo->dev_name,
			 count, nonblock ? " (nonblock)" : "");

	do {
		ret = get_tx_buffer(pdevinfo, &bufs);

		if (ret == 0) {
			dmab = mlb150_ext_get_dmabuf(&bufs);
			poison_dmab(dmab, DMAB_TX_POISON, ci->buf_size);
			ret = copy_from_user(dmab->virt, buf, count);

			if (unlikely(ret)) {
				free_channel_dmabuf(ci, dmab);
				pr_err(DRIVER_NAME": copy from user failed\n");
				ret = -EFAULT;
				break;
			}
#if MLB_RX_TX_DUMP
			/*
			 * This is to make the crc16 calculation to produce
			 * usable output. It is not needed otherwise.
			 */
			if (pdevinfo->channel_type == MLB150_CTYPE_SYNC &&
			    (debug_mask & DEBUG_TRACE_SYNC) &&
			    count < ci->buf_size)
				memset(dmab->virt + count, 0, ci->buf_size - count);
#endif
			// This packet is also dumped later in put_tx_buffers
			// in the TX ISR. The code left here to find out if
			// something corrupts the data inbetween.
			// lld_dump(pdevinfo, dmab->virt, ci->buf_size, "T? %lx", dmab->phys);
			mlb150_ext_put_dmabuf(&bufs, dmab);
			ret = count;
		} else if (nonblock) {
			/* ret is either -EAGAIN or -ESHUTDOWN */
			break;
		} else {
			unsigned long flags;
			int is_on;

			spin_lock_irqsave(&ci->free_slock, flags);
			ret = wait_event_interruptible_lock_irq(pdevinfo->tx_wq,
				ci->closing_tx ||
				!list_empty(&ci->free) ||
				!(is_on = atomic_read(&pdevinfo->on)),
				ci->free_slock);

			if (ci->closing_tx || !is_on)
				ret = -ESHUTDOWN;

			spin_unlock_irqrestore(&ci->free_slock, flags);

			if (ret) /* -ERESTARTSYS */
				break;
		}
	} while (!dmab);

	if (dmab)
		put_tx_buffers(pdevinfo, &bufs);

	if (debug_channel(pdevinfo))
		pr_debug("%ld: %s: write = %d\n", (long)current->pid,
			 pdevinfo->dev_name, ret);
	return ret;
}

static ssize_t mlb150_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *f_pos)
{
	int minor = MINOR(file_inode(filp)->i_rdev);
	struct mlb_data *drvdata = filp->private_data;
	struct mlb_dev_info *pdevinfo = &drvdata->devinfo[minor];
	struct mlb_channel_info *pchinfo = &pdevinfo->channels[TX_CHANNEL];

	if (debug_channel(pdevinfo))
		pr_debug("write %s count %zd\n", pdevinfo->dev_name, count);

	if (unlikely(count > pchinfo->buf_size)) {
		pr_debug("attempt to write %zd bytes, buffer %u\n",
			 count, pchinfo->buf_size);
		return -EFBIG;
	}

	if (unlikely(!(READ_MLB_REG(REG_MLBC0) & MLBC0_MLBLK))) {
		pr_err("write %s count %zd without MLB Lock!\n", pdevinfo->dev_name, count);
		printk(KERN_DEBUG "write %s count %zd without MLB Lock!\n", pdevinfo->dev_name, count);
	}

	*f_pos = 0;
	return do_write(pdevinfo, buf, count, !!(filp->f_flags & O_NONBLOCK));
}

static unsigned int mlb150_poll(struct file *filp, poll_table *wait)
{
	unsigned int ret = 0;
	unsigned long flags, rqevents = poll_requested_events(wait);
	int minor = MINOR(file_inode(filp)->i_rdev);
	struct mlb_data *drvdata = filp->private_data;
	struct mlb_dev_info *pdevinfo = &drvdata->devinfo[minor];

	if (debug_channel(pdevinfo))
		pr_debug("poll %s tx ch %d, rx ch %d\n", pdevinfo->dev_name,
			 pdevinfo->channels[TX_CHANNEL].address,
			 pdevinfo->channels[RX_CHANNEL].address);

	if (rqevents & (POLLOUT | POLLWRNORM)) {
		struct mlb_channel_info *tx_ci = &pdevinfo->channels[TX_CHANNEL];

		poll_wait(filp, &pdevinfo->tx_wq, wait);
		/* check the tx buffer is avaiable or not */
		spin_lock_irqsave(&tx_ci->free_slock, flags);
		if (!list_empty(&tx_ci->free))
			ret |= POLLOUT | POLLWRNORM;
		spin_unlock_irqrestore(&tx_ci->free_slock, flags);
	}
	if (rqevents & (POLLIN | POLLRDNORM)) {
		struct mlb_channel_info *rx_ci = &pdevinfo->channels[RX_CHANNEL];

		poll_wait(filp, &pdevinfo->rx_wq, wait);
		spin_lock_irqsave(&rx_ci->usr_slock, flags);
		if (!list_empty(&rx_ci->usr))
			ret |= POLLIN | POLLRDNORM;
		spin_unlock_irqrestore(&rx_ci->usr_slock, flags);
	}
	/* check the exception event */
	spin_lock_irqsave(&pdevinfo->event_lock, flags);
	if (pdevinfo->ex_event)
		ret |= POLLIN | POLLRDNORM; /* FIXME maybe better POLLPRI or POLLRDBAND? */
	spin_unlock_irqrestore(&pdevinfo->event_lock, flags);

	/*
	 * POLLHUP is not linked to the channel being enabled
	 * (mlb_dev_info->on) because the MLB channels can be re-enabled,
	 * which is not a normal behaviour for the handles, which report
	 * POLLHUP: the MLB channel has no "end-of-file" condition.
	 */
	return ret;
}

/*!
 * char dev file operations structure
 */
static const struct file_operations mlb150_fops = {
	.owner = THIS_MODULE,
	.open = mlb150_open,
	.release = mlb150_release,
	.unlocked_ioctl = mlb150_ioctl,
	.poll = mlb150_poll,
	.read = mlb150_read,
	.write = mlb150_write,
};

static void init_dev_info(struct mlb_dev_info *pdevinfo,
			  enum mlb150_channel_type ctype)
{
	struct mlb_channel_info *tx_ci = &pdevinfo->channels[TX_CHANNEL];
	struct mlb_channel_info *rx_ci = &pdevinfo->channels[RX_CHANNEL];

	pdevinfo->channel_type = ctype;
	if (ctype != MLB150_CTYPE_SYNC)
		pdevinfo->bytes_per_frame = 0;
	spin_lock_init(&tx_ci->dmabuf_slock);
	spin_lock_init(&tx_ci->free_slock);
	spin_lock_init(&tx_ci->usr_slock);
	init_mlb_channel_info_dma(tx_ci);
	free_list_size_set(tx_ci);
	spin_lock_init(&rx_ci->dmabuf_slock);
	spin_lock_init(&rx_ci->free_slock);
	spin_lock_init(&rx_ci->usr_slock);
	init_mlb_channel_info_dma(rx_ci);
	free_list_size_set(rx_ci);
	atomic_set(&pdevinfo->on, 0);
	atomic_set(&pdevinfo->opencnt, 0);
	init_waitqueue_head(&pdevinfo->rx_wq);
	init_waitqueue_head(&pdevinfo->tx_wq);
	spin_lock_init(&pdevinfo->event_lock);
	setup_timer(&rx_ci->isr_timer, rx_isr_defer_poll, (ulong)pdevinfo);
	setup_timer(&tx_ci->isr_timer, tx_isr_defer_poll, (ulong)pdevinfo);
	/*
	 * In all cases, set the isoc sizes to invalid values as this
	 * will be recalculated when opening the device.
	 */
	pdevinfo->isoc_blk_size = 0;
	pdevinfo->isoc_blk_num  = 0;
#if DEBUG_POISON
	spin_lock_init(&rx_ci->pmp_lock);
	INIT_LIST_HEAD(&rx_ci->pmp);
	spin_lock_init(&tx_ci->pmp_lock);
	INIT_LIST_HEAD(&tx_ci->pmp);
#endif
}

static uint set_channel_buf_size(struct mlb_channel_info *ci,
				 uint buf_size,
				 u32 dbr_buf_head,
				 uint dbr_size)
{
	ci->buf_size = buf_size;
	ci->dbr_size = dbr_size;
	ci->dbr_buf_head = dbr_buf_head;
	return ci->dbr_size;
}

#define DBR_ALLOC(size) \
	({ u32 __start = dbr_buf_head; dbr_buf_head += (size); __start; })

static int alloc_minor_devices(struct mlb_data *drvdata)
{
	uint i;
	u32 dbr_buf_head;
	struct mlb_dev_info *d;

	used_minor_devices = 1 + !!opt3_flag /* ctrl */ + 1 /* async */ +
		number_sync_channels + number_isoc_channels +
		!!isoc_sync_quirk;

	drvdata->devinfo = kzalloc(sizeof(struct mlb_dev_info) *
				   used_minor_devices, GFP_KERNEL);

	if (!drvdata->devinfo) {
		dev_err(drvdata->dev,
			"cannot allocate enough memory for %d minor devices\n",
			used_minor_devices);
		return -ENOMEM;
	}

	for (d = drvdata->devinfo; d < drvdata->devinfo + used_minor_devices; ++d)
		d->drvdata = drvdata;

	d = drvdata->devinfo;
	dbr_buf_head = CH_CTRL_DBR_BUF_OFFSET;

	drvdata->minor_ctrl0 = 0;
	init_dev_info(d, MLB150_CTYPE_CTRL);
	set_channel_buf_size(&d->channels[TX_CHANNEL], CH_CTRL_BUF_DEP,
			     DBR_ALLOC(CH_CTRL_BUF_DEP), CH_CTRL_BUF_DEP);
	set_channel_buf_size(&d->channels[RX_CHANNEL], CH_CTRL_BUF_DEP,
			     DBR_ALLOC(CH_CTRL_BUF_DEP), CH_CTRL_BUF_DEP);
	sprintf(d->dev_name, "%s", ctype_names[d->channel_type]);
	++d;
	if (opt3_flag) {
		drvdata->minor_opt30 = d - drvdata->devinfo;
		init_dev_info(d, MLB150_CTYPE_CTRL);
		set_channel_buf_size(&d->channels[TX_CHANNEL], CH_CTRL_BUF_DEP,
				     DBR_ALLOC(CH_CTRL_BUF_DEP),
				     CH_CTRL_BUF_DEP);
		set_channel_buf_size(&d->channels[RX_CHANNEL], CH_CTRL_BUF_DEP,
				     d->channels[TX_CHANNEL].dbr_buf_head,
				     d->channels[TX_CHANNEL].dbr_size);
		strcpy(d->dev_name, "opt3");
		++d;
	}
	dev_info(drvdata->dev, "used DBR after ctrl: %u\n", dbr_buf_head);

	drvdata->minor_async0 = d - drvdata->devinfo;
	init_dev_info(d, MLB150_CTYPE_ASYNC);
	set_channel_buf_size(&d->channels[TX_CHANNEL], CH_ASYNC_BUF_DEP,
			     DBR_ALLOC(2 * CH_ASYNC_BUF_DEP),
			     2 * CH_ASYNC_BUF_DEP);
	set_channel_buf_size(&d->channels[RX_CHANNEL], CH_ASYNC_BUF_DEP,
			     DBR_ALLOC(CH_ASYNC_BUF_DEP), CH_ASYNC_BUF_DEP);
	sprintf(d->dev_name, "%s", ctype_names[d->channel_type]);
	dev_info(drvdata->dev, "used DBR after async: %u\n", dbr_buf_head);
	++d;

	/*
	 * Sync devices have restrictions placed on DBR sizes defined in table
	 * mlb150_max_sync_dev_constraints.
	 */
	drvdata->minor_sync0 = d - drvdata->devinfo;
	for (i = 0; i < number_sync_channels; ++i, ++d) {
		const unsigned bpf =
			mlb150_max_sync_dev_constraints[i].bytes_per_ch *
			mlb150_max_sync_dev_constraints[i].num_ch;

		d->bytes_per_frame = SYNC_MIN_FRAME_SIZE;
		d->sync_buf_size = SYNC_DMA_MIN_SIZE;
		init_dev_info(d, MLB150_CTYPE_SYNC);
		set_channel_buf_size(&d->channels[TX_CHANNEL],
				     SYNC_BUFFER_DEP(bpf),
				     DBR_ALLOC(SYNC_BUFFER_DEP(bpf)),
				     SYNC_BUFFER_DEP(bpf));
		set_channel_buf_size(&d->channels[RX_CHANNEL],
				     SYNC_BUFFER_DEP(bpf),
				     d->channels[TX_CHANNEL].dbr_buf_head,
				     d->channels[TX_CHANNEL].dbr_size);
		sprintf(d->dev_name, "%s%d", ctype_names[d->channel_type], i);
		/* Buffer depth in ADT entries for sync channels is 13 bits */
		if (d->sync_buf_size > 0xfffu) {
			dev_err(drvdata->dev,
				"%s: required DBR space (%u) overflows CDT buffer depth\n",
				d->dev_name, d->sync_buf_size);
			goto fail;
		}
		dev_info(drvdata->dev,
			 "used DBR after sync%u: %u (%u bytes) (DMA %u, max %u)\n",
			 i, dbr_buf_head, d->channels[TX_CHANNEL].dbr_size,
			 d->sync_buf_size, sync_max_dma_size(bpf));
	}
	
	if (dbr_buf_head > DBR_CAPACITY) {
		dev_err(drvdata->dev,
			"Not enough DBR for %u sync channels\n",
			number_sync_channels);
		goto fail;
	}

	if (isoc_sync_quirk) {
		struct mlb_dev_info *pdevinfo = ISOC_SYNC_DEV(drvdata);

		pdevinfo->drvdata = drvdata;
		pdevinfo->bytes_per_frame = SYNC_MIN_FRAME_SIZE;
		pdevinfo->sync_buf_size = SYNC_BUFFER_DEP(pdevinfo->bytes_per_frame);
		atomic_set(&pdevinfo->opencnt, 1);
		init_dev_info(pdevinfo, MLB150_CTYPE_SYNC);
		set_channel_buf_size(&pdevinfo->channels[TX_CHANNEL],
				     pdevinfo->sync_buf_size,
				     DBR_ALLOC(pdevinfo->sync_buf_size),
				     pdevinfo->sync_buf_size);
		set_channel_buf_size(&pdevinfo->channels[RX_CHANNEL],
				     pdevinfo->sync_buf_size,
				     d->channels[TX_CHANNEL].dbr_buf_head,
				     d->channels[TX_CHANNEL].dbr_size);
		strcpy(pdevinfo->dev_name, "isoc_sync");
		dev_info(drvdata->dev, "used DBR after %s: %u\n",
			 pdevinfo->dev_name, dbr_buf_head);
	}

	/* Put ISOC as last because of DBR allocation: ISOC takes the remaining size.
	 * Divide remaining size by number of isoc channels */
	if (number_isoc_channels)
		dbr_size_per_isoc = (DBR_CAPACITY - dbr_buf_head) / number_isoc_channels;
	dev_info(drvdata->dev, "DBR: remaining: %u, per isoc: %u\n",
		(DBR_CAPACITY - dbr_buf_head), dbr_size_per_isoc);

	/* check minumum isoc buffer size */
	if (number_isoc_channels &&
	    dbr_size_per_isoc < CH_ISOC_BLK_NUM_MIN * CH_ISOC_BLK_SIZE_MAX) {
		dev_err(drvdata->dev,
			"Not enough DBR space for %u isoc channels\n",
			number_isoc_channels);
		goto fail;
	}

	/* Set maximum number of isoc buffers possible */
	set_current_isoc_blk_num(dbr_size_per_isoc / CH_ISOC_BLK_SIZE_MAX);

	drvdata->minor_isoc0 = d - drvdata->devinfo;
	for (i = 0; i < number_isoc_channels; ++i, ++d) {
		init_dev_info(d, MLB150_CTYPE_ISOC);
		/* The DBR size equals the DMA buffer size for all other
		 * channel types than isoc. Isoc DBR size can be larger
		 * than the isoc buffer size as it is just the remaining
		 * DBR size.
		 * Moreover, isoc buffer size varies due to parameters
		 * being changeable through sysfs. Isoc buffer size is
		 * recalculated in mlb150_do_open() using the current
		 * sysfs settings at that time. Thus, we can set it to
		 * zero now. */
		set_channel_buf_size(&d->channels[TX_CHANNEL], 0,
				     DBR_ALLOC(dbr_size_per_isoc),
				     dbr_size_per_isoc);
		set_channel_buf_size(&d->channels[RX_CHANNEL], 0,
				     d->channels[TX_CHANNEL].dbr_buf_head,
				     d->channels[TX_CHANNEL].dbr_size);
		strcpy(d->dev_name, ctype_names[d->channel_type]);
		if (number_isoc_channels > 1)
			sprintf(d->dev_name + strlen(d->dev_name), "%d", i);
	}

	if (dbr_buf_head > DBR_CAPACITY) {
		dev_err(drvdata->dev,
			"DMA buffer space for %d minor devices exceeds the available DBR capacity\n",
			used_minor_devices);
		goto fail;
	}

	dev_info(drvdata->dev, "minors: ctrl0 %d, opt30 %d, async0 %d, isoc0 %d, sync0 %d\n",
		drvdata->minor_ctrl0, drvdata->minor_opt30, drvdata->minor_async0,
		drvdata->minor_isoc0, drvdata->minor_sync0);
	return 0;
fail:
	kfree(drvdata->devinfo);
	drvdata->devinfo = NULL;
	return -EFBIG;
}

#if MLB_RX_TX_STATISTICS
static void update_stats(struct mlb_dev_info *pdevinfo, int rxtx, unsigned pkt_size)
{
	unsigned long flags;

	write_lock_irqsave(&pdevinfo->stat_lock, flags);

	if (rxtx == RX_CHANNEL) {
		pdevinfo->rx_bytes += pkt_size;
		pdevinfo->rx_pkts++;
	} else {
		pdevinfo->tx_bytes += pkt_size;
		pdevinfo->tx_pkts++;
	}

	write_unlock_irqrestore(&pdevinfo->stat_lock, flags);
}

static void update_pmp_stats(struct mlb_dev_info *pdevinfo, int rxtx, const void *pmh)
{
	unsigned buf_size = pdevinfo->channels[rxtx].buf_size;
	u16 pml = be16_to_cpup(pmh);

	if (unlikely(2u + pml > buf_size || pml < 3u))
		pr_err_ratelimited(DRIVER_NAME ": invalid PML 0x%02x%02x (0x%04x)\n",
				   *(u8 *)pmh, *((u8 *)pmh + 1), pml);
	else
		buf_size = 2u + pml;

	update_stats(pdevinfo, rxtx, buf_size);
}

static void update_drop_stats(struct mlb_dev_info *pdevinfo, int rxtx)
{
	unsigned long flags;

	write_lock_irqsave(&pdevinfo->stat_lock, flags);

	if (rxtx == RX_CHANNEL)
		pdevinfo->rx_drops++;
	else
		pdevinfo->tx_drops++;

	write_unlock_irqrestore(&pdevinfo->stat_lock, flags);
}

static ssize_t show_dev_stats(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mlb_dev_info *devinfo = container_of(attr, struct mlb_dev_info, attr);
	ssize_t ret;
	unsigned long flags;

	read_lock_irqsave(&devinfo->stat_lock, flags);
	ret = scnprintf(buf, PAGE_SIZE, "%lld %lld %lld %lld %lld %lld\n",
			devinfo->rx_bytes, devinfo->tx_bytes,
			devinfo->rx_pkts, devinfo->tx_pkts,
			devinfo->rx_drops, devinfo->tx_drops);
	read_unlock_irqrestore(&devinfo->stat_lock, flags);
	return ret;
}

static ssize_t show_buffer_size(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mlb_dev_info *pdevinfo = container_of(attr, struct mlb_dev_info, bufsize_attr);

	return snprintf(buf, PAGE_SIZE, "%u", pdevinfo->sync_buf_size);
}

static ssize_t store_buffer_size(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct mlb_dev_info *pdevinfo = container_of(attr, struct mlb_dev_info, bufsize_attr);
	unsigned val;
	int ret = kstrtouint(buf, 0, &val);

	/*
	 * Only do the most generic range check, the complete validation
	 * happens after the bytes-per-frame is known when allocating DMA
	 * buffers.
	 */
	if (ret == 0) {
		ret = len;
		if (SYNC_DMA_MIN_SIZE <= val && val <= SYNC_DMA_MAX_SIZE)
			pdevinfo->sync_buf_size = val;
		else {
			dev_err(dev, "buffer size (%u) out of range [%u,%u]\n",
				val, SYNC_DMA_MIN_SIZE, SYNC_DMA_MAX_SIZE);
			ret = -EINVAL;
		}
	}
	return ret;
}

static ssize_t show_dev_dump(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "# write 1 to dump\n");
}

static ssize_t store_dev_dump(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct mlb_dev_info *pdevinfo = container_of(attr, struct mlb_dev_info,
						     dumpattr);
	int val, ret, pfx;
	char tmp[32];

	ret = min_t(size_t, ARRAY_SIZE(tmp) - 1, len);
	memcpy(tmp, buf, ret);
	tmp[ret] = '\0';
	pfx = strspn(tmp, "0123456789xboh");
	tmp[pfx] = '\0';
	ret = kstrtoint(tmp, 0, &val);
	if (ret)
		return ret;
	switch (val) {
	case 0:
		dump_mlb_diag(pdevinfo, 0);
		break;
	case 8:
		dump_driver_diag(pdevinfo, buf + pfx, len - pfx);
		break;
	default:
		dump_mlb_diag(pdevinfo, DIAG_DBR | DIAG_CTR);
	}
	return len;
}

static void init_statistics(struct device *dev, struct mlb_dev_info *devinfo)
{
	int ret;

	rwlock_init(&devinfo->stat_lock);
	sysfs_attr_init(&devinfo->attr.attr);
	devinfo->attr.attr.mode = 0444;
	devinfo->attr.attr.name = "stat";
	devinfo->attr.show = show_dev_stats;
	ret = device_create_file(dev, &devinfo->attr);
	if (ret) {
		dev_err(dev, "cannot create attribute '%s': %d\n",
			devinfo->attr.attr.name, ret);
		devinfo->attr.attr.mode = 0;
	}

	if (devinfo->channel_type == MLB150_CTYPE_SYNC) {
		sysfs_attr_init(&devinfo->bufsize_attr.attr);
		devinfo->bufsize_attr.attr.mode = 0644;
		devinfo->bufsize_attr.attr.name = "buffer_size";
		devinfo->bufsize_attr.show = show_buffer_size;
		devinfo->bufsize_attr.store = store_buffer_size;
		ret = device_create_file(dev, &devinfo->bufsize_attr);
		if (ret) {
			dev_err(dev, "cannot create attribute '%s': %d\n",
				devinfo->bufsize_attr.attr.name, ret);
			devinfo->bufsize_attr.attr.mode = 0;
		}
	}

	sysfs_attr_init(&devinfo->dumpattr.attr);
	devinfo->dumpattr.attr.mode = 0600;
	devinfo->dumpattr.attr.name = "dump";
	devinfo->dumpattr.show = show_dev_dump;
	devinfo->dumpattr.store = store_dev_dump;
	ret = device_create_file(dev, &devinfo->dumpattr);
	if (ret) {
		dev_err(dev, "cannot create attribute '%s': %d\n",
			devinfo->dumpattr.attr.name, ret);
		devinfo->dumpattr.attr.mode = 0;
	}
}

static void remove_statistics(struct mlb_data *drvdata, unsigned int count)
{
	unsigned i;

	for (i = 0; i < used_minor_devices; ++i) {
		if (drvdata->devinfo[i].attr.attr.mode)
			device_remove_file(drvdata->dev,
					   &drvdata->devinfo[i].attr);
		if (drvdata->devinfo[i].bufsize_attr.attr.mode)
			device_remove_file(drvdata->dev,
					   &drvdata->devinfo[i].bufsize_attr);
		if (drvdata->devinfo[i].dumpattr.attr.mode)
			device_remove_file(drvdata->dev,
					   &drvdata->devinfo[i].dumpattr);
	}
}
#else
static void init_statistics(struct device *dev, struct mlb_dev_info *pdevinfo) {}
static void remove_statistics(struct mlb_data *drvdata, unsigned int count) {}
static void update_stats(struct mlb_dev_info *pdevinfo, int rxtx, unsigned buf_size) {}
static void update_pmp_stats(struct mlb_dev_info *pdevinfo, int rxtx, const void *pmh) {}
static void update_drop_stats(struct mlb_dev_info *pdevinfo, int rxtx) {}
#endif /* MLB_RX_TX_STATISTICS */

/*!
 * This function is called whenever the MLB device is detected.
 * TODO It can be made __init, the MLB hardware is not usually hotpluggable
 */
static int mlb150_probe(struct platform_device *pdev)
{
	int ret, mlb_major, target_cpu, i;
	struct mlb_data *drvdata;

	if (mlb_base) {
		pr_debug("already initialized\n");
		return 0;
	}
	pr_debug("mlb150_probe() called!\n");

	drvdata = kzalloc(sizeof(struct mlb_data), GFP_KERNEL);
	if (!drvdata) {
		dev_err(&pdev->dev, "can't allocate enough memory\n");
		return -ENOMEM;
	}

	drvdata->dev = &pdev->dev;
	ret = alloc_minor_devices(drvdata);

	if (unlikely(ret))
		goto err_mem;

	ret = alloc_chrdev_region(&drvdata->firstdev, 0, used_minor_devices, "mlb150");

	if (unlikely(ret < 0)) {
		dev_err(&pdev->dev, "cannot allocate chardev data\n");
		goto err_reg;
	}

	mlb_major = MAJOR(drvdata->firstdev);
	dev_dbg(&pdev->dev, "MLB device major: %d\n", mlb_major);

	cdev_init(&drvdata->cdev, &mlb150_fops);
	drvdata->cdev.owner = THIS_MODULE;

	ret = cdev_add(&drvdata->cdev, drvdata->firstdev, used_minor_devices);
	if (unlikely(ret < 0)) {
		dev_err(&pdev->dev, "can't add cdev\n");
		goto err_reg;
	}

	/* create class and device for udev information */
	drvdata->class.name = DRIVER_NAME;
	drvdata->class.owner = THIS_MODULE;
	ret = class_register(&drvdata->class);
	if (unlikely(ret)) {
		dev_err(&pdev->dev, "failed to create device class: %d\n", ret);
		goto err_reg;
	}

	for (i = 0; i < used_minor_devices; i++) {
		struct device *class_dev;

		class_dev = device_create(&drvdata->class, NULL,
				MKDEV(mlb_major, i),
				NULL, drvdata->devinfo[i].dev_name);
		if (unlikely(IS_ERR(class_dev))) {
			dev_err(&pdev->dev, "failed to create mlb150 %s class device\n",
				drvdata->devinfo[i].dev_name);
			ret = -ENOMEM;
			goto err_dev;
		}

		init_statistics(class_dev, &drvdata->devinfo[i]);
	}

	ret = mlb150_get_mlb_io(drvdata);
	if (ret)
		goto err_dev;

	/* AHB0 IRQ */
	dev_dbg(&pdev->dev, "ahb0_virq: %d\n", drvdata->irq_ahb0);
	ret = request_irq(drvdata->irq_ahb0, mlb_ahb_isr, IRQF_TRIGGER_HIGH, "mlb_ahb0", drvdata);
	if (ret) {
		dev_err(&pdev->dev, "can't claim ahb0 irq %d\n", drvdata->irq_ahb0);
		drvdata->irq_ahb0 = 0;
		drvdata->irq_ahb1 = 0;
		drvdata->irq_mlb = 0;
		goto err_irq;
	}

	target_cpu = irq_cpu;
	if (target_cpu >= 0) {
		if (!cpu_online(target_cpu)) {
			dev_err(&pdev->dev, "irq_cpu %i is not online. "
				"Not setting irq and kthread CPU affinity.",
				target_cpu);
			target_cpu = -1;
		}
	} else
		dev_info(&pdev->dev, "Not configuring CPU affinity");

	if (target_cpu >= 0)
		irq_set_affinity(drvdata->irq_ahb0, cpumask_of(target_cpu));

	/* AHB1 IRQ */
	dev_dbg(&pdev->dev, "ahb1_virq: %d\n", drvdata->irq_ahb1);
	ret = request_irq(drvdata->irq_ahb1, mlb_ahb_isr, IRQF_TRIGGER_HIGH, "mlb_ahb1", drvdata);
	if (ret) {
		dev_err(&pdev->dev, "can't claim ahb1 irq %d\n", drvdata->irq_ahb1);
		drvdata->irq_ahb1 = 0;
		drvdata->irq_mlb = 0;
		goto err_irq;
	}

	if (target_cpu >= 0)
		irq_set_affinity(drvdata->irq_ahb1, cpumask_of(target_cpu));

	/* MLB IRQ */
	dev_dbg(&pdev->dev, "mlb_virq: %d\n", drvdata->irq_mlb);
	ret = request_irq(drvdata->irq_mlb, mlb_isr, IRQF_TRIGGER_HIGH, "mlb", drvdata);
	if (ret) {
		dev_err(&pdev->dev, "can't claim mlb irq %d\n", drvdata->irq_mlb);
		drvdata->irq_mlb = 0;
		goto err_irq;
	}

	if (target_cpu >= 0)
		irq_set_affinity(drvdata->irq_mlb, cpumask_of(target_cpu));

	ret = mlb150_init_mlb_io(drvdata);
	if (ret)
		goto err_irq;

	init_kthread_worker(&drvdata->isr_wrk);
	drvdata->isr_task = kthread_create(kthread_worker_fn,
		&drvdata->isr_wrk, DRIVER_NAME "_ahb_isr");
	if (likely(!IS_ERR(drvdata->isr_task))) {
		if (target_cpu >= 0)
			kthread_bind(drvdata->isr_task, target_cpu);

		wake_up_process(drvdata->isr_task);
	}

	if (IS_ERR(drvdata->isr_task))
		goto err_irq;

	sched_setscheduler(drvdata->isr_task, SCHED_FIFO,
			   &(struct sched_param){ .sched_priority = MAX_RT_PRIO / 2 });

	mlb_base = drvdata->mlbregs;
	mlb150_dev_init(drvdata);

	dump_registers();

	/* Enable access to the driver extension interface */
	ext_set_drvdata(drvdata);
	platform_set_drvdata(pdev, drvdata);

#if DEBUG_POISON
	invalid_pmp_task = kthread_run(invalid_pmp_fn, drvdata, DRIVER_NAME "_pmp_chk");
	{
		const struct sched_param param = {
			.sched_priority = MAX_RT_PRIO / 2 -1
		};
		sched_setscheduler(invalid_pmp_task, SCHED_FIFO, &param);
	}
#endif
	pr_debug("mlb150_probe(): success!\n");
	return 0;

err_irq:
	mlb150_free_mlb_io(drvdata);
	remove_statistics(drvdata, used_minor_devices);
err_dev:
	while (i-- > 0)
		device_destroy(&drvdata->class, MKDEV(mlb_major, i));
	class_unregister(&drvdata->class);
err_reg:
	unregister_chrdev_region(drvdata->firstdev, used_minor_devices);
err_mem:
	kfree(drvdata->devinfo);
	kfree(drvdata);
	pr_debug("mlb150_probe() failed! :-(\n");

	return ret;
}

static int mlb150_remove(struct platform_device *pdev)
{
	int i;
	struct mlb_data *drvdata = platform_get_drvdata(pdev);

#if DEBUG_POISON
	if (!IS_ERR_OR_NULL(invalid_pmp_task))
		kthread_stop(invalid_pmp_task);
#endif
	if (mlb_base)
		mlb150_dev_exit();

	if (!drvdata)
		return 0;

	if (!IS_ERR_OR_NULL(drvdata->isr_task))
		kthread_stop(drvdata->isr_task);

	remove_statistics(drvdata, used_minor_devices);
	mlb150_free_mlb_io(drvdata);

	for (i = used_minor_devices - 1; i >= 0; i--)
		device_destroy(&drvdata->class,
			       MKDEV(MAJOR(drvdata->firstdev), i));
	class_unregister(&drvdata->class);

	cdev_del(&drvdata->cdev);

	/* Unregister the two MLB devices */
	unregister_chrdev_region(drvdata->firstdev, used_minor_devices);

	kfree(drvdata->devinfo);
	kfree(drvdata);

	return 0;
}

#ifdef CONFIG_PM
static int mlb150_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int mlb150_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define mlb150_suspend NULL
#define mlb150_resume NULL
#endif


/* Attributes are available at [sysfs]/bus/platform/drivers/mlb150/ */
static struct attribute *mlb150_attrs[] = {
	&driver_attr_isoc_blk_sz.attr,
	&driver_attr_isoc_blk_num.attr,
	NULL
};
ATTRIBUTE_GROUPS(mlb150);

/*!
 * platform driver structure for MLB
 */
static struct platform_driver mlb150_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mlb150_of_device_ids,
		.groups = mlb150_groups,
	},
	.remove = mlb150_remove,
	.suspend = mlb150_suspend,
	.resume = mlb150_resume,
};

static int __init mlb150_init(void)
{
	const struct module *mod = THIS_MODULE;

	pr_info(DRIVER_NAME " version %s %s\n",
		mod ? mod->version : MLB_DRIVER_VERSION" (built-in)", mod ? mod->srcversion : "");
	if (number_sync_channels > MLB_MAX_SYNC_DEVICES) {
		pr_warn("Requested number of sync channels (%d) too large; limited to %d.\n",
			number_sync_channels, MLB_MAX_SYNC_DEVICES);
		number_sync_channels = MLB_MAX_SYNC_DEVICES;
	}
	if (number_isoc_channels > MLB_MAX_ISOC_DEVICES) {
		pr_warn("Requested number of isochronous channels (%u) too large; limited to %d.\n",
			number_isoc_channels, MLB_MAX_ISOC_DEVICES);
		number_isoc_channels = MLB_MAX_ISOC_DEVICES;
	}
	if (number_isoc_channels == 0) {
		pr_warn("Requested number of isochronous channels is zero. Must be at least one!\n");
		number_isoc_channels = 1; /* minimum required. */
	}

	return mlb150_init_driver(&mlb150_driver, mlb150_probe);
}

static void __exit mlb150_exit(void)
{
	mlb150_exit_driver(&mlb150_driver);
	pr_debug("MLB driver removed\n");
}

module_init(mlb150_init);
module_exit(mlb150_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc. && Cetitec GmbH");
MODULE_DESCRIPTION("MLB150 low level driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(MLB_DRIVER_VERSION);
