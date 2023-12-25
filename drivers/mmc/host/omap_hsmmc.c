/*
 * drivers/mmc/host/omap_hsmmc.c
 *
 * Driver for OMAP2430/3430 MMC controller.
 *
 * Copyright (C) 2007 Texas Instruments.
 *
 * Authors:
 *	Syed Mohammed Khasim	<x0khasim@ti.com>
 *	Madhusudhan		<madhu.cr@ti.com>
 *	Mohit Jalori		<mjalori@ti.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/dmaengine.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/omap-dmaengine.h>
#include <linux/mmc/host.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/platform_data/mmc-omap.h>
#include <linux/thermal.h>
#include <linux/pinctrl/pinctrl.h>
#include "denso_health.h"
#include <linux/mtd/mtd.h>
#include "../../misc/denso_hw_ctrl/src/mtd_api.h"

/* OMAP HSMMC Host Controller Registers */
#define OMAP_HSMMC_SYSSTATUS	0x0014
#define OMAP_HSMMC_CON		0x002C
#define OMAP_HSMMC_DLL		0x0034
#define OMAP_HSMMC_SDMASA	0x0100
#define OMAP_HSMMC_BLK		0x0104
#define OMAP_HSMMC_ARG		0x0108
#define OMAP_HSMMC_CMD		0x010C
#define OMAP_HSMMC_RSP10	0x0110
#define OMAP_HSMMC_RSP32	0x0114
#define OMAP_HSMMC_RSP54	0x0118
#define OMAP_HSMMC_RSP76	0x011C
#define OMAP_HSMMC_DATA		0x0120
#define OMAP_HSMMC_PSTATE	0x0124
#define OMAP_HSMMC_HCTL		0x0128
#define OMAP_HSMMC_SYSCTL	0x012C
#define OMAP_HSMMC_STAT		0x0130
#define OMAP_HSMMC_IE		0x0134
#define OMAP_HSMMC_ISE		0x0138
#define OMAP_HSMMC_AC12		0x013C
#define OMAP_HSMMC_CAPA		0x0140
#define OMAP_HSMMC_CAPA2	0x0144

#define VS18			(1 << 26)
#define VS30			(1 << 25)
#define HSS			(1 << 21)
#define SDVS18			(0x5 << 9)
#define SDVS30			(0x6 << 9)
#define SDVS33			(0x7 << 9)
#define SDVS_MASK		0x00000E00
#define SDVSCLR			0xFFFFF1FF
#define SDVSDET			0x00000400
#define AUTOIDLE		0x1
#define SDBP			(1 << 8)
#define DTO			0xe
#define ICE			0x1
#define ICS			0x2
#define CEN			(1 << 2)
#define CLKD_MAX		0x3FF		/* max clock divisor: 1023 */
#define CLKD_MASK		0x0000FFC0
#define CLKD_SHIFT		6
#define DTO_MASK		0x000F0000
#define DTO_SHIFT		16
#define INIT_STREAM		(1 << 1)
#define ACEN_ACMD23		(2 << 2)
#define DP_SELECT		(1 << 21)
#define DDIR			(1 << 4)
#define DMAE			0x1
#define MSBS			(1 << 5)
#define BCE			(1 << 1)
#define FOUR_BIT		(1 << 1)
#define HSPE			(1 << 2)
#define DDR			(1 << 19)
#define CLKEXTFREE		(1 << 16)
#define PADEN			(1 << 15)
#define CLEV			(1 << 24)
#define DLEV			(0xF << 20)
#define DW8			(1 << 5)
#define BRR                    (1 << 5)
#define OD			0x1
#define STAT_CLEAR		0xFFFFFFFF
#define INIT_STREAM_CMD		0x00000000
#define DUAL_VOLT_OCR_BIT	7
#define SRC			(1 << 25)
#define SRD			(1 << 26)
#define SOFTRESET		(1 << 1)

/* Interrupt masks for IE and ISE register */
#define CC_EN			(1 << 0)
#define TC_EN			(1 << 1)
#define BWR_EN			(1 << 4)
#define BRR_EN			(1 << 5)
#define ERR_EN			(1 << 15)
#define CTO_EN			(1 << 16)
#define CCRC_EN			(1 << 17)
#define CEB_EN			(1 << 18)
#define CIE_EN			(1 << 19)
#define DTO_EN			(1 << 20)
#define DCRC_EN			(1 << 21)
#define DEB_EN			(1 << 22)
#define ACE_EN			(1 << 24)
#define CERR_EN			(1 << 28)
#define BADA_EN			(1 << 29)

#define V1V8_SIGEN             (1 << 19)
#define AC12_SCLK_SEL          (1 << 23)
#define AC12_UHSMC_MASK                (7 << 16)
#define AC12_UHSMC_SDR50       (2 << 16)
#define AC12_UHSMC_SDR104      (3 << 16)
#define DLL_LOCK               (1 << 0)
#define DLL_CALIB              (1 << 1)
#define DLL_UNLOCK_STICKY      (1 << 2)
#define DLL_SWT                        (1 << 20)
#define DLL_FORCE_SR_C_MASK    (0x7F << 13)
#define DLL_FORCE_SR_C_SHIFT   13
#define DLL_FORCE_VALUE                (1 << 12)
#define DLL_RESET              (1 << 31)

#define INT_EN_MASK (BADA_EN | CERR_EN | ACE_EN | DEB_EN | DCRC_EN |\
		DTO_EN | CIE_EN | CEB_EN | CCRC_EN | CTO_EN | \
		BRR_EN | BWR_EN | TC_EN | CC_EN)

#define CNI	(1 << 7)
#define ACIE	(1 << 4)
#define ACEB	(1 << 3)
#define ACCE	(1 << 2)
#define ACTO	(1 << 1)
#define ACNE	(1 << 0)

#define CAPA2_TSDR50		(1 << 13)
#define MMC_AUTOSUSPEND_DELAY	100
#define MMC_SOFT_TIMER_SLACK	1000000		/* ns */
#define MMC_TIMEOUT_MS		20		/* 20 mSec */
#define MMC_TIMEOUT_US		20000		/* 20000 micro Sec */
#define OMAP_MMC_MIN_CLOCK	400000
#define OMAP_MMC_MAX_CLOCK	52000000
#ifdef CONFIG_MACH_DENSOJ6REF
#define MAX_PHASE_DELAY		0x3B
#else
#define MAX_PHASE_DELAY		0x7C
#endif   /*CONFIG_MACH_DENSOJ6REF*/
#define TEMP_MAX_PHASE_DELAY	0x50
#define DRIVER_NAME		"omap_hsmmc"

#define VDD_1V8			1800000		/* 180000 uV */
#define VDD_3V3			3300000		/* 330000 uV */
#define VDD_165_195		(ffs(MMC_VDD_165_195) - 1)

#define EMMC_HSDDR_SD_SDR25_MAX	52000000
#define SD_SDR50_MAX_FREQ	104000000

#define AUTO_CMD23		(1 << 1)	/* Auto CMD23 support */
/*
 * One controller can have multiple slots, like on some omap boards using
 * omap.c controller driver. Luckily this is not currently done on any known
 * omap_hsmmc.c device.
 */
#define mmc_slot(host)		(host->pdata->slots[host->slot_id])

/*
 * MMC Host controller read/write API's
 */
#define OMAP_HSMMC_READ(base, reg)	\
	__raw_readl((base) + OMAP_HSMMC_##reg)

#define OMAP_HSMMC_WRITE(base, reg, val) \
	__raw_writel((val), (base) + OMAP_HSMMC_##reg)

#if 1 /* Murata */
struct mmc_host *wifi_mmc_host;
void wifi_card_detect(void)
{
               mmc_detect_change(wifi_mmc_host, 0);
}
EXPORT_SYMBOL(wifi_card_detect);
#endif /* Murata */

struct omap_hsmmc_next {
	unsigned int	dma_len;
	s32		cookie;
};

struct omap_hsmmc_host {
	struct	device		*dev;
	struct	mmc_host	*mmc;
	struct	mmc_request	*mrq;
	struct	mmc_command	*cmd;
	struct	mmc_data	*data;
	struct	clk		*fclk;
	struct	clk		*dbclk;
	/*
	 * vcc == configured supply
	 * vcc_aux == optional
	 *   -	MMC1, supply for DAT4..DAT7
	 *   -	MMC2/MMC2, external level shifter voltage supply, for
	 *	chip (SDIO, eMMC, etc) or transceiver (MMC2 only)
	 */
	struct	regulator	*vcc;
	struct	regulator	*vcc_aux;
	struct	regulator	*pbias;
	bool			pbias_enabled;
	void	__iomem		*base;
	resource_size_t		mapbase;
	spinlock_t		irq_lock; /* Prevent races with irq handler */
	struct completion	buf_ready;
	unsigned int		dma_len;
	unsigned int		dma_sg_idx;
	unsigned char		bus_mode;
	unsigned char		power_mode;
	int			suspended;
	u32			con;
	u32			hctl;
	u32			sysctl;
	u32			capa;
	int			irq;
	int			use_dma, dma_ch;
	struct dma_chan		*tx_chan;
	struct dma_chan		*rx_chan;
	int			slot_id;
	int			response_busy;
	int			context_loss;
	int			protect_card;
	int			reqs_blocked;
	int			use_reg;
	int			req_in_progress;
	int                     regulator_enabled;
	unsigned long		clk_rate;
	unsigned int		flags;
	u32			*tuning_data;
	int			tuning_size;
	int			tuning_done;
	int			tuning_fsrc;
	u32			tuning_uhsmc;
	u32			tuning_opcode;
	bool			require_io_delay;
	struct pinctrl		*pinctrl;
	/*
	 * flag to determine if the pinctrl is in default state.
	 * It is required to set the pinctrl to default state on
	 * card removal if the pinctrl is not in default state.
	 */
	bool			pinctrl_default_state;
	struct pinctrl_state	*pinctrl_state;
	/*
	 * flag to determine whether card was removed during data
	 * transfer
	 */
	bool			transfer_incomplete;

/*
 * DENSO retention refresh
 */
    char        *retention_buffer;
    unsigned int    retention_buffer_size;
    atomic_t    debug_flags;
#define OMAP_HSMMC_DEBUG_LOG_WRITE  1
#define OMAP_HSMMC_DEBUG_LOG_READ   2
#define OMAP_HSMMC_DEBUG_LOG  \
    (OMAP_HSMMC_DEBUG_LOG_WRITE | OMAP_HSMMC_DEBUG_LOG_READ)
#define OMAP_HSMMC_DEBUG_LOG_RETENTION  4
#define OMAP_HSMMC_DEBUG_LOG_OPT (OMAP_HSMMC_DEBUG_LOG_RETENTION)
/*
 * DENSO eMMC health reporting
 */
#define DENSO_HEALTH_INIT_NOT_READY         0
#define DENSO_HEALTH_INIT_READY             1
#define DENSO_HEALTH_INIT_IN_PROGRESS       2
#define DENSO_HEALTH_INIT_DONE              3
#define DENSO_HEALTH_INIT_ERROR             -1
    char            *denso_health_blkerase_info;
    unsigned long   denso_health_blkerase_len;

	struct omap_hsmmc_next	next_data;
	struct	omap_mmc_platform_data	*pdata;
	struct thermal_zone_device *tzd;
	bool temp_tuning;
	struct timer_list	timer;
	unsigned long		data_timeout;
	bool                    is_tuning;
	unsigned int		need_i834_errata:1;
	unsigned int		last_cmd;
};

struct omap_mmc_of_data {
	u32 reg_offset;
	u8 controller_flags;
};

static const u32 ref_tuning_4bits[] = {
	0x00FF0FFF, 0xCCC3CCFF, 0xFFCC3CC3, 0xEFFEFFFE,
	0xDDFFDFFF, 0xFBFFFBFF, 0xFF7FFFBF, 0xEFBDF777,
	0xF0FFF0FF, 0x3CCCFC0F, 0xCFCC33CC, 0xEEFFEFFF,
	0xFDFFFDFF, 0xFFBFFFDF, 0xFFF7FFBB, 0xDE7B7FF7
};

static const u32 ref_tuning_8bits[] = {
	0xFF00FFFF, 0x0000FFFF, 0xCCCCFFFF, 0xCCCC33CC,
	0xCC3333CC, 0xFFFFCCCC, 0xFFFFEEFF, 0xFFEEEEFF,
	0xFFDDFFFF, 0xDDDDFFFF, 0xBBFFFFFF, 0xBBFFFFFF,
	0xFFFFFFBB, 0XFFFFFF77, 0x77FF7777, 0xFFEEDDBB,
	0x00FFFFFF, 0x00FFFFFF, 0xCCFFFF00, 0xCC33CCCC,
	0x3333CCCC, 0xFFCCCCCC, 0xFFEEFFFF, 0xEEEEFFFF,
	0xDDFFFFFF, 0xDDFFFFFF, 0xFFFFFFDD, 0XFFFFFFBB,
	0xFFFFBBBB, 0xFFFF77FF, 0xFF7777FF, 0xEEDDBB77
};

#ifdef CONFIG_DEBUG_FS
static void *denso_health_blkerase_start(struct seq_file *s, loff_t *pos)
{
	struct omap_hsmmc_host *host = (struct omap_hsmmc_host *) s->private;
    struct mmc_host *mmc = host->mmc;
    size_t alloc, len;
    char *buffer = NULL, *p = NULL;
    int rc = 0;

    if(!*pos && !host->denso_health_blkerase_info) {
        len = 0;
        rc = denso_report_block_erasure_data(mmc, &buffer, &alloc, &len);
        if(!rc) {
            host->denso_health_blkerase_info = buffer;
            host->denso_health_blkerase_len = len;
            dev_dbg(mmc_dev(mmc),
                    ".%s() block erasure data allocated %u, used %u bytes\n",
                    __func__, alloc, len);
        } else {
            if(buffer)
                kfree(buffer);
            dev_warn(mmc_dev(mmc),
                    "!%s() cannot retrieve erasure data. "
                    "rc=%d, alloc=%u, len=%u\n",
                    __func__, rc, alloc, len);
        }
    } else {
        len = host->denso_health_blkerase_len;
    }
    if(host->denso_health_blkerase_info && *pos < len)
        p = host->denso_health_blkerase_info  + (*pos ? *pos - 1 : *pos);
    dev_dbg(mmc_dev(mmc), ".%s() base=0x%p, pos=%lld, p=0x%p, len = %u\n",
            __func__, host->denso_health_blkerase_info,
            *pos, p, len);

    return p;
}

static void denso_health_blkerase_stop(struct seq_file *s, void *v)
{
    /* would kfree(host->denso_health_blkerase) but instead
      we save it (one invocation per boot) */
}

static void *denso_health_blkerase_next(struct seq_file *s, void *v, loff_t *pos)
{
    char *p;
	struct omap_hsmmc_host *host = (struct omap_hsmmc_host *) s->private;
    unsigned long max = host->denso_health_blkerase_len;
    p = (*pos < max) ? strchr((char *) v, '\n') : NULL;
    if(p && *(p+1)) {
        p += 1; /* advance past '\n' */
        *pos = (loff_t) (p - host->denso_health_blkerase_info);
    } else {
        *pos = (loff_t) max;
    }
    return p;
}

static int denso_health_blkerase_show(struct seq_file *s, void *v)
{
    char *nl = strchr((char *) v, '\n');
    if(nl)
        seq_write(s, (char *) v, (nl - ((char*)v)) + 1);
    return 0;
}

static const struct seq_operations denso_health_sops = {
    .start = denso_health_blkerase_start,
    .stop = denso_health_blkerase_stop,
    .next = denso_health_blkerase_next,
    .show = denso_health_blkerase_show,
};

static int denso_health_blkerase_open(struct inode *inode, struct file *file)
{
    int rc = seq_open(file, &denso_health_sops);
    if(!rc)
        ((struct seq_file *) file->private_data)->private = inode->i_private;
    return rc;
}

static const struct file_operations denso_health_fops = {
    .owner   = THIS_MODULE,
    .open    = denso_health_blkerase_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = seq_release
};
#endif /* CONFIG_DEBUG_FS */

/*---------------------------------------
 * SD retention refresh facilities
 * Copyright (c) 2014,2015 DENSO Corporation.  ALL RIGHTS RESERVED.
 * Aug-20-2014  v1.0 S. OGAWA
 * May-15-2015  v1.1 T. THOMAS (port to hsmmc)
 *---------------------------------------*/
static const char* const    retention_vers =
    "omap_hsmmc retention refresh version 1.1. Copyright (c) 2014,2015 DENSO Corporation.  ALL RIGHTS RESERVED.";

static int omap_hsmmc_retention_refresh(struct mmc_card *card,
        sector_t pos, unsigned int bytesize)
/*
 * see mmc/card/block.c : mmc_blk_issue_rq_rq
 * address : cmd.arg = blk_rq_pos(rq);
 * blocks  : data.blocks = blk_rq_sectors(rq);
 */
{
   int ret;
   struct mmc_host     *mmc;
   struct omap_hsmmc_host   *host;

   struct mmc_request  mrq;
   struct mmc_command  cmd, stop;
   struct mmc_data     data;

   u32 readcmd, writecmd;

   struct scatterlist  sg;
   unsigned int    buffer_size, debug_flags;

   ret = 0;
   mmc = card->host;
   host = mmc_priv(mmc);
   debug_flags = 0;

   dev_dbg(mmc_dev(mmc), "+%s(%lu, %u)\n", __func__,
           (unsigned long) pos, bytesize);

   /*
    * make request parameters
    */
   memset(&mrq, 0, sizeof(mrq));
   memset(&cmd, 0, sizeof(cmd));
   memset(&data, 0, sizeof(data));
   memset(&stop, 0, sizeof(stop));

   mrq.cmd = &cmd;
   mrq.data = &data;

   cmd.arg = pos;
   if (!mmc_card_blockaddr(card)) {
       /*
        * mmc_card_blockaddr() is true,
        * if mmc_card_set_blockaddr() is called :
        * mmc/core/mmc.c density > 2GB
        * mmc/core/sd.c  block-addressed SDHC card
        */
       cmd.arg <<= 9;  /* not >2GB nor SDHC */
   }
   cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;
   data.blksz = 512;
   data.blocks = bytesize >> 9;
       /* divide 512, see blk_rq_sectors(req) */
   stop.opcode = MMC_STOP_TRANSMISSION;
   stop.arg = 0;
   stop.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;

   if (data.blocks > mmc->max_blk_count) {
       data.blocks = mmc->max_blk_count;
   }

   if (data.blocks > 1) {
       readcmd = MMC_READ_MULTIPLE_BLOCK;
       writecmd = MMC_WRITE_MULTIPLE_BLOCK;
   } else {
       readcmd = MMC_READ_SINGLE_BLOCK;
       writecmd = MMC_WRITE_BLOCK;
   }

   /*
    * prepare buffer
    */
   mmc_claim_host(mmc);

#ifdef CONFIG_DEBUG_FS
   debug_flags = atomic_read(&host->debug_flags);
   if (debug_flags & OMAP_HSMMC_DEBUG_LOG_RETENTION) {
       atomic_set(&host->debug_flags, debug_flags | OMAP_HSMMC_DEBUG_LOG);
   } else {
       debug_flags = 0;
   }
#endif
   buffer_size = data.blksz * data.blocks;
   if (unlikely(host->retention_buffer_size < buffer_size)) {
       char    *buf = kmalloc(buffer_size, GFP_KERNEL);
       if (!buf) {
           printk(KERN_ERR "%s %s unable to "
                   "allocate buffer,size=%d\n",
                   mmc_hostname(mmc),
                   __func__,
                   buffer_size);
           ret = -ENOMEM;
           goto fin;
       }

       if (host->retention_buffer) {
           kfree(host->retention_buffer);
       } else {
           printk(KERN_INFO "%s\n",
                   retention_vers);
       }
       host->retention_buffer = buf;
       host->retention_buffer_size = buffer_size;
   }
   sg_init_one(&sg, host->retention_buffer, buffer_size);
   data.sg = &sg;
   data.sg_len = 1;

   /*
    * read
    */
   if (readcmd == MMC_READ_MULTIPLE_BLOCK) {
       mrq.stop = &stop;
   } else {
       mrq.stop = NULL;
   }
   mrq.cmd->opcode = readcmd;
   mrq.data->flags = MMC_DATA_READ;

   mmc_set_data_timeout(mrq.data, card);
   dev_dbg(mmc_dev(mmc),
           ".%s() mmc_set_data_timeout() done. to_ns=%u, to_clks=%u. About to mmc_wait_for_req()\n",
            __func__, mrq.data->timeout_ns, mrq.data->timeout_clks);
   mmc_wait_for_req(mmc, &mrq);

   ret = cmd.error ?: data.error ?: stop.error;
   if (unlikely(ret)) {
       goto fin_error;
   }

   /*
    * write
    */
   data.blocks = data.bytes_xfered / data.blksz; /* write the same buffer with read actual size */
    data.bytes_xfered = 0;
   if (writecmd == MMC_WRITE_MULTIPLE_BLOCK &&
           !mmc_host_is_spi(mmc)) {
       mrq.stop = &stop;
   } else {
       mrq.stop = NULL;
   }
   mrq.cmd->opcode = writecmd;
   mrq.data->flags = MMC_DATA_WRITE;
   mmc_set_data_timeout(mrq.data, card);

   /*
    * retention log-io
    */

   mmc_wait_for_req(mmc, &mrq);

   ret = mrq.cmd->error ?: mrq.data->error;
   if (unlikely(ret)) {
       goto fin_error;
   }

   if (!mmc_host_is_spi(card->host)) do {
       cmd.opcode = MMC_SEND_STATUS;
       cmd.arg = card->rca << 16;
       cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
       ret = mmc_wait_for_cmd(card->host, &cmd, 5);
       if (ret) {
           printk(KERN_ERR "%s %s ERROR ret=%d cmd=MMC_SEND_STATUS\n",
               mmc_hostname(mmc), __func__, ret);
           goto fin;
       }
   } while (!(cmd.resp[0] & R1_READY_FOR_DATA) ||
               (R1_CURRENT_STATE(cmd.resp[0]) == 7));

   goto fin;

fin_error:
   printk(KERN_ERR "%s %s ERROR ret=%d cmd=%d pos=%d size=%d\n",
           mmc_hostname(mmc), __func__, ret,
           mrq.cmd->opcode,
           mrq.cmd->arg,
           buffer_size);
fin:
#ifdef CONFIG_DEBUG_FS
   if (debug_flags) {
       atomic_set(&host->debug_flags, debug_flags);
   }
   dev_dbg(mmc_dev(mmc), ".%s() set host->debug_flags = %u\n", __func__, debug_flags);
#endif /* CONFIG_DEBUG_FS */
   mmc_release_host(mmc);
   dev_dbg(mmc_dev(mmc), "-%s(%lu, %u) = %d\n", __func__,
           (unsigned long) pos, bytesize, ret);
   return ret;
}

/*
 * debug fs
 * Copyright (c) 2014,2015 DENSO Corporation.  ALL RIGHTS RESERVED.
 */
#ifdef CONFIG_DEBUG_FS
static int omap_hsmmc_log_io_get(void *data, u64 *val)
{
   struct omap_hsmmc_host   *host = data;
   unsigned long   f, mask;

   mask = OMAP_HSMMC_DEBUG_LOG | OMAP_HSMMC_DEBUG_LOG_OPT;
   f = atomic_read(&host->debug_flags) & mask;
   *val = f;
   dev_dbg(mmc_dev(host->mmc), ".%s(%s) set host->debug_flags = 0x%lx\n",
           __func__, mmc_hostname(host->mmc), f);
   return 0;
}
static int omap_hsmmc_log_io_set(void *data, u64 val)
{
   struct omap_hsmmc_host   *host = data;
   unsigned long   f = atomic_read(&host->debug_flags);
   unsigned long   f0 = f, mask;

   mask = OMAP_HSMMC_DEBUG_LOG | OMAP_HSMMC_DEBUG_LOG_OPT;
   f = (f &~ mask) | (val & mask);

   if (f != f0) {
       atomic_set(&host->debug_flags, f);
       dev_dbg(mmc_dev(host->mmc), ".%s() set host->debug_flags to 0x%lx\n",
             __func__, f);
   }
   return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(omap_hsmmc_log_io,
       omap_hsmmc_log_io_get,
       omap_hsmmc_log_io_set,
       "%llu\n");
#endif /* CONFIG_DEBUG_FS */

/*
 * omap_hsmmc attribute group
 * Copyright (c) 2015 DENSO Corporation.  ALL RIGHTS RESERVED.
 * see drivers/gpio/gpiolib.c for reference of device_attribute
 */
static ssize_t omap_hsmmc_retention_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct mmc_host *mmc;
    struct mmc_card *card;
    unsigned long long  pos;
    unsigned int    bytesize;

    mmc = container_of(dev, struct mmc_host, class_dev);
    dev_dbg(mmc_dev(mmc), "+%s()\n", __func__);
    card = mmc->card;
    if (!card) {
        printk(KERN_ERR "%s %s not detect card\n",
            mmc_hostname(mmc), __func__);
        return count;
    } else {
        dev_dbg(mmc_dev(mmc), "%s %s card=%p mmc=%p dev=%p\n",
            mmc_hostname(mmc), __func__,
            card, mmc, dev);
    }

    bytesize = PAGE_SIZE;   /* default size */
    if (unlikely(!sscanf(buf, "%Lu %u", &pos, &bytesize))) {
        printk(KERN_ERR "%s %s:parameter must be "
            "pos-value [size] :%s\n",
        mmc_hostname(card->host), __func__, buf);
        return count;
    }
    if (bytesize < 512) {
        bytesize = 512;
    }

    omap_hsmmc_retention_refresh(card, (sector_t)pos, bytesize);
    dev_dbg(mmc_dev(mmc), "-%s()\n", __func__);

    return count;
}

/* tthomas
* @see Documentation/driver-model/device.txt
* #define DEVICE_ATTR(name,mode,show,store)
* creates device_attribute "dev_attr_retention"
* with .store function pointer pointed at
* sdhci_retention_store, and no .show function.
*/
static const DEVICE_ATTR(retention, S_IWUSR, NULL, omap_hsmmc_retention_store);
static const struct attribute *attrs[] = {
    &dev_attr_retention.attr,
    NULL,
};
static const struct attribute_group attr_group = {
    .attrs = (struct attribute**) attrs,
};

static void omap_hsmmc_attr_init(struct mmc_host *host)
{
    struct kobject  *kobj = &host->class_dev.kobj;

    dev_dbg(mmc_dev(host), "+%s()\n", __func__);

    if (!sysfs_create_group(kobj, &attr_group)) {
        dev_dbg(mmc_dev(host), ".%s() did sysfs_create_group() = 0\n",
                __func__);
    } else {
        printk(KERN_WARNING "!%s() could not sysfs_create_group()\n",
                __func__);
    }

    dev_dbg(mmc_dev(host), "-%s()\n", __func__);
}

static void omap_hsmmc_attr_term(struct omap_hsmmc_host *host)
{
    struct mmc_host *mmc = host->mmc;
    struct kobject  *kobj = &mmc->class_dev.kobj;

    sysfs_remove_group(kobj, &attr_group);
}

static inline int omap_hsmmc_set_dll(struct omap_hsmmc_host *host, int count)
{
	int i;
	u32 dll;
	unsigned int retries = 1000;

	dll = OMAP_HSMMC_READ(host->base, DLL);
	dll &= ~(DLL_FORCE_SR_C_MASK);
	dll &= ~DLL_CALIB;
	dll |= (count << DLL_FORCE_SR_C_SHIFT);
	OMAP_HSMMC_WRITE(host->base, DLL, dll);
	dll |= DLL_FORCE_VALUE;
	OMAP_HSMMC_WRITE(host->base, DLL, dll);
	dll |= DLL_CALIB;
	OMAP_HSMMC_WRITE(host->base, DLL, dll);
	for (i = 0; i < retries; i++) {
		if (OMAP_HSMMC_READ(host->base, DLL) & DLL_CALIB)
			break;
		usleep_range(10, 20);
	}
	dll &= ~DLL_CALIB;
	dll = OMAP_HSMMC_READ(host->base, DLL);

	return 0;
}

static void omap_hsmmc_start_dma_transfer(struct omap_hsmmc_host *host);

static int omap_hsmmc_card_detect(struct device *dev, int slot)
{
	struct omap_hsmmc_host *host = dev_get_drvdata(dev);
	struct omap_mmc_platform_data *mmc = host->pdata;

	/* NOTE: assumes card detect signal is active-low */
	return !gpio_get_value_cansleep(mmc->slots[0].switch_pin);
}

static int omap_hsmmc_get_wp(struct device *dev, int slot)
{
	struct omap_hsmmc_host *host = dev_get_drvdata(dev);
	struct omap_mmc_platform_data *mmc = host->pdata;

	/* NOTE: assumes write protect signal is active-high */
	return gpio_get_value_cansleep(mmc->slots[0].gpio_wp);
}

static int omap_hsmmc_get_cover_state(struct device *dev, int slot)
{
	struct omap_hsmmc_host *host = dev_get_drvdata(dev);
	struct omap_mmc_platform_data *mmc = host->pdata;

	/* NOTE: assumes card detect signal is active-low */
	return !gpio_get_value_cansleep(mmc->slots[0].switch_pin);
}

#ifdef CONFIG_PM

static int omap_hsmmc_suspend_cdirq(struct device *dev, int slot)
{
	struct omap_hsmmc_host *host = dev_get_drvdata(dev);
	struct omap_mmc_platform_data *mmc = host->pdata;

	disable_irq(mmc->slots[0].card_detect_irq);
	return 0;
}

static int omap_hsmmc_resume_cdirq(struct device *dev, int slot)
{
	struct omap_hsmmc_host *host = dev_get_drvdata(dev);
	struct omap_mmc_platform_data *mmc = host->pdata;

	enable_irq(mmc->slots[0].card_detect_irq);
	return 0;
}

#else

#define omap_hsmmc_suspend_cdirq	NULL
#define omap_hsmmc_resume_cdirq		NULL

#endif

#ifdef CONFIG_REGULATOR

static int omap_hsmmc_set_power(struct device *dev, int slot, int power_on,
				int vdd_iopower)
{
	struct omap_hsmmc_host *host =
		platform_get_drvdata(to_platform_device(dev));
	struct mmc_ios *ios = &host->mmc->ios;
	int ret = 0;
	u32 ac12 = 0;

	/*
	 * If we don't see a Vcc regulator, assume it's a fixed
	 * voltage always-on regulator.
	 */
	if (!host->vcc)
		return 0;

	if (mmc_slot(host).before_set_reg)
		mmc_slot(host).before_set_reg(dev, slot, power_on, vdd_iopower);

	if (host->pbias) {
		if (host->pbias_enabled == 1) {
			ret = regulator_disable(host->pbias);
			if (!ret)
				host->pbias_enabled = 0;
		}
		regulator_set_voltage(host->pbias, VDD_3V3, VDD_3V3);
	}

	/*
	 * Assume Vcc regulator is used only to power the card ... OMAP
	 * VDDS is used to power the pins, optionally with a transceiver to
	 * support cards using voltages other than VDDS (1.8V nominal).  When a
	 * transceiver is used, DAT3..7 are muxed as transceiver control pins.
	 *
	 * In some cases this regulator won't support enable/disable;
	 * e.g. it's a fixed rail for a WLAN chip.
	 *
	 * In other cases vcc_aux switches interface power.  Example, for
	 * eMMC cards it represents VccQ.  Sometimes transceivers or SDIO
	 * chips/cards need an interface voltage rail too.
	 */
	if (power_on) {
		if (host->vcc) {
			ret = mmc_regulator_set_ocr(host->mmc, host->vcc,
						    ios->vdd);
			if (ret < 0)
				return ret;
		}

		/* Enable interface voltage rail, if needed */
		if (host->vcc_aux) {
			ac12 = OMAP_HSMMC_READ(host->base, AC12);

			if (vdd_iopower == VDD_165_195) {
				if (host->regulator_enabled) {
					ret = regulator_disable(host->vcc_aux);
					if (ret < 0)
						goto error_set_power;
					host->regulator_enabled = 0;
				}
				ret = regulator_set_voltage(host->vcc_aux,
							    VDD_1V8, VDD_1V8);
				if (ret < 0)
					goto error_set_power;
				ac12 |= V1V8_SIGEN;
			} else {
				ret = regulator_set_voltage(host->vcc_aux,
							    VDD_3V3, VDD_3V3);
				if (ret < 0)
					goto error_set_power;
				ac12 &= ~V1V8_SIGEN;
			}
			OMAP_HSMMC_WRITE(host->base, AC12, ac12);
		}
		if (host->vcc_aux && !host->regulator_enabled) {
			ret = regulator_enable(host->vcc_aux);

			if (!ret) {
				host->regulator_enabled = 1;
			} else {
				mmc_regulator_set_ocr(host->mmc, host->vcc, 0);
				goto error_set_power;
			}
		}
	} else {
		/* Shut down the rail */
		if (host->vcc_aux && host->regulator_enabled) {
			ret = regulator_disable(host->vcc_aux);

			if (!ret)
				host->regulator_enabled = 0;
		}

		if (host->vcc) {
			/* Then proceed to shut down the local regulator */
			ret = mmc_regulator_set_ocr(host->mmc,
						host->vcc, 0);
		}
	}

	if (host->pbias) {
		if (vdd_iopower == VDD_165_195)
			ret = regulator_set_voltage(host->pbias, VDD_1V8,
								VDD_1V8);
		else
			ret = regulator_set_voltage(host->pbias, VDD_3V3,
								VDD_3V3);
		if (ret < 0)
			goto error_set_power;

		if (host->pbias_enabled == 0) {
			ret = regulator_enable(host->pbias);
			if (!ret)
				host->pbias_enabled = 1;
		}
	}

	if (mmc_slot(host).after_set_reg)
		mmc_slot(host).after_set_reg(dev, slot, power_on, vdd_iopower);

error_set_power:
	return ret;
}

static int omap_hsmmc_reg_get(struct omap_hsmmc_host *host)
{
	struct regulator *reg;
	int ocr_value = 0;

	reg = devm_regulator_get(host->dev, "vmmc");
	if (IS_ERR(reg)) {
		dev_err(host->dev, "unable to get vmmc regulator %ld\n",
			PTR_ERR(reg));
		return PTR_ERR(reg);
	} else {
		host->vcc = reg;
		ocr_value = mmc_regulator_get_ocrmask(reg);
		if (!mmc_slot(host).ocr_mask) {
			mmc_slot(host).ocr_mask = ocr_value;
		} else {
			if (!(mmc_slot(host).ocr_mask & ocr_value)) {
				dev_err(host->dev, "ocrmask %x is not supported\n",
					mmc_slot(host).ocr_mask);
				mmc_slot(host).ocr_mask = 0;
				return -EINVAL;
			}
		}
	}
	mmc_slot(host).set_power = omap_hsmmc_set_power;

	/* Allow an aux regulator */
	reg = devm_regulator_get_optional(host->dev, "vmmc_aux");
	host->vcc_aux = IS_ERR(reg) ? NULL : reg;

	reg = devm_regulator_get_optional(host->dev, "pbias");
	host->pbias = IS_ERR(reg) ? NULL : reg;

	/* For eMMC do not power off when not in sleep state */
	if (mmc_slot(host).no_regulator_off_init)
		return 0;
	/*
	 * To disable boot_on regulator, enable regulator
	 * to increase usecount and then disable it.
	 */
	if ((host->vcc && regulator_is_enabled(host->vcc) > 0) ||
	    (host->vcc_aux && regulator_is_enabled(host->vcc_aux))) {
		int vdd = ffs(mmc_slot(host).ocr_mask) - 1;

		mmc_slot(host).set_power(host->dev, host->slot_id, 1, vdd);
		mmc_slot(host).set_power(host->dev, host->slot_id, 0, 0);
	}

	return 0;
}

static void omap_hsmmc_reg_put(struct omap_hsmmc_host *host)
{
	mmc_slot(host).set_power = NULL;
}

static inline int omap_hsmmc_have_reg(void)
{
	return 1;
}

#else

static inline int omap_hsmmc_reg_get(struct omap_hsmmc_host *host)
{
	return -EINVAL;
}

static inline void omap_hsmmc_reg_put(struct omap_hsmmc_host *host)
{
}

static inline int omap_hsmmc_have_reg(void)
{
	return 0;
}

#endif

static int omap_hsmmc_gpio_init(struct omap_mmc_platform_data *pdata)
{
	int ret;

	if (gpio_is_valid(pdata->slots[0].switch_pin)) {
		if (pdata->slots[0].cover)
			pdata->slots[0].get_cover_state =
					omap_hsmmc_get_cover_state;
		else
			pdata->slots[0].card_detect = omap_hsmmc_card_detect;
		pdata->slots[0].card_detect_irq =
				gpio_to_irq(pdata->slots[0].switch_pin);
		ret = gpio_request(pdata->slots[0].switch_pin, "mmc_cd");
		if (ret)
			return ret;
		ret = gpio_direction_input(pdata->slots[0].switch_pin);
		if (ret)
			goto err_free_sp;
	} else
		pdata->slots[0].switch_pin = -EINVAL;

	if (gpio_is_valid(pdata->slots[0].gpio_wp)) {
		pdata->slots[0].get_ro = omap_hsmmc_get_wp;
		ret = gpio_request(pdata->slots[0].gpio_wp, "mmc_wp");
		if (ret)
			goto err_free_cd;
		ret = gpio_direction_input(pdata->slots[0].gpio_wp);
		if (ret)
			goto err_free_wp;
	} else
		pdata->slots[0].gpio_wp = -EINVAL;

	return 0;

err_free_wp:
	gpio_free(pdata->slots[0].gpio_wp);
err_free_cd:
	if (gpio_is_valid(pdata->slots[0].switch_pin))
err_free_sp:
		gpio_free(pdata->slots[0].switch_pin);
	return ret;
}

static void omap_hsmmc_gpio_free(struct omap_mmc_platform_data *pdata)
{
	if (gpio_is_valid(pdata->slots[0].gpio_wp))
		gpio_free(pdata->slots[0].gpio_wp);
	if (gpio_is_valid(pdata->slots[0].switch_pin))
		gpio_free(pdata->slots[0].switch_pin);
}

/*
 * Start clock to the card
 */
static void omap_hsmmc_start_clock(struct omap_hsmmc_host *host)
{
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) | CEN);
}

/*
 * Stop clock to the card
 */
static void omap_hsmmc_stop_clock(struct omap_hsmmc_host *host)
{
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) & ~CEN);
	if ((OMAP_HSMMC_READ(host->base, SYSCTL) & CEN) != 0x0)
		dev_dbg(mmc_dev(host->mmc), "MMC Clock is not stopped\n");
}

static void omap_hsmmc_enable_irq(struct omap_hsmmc_host *host,
				  struct mmc_command *cmd)
{
	unsigned int irq_mask;

	if (host->is_tuning)
		/*
		 * OMAP5/DRA74X/DRA72x Errata i802:
		 * DCRC error interrupts (MMCHS_STAT[21] DCRC=0x1) can occur
		 * during the tuning procedure. So disable it during the
		 * tuning procedure.
		 */
		irq_mask = (INT_EN_MASK | BRR_EN) & ~DCRC_EN;
	else if (host->use_dma)
		irq_mask = INT_EN_MASK & ~(BRR_EN | BWR_EN);
	else
		irq_mask = INT_EN_MASK;

	/* Disable timeout for erases or when using software timeout */
	if (cmd->opcode == MMC_ERASE || host->need_i834_errata)
		irq_mask &= ~DTO_EN;

	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);
	OMAP_HSMMC_WRITE(host->base, ISE, irq_mask);
	OMAP_HSMMC_WRITE(host->base, IE, irq_mask);
}

static void omap_hsmmc_disable_irq(struct omap_hsmmc_host *host)
{
	OMAP_HSMMC_WRITE(host->base, ISE, 0);
	OMAP_HSMMC_WRITE(host->base, IE, 0);
	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);
}

/* Calculate divisor for the given clock frequency */
static u16 calc_divisor(struct omap_hsmmc_host *host, struct mmc_ios *ios)
{
	u16 dsor = 0;

	if (ios->clock) {
		dsor = DIV_ROUND_UP(clk_get_rate(host->fclk), ios->clock);
		if (dsor > CLKD_MAX)
			dsor = CLKD_MAX;
	}

	return dsor;
}

static inline int omap_hsmmc_restore_dll(struct omap_hsmmc_host *host)
{
	u32 ac12;
	u32 dll;

	ac12 = OMAP_HSMMC_READ(host->base, AC12);
	ac12 |= host->tuning_uhsmc;
	OMAP_HSMMC_WRITE(host->base, AC12, ac12);

	dll = OMAP_HSMMC_READ(host->base, DLL);
	dll |= DLL_FORCE_VALUE;
	OMAP_HSMMC_WRITE(host->base, DLL, dll);

	if (omap_hsmmc_set_dll(host, host->tuning_fsrc))
		return -EIO;
	return 0;
}

static inline void omap_hsmmc_save_dll(struct omap_hsmmc_host *host)
{
	u32 ac12;

	ac12 = OMAP_HSMMC_READ(host->base, AC12);
	ac12 &= ~AC12_UHSMC_MASK;
	OMAP_HSMMC_WRITE(host->base, AC12, ac12);
}

static int omap_hsmmc_pinctrl_set_state(struct omap_hsmmc_host *host,
					char *mode)
{
	int ret;

	if (IS_ERR(host->pinctrl)) {
		dev_vdbg(mmc_dev(host->mmc),
			 "pins are not configured from the driver\n");
		return -ENODEV;
	}

	if (!mode) {
		dev_err(mmc_dev(host->mmc), "empty mode string\n");
		return -EINVAL;
	}

	host->pinctrl_state = pinctrl_lookup_state(host->pinctrl, mode);
	if (IS_ERR(host->pinctrl_state)) {
		dev_err(mmc_dev(host->mmc),
			"no pinctrl state for %s mode\n", mode);
		return PTR_ERR(host->pinctrl_state);
	}

	ret = pinctrl_select_state(host->pinctrl, host->pinctrl_state);
	if (ret) {
		dev_err(mmc_dev(host->mmc),
			"failed to activate pinctrl state\n");
		return ret;
	}

	return 0;
}

static void omap_hsmmc_set_clock(struct omap_hsmmc_host *host)
{
	struct mmc_ios *ios = &host->mmc->ios;
	unsigned long regval;
	unsigned long timeout;
	unsigned long clkdiv;
	char *mode;

	dev_vdbg(mmc_dev(host->mmc), "Set clock to %uHz\n", ios->clock);

	omap_hsmmc_stop_clock(host);

	if (!host->require_io_delay)
		goto no_io_delay;

	/*
	 * DRA7 Errata No i834: When using high speed HS200 and SDR104
	 * cards, the functional clock for MMC module will be 192MHz.
	 * At this frequency, the maximum obtainable timeout (DTO =0xE)
	 * in hardware is (1/192MHz)*2^27 = 700ms. Commands taking longer
	 * than 700ms will be affected by this small window frame and
	 * will be timing out frequently even without a genune timeout
	 * from the card. Workarround for this errata is use a software
	 * timer instead of hardware timer to provide the delay requested
	 * by the upper layer
	 */
	host->need_i834_errata = false;

	switch (ios->timing) {
	case MMC_TIMING_UHS_SDR104:
		mode = kstrdup("sdr104", GFP_KERNEL);
		host->need_i834_errata = true;
		break;
	case MMC_TIMING_UHS_DDR50:
		mode = kstrdup("ddr50", GFP_KERNEL);
		break;
	case MMC_TIMING_UHS_SDR50:
		mode = kstrdup("sdr50", GFP_KERNEL);
		break;
	case MMC_TIMING_SD_HS:
	case MMC_TIMING_MMC_HS:
		mode = kstrdup("hs", GFP_KERNEL);
		break;
	case MMC_TIMING_UHS_SDR25:
		mode = kstrdup("sdr25", GFP_KERNEL);
		break;
	case MMC_TIMING_UHS_SDR12:
		mode = kstrdup("sdr12", GFP_KERNEL);
		break;
	case MMC_TIMING_MMC_HS200:
		mode = kstrdup("hs200", GFP_KERNEL);
		host->need_i834_errata = true;
		break;
	case MMC_TIMING_MMC_DDR52:
		mode = kstrdup("ddr_3_3v", GFP_KERNEL);
		break;
	default:
		dev_dbg(mmc_dev(host->mmc), "no io delay setting\n");
		goto no_io_delay;
	}

	omap_hsmmc_pinctrl_set_state(host, mode);
	host->pinctrl_default_state = false;
	kfree(mode);

no_io_delay:
	regval = OMAP_HSMMC_READ(host->base, SYSCTL);
	regval = regval & ~(CLKD_MASK | DTO_MASK);
	clkdiv = calc_divisor(host, ios);
	regval = regval | (clkdiv << 6) | (DTO << 16);
	OMAP_HSMMC_WRITE(host->base, SYSCTL, regval);
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) | ICE);

	/* Wait till the ICS bit is set */
	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((OMAP_HSMMC_READ(host->base, SYSCTL) & ICS) != ICS
		&& time_before(jiffies, timeout))
		cpu_relax();

	/*
	 * Enable High-Speed Support
	 * Pre-Requisites
	 *	- Controller should support High-Speed-Enable Bit
	 *	- Controller should not be using DDR Mode
	 *	- Controller should advertise that it supports High Speed
	 *	  in capabilities register
	 *	- MMC/SD clock coming out of controller > 25MHz
	 */
	if ((mmc_slot(host).features & HSMMC_HAS_HSPE_SUPPORT) &&
	    (ios->timing != MMC_TIMING_MMC_DDR52) &&
	    (ios->timing != MMC_TIMING_UHS_DDR50) &&
	    ((OMAP_HSMMC_READ(host->base, CAPA) & HSS) == HSS)) {
		regval = OMAP_HSMMC_READ(host->base, HCTL);
		if (clkdiv && (clk_get_rate(host->fclk)/clkdiv) > 25000000)
			regval |= HSPE;
		else
			regval &= ~HSPE;

		OMAP_HSMMC_WRITE(host->base, HCTL, regval);
	}

	omap_hsmmc_start_clock(host);
}

static void omap_hsmmc_set_bus_width(struct omap_hsmmc_host *host)
{
	struct mmc_ios *ios = &host->mmc->ios;
	u32 con;

	con = OMAP_HSMMC_READ(host->base, CON);
	if (ios->timing == MMC_TIMING_MMC_DDR52 ||
	    ios->timing == MMC_TIMING_UHS_DDR50)
		con |= DDR;	/* configure in DDR mode */
	else
		con &= ~DDR;
	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_8:
		OMAP_HSMMC_WRITE(host->base, CON, con | DW8);
		break;
	case MMC_BUS_WIDTH_4:
		OMAP_HSMMC_WRITE(host->base, CON, con & ~DW8);
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | FOUR_BIT);
		break;
	case MMC_BUS_WIDTH_1:
		OMAP_HSMMC_WRITE(host->base, CON, con & ~DW8);
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) & ~FOUR_BIT);
		break;
	}
}

static void omap_hsmmc_set_bus_mode(struct omap_hsmmc_host *host)
{
	struct mmc_ios *ios = &host->mmc->ios;
	u32 con;

	con = OMAP_HSMMC_READ(host->base, CON);
	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN)
		OMAP_HSMMC_WRITE(host->base, CON, con | OD);
	else
		OMAP_HSMMC_WRITE(host->base, CON, con & ~OD);
}

#ifdef CONFIG_PM
/*
 * Restore the MMC host context, if it was lost as result of a
 * power state change.
 */
static int omap_hsmmc_context_restore(struct omap_hsmmc_host *host)
{
	struct mmc_ios *ios = &host->mmc->ios;
	u32 hctl, capa;
	u32 value;
	unsigned long timeout;

	if (host->con == OMAP_HSMMC_READ(host->base, CON) &&
	    host->hctl == OMAP_HSMMC_READ(host->base, HCTL) &&
	    host->sysctl == OMAP_HSMMC_READ(host->base, SYSCTL) &&
	    host->capa == OMAP_HSMMC_READ(host->base, CAPA))
		return 0;

	host->context_loss++;

	if (host->pdata->controller_flags & OMAP_HSMMC_SUPPORTS_DUAL_VOLT) {
		if (host->power_mode != MMC_POWER_OFF &&
		    ((1 << ios->vdd) <= MMC_VDD_23_24 ||
		    (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180)))
			hctl = SDVS18;
		else
			hctl = SDVS30;
		capa = VS30 | VS18;
	} else {
		hctl = SDVS18;
		capa = VS18;
	}

	OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | hctl);

	OMAP_HSMMC_WRITE(host->base, CAPA,
			OMAP_HSMMC_READ(host->base, CAPA) | capa);

	OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | SDBP);

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((OMAP_HSMMC_READ(host->base, HCTL) & SDBP) != SDBP
		&& time_before(jiffies, timeout))
		;

	omap_hsmmc_disable_irq(host);

	/* Do not initialize card-specific things if the power is off */
	if (host->power_mode == MMC_POWER_OFF)
		goto out;

	omap_hsmmc_set_bus_width(host);

	omap_hsmmc_set_clock(host);

	omap_hsmmc_set_bus_mode(host);

	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
		value = OMAP_HSMMC_READ(host->base, HCTL);
		value &= ~SDVS_MASK;
		value |= SDVS18;
		OMAP_HSMMC_WRITE(host->base, HCTL, value);

		value = OMAP_HSMMC_READ(host->base, AC12);
		value |= V1V8_SIGEN;
		OMAP_HSMMC_WRITE(host->base, AC12, value);
	}

out:
	dev_dbg(mmc_dev(host->mmc), "context is restored: restore count %d\n",
		host->context_loss);
	return 0;
}

/*
 * Save the MMC host context (store the number of power state changes so far).
 */
static void omap_hsmmc_context_save(struct omap_hsmmc_host *host)
{
	host->con =  OMAP_HSMMC_READ(host->base, CON);
	host->hctl = OMAP_HSMMC_READ(host->base, HCTL);
	host->sysctl =  OMAP_HSMMC_READ(host->base, SYSCTL);
	host->capa = OMAP_HSMMC_READ(host->base, CAPA);
}

#else

static int omap_hsmmc_context_restore(struct omap_hsmmc_host *host)
{
	return 0;
}

static void omap_hsmmc_context_save(struct omap_hsmmc_host *host)
{
}

#endif

/*
 * Send init stream sequence to card
 * before sending IDLE command
 */
static void send_init_stream(struct omap_hsmmc_host *host)
{
	int reg = 0;
	unsigned long timeout;

	if (host->protect_card)
		return;

	disable_irq(host->irq);

	OMAP_HSMMC_WRITE(host->base, IE, INT_EN_MASK);
	OMAP_HSMMC_WRITE(host->base, CON,
		OMAP_HSMMC_READ(host->base, CON) | INIT_STREAM);
	OMAP_HSMMC_WRITE(host->base, CMD, INIT_STREAM_CMD);

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((reg != CC_EN) && time_before(jiffies, timeout))
		reg = OMAP_HSMMC_READ(host->base, STAT) & CC_EN;

	OMAP_HSMMC_WRITE(host->base, CON,
		OMAP_HSMMC_READ(host->base, CON) & ~INIT_STREAM);

	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);
	OMAP_HSMMC_READ(host->base, STAT);

	enable_irq(host->irq);
}

static inline
int omap_hsmmc_cover_is_closed(struct omap_hsmmc_host *host)
{
	int r = 1;

	if (mmc_slot(host).get_cover_state)
		r = mmc_slot(host).get_cover_state(host->dev, host->slot_id);
	return r;
}

static ssize_t
omap_hsmmc_show_cover_switch(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct mmc_host *mmc = container_of(dev, struct mmc_host, class_dev);
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	return sprintf(buf, "%s\n",
			omap_hsmmc_cover_is_closed(host) ? "closed" : "open");
}

static DEVICE_ATTR(cover_switch, S_IRUGO, omap_hsmmc_show_cover_switch, NULL);

static ssize_t
omap_hsmmc_show_slot_name(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct mmc_host *mmc = container_of(dev, struct mmc_host, class_dev);
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	return sprintf(buf, "%s\n", mmc_slot(host).name);
}

static DEVICE_ATTR(slot_name, S_IRUGO, omap_hsmmc_show_slot_name, NULL);

/*
 * Configure the response type and send the cmd.
 */
static void
omap_hsmmc_start_command(struct omap_hsmmc_host *host, struct mmc_command *cmd,
	struct mmc_data *data)
{
	int cmdreg = 0, resptype = 0, cmdtype = 0;
	unsigned long flags;

	dev_vdbg(mmc_dev(host->mmc), "%s: CMD%d, argument 0x%08x\n",
		mmc_hostname(host->mmc), cmd->opcode, cmd->arg);
	host->cmd = cmd;
	host->last_cmd = cmd->opcode;

	omap_hsmmc_enable_irq(host, cmd);

	host->response_busy = 0;
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			resptype = 1;
		else if (cmd->flags & MMC_RSP_BUSY) {
			resptype = 3;
			host->response_busy = 1;
		} else
			resptype = 2;
	}

	/*
	 * Unlike OMAP1 controller, the cmdtype does not seem to be based on
	 * ac, bc, adtc, bcr. Only commands ending an open ended transfer need
	 * a val of 0x3, rest 0x0.
	 */
	if (cmd == host->mrq->stop)
		cmdtype = 0x3;

	cmdreg = (cmd->opcode << 24) | (resptype << 16) | (cmdtype << 22);

	if ((host->flags & AUTO_CMD23) && mmc_op_multi(cmd->opcode) &&
	    host->mrq->sbc) {
		cmdreg |= ACEN_ACMD23;
		OMAP_HSMMC_WRITE(host->base, SDMASA, host->mrq->sbc->arg);
	}
	if (data) {
		cmdreg |= DP_SELECT | MSBS | BCE;
		if (data->flags & MMC_DATA_READ)
			cmdreg |= DDIR;
		else
			cmdreg &= ~(DDIR);
	}

	if (host->use_dma)
		cmdreg |= DMAE;

	/* Tuning command is special. Data Present Select should be set */
	if ((cmd->opcode == MMC_SEND_TUNING_BLOCK) ||
	    (cmd->opcode == MMC_SEND_TUNING_BLOCK_HS200)) {
		cmdreg = (cmd->opcode << 24) | (resptype << 16) |
			(cmdtype << 22) | DP_SELECT | DDIR;
	}

	spin_lock_irqsave(&host->irq_lock, flags);
	host->req_in_progress = 1;
	spin_unlock_irqrestore(&host->irq_lock, flags);

	OMAP_HSMMC_WRITE(host->base, ARG, cmd->arg);
	OMAP_HSMMC_WRITE(host->base, CMD, cmdreg);
}

static int
omap_hsmmc_get_dma_dir(struct omap_hsmmc_host *host, struct mmc_data *data)
{
	if (data->flags & MMC_DATA_WRITE)
		return DMA_TO_DEVICE;
	else
		return DMA_FROM_DEVICE;
}

static struct dma_chan *omap_hsmmc_get_dma_chan(struct omap_hsmmc_host *host,
	struct mmc_data *data)
{
	return data->flags & MMC_DATA_WRITE ? host->tx_chan : host->rx_chan;
}

static void omap_hsmmc_request_done(struct omap_hsmmc_host *host, struct mmc_request *mrq)
{
	int dma_ch;
	unsigned long flags;

	spin_lock_irqsave(&host->irq_lock, flags);
	host->req_in_progress = 0;
	dma_ch = host->dma_ch;
	spin_unlock_irqrestore(&host->irq_lock, flags);

	omap_hsmmc_disable_irq(host);
	/* Do not complete the request if DMA is still in progress */
	if (mrq->data && host->use_dma && dma_ch != -1)
		return;
	host->mrq = NULL;
	mmc_request_done(host->mmc, mrq);
}

static void omap_hsmmc_request_clear(struct omap_hsmmc_host *host,
					struct mmc_request *mrq)
{
	unsigned long flags;

	spin_lock_irqsave(&host->irq_lock, flags);
	host->req_in_progress = 0;
	host->dma_ch = -1;
	spin_unlock_irqrestore(&host->irq_lock, flags);

	omap_hsmmc_disable_irq(host);
	if (mrq) {
		host->mrq->done(mrq);
		host->mrq = NULL;
	}
}

/*
 * Notify the transfer complete to MMC core
 */
static void
omap_hsmmc_xfer_done(struct omap_hsmmc_host *host, struct mmc_data *data)
{
	if (!data) {
		struct mmc_request *mrq = host->mrq;

		/* TC before CC from CMD6 - don't know why, but it happens */
		if (host->cmd && host->cmd->opcode == 6 &&
		    host->response_busy) {
			host->response_busy = 0;
			return;
		}

		omap_hsmmc_request_done(host, mrq);
		return;
	}

	host->data = NULL;

	if (!data->error)
		data->bytes_xfered += data->blocks * (data->blksz);
	else
		data->bytes_xfered = 0;

    /* DENSO retention refresh */
#ifdef CONFIG_DEBUG_FS
    do {
        unsigned long   flags = atomic_read(&host->debug_flags);
        u32 opcode = host->mrq->cmd ? host->mrq->cmd->opcode : 0;
        const char *op;

        if(!opcode || !(flags & OMAP_HSMMC_DEBUG_LOG) || !host->mrq->data)
            break;

        if(opcode == MMC_WRITE_MULTIPLE_BLOCK || opcode == MMC_WRITE_BLOCK) {
            if(!(flags & OMAP_HSMMC_DEBUG_LOG_WRITE))
                break;
            op = "WRITE";
        } else if (opcode == MMC_READ_MULTIPLE_BLOCK || opcode == MMC_READ_SINGLE_BLOCK) {
            if(!(flags & OMAP_HSMMC_DEBUG_LOG_READ))
                break;
            op = "READ";
        } else {
            break;
        }

        printk(KERN_INFO "%s(%s) LOG_IO %s(%d) rc=%d offset=%d "
            "blocks=%d bytes=%d\n",
            __func__, mmc_hostname(host->mmc),
            op, opcode,
            host->mrq->cmd->error ?: host->mrq->data->error,
            host->mrq->cmd->arg,
            host->mrq->data->blocks,
            host->mrq->data->bytes_xfered);
    } while(0);
#endif /* CONFIG_DEBUG_FS */

	if (data->stop && (data->error || !host->mrq->sbc))
		omap_hsmmc_start_command(host, data->stop, NULL);
	else
		omap_hsmmc_request_done(host, data->mrq);
}

/*
 * Notify the core about command completion
 */
static void
omap_hsmmc_cmd_done(struct omap_hsmmc_host *host, struct mmc_command *cmd)
{
	if (host->mrq->sbc && (host->cmd == host->mrq->sbc) &&
	    !host->mrq->sbc->error && !(host->flags & AUTO_CMD23)) {
		host->cmd = NULL;
		omap_hsmmc_start_dma_transfer(host);
		omap_hsmmc_start_command(host, host->mrq->cmd,
						host->mrq->data);
		return;
	}

	if (host->cmd->opcode == MMC_SEND_TUNING_BLOCK)
		return;

	host->cmd = NULL;

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			/* response type 2 */
			cmd->resp[3] = OMAP_HSMMC_READ(host->base, RSP10);
			cmd->resp[2] = OMAP_HSMMC_READ(host->base, RSP32);
			cmd->resp[1] = OMAP_HSMMC_READ(host->base, RSP54);
			cmd->resp[0] = OMAP_HSMMC_READ(host->base, RSP76);
		} else {
			/* response types 1, 1b, 3, 4, 5, 6 */
			cmd->resp[0] = OMAP_HSMMC_READ(host->base, RSP10);
		}
	}
	if ((host->data == NULL && !host->response_busy) || cmd->error)
		omap_hsmmc_request_done(host, host->mrq);
}

/*
 * DMA clean up for command errors
 */
static void omap_hsmmc_dma_cleanup(struct omap_hsmmc_host *host, int errno)
{
	int dma_ch;
	unsigned long flags;

	host->data->error = errno;

	spin_lock_irqsave(&host->irq_lock, flags);
	dma_ch = host->dma_ch;
	host->dma_ch = -1;
	spin_unlock_irqrestore(&host->irq_lock, flags);

	if (host->use_dma && dma_ch != -1) {
		struct dma_chan *chan = omap_hsmmc_get_dma_chan(host, host->data);

		dmaengine_terminate_all(chan);
		dma_unmap_sg(chan->device->dev,
			host->data->sg, host->data->sg_len,
			omap_hsmmc_get_dma_dir(host, host->data));

		host->data->host_cookie = 0;
	}
	host->data = NULL;
}

/*
 * Readable error output
 */
#ifdef CONFIG_MMC_DEBUG
static void omap_hsmmc_dbg_report_irq(struct omap_hsmmc_host *host, u32 status)
{
	/* --- means reserved bit without definition at documentation */
	static const char *omap_hsmmc_status_bits[] = {
		"CC"  , "TC"  , "BGE", "---", "BWR" , "BRR" , "---" , "---" ,
		"CIRQ",	"OBI" , "---", "---", "---" , "---" , "---" , "ERRI",
		"CTO" , "CCRC", "CEB", "CIE", "DTO" , "DCRC", "DEB" , "---" ,
		"ACE" , "---" , "---", "---", "CERR", "BADA", "---" , "---"
	};
	char res[256];
	char *buf = res;
	int len, i;

	len = sprintf(buf, "MMC IRQ 0x%x :", status);
	buf += len;

	for (i = 0; i < ARRAY_SIZE(omap_hsmmc_status_bits); i++)
		if (status & (1 << i)) {
			len = sprintf(buf, " %s", omap_hsmmc_status_bits[i]);
			buf += len;
		}

	dev_vdbg(mmc_dev(host->mmc), "%s\n", res);
}
#else
static inline void omap_hsmmc_dbg_report_irq(struct omap_hsmmc_host *host,
					     u32 status)
{
}
#endif  /* CONFIG_MMC_DEBUG */

/*
 * MMC controller internal state machines reset
 *
 * Used to reset command or data internal state machines, using respectively
 *  SRC or SRD bit of SYSCTL register
 * Can be called from interrupt context
 */
static inline void omap_hsmmc_reset_controller_fsm(struct omap_hsmmc_host *host,
						   unsigned long bit)
{
	unsigned long i = 0;
	unsigned long limit = MMC_TIMEOUT_US;

	OMAP_HSMMC_WRITE(host->base, SYSCTL,
			 OMAP_HSMMC_READ(host->base, SYSCTL) | bit);

	/*
	 * OMAP4 ES2 and greater has an updated reset logic.
	 * Monitor a 0->1 transition first
	 */
	if (mmc_slot(host).features & HSMMC_HAS_UPDATED_RESET) {
		while ((!(OMAP_HSMMC_READ(host->base, SYSCTL) & bit))
					&& (i++ < limit))
			udelay(1);
	}
	i = 0;

	while ((OMAP_HSMMC_READ(host->base, SYSCTL) & bit) &&
		(i++ < limit))
		udelay(1);

	if (OMAP_HSMMC_READ(host->base, SYSCTL) & bit)
		dev_err(mmc_dev(host->mmc),
			"Timeout waiting on controller reset in %s\n",
			__func__);
}

static void hsmmc_command_incomplete(struct omap_hsmmc_host *host,
					int err, int end_cmd)
{
	if (end_cmd) {
		omap_hsmmc_reset_controller_fsm(host, SRC);
		if (host->cmd)
			host->cmd->error = err;
	}

	if (host->data) {
		omap_hsmmc_reset_controller_fsm(host, SRD);
		omap_hsmmc_dma_cleanup(host, err);
	} else if (host->mrq && host->mrq->cmd)
		host->mrq->cmd->error = err;
}

static void omap_hsmmc_do_irq(struct omap_hsmmc_host *host, int status)
{
	struct mmc_data *data;
	int i = 0;

	data = host->data;

	OMAP_HSMMC_WRITE(host->base, STAT, status);

	if (status & BRR_EN) {
		for (i = 0; i < host->tuning_size/4; i++)
			host->tuning_data[i] =
				OMAP_HSMMC_READ(host->base, DATA);
		complete(&host->buf_ready);
		return;
	}

	if ((status & CC_EN) && host->cmd)
		omap_hsmmc_cmd_done(host, host->cmd);
	if ((status & TC_EN) && host->mrq)
		omap_hsmmc_xfer_done(host, data);
}

static void omap_hsmmc_do_irq_err(struct omap_hsmmc_host *host, int status)
{
	int end_cmd = 0, end_trans = 0;
	int error = 0;

	omap_hsmmc_dbg_report_irq(host, status);

	if (status & (CTO_EN | CCRC_EN))
		end_cmd = 1;
	if (status & (CTO_EN | DTO_EN))
		hsmmc_command_incomplete(host, -ETIMEDOUT, end_cmd);
	else if (status & (CCRC_EN | DCRC_EN))
		hsmmc_command_incomplete(host, -EILSEQ, end_cmd);

	if (status & ACE_EN) {
		u32 ac12;
		ac12 = OMAP_HSMMC_READ(host->base, AC12);
		if (!(ac12 & ACNE) && host->mrq->sbc) {
			end_cmd = 1;
			if (ac12 & ACTO)
				error =  -ETIMEDOUT;
			else if (ac12 & (ACCE | ACEB | ACIE))
				error = -EILSEQ;
			host->mrq->sbc->error = error;
			hsmmc_command_incomplete(host, error, end_cmd);
		}
		if (!(ac12 & ACNE) && !host->mrq->sbc &&
		    host->mrq->data) {
			end_trans = 1;
			if (ac12 & ACTO)
				host->mrq->data->error = -ETIMEDOUT;
			else if (ac12 & (ACCE | ACEB | ACIE))
				host->mrq->data->error = -EILSEQ;
			omap_hsmmc_reset_controller_fsm(host, SRC);
		}
		dev_dbg(mmc_dev(host->mmc), "AC12 err: 0x%x\n", ac12);
	}

	if (host->data || host->response_busy) {
		end_trans = !end_cmd;
		host->response_busy = 0;
	}

	if (end_cmd)
		omap_hsmmc_cmd_done(host, host->cmd);
	if (end_trans && host->mrq)
		omap_hsmmc_xfer_done(host, host->data);
}

static irqreturn_t omap_hsmmc_irq_thread_handler(int irq, void *dev_id)
{
	struct omap_hsmmc_host *host = dev_id;
	int status;

	status = OMAP_HSMMC_READ(host->base, STAT);
	while (status & INT_EN_MASK && host->req_in_progress) {
		dev_vdbg(mmc_dev(host->mmc), "IRQ Status is %x\n", status);
		omap_hsmmc_do_irq(host, status);
		omap_hsmmc_do_irq_err(host, status);

		/* Flush posted write */
		OMAP_HSMMC_WRITE(host->base, STAT, status);
		status = OMAP_HSMMC_READ(host->base, STAT);
	}

	return IRQ_HANDLED;
}

/*
 * MMC controller IRQ handler
 */
static irqreturn_t omap_hsmmc_irq(int irq, void *dev_id)
{
	struct omap_hsmmc_host *host = dev_id;
	int status;

	status = OMAP_HSMMC_READ(host->base, STAT);

	/*
	 * During a successful bulk data transfer command-completion
	 * interrupt and transfer-completion interrupt will be generated,
	 * but software-timeout timer should be deleted only on non-CC
	 * interrupts (transfer complete or error)
	 */
	if (host->need_i834_errata && (status & (~CC_EN)))
		del_timer_sync(&host->timer);
	/* Error handling might take longer, run it in the threaded handler */
	if (status & ERR_EN)
		return IRQ_WAKE_THREAD;

	while (status & INT_EN_MASK && host->req_in_progress) {
		dev_vdbg(mmc_dev(host->mmc), "IRQ Status is %x\n", status);
		omap_hsmmc_do_irq(host, status);

		/* Flush posted write */
		status = OMAP_HSMMC_READ(host->base, STAT);
	}

	return IRQ_HANDLED;
}

static void omap_hsmmc_soft_timeout(unsigned long data)
{
	struct omap_hsmmc_host *host = (struct omap_hsmmc_host *)data;

	hsmmc_command_incomplete(host, -ETIMEDOUT, 0);

	if (host->cmd)
		omap_hsmmc_cmd_done(host, host->cmd);
	if (host->mrq)
		omap_hsmmc_xfer_done(host, host->data);
}

static void set_sd_bus_power(struct omap_hsmmc_host *host)
{
	unsigned long i;

	OMAP_HSMMC_WRITE(host->base, HCTL,
			 OMAP_HSMMC_READ(host->base, HCTL) | SDBP);
	for (i = 0; i < loops_per_jiffy; i++) {
		if (OMAP_HSMMC_READ(host->base, HCTL) & SDBP)
			break;
		cpu_relax();
	}
}

/*
 * Switch MMC interface voltage ... only relevant for MMC1.
 *
 * MMC2 and MMC3 use fixed 1.8V levels, and maybe a transceiver.
 * The MMC2 transceiver controls are used instead of DAT4..DAT7.
 * Some chips, like eMMC ones, use internal transceivers.
 */
static int omap_hsmmc_switch_opcond(struct omap_hsmmc_host *host, int vdd)
{
	u32 reg_val = 0;
	int ret;

	/* Disable the clocks */
	pm_runtime_put_sync(host->dev);
	if (host->dbclk)
		clk_disable_unprepare(host->dbclk);

	/* Turn the power off */
	ret = mmc_slot(host).set_power(host->dev, host->slot_id, 0, 0);

	/* Turn the power ON with given VDD 1.8 or 3.0v */
	if (!ret)
		ret = mmc_slot(host).set_power(host->dev, host->slot_id, 1,
					       vdd);
	pm_runtime_get_sync(host->dev);
	if (host->dbclk)
		clk_prepare_enable(host->dbclk);

	if (ret != 0)
		goto err;

	OMAP_HSMMC_WRITE(host->base, HCTL,
		OMAP_HSMMC_READ(host->base, HCTL) & SDVSCLR);
	reg_val = OMAP_HSMMC_READ(host->base, HCTL);

	/*
	 * If a MMC dual voltage card is detected, the set_ios fn calls
	 * this fn with VDD bit set for 1.8V. Upon card removal from the
	 * slot, omap_hsmmc_set_ios sets the VDD back to 3V on MMC_POWER_OFF.
	 *
	 * Cope with a bit of slop in the range ... per data sheets:
	 *  - "1.8V" for vdds_mmc1/vdds_mmc1a can be up to 2.45V max,
	 *    but recommended values are 1.71V to 1.89V
	 *  - "3.0V" for vdds_mmc1/vdds_mmc1a can be up to 3.5V max,
	 *    but recommended values are 2.7V to 3.3V
	 *
	 * Board setup code shouldn't permit anything very out-of-range.
	 * TWL4030-family VMMC1 and VSIM regulators are fine (avoiding the
	 * middle range) but VSIM can't power DAT4..DAT7 at more than 3V.
	 */
	if ((1 << vdd) <= MMC_VDD_23_24)
		reg_val |= SDVS18;
	else
		reg_val |= SDVS30;

	OMAP_HSMMC_WRITE(host->base, HCTL, reg_val);
	set_sd_bus_power(host);

	return 0;
err:
	dev_err(mmc_dev(host->mmc), "Unable to switch operating voltage\n");
	return ret;
}

/* Protect the card while the cover is open */
static void omap_hsmmc_protect_card(struct omap_hsmmc_host *host)
{
	if (!mmc_slot(host).get_cover_state)
		return;

	host->reqs_blocked = 0;
	if (mmc_slot(host).get_cover_state(host->dev, host->slot_id)) {
		if (host->protect_card) {
			dev_info(host->dev, "%s: cover is closed, "
					 "card is now accessible\n",
					 mmc_hostname(host->mmc));
			host->protect_card = 0;
		}
	} else {
		if (!host->protect_card) {
			dev_info(host->dev, "%s: cover is open, "
					 "card is now inaccessible\n",
					 mmc_hostname(host->mmc));
			host->protect_card = 1;
		}
	}
}

/*
 * irq handler to notify the core about card insertion/removal
 */
static irqreturn_t omap_hsmmc_detect(int irq, void *dev_id)
{
	struct omap_hsmmc_host *host = dev_id;
	struct omap_mmc_slot_data *slot = &mmc_slot(host);
	struct mmc_request *mrq = host->mrq;
	int carddetect;

	sysfs_notify(&host->mmc->class_dev.kobj, NULL, "cover_switch");

	if (slot->card_detect)
		carddetect = slot->card_detect(host->dev, host->slot_id);
	else {
		omap_hsmmc_protect_card(host);
		carddetect = -ENOSYS;
	}

	/*
	 * If the card was removed in the middle of data transfer last
	 * time, the TC/CC/timeout interrupt is not raised due to which
	 * mmc_request is not cleared. Hence, this card insertion will
	 * still see pending mmc_request. Clear the request to make sure
	 * that this card enumeration is successful.
	 */
	if (carddetect && mrq && host->transfer_incomplete) {
		dev_info(host->dev,
			 "card removed during transfer last time\n");
		dev_dbg(host->dev, "last command was !!! %d !!!\n",
			host->last_cmd);
		hsmmc_command_incomplete(host, -ENOMEDIUM, 1);
		omap_hsmmc_request_clear(host, mrq);
		dev_info(host->dev, "recovery done\n");
		return IRQ_HANDLED;
	 }
	host->transfer_incomplete = false;

	if (carddetect)
		mmc_detect_change(host->mmc, (HZ * 200) / 1000);
	else
		mmc_detect_change(host->mmc, (HZ * 50) / 1000);

	if (host->require_io_delay && !host->mmc->card &&
	    !host->pinctrl_default_state) {
		omap_hsmmc_pinctrl_set_state(host, "default");
		host->pinctrl_default_state = true;
	}

	/*
	 * The current mmc_request is usually null before card removal
	 * sequence is complete. It may not be null if TC/CC interrupt
	 * never happens due to removal of card during a data
	 * transfer. Set a flag to indicate mmc_request was not null
	 * in order to do cleanup on next card insertion.
	 */
	if (!carddetect && mrq)
		host->transfer_incomplete = true;

	return IRQ_HANDLED;
}

static void omap_hsmmc_dma_callback(void *param)
{
	struct omap_hsmmc_host *host = param;
	struct dma_chan *chan;
	struct mmc_data *data;
	int req_in_progress;

	spin_lock_irq(&host->irq_lock);
	if (host->dma_ch < 0) {
		spin_unlock_irq(&host->irq_lock);
		return;
	}

	data = host->mrq->data;
	chan = omap_hsmmc_get_dma_chan(host, data);
	if (!data->host_cookie)
		dma_unmap_sg(chan->device->dev,
			     data->sg, data->sg_len,
			     omap_hsmmc_get_dma_dir(host, data));

	req_in_progress = host->req_in_progress;
	host->dma_ch = -1;
	spin_unlock_irq(&host->irq_lock);

	/* If DMA has finished after TC, complete the request */
	if (!req_in_progress) {
		struct mmc_request *mrq = host->mrq;

		host->mrq = NULL;
		mmc_request_done(host->mmc, mrq);
	}
}

static int omap_hsmmc_pre_dma_transfer(struct omap_hsmmc_host *host,
				       struct mmc_data *data,
				       struct omap_hsmmc_next *next,
				       struct dma_chan *chan)
{
	int dma_len;

	if (!next && data->host_cookie &&
	    data->host_cookie != host->next_data.cookie) {
		dev_warn(host->dev, "[%s] invalid cookie: data->host_cookie %d"
		       " host->next_data.cookie %d\n",
		       __func__, data->host_cookie, host->next_data.cookie);
		data->host_cookie = 0;
	}

	/* Check if next job is already prepared */
	if (next || data->host_cookie != host->next_data.cookie) {
		dma_len = dma_map_sg(chan->device->dev, data->sg, data->sg_len,
				     omap_hsmmc_get_dma_dir(host, data));

	} else {
		dma_len = host->next_data.dma_len;
		host->next_data.dma_len = 0;
	}


	if (dma_len == 0)
		return -EINVAL;

	if (next) {
		next->dma_len = dma_len;
		data->host_cookie = ++next->cookie < 0 ? 1 : next->cookie;
	} else
		host->dma_len = dma_len;

	return 0;
}

/*
 * Routine to configure and start DMA for the MMC card
 */
static int omap_hsmmc_setup_dma_transfer(struct omap_hsmmc_host *host,
					struct mmc_request *req)
{
	struct dma_slave_config cfg;
	struct dma_async_tx_descriptor *tx;
	int ret = 0, i;
	struct mmc_data *data = req->data;
	struct dma_chan *chan;

	/* Sanity check: all the SG entries must be aligned by block size. */
	for (i = 0; i < data->sg_len; i++) {
		struct scatterlist *sgl;

		sgl = data->sg + i;
		if (sgl->length % data->blksz)
			return -EINVAL;
	}
	if ((data->blksz % 4) != 0)
		/* REVISIT: The MMC buffer increments only when MSB is written.
		 * Return error for blksz which is non multiple of four.
		 */
		return -EINVAL;

	BUG_ON(host->dma_ch != -1);

	chan = omap_hsmmc_get_dma_chan(host, data);

	cfg.src_addr = host->mapbase + OMAP_HSMMC_DATA;
	cfg.dst_addr = host->mapbase + OMAP_HSMMC_DATA;
	cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.src_maxburst = data->blksz / 4;
	cfg.dst_maxburst = data->blksz / 4;

	ret = dmaengine_slave_config(chan, &cfg);
	if (ret)
		return ret;

	ret = omap_hsmmc_pre_dma_transfer(host, data, NULL, chan);
	if (ret)
		return ret;

	tx = dmaengine_prep_slave_sg(chan, data->sg, data->sg_len,
		data->flags & MMC_DATA_WRITE ? DMA_MEM_TO_DEV : DMA_DEV_TO_MEM,
		DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!tx) {
		dev_err(mmc_dev(host->mmc), "prep_slave_sg() failed\n");
		/* FIXME: cleanup */
		return -1;
	}

	tx->callback = omap_hsmmc_dma_callback;
	tx->callback_param = host;

	/* Does not fail */
	dmaengine_submit(tx);

	host->dma_ch = 1;

	return 0;
}

static void set_data_timeout(struct omap_hsmmc_host *host,
			     unsigned int timeout_ns,
			     unsigned int timeout_clks)
{
	unsigned int timeout, cycle_ns;
	uint32_t reg, clkd, dto = 0;
	unsigned int wr_mult = 1;

	reg = OMAP_HSMMC_READ(host->base, SYSCTL);
	clkd = (reg & CLKD_MASK) >> CLKD_SHIFT;
	if (clkd == 0)
		clkd = 1;

	if (host->need_i834_errata &&
	    host->data && host->data->flags == MMC_DATA_WRITE)
		wr_mult = 2;

	cycle_ns = 1000000000 / (host->clk_rate / clkd);
	timeout = (timeout_ns * wr_mult) / cycle_ns;
	timeout += timeout_clks;

	if (host->need_i834_errata) {
		host->data_timeout = timeout * cycle_ns;
	} else {
		if (timeout) {
			while ((timeout & 0x80000000) == 0) {
				dto += 1;
				timeout <<= 1;
			}
			dto = 31 - dto;
			timeout <<= 1;
			if (timeout && dto)
				dto += 1;
			if (dto >= 13)
				dto -= 13;
			else
				dto = 0;
		}

		if (dto > 14)
			dto = 14;
		reg &= ~DTO_MASK;
		reg |= dto << DTO_SHIFT;
		OMAP_HSMMC_WRITE(host->base, SYSCTL, reg);
	}
}

static void omap_hsmmc_start_dma_transfer(struct omap_hsmmc_host *host)
{
	struct mmc_request *req = host->mrq;
	struct dma_chan *chan;

	if (!req->data)
		return;
	OMAP_HSMMC_WRITE(host->base, BLK, (req->data->blksz)
				| (req->data->blocks << 16));
	set_data_timeout(host, req->data->timeout_ns,
				req->data->timeout_clks);
	chan = omap_hsmmc_get_dma_chan(host, req->data);
	dma_async_issue_pending(chan);

	if (host->need_i834_errata) {
		unsigned long timeout;

		timeout = jiffies + nsecs_to_jiffies(host->data_timeout);
		mod_timer(&host->timer, timeout);
	}
}

/*
 * Configure block length for MMC/SD cards and initiate the transfer.
 */
static int
omap_hsmmc_prepare_data(struct omap_hsmmc_host *host, struct mmc_request *req)
{
	int ret;
	host->data = req->data;

	if (req->data == NULL) {
		OMAP_HSMMC_WRITE(host->base, BLK, 0);
		/*
		 * Set an arbitrary 100ms data timeout for commands with
		 * busy signal.
		 */
		if (req->cmd->flags & MMC_RSP_BUSY)
			set_data_timeout(host, 100000000U, 0);
		return 0;
	}

	if (host->use_dma) {
		ret = omap_hsmmc_setup_dma_transfer(host, req);
		if (ret != 0) {
			dev_err(mmc_dev(host->mmc), "MMC start dma failure\n");
			return ret;
		}
	}
	return 0;
}

static void omap_hsmmc_post_req(struct mmc_host *mmc, struct mmc_request *mrq,
				int err)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;

	if (host->use_dma && data->host_cookie) {
		struct dma_chan *c = omap_hsmmc_get_dma_chan(host, data);

		dma_unmap_sg(c->device->dev, data->sg, data->sg_len,
			     omap_hsmmc_get_dma_dir(host, data));
		data->host_cookie = 0;
	}
}

static void omap_hsmmc_pre_req(struct mmc_host *mmc, struct mmc_request *mrq,
			       bool is_first_req)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	if (mrq->data->host_cookie) {
		mrq->data->host_cookie = 0;
		return ;
	}

	if (host->use_dma) {
		struct dma_chan *c = omap_hsmmc_get_dma_chan(host, mrq->data);

		if (omap_hsmmc_pre_dma_transfer(host, mrq->data,
						&host->next_data, c))
			mrq->data->host_cookie = 0;
	}
}

/*
 * Request function. for read/write operation
 */
static void omap_hsmmc_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	int err;

	BUG_ON(host->req_in_progress);
	BUG_ON(host->dma_ch != -1);
	if (host->protect_card) {
		if (host->reqs_blocked < 3) {
			/*
			 * Ensure the controller is left in a consistent
			 * state by resetting the command and data state
			 * machines.
			 */
			omap_hsmmc_reset_controller_fsm(host, SRD);
			omap_hsmmc_reset_controller_fsm(host, SRC);
			host->reqs_blocked += 1;
		}
		req->cmd->error = -EBADF;
		if (req->data)
			req->data->error = -EBADF;
		req->cmd->retries = 0;
		mmc_request_done(mmc, req);
		return;
	} else if (host->reqs_blocked)
		host->reqs_blocked = 0;
	WARN_ON(host->mrq != NULL);
	host->mrq = req;
	host->clk_rate = clk_get_rate(host->fclk);
	err = omap_hsmmc_prepare_data(host, req);
	if (err) {
		req->cmd->error = err;
		if (req->data)
			req->data->error = err;
		host->mrq = NULL;
		mmc_request_done(mmc, req);
		return;
	}
	if (req->sbc && !(host->flags & AUTO_CMD23)) {
		omap_hsmmc_start_command(host, req->sbc, NULL);
		return;
	}

	omap_hsmmc_start_dma_transfer(host);
	omap_hsmmc_start_command(host, req->cmd, req->data);
}

/* Routine to configure clock values. Exposed API to core */
static void omap_hsmmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	int do_send_init_stream = 0;

	pm_runtime_get_sync(host->dev);

	if (ios->power_mode != host->power_mode) {
		switch (ios->power_mode) {
		case MMC_POWER_OFF:
#if 0 /* Murata */
			dev_err(host->dev, "MMC_POWER_OFF\n");
#endif
			mmc_slot(host).set_power(host->dev, host->slot_id,
						 0, 0);
			break;
		case MMC_POWER_UP:
#if 0 /* Murata */
			dev_err(host->dev, "MMC_POWER_UP with Vdd: %d\n",ios->vdd);
#endif
			mmc_slot(host).set_power(host->dev, host->slot_id,
						 1, ios->vdd);
			break;
		case MMC_POWER_ON:
#if 0 /* Murata */
			dev_err(host->dev, "MMC_POWER_ON\n");
#endif
			do_send_init_stream = 1;
			break;
		}
		host->power_mode = ios->power_mode;
	}

	/* FIXME: set registers based only on changes to ios */

	omap_hsmmc_set_bus_width(host);

	if (host->pdata->controller_flags & OMAP_HSMMC_SUPPORTS_DUAL_VOLT) {
		/* Only MMC1 can interface at 3V without some flavor
		 * of external transceiver; but they all handle 1.8V.
		 */
		if ((OMAP_HSMMC_READ(host->base, HCTL) & SDVSDET) &&
			(ios->vdd == DUAL_VOLT_OCR_BIT)) {
				/*
				 * The mmc_select_voltage fn of the core does
				 * not seem to set the power_mode to
				 * MMC_POWER_UP upon recalculating the voltage.
				 * vdd 1.8v.
				 */
			if (omap_hsmmc_switch_opcond(host, ios->vdd) != 0)
				dev_dbg(mmc_dev(host->mmc),
						"Switch operation failed\n");
		}
	}

	if (!ios->clock)
		goto end_ios;
	omap_hsmmc_set_clock(host);

	if (do_send_init_stream)
		send_init_stream(host);

	omap_hsmmc_set_bus_mode(host);

end_ios:
	pm_runtime_put_autosuspend(host->dev);
}

static int omap_hsmmc_get_cd(struct mmc_host *mmc)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	if (!mmc_slot(host).card_detect)
		return -ENOSYS;
	return mmc_slot(host).card_detect(host->dev, host->slot_id);
}

static int omap_hsmmc_get_ro(struct mmc_host *mmc)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	if (!mmc_slot(host).get_ro)
		return -ENOSYS;
	return mmc_slot(host).get_ro(host->dev, 0);
}

static void omap_hsmmc_init_card(struct mmc_host *mmc, struct mmc_card *card)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	if (mmc_slot(host).init_card)
		mmc_slot(host).init_card(card);
}

static void omap_hsmmc_conf_bus_power(struct omap_hsmmc_host *host)
{
	u32 hctl, capa, value;

	/* Only MMC1 supports 3.0V */
	if (host->pdata->controller_flags & OMAP_HSMMC_SUPPORTS_DUAL_VOLT) {
		hctl = SDVS30;
		capa = VS30 | VS18;
	} else {
		hctl = SDVS18;
		capa = VS18;
	}

	value = OMAP_HSMMC_READ(host->base, HCTL) & ~SDVS_MASK;
	OMAP_HSMMC_WRITE(host->base, HCTL, value | hctl);

	value = OMAP_HSMMC_READ(host->base, CAPA);
	OMAP_HSMMC_WRITE(host->base, CAPA, value | capa);

	/* Set SD bus power bit */
	set_sd_bus_power(host);
}

static int omap_hsmmc_enable_fclk(struct mmc_host *mmc)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	pm_runtime_get_sync(host->dev);

	return 0;
}

static int omap_hsmmc_disable_fclk(struct mmc_host *mmc)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	pm_runtime_mark_last_busy(host->dev);
	pm_runtime_put_autosuspend(host->dev);

	return 0;
}

/* TJBA Temperature based tuning logic */
static int omap_hsmmc_tuning_logic_30c(int w1_start_window, int w1_len, int w2_start_window, int w2_len)
{
	int phase_delay = -EINVAL;

	if (w2_len < 2) {
		phase_delay = w1_start_window + 4 * (w1_len * 3/4);
	} else if (w1_len >= w2_len) {
		phase_delay = w1_start_window + 4 * (w1_len * 3/16);
	} else if (w1_len >= 2) {
		phase_delay = 0;
	} else if (((w2_len <= 9) && (w1_len < 2))) {
		phase_delay = 80;
	} else {
		phase_delay = w2_start_window + 4 * ((w2_len * 3 + 3)/4);
	}
	return phase_delay;
}

static int omap_hsmmc_tuning_logic_85c(int w1_start_window, int w1_len, int w2_start_window, int w2_len)
{
	int phase_delay = -EINVAL;
	if (w1_len > 12) {
		phase_delay = w1_start_window + 4 * (w1_len * 1/2);
	} else if (w1_len >= (w2_len + 2)) {
		phase_delay = w1_start_window + 4 * (w1_len * 3/16);
	} else if (w1_len >= (w2_len - 2)) {
		phase_delay = 0;
	} else if (w2_len <= 12) {
		phase_delay = 80;
	} else {
		phase_delay = w2_start_window + 4 * ((w2_len * 1 + 1) / 2);
	}

	return phase_delay;
}


static int omap_hsmmc_tuning_logic_125c(int w1_start_window, int w1_len, int w2_start_window, int w2_len)
{
	int phase_delay = -EINVAL;
	if (w1_len > 15) {
		phase_delay = w1_start_window + 4 * (w1_len * 1/2);
	} else if (w1_len >= (w2_len + 4)) {
		phase_delay = w1_start_window + 4 * (w1_len * 3/16);
	} else {
		phase_delay = w2_start_window + 4 * (w2_len * 1/2);
	}
	return phase_delay;
}

struct mmc_iodelay_device {
	struct device *dev;
	unsigned long phys_base;
	void __iomem *reg_base;
};

/*
*select tuning algorithm based on production ID
*Blank -- Old tuning
*THRA -- Old tuning
*Any other -- New tuning.
*/
static int omap_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	u32 val;
	u8 cur_match, prev_match = 0;
	int ret;
	u32 phase_delay = 0;
	u32 start_window = 0;
	int length = 0;
	struct mmc_ios *ios = &mmc->ios;
	struct omap_hsmmc_host *host;
	u32 w1_start_window = 0, w2_start_window = 80;
	u32 w1_len = 0, w2_len = 0;
	int failure_length = 0;
	u32 failure_phase = 0;
	int temp_window = 0, curr_temp = 0;
	unsigned long flags;
	u32 ac12,capa2,dll = 0;
	const u32 *tuning_ref;
	int note_index = 0xFF;
	int max_index = 0;
	int max_window = 0;
	int count = 0;
	u32 max_phase_delay = 0;
	host  = mmc_priv(mmc);

	/* clock tuning is not needed for upto 52MHz */
	if (ios->clock <= EMMC_HSDDR_SD_SDR25_MAX)
		return 0;

	/*
	 * Host Controller needs tuning only in case of SDR104 mode
	 * and for SDR50 mode when Use Tuning for SDR50 is set in
	 * Capabilities register.
	 */
	capa2 = OMAP_HSMMC_READ(host->base, CAPA2);
	if ((ios->clock <= SD_SDR50_MAX_FREQ) && !(capa2 & CAPA2_TSDR50))
		return 0;

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_8:
		tuning_ref = ref_tuning_8bits;
		host->tuning_size = sizeof(ref_tuning_8bits);
		break;
	case MMC_BUS_WIDTH_4:
		tuning_ref = ref_tuning_4bits;
		host->tuning_size = sizeof(ref_tuning_4bits);
		break;
	default:
		return -EINVAL;
	}

	host->tuning_data = kzalloc(host->tuning_size, GFP_KERNEL);
	if (!host->tuning_data)
		return -ENOMEM;

	host->tuning_done = 0;
	omap_hsmmc_stop_clock(host);

	ac12 = OMAP_HSMMC_READ(host->base, AC12);
	ac12 &= ~AC12_UHSMC_MASK;
	OMAP_HSMMC_WRITE(host->base, AC12, ac12);

	if (ios->clock <= SD_SDR50_MAX_FREQ)
		ac12 |= AC12_UHSMC_SDR50;
	else
		ac12 |= AC12_UHSMC_SDR104;

	ac12 |= AC12_UHSMC_SDR104;
	ac12 |= V1V8_SIGEN;

	/* Enable SDR50/SDR104 mode */
	OMAP_HSMMC_WRITE(host->base, AC12, ac12);
	omap_hsmmc_start_clock(host);

	/* Start software tuning Procedure */
	dll |= DLL_SWT;
	OMAP_HSMMC_WRITE(host->base, DLL, dll);
	host->is_tuning =  true;
	if (host->temp_tuning)
		max_phase_delay = TEMP_MAX_PHASE_DELAY;
	else
		max_phase_delay = MAX_PHASE_DELAY;
	dev_info(mmc_dev(host->mmc),"max_phase_delay = 0x%x host->temp_tuning = %d\n",
		max_phase_delay,host->temp_tuning);
	while (phase_delay <= max_phase_delay) {
		struct mmc_command cmd;
		struct mmc_request mrq;

		memset(&cmd, 0, sizeof(struct mmc_command));
		memset(&mrq, 0, sizeof(struct mmc_request));

		if (phase_delay > max_phase_delay)
			break;

		omap_hsmmc_set_dll(host, phase_delay);

		cmd.opcode = opcode;
		cmd.arg = 0;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
		cmd.retries = 0;
		cmd.data = NULL;
		cmd.error = 0;

		mrq.cmd = &cmd;
		host->mrq = &mrq;

		OMAP_HSMMC_WRITE(host->base, BLK, host->tuning_size);
		set_data_timeout(host, 50000000, 0);
		omap_hsmmc_start_command(host, &cmd, NULL);

		host->cmd = NULL;
		host->mrq = NULL;

		/* Wait for Buffer Read Ready interrupt */
		ret = wait_for_completion_timeout(&host->buf_ready,
				msecs_to_jiffies(5000));
		omap_hsmmc_disable_irq(host);
		spin_lock_irqsave(&host->irq_lock, flags);
		host->req_in_progress = 0;
		spin_unlock_irqrestore(&host->irq_lock, flags);

		if (ret == 0) {
			dev_err(mmc_dev(host->mmc),
					"Tuning BRR timeout. phase_delay=%x",
					phase_delay);
			ret = -ETIMEDOUT;
			goto tuning_error;
		}

		cur_match = !memcmp(host->tuning_data,
				tuning_ref,host->tuning_size);

		if (host->temp_tuning) {

			if (cur_match) {
				if (prev_match) {
					length++;
				} else {
					/* Start of first pass window */
					start_window = phase_delay;
					length = 1;
				}

			} else {

				failure_length++;
				failure_phase = phase_delay;
			}

			if (length > 0 &&
					(!cur_match || phase_delay == max_phase_delay)) {
				if (failure_length <= 1 && start_window == 0) {
					w1_start_window = start_window;
					w1_len = length - 1;
					length = 0;
				} else {
					w2_start_window = min((int)(start_window + 12),80);
					w2_len = max(length - 3, 0);
				}
			}
			prev_match = cur_match;
			phase_delay += 4;

		}
		else { /*regular tuning */

			if (cur_match == true) {
				if (prev_match == false) {
					/* new window */
					note_index = count;
					length = 1;
				} else {
					length++;
				}
				prev_match = true;
				if (length > max_window) {
					max_index = note_index;
					max_window = length;
				}
			}else {
				prev_match = false;
			}
			phase_delay += 4;
			count++;
		}
	}
	host->is_tuning = false;

	if (host->temp_tuning) {
		if (!(w1_len || w2_len)) { /*max_len*/
			dev_err(mmc_dev(host->mmc), "Unable to find match\n");
			ret = -EIO;
			goto tuning_error;
		}

		val = OMAP_HSMMC_READ(host->base, AC12);
		if (!(val & AC12_SCLK_SEL)) {
			ret = -EIO;
			goto tuning_error;
		}
		/* Read temperature and select OTV DLL ratio */
		ret = thermal_zone_get_temp(host->tzd, &curr_temp);
		dev_info(mmc_dev(host->mmc),
				"temp=%dc,w1_start_window=%d w1_len=%d 	w2_start_window=%d,w2_len=%d \n",
				curr_temp/1000,w1_start_window,w1_len, w2_start_window,w2_len);
		if (ret < 0)
			return -EPROBE_DEFER;

		if (curr_temp < 30000) {	/* T < 30 C (30000 milli C)*/
			phase_delay = omap_hsmmc_tuning_logic_30c(w1_start_window,
					w1_len, w2_start_window, w2_len);
			temp_window = 0;
		} else if (curr_temp < 85000) {		/* T < ~85 C */
			phase_delay = omap_hsmmc_tuning_logic_85c(w1_start_window,
					w1_len, w2_start_window, w2_len);
			temp_window = 1;
		} else {					/* T >= ~85 C*/
			phase_delay = omap_hsmmc_tuning_logic_125c(w1_start_window,
					w1_len, w2_start_window, w2_len);
			temp_window = 2;
		}
		dev_info(mmc_dev(host->mmc),
				"final phase_delay after calculating temp factor %d \n",phase_delay);
		omap_hsmmc_set_dll(host, phase_delay);
	} else {

		if (!max_window) {
			dev_err(mmc_dev(host->mmc), "Unable to find match\n");
			ret = -EIO;
			goto tuning_error;
		}

		ac12 = OMAP_HSMMC_READ(host->base, AC12);
		if (!(ac12 & AC12_SCLK_SEL)) {
			ret = -EIO;
			goto tuning_error;
		}

		dll = OMAP_HSMMC_READ(host->base, DLL);
		dll &= ~DLL_SWT;
		OMAP_HSMMC_WRITE(host->base, DLL, dll);
		count = 4 * (max_index + (max_window >> 1));
		dev_info(mmc_dev(host->mmc), "selected tuning value 0x%x"
				" max_index %d, max_window %d\n", count,
				max_index, max_window);
		if (omap_hsmmc_set_dll(host, count)) {
			ret = -EIO;
			goto tuning_error;
		}
		host->tuning_fsrc = count;
		host->tuning_uhsmc = (OMAP_HSMMC_READ(host->base, AC12)
				& AC12_UHSMC_MASK);
		host->tuning_opcode = opcode;
		host->tuning_done = 1;
	}
	omap_hsmmc_reset_controller_fsm(host, SRD);
	omap_hsmmc_reset_controller_fsm(host, SRC);

	return 0;

tuning_error:
	dev_err(mmc_dev(host->mmc),
		"Tuning failed. Using fixed sampling clock\n");
	ac12 = OMAP_HSMMC_READ(host->base, AC12);
	ac12 &= ~(AC12_UHSMC_MASK | AC12_SCLK_SEL);
	OMAP_HSMMC_WRITE(host->base, AC12, ac12);

	dll = OMAP_HSMMC_READ(host->base, DLL);
	dll &= ~(DLL_FORCE_VALUE | DLL_SWT);
	OMAP_HSMMC_WRITE(host->base, DLL, dll);

	omap_hsmmc_reset_controller_fsm(host, SRD);
	omap_hsmmc_reset_controller_fsm(host, SRC);
	return ret;
}

static int omap_start_signal_voltage_switch(struct mmc_host *mmc,
			struct mmc_ios *ios)
{
	struct omap_hsmmc_host *host;
	u32 value = 0;
	int ret = 0;

	if (!(mmc->caps & (MMC_CAP_UHS_SDR12 |
	      MMC_CAP_UHS_SDR25 | MMC_CAP_UHS_DDR50)))
		return 0;

	host  = mmc_priv(mmc);

	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
		omap_hsmmc_conf_bus_power(host);

		value = OMAP_HSMMC_READ(host->base, AC12);
		value &= ~V1V8_SIGEN;
		OMAP_HSMMC_WRITE(host->base, AC12, value);
		dev_dbg(mmc_dev(host->mmc), " i/o voltage switch to 3V\n");
		return 0;
	}

	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
		value = OMAP_HSMMC_READ(host->base, HCTL);
		value &= ~SDVS_MASK;
		value |= SDVS18;
		OMAP_HSMMC_WRITE(host->base, HCTL, value);
		value |= SDBP;
		OMAP_HSMMC_WRITE(host->base, HCTL, value);

		ret = mmc_slot(host).set_power(host->dev, host->slot_id,
						 1, VDD_165_195);
		if (ret < 0) {
			dev_dbg(mmc_dev(host->mmc), "failed to switch 1.8v\n");
			goto voltage_switch_error;
		}
	}

voltage_switch_error:
	return ret;
}

static int omap_hsmmc_card_busy_low(struct omap_hsmmc_host *host)
{
	u32 value;
	unsigned long timeout;

	value = OMAP_HSMMC_READ(host->base, CON);
	value &= ~CLKEXTFREE;
	value |= PADEN;
	OMAP_HSMMC_WRITE(host->base, CON, value);

	timeout = jiffies + msecs_to_jiffies(1);
	do {
		value = OMAP_HSMMC_READ(host->base, PSTATE);
		if (!(value & (CLEV | DLEV)))
			return true;

		usleep_range(100, 200);
	} while (!time_after(jiffies, timeout));

	dev_err(mmc_dev(host->mmc), "timeout : i/o low 0x%x\n", value);

	return false;
}

static int omap_hsmmc_card_busy_high(struct omap_hsmmc_host *host)
{
	u32 value;
	unsigned long timeout;

	value = OMAP_HSMMC_READ(host->base, CON);
	value |= CLKEXTFREE;
	OMAP_HSMMC_WRITE(host->base, CON, value);

	timeout = jiffies + msecs_to_jiffies(1);
	do {
		value = OMAP_HSMMC_READ(host->base, PSTATE);
		if ((value & CLEV) && (value & DLEV)) {
			value = OMAP_HSMMC_READ(host->base, CON);
			value &= ~(CLKEXTFREE | PADEN);
			OMAP_HSMMC_WRITE(host->base, CON,
					 (value & ~(CLKEXTFREE | PADEN)));
			return false;
		}

		usleep_range(100, 200);
	} while (!time_after(jiffies, timeout));

	dev_dbg(mmc_dev(host->mmc), "timeout : i/o high 0x%x\n", value);

	return true;
}

static int omap_hsmmc_card_busy(struct mmc_host *mmc)
{
	struct omap_hsmmc_host *host;
	u32 value;
	int ret;

	host  = mmc_priv(mmc);
	value = OMAP_HSMMC_READ(host->base, AC12);

	if (value & V1V8_SIGEN)
		ret = omap_hsmmc_card_busy_high(host);
	else
		ret = omap_hsmmc_card_busy_low(host);

	return ret;
}

static const struct mmc_host_ops omap_hsmmc_ops = {
	.enable = omap_hsmmc_enable_fclk,
	.disable = omap_hsmmc_disable_fclk,
	.post_req = omap_hsmmc_post_req,
	.pre_req = omap_hsmmc_pre_req,
	.request = omap_hsmmc_request,
	.set_ios = omap_hsmmc_set_ios,
	.get_cd = omap_hsmmc_get_cd,
	.get_ro = omap_hsmmc_get_ro,
	.init_card = omap_hsmmc_init_card,
	.start_signal_voltage_switch = omap_start_signal_voltage_switch,
	.card_busy = omap_hsmmc_card_busy,
	.execute_tuning = omap_execute_tuning,
	/* NYET -- enable_sdio_irq */
};

#ifdef CONFIG_DEBUG_FS

static int omap_hsmmc_regs_show(struct seq_file *s, void *data)
{
	struct mmc_host *mmc = s->private;
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	seq_printf(s, "mmc%d:\n ctx_loss:\t%d\n\nregs:\n",
			mmc->index, host->context_loss);

	pm_runtime_get_sync(host->dev);

	seq_printf(s, "CON:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, CON));
	seq_printf(s, "HCTL:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, HCTL));
	seq_printf(s, "SYSCTL:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, SYSCTL));
	seq_printf(s, "IE:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, IE));
	seq_printf(s, "ISE:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, ISE));
	seq_printf(s, "CAPA:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, CAPA));

	pm_runtime_mark_last_busy(host->dev);
	pm_runtime_put_autosuspend(host->dev);

	return 0;
}

static int omap_hsmmc_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, omap_hsmmc_regs_show, inode->i_private);
}

static const struct file_operations mmc_regs_fops = {
	.open           = omap_hsmmc_regs_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int denso_health_bad_blocks_show(struct seq_file *s, void *data)
{
	struct omap_hsmmc_host *host = (struct omap_hsmmc_host *) s->private;
    struct mmc_host *mmc = host->mmc;
    struct denso_mmc_health health_report;
    int rc;

    memset(&health_report, 0, sizeof(health_report));
    rc = denso_get_bad_block_counters(mmc, &health_report);
    if(!rc) {
        seq_printf(s, "Intial Bad Block Count: %u\n",
                health_report.initial_bad_block_count);
        seq_printf(s, "Runtime Bad Block Count: %u\n",
                health_report.runtime_bad_block_count);
        seq_printf(s, "Remaining Spare Block Count: %u\n",
                health_report.remaining_spare_block_count);
    } else {
        printk(KERN_WARNING "!%s() cannot get bad block counters (%d)\n",
                    __func__, rc);
        /* per fs/seq_file.c: ->show() returns 0 if success and negative
            number in case of error.  Using -EIO */
        rc = -EIO;
    }

    return rc;
}

static int denso_health_bad_blocks_open(struct inode *inode, struct file *file)
{
	return single_open(file, denso_health_bad_blocks_show, inode->i_private);
}

static const struct file_operations denso_health_bad_blocks_fops = {
	.open           = denso_health_bad_blocks_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static void omap_hsmmc_debugfs(struct omap_hsmmc_host *host)
{
    struct mmc_host *mmc = host->mmc;
    dev_dbg(mmc_dev(host->mmc), "+%s()\n", __func__);
	if (mmc->debugfs_root) {
		debugfs_create_file("regs", S_IRUSR, mmc->debugfs_root,
			mmc, &mmc_regs_fops);
        /* DENSO NAND retention refresh */
        debugfs_create_file("log_io", S_IRUSR | S_IWUSR,
            mmc->debugfs_root, host, &omap_hsmmc_log_io);
        /* DENSO eMMC device health reporting */
        debugfs_create_file(DENSO_HEALTH_DEBUGFS_ERASURES,
                                    S_IRUSR | S_IRGRP,
                                    mmc->debugfs_root,
                                    host, &denso_health_fops);
        debugfs_create_file(DENSO_HEALTH_DEBUGFS_BAD_BLOCKS,
                            S_IRUSR,
                            mmc->debugfs_root, host,
                            &denso_health_bad_blocks_fops);
    }
    dev_dbg(mmc_dev(host->mmc), "-%s()\n", __func__);
}

#else

static void omap_hsmmc_debugfs(struct omap_hsmmc_host *mmc)
{
}

#endif /* CONFIG_DEBUG_FS */

#ifdef CONFIG_OF
static const struct omap_mmc_of_data omap3_pre_es3_mmc_of_data = {
	/* See 35xx errata 2.1.1.128 in SPRZ278F */
	.controller_flags = OMAP_HSMMC_BROKEN_MULTIBLOCK_READ,
};

static const struct omap_mmc_of_data omap4_mmc_of_data = {
	.reg_offset = 0x100,
};

static const struct of_device_id omap_mmc_of_match[] = {
	{
		.compatible = "ti,omap2-hsmmc",
	},
	{
		.compatible = "ti,omap3-pre-es3-hsmmc",
		.data = &omap3_pre_es3_mmc_of_data,
	},
	{
		.compatible = "ti,omap3-hsmmc",
	},
	{
		.compatible = "ti,omap4-hsmmc",
		.data = &omap4_mmc_of_data,
	},
	{
		.compatible = "ti,dra7-hsmmc",
		.data = &omap4_mmc_of_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, omap_mmc_of_match);

static struct omap_mmc_platform_data *of_get_hsmmc_pdata(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;
	struct device_node *np = dev->of_node;
	u32 bus_width, max_freq;
	int cd_gpio, wp_gpio;

	cd_gpio = of_get_named_gpio(np, "cd-gpios", 0);
	wp_gpio = of_get_named_gpio(np, "wp-gpios", 0);
	if (cd_gpio == -EPROBE_DEFER || wp_gpio == -EPROBE_DEFER)
		return ERR_PTR(-EPROBE_DEFER);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM); /* out of memory */

	if (of_find_property(np, "ti,dual-volt", NULL))
		pdata->controller_flags |= OMAP_HSMMC_SUPPORTS_DUAL_VOLT;

	/* This driver only supports 1 slot */
	pdata->nr_slots = 1;
	pdata->slots[0].switch_pin = cd_gpio;
	pdata->slots[0].gpio_wp = wp_gpio;

	if (of_find_property(np, "ti,non-removable", NULL)) {
		pdata->slots[0].nonremovable = true;
		pdata->slots[0].no_regulator_off_init = true;
	}
	of_property_read_u32(np, "bus-width", &bus_width);
	if (bus_width == 4)
		pdata->slots[0].caps |= MMC_CAP_4_BIT_DATA;
	else if (bus_width == 8)
		pdata->slots[0].caps |= MMC_CAP_8_BIT_DATA;

	if (of_find_property(np, "cap-power-off-card", NULL))
		pdata->slots[0].caps |= MMC_CAP_POWER_OFF_CARD;

	if (of_find_property(np, "keep-power-in-suspend", NULL))
		pdata->slots[0].pm_caps |= MMC_PM_KEEP_POWER;

	if (of_find_property(np, "ti,needs-special-reset", NULL))
		pdata->slots[0].features |= HSMMC_HAS_UPDATED_RESET;

	if (!of_property_read_u32(np, "max-frequency", &max_freq))
		pdata->max_freq = max_freq;

	if (of_find_property(np, "ti,needs-special-hs-handling", NULL))
		pdata->slots[0].features |= HSMMC_HAS_HSPE_SUPPORT;

	if (of_find_property(np, "keep-power-in-suspend", NULL))
		pdata->slots[0].pm_caps |= MMC_PM_KEEP_POWER;

	if (of_find_property(np, "enable-sdio-wakeup", NULL))
		pdata->slots[0].pm_caps |= MMC_PM_WAKE_SDIO_IRQ;

	return pdata;
}
#else
static inline struct omap_mmc_platform_data
			*of_get_hsmmc_pdata(struct device *dev)
{
	return ERR_PTR(-EINVAL);
}
#endif

#define DENSO_QSPI_PARAMETER_SIZE 32
static void get_denso_production_info(struct omap_hsmmc_host *host)
{
        char product[DENSO_QSPI_PARAMETER_SIZE + 1];
        char product_revision[DENSO_QSPI_PARAMETER_SIZE + 1];
        char model[DENSO_QSPI_PARAMETER_SIZE + 1];
        uint counter;

        read_nor(BOOTFLAG, 0x20, product, DENSO_QSPI_PARAMETER_SIZE);
        read_nor(BOOTFLAG, 0x40, product_revision, DENSO_QSPI_PARAMETER_SIZE);
        read_nor(BOOTFLAG, 0x60, model, DENSO_QSPI_PARAMETER_SIZE);
        printk(KERN_INFO "\nRCG EMMC driver Bootparameters PRODUCT=%s REVISION=%s MODEL=%s \n",
                 product,product_revision, model);

        host->temp_tuning = true;
        /*
         *  This will cover default memory value 0xFF and garbage value
         *  THRA board should be have default value or "THRA"
         *  All other string is handled as new tuning
         */

        for (counter = 0; counter < DENSO_QSPI_PARAMETER_SIZE ; counter++){
                if ((0x20 > product[counter]) || (0x7D < product[counter])){
                        if (0 == product[counter]){
                                if (0 == counter)
                                        host->temp_tuning = false;
                                else
                                        break;
                        }
                        host->temp_tuning = false;
                        return;
                }
        }
        if ( !strcmp(product, "THRA"))
                host->temp_tuning = false;
}

static int omap_hsmmc_probe(struct platform_device *pdev)
{
	struct omap_mmc_platform_data *pdata = pdev->dev.platform_data;
	struct mmc_host *mmc;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct omap_hsmmc_host *host = NULL;
	struct resource *res;
	int ret, irq;
	const struct of_device_id *match;
	dma_cap_mask_t mask;
	unsigned tx_req, rx_req;
	const struct omap_mmc_of_data *data;
	void __iomem *base;

#if 1 /* Murata */
	pr_info("!!!!!!!!!!!!!!!!!!!!!%s called!!!!!!!!!!!!!!!!!!!!!\n",__FUNCTION__);
#endif /* Murata */

	match = of_match_device(of_match_ptr(omap_mmc_of_match), &pdev->dev);
	if (match) {
		pdata = of_get_hsmmc_pdata(&pdev->dev);

		if (IS_ERR(pdata))
			return PTR_ERR(pdata);

		if (match->data) {
			data = match->data;
			pdata->reg_offset = data->reg_offset;
			pdata->controller_flags |= data->controller_flags;
		}
	}

	if (pdata == NULL) {
		dev_err(&pdev->dev, "Platform Data is missing\n");
		return -ENXIO;
	}

	if (pdata->nr_slots == 0) {
		dev_err(&pdev->dev, "No Slots\n");
		return -ENXIO;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (res == NULL || irq < 0)
		return -ENXIO;

	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	ret = omap_hsmmc_gpio_init(pdata);
	if (ret)
		goto err;

	mmc = mmc_alloc_host(sizeof(struct omap_hsmmc_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto err_alloc;
	}

#if 1 /* Murata */
	if (strcmp(mmc_hostname(mmc), "mmc2") == 0) {
               wifi_mmc_host = mmc;
	}
#endif /* Murata */

	host		= mmc_priv(mmc);
	host->mmc	= mmc;
	host->pdata	= pdata;
	host->dev	= &pdev->dev;
	host->use_dma	= 1;
	host->dma_ch	= -1;
	host->irq	= irq;
	host->slot_id	= 0;
	host->mapbase	= res->start + pdata->reg_offset;
	host->base	= base + pdata->reg_offset;
	host->power_mode = MMC_POWER_OFF;
	host->next_data.cookie = 1;
	host->pbias_enabled = 0;
	host->regulator_enabled = 0;
	host->last_cmd = 0;
	__setup_timer(&host->timer, omap_hsmmc_soft_timeout,
		    (unsigned long) host, TIMER_IRQSAFE);

	platform_set_drvdata(pdev, host);

	mmc->ops	= &omap_hsmmc_ops;

	/*select tuning algorithm based on production id*/
	get_denso_production_info(host);

	mmc->f_min = OMAP_MMC_MIN_CLOCK;

	spin_lock_init(&host->irq_lock);
	init_completion(&host->buf_ready);

	host->fclk = devm_clk_get(&pdev->dev, "fck");
	if (IS_ERR(host->fclk)) {
		ret = PTR_ERR(host->fclk);
		host->fclk = NULL;
		goto err1;
	}

	if (pdata->max_freq > 0) {
		mmc->f_max = pdata->max_freq;
		ret = clk_set_rate(host->fclk, pdata->max_freq);
		if (ret) {
			dev_err(dev, "failed to set clock to %d\n",
				pdata->max_freq);
			goto err1;
		}
	} else {
		mmc->f_max = OMAP_MMC_MAX_CLOCK;
	}

	if (host->pdata->controller_flags & OMAP_HSMMC_BROKEN_MULTIBLOCK_READ) {
		dev_info(&pdev->dev, "multiblock reads disabled due to 35xx erratum 2.1.1.128; MMC read performance may suffer\n");
		mmc->caps2 |= MMC_CAP2_NO_MULTI_READ;
	}

	host->pinctrl = devm_pinctrl_get(&pdev->dev);
	omap_hsmmc_pinctrl_set_state(host, "default");
	host->pinctrl_default_state = true;

	pm_runtime_enable(host->dev);
	pm_runtime_get_sync(host->dev);
	pm_runtime_set_autosuspend_delay(host->dev, MMC_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(host->dev);

	omap_hsmmc_context_save(host);

	host->dbclk = devm_clk_get(&pdev->dev, "mmchsdb_fck");
	/*
	 * MMC can still work without debounce clock.
	 */
	if (IS_ERR(host->dbclk)) {
		host->dbclk = NULL;
	} else if (clk_prepare_enable(host->dbclk) != 0) {
		dev_warn(mmc_dev(host->mmc), "Failed to enable debounce clk\n");
		host->dbclk = NULL;
	}

	/* Since we do only SG emulation, we can have as many segs
	 * as we want. */
	mmc->max_segs = 1024;

	mmc->max_blk_size = 512;       /* Block Length at max can be 1024 */
	mmc->max_blk_count = 0xFFFF;    /* No. of Blocks is 16 bits */
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size = mmc->max_req_size;

	mmc->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED |
		     MMC_CAP_WAIT_WHILE_BUSY | MMC_CAP_ERASE;

	mmc->caps |= mmc_slot(host).caps;
	if (mmc->caps & MMC_CAP_8_BIT_DATA)
		mmc->caps |= MMC_CAP_4_BIT_DATA;

	if (mmc_slot(host).nonremovable)
		mmc->caps |= MMC_CAP_NONREMOVABLE;

	mmc->pm_caps = mmc_slot(host).pm_caps;

	if (of_property_read_bool(np, "sd-uhs-sdr12"))
		mmc->caps |= MMC_CAP_UHS_SDR12;
	if (of_property_read_bool(np, "sd-uhs-sdr25"))
		mmc->caps |= MMC_CAP_UHS_SDR25;
	if (of_property_read_bool(np, "sd-uhs-sdr50"))
		mmc->caps |= MMC_CAP_UHS_SDR50;
	if (of_property_read_bool(np, "sd-uhs-sdr104"))
		mmc->caps |= MMC_CAP_UHS_SDR104;
	if (of_property_read_bool(np, "sd-uhs-ddr50"))
		mmc->caps |= MMC_CAP_UHS_DDR50;
	if (of_property_read_bool(np, "mmc-ddr-1_8v"))
		mmc->caps |= MMC_CAP_1_8V_DDR;
	if (of_property_read_bool(np, "mmc-hs200-1_8v"))
		mmc->caps2 |= MMC_CAP2_HS200_1_8V_SDR;

	mmc->caps |= MMC_CAP_DRIVER_TYPE_A | MMC_CAP_DRIVER_TYPE_C |
			MMC_CAP_DRIVER_TYPE_D;

	omap_hsmmc_conf_bus_power(host);

	if (!pdev->dev.of_node) {
		res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "tx");
		if (!res) {
			dev_err(mmc_dev(host->mmc), "cannot get DMA TX channel\n");
			ret = -ENXIO;
			goto err_irq;
		}
		tx_req = res->start;

		res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "rx");
		if (!res) {
			dev_err(mmc_dev(host->mmc), "cannot get DMA RX channel\n");
			ret = -ENXIO;
			goto err_irq;
		}
		rx_req = res->start;
	}

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	host->rx_chan =
		dma_request_slave_channel_compat(mask, omap_dma_filter_fn,
						 &rx_req, &pdev->dev, "rx");

	if (!host->rx_chan) {
		dev_err(mmc_dev(host->mmc), "unable to obtain RX DMA engine channel %u\n", rx_req);
		ret = -ENXIO;
		goto err_irq;
	}

	host->tx_chan =
		dma_request_slave_channel_compat(mask, omap_dma_filter_fn,
						 &tx_req, &pdev->dev, "tx");

	if (!host->tx_chan) {
		dev_err(mmc_dev(host->mmc), "unable to obtain TX DMA engine channel %u\n", tx_req);
		ret = -ENXIO;
		goto err_irq;
	}

	/* Request IRQ for MMC operations */
	ret = devm_request_threaded_irq(&pdev->dev, host->irq,
				omap_hsmmc_irq, omap_hsmmc_irq_thread_handler,
				IRQF_ONESHOT, mmc_hostname(mmc), host);

	if (ret) {
		dev_err(mmc_dev(host->mmc), "Unable to grab HSMMC IRQ\n");
		goto err_irq;
	}

	if (pdata->init != NULL) {
		if (pdata->init(&pdev->dev) != 0) {
			dev_err(mmc_dev(host->mmc),
				"Unable to configure MMC IRQs\n");
			goto err_irq;
		}
	}

	if (omap_hsmmc_have_reg() && !mmc_slot(host).set_power) {
		ret = omap_hsmmc_reg_get(host);
		if (ret)
			goto err_reg;
		host->use_reg = 1;
	}

	mmc->ocr_avail = mmc_slot(host).ocr_mask;

	/* Request IRQ for card detect */
	if ((mmc_slot(host).card_detect_irq)) {
		ret = devm_request_threaded_irq(&pdev->dev,
						mmc_slot(host).card_detect_irq,
						NULL, omap_hsmmc_detect,
					   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					   mmc_hostname(mmc), host);
		if (ret) {
			dev_err(mmc_dev(host->mmc),
				"Unable to grab MMC CD IRQ\n");
			goto err_irq_cd;
		}
		pdata->suspend = omap_hsmmc_suspend_cdirq;
		pdata->resume = omap_hsmmc_resume_cdirq;
	}

	omap_hsmmc_disable_irq(host);

	if (of_device_is_compatible(np, "ti,dra7-hsmmc"))
		host->require_io_delay = true;

	omap_hsmmc_protect_card(host);

	mmc_add_host(mmc);

	if (mmc_slot(host).name != NULL) {
		ret = device_create_file(&mmc->class_dev, &dev_attr_slot_name);
		if (ret < 0)
			goto err_slot_name;
	}
	if (mmc_slot(host).card_detect_irq && mmc_slot(host).get_cover_state) {
		ret = device_create_file(&mmc->class_dev,
					&dev_attr_cover_switch);
		if (ret < 0)
			goto err_slot_name;
	}

    omap_hsmmc_attr_init(mmc);  /* DENSO retention refresh */
	host->tzd = thermal_zone_get_zone_by_name("cpu_thermal");
	if (!host->tzd)
		return -EPROBE_DEFER;
	omap_hsmmc_debugfs(host);
	pm_runtime_mark_last_busy(host->dev);
	pm_runtime_put_autosuspend(host->dev);

#if 1 /* Murata */
	pr_info("exiting omap_hsmmc_probe successfully!!!!!!!!!!!!!!!!!!!!!\n");
#endif
	return 0;

err_slot_name:
	mmc_remove_host(mmc);
err_irq_cd:
	if (host->use_reg)
		omap_hsmmc_reg_put(host);
err_reg:
	if (host->pdata->cleanup)
		host->pdata->cleanup(&pdev->dev);
err_irq:
	if (host->tx_chan)
		dma_release_channel(host->tx_chan);
	if (host->rx_chan)
		dma_release_channel(host->rx_chan);
	pm_runtime_put_sync(host->dev);
	pm_runtime_disable(host->dev);
	if (host->dbclk)
		clk_disable_unprepare(host->dbclk);
err1:
	mmc_free_host(mmc);
err_alloc:
	omap_hsmmc_gpio_free(pdata);
err:
	return ret;
}

static int omap_hsmmc_remove(struct platform_device *pdev)
{
	struct omap_hsmmc_host *host = platform_get_drvdata(pdev);

	pm_runtime_get_sync(host->dev);
    omap_hsmmc_attr_term(host);    /* DENSO retention refresh */
	mmc_remove_host(host->mmc);
	if (host->use_reg)
		omap_hsmmc_reg_put(host);
	if (host->pdata->cleanup)
		host->pdata->cleanup(&pdev->dev);

	if (host->tx_chan)
		dma_release_channel(host->tx_chan);
	if (host->rx_chan)
		dma_release_channel(host->rx_chan);

	del_timer_sync(&host->timer);

	pm_runtime_put_sync(host->dev);
	pm_runtime_disable(host->dev);
	if (host->dbclk)
		clk_disable_unprepare(host->dbclk);

	omap_hsmmc_gpio_free(host->pdata);
	mmc_free_host(host->mmc);

	return 0;
}

#ifdef CONFIG_PM
static int omap_hsmmc_prepare(struct device *dev)
{
	struct omap_hsmmc_host *host = dev_get_drvdata(dev);

	if (host->pdata->suspend)
		return host->pdata->suspend(dev, host->slot_id);

	return 0;
}

static void omap_hsmmc_complete(struct device *dev)
{
	struct omap_hsmmc_host *host = dev_get_drvdata(dev);

	if (host->pdata->resume)
		host->pdata->resume(dev, host->slot_id);

}

static int omap_hsmmc_suspend(struct device *dev)
{
	struct omap_hsmmc_host *host = dev_get_drvdata(dev);

	if (!host)
		return 0;

	pm_runtime_get_sync(host->dev);

	if (!(host->mmc->pm_flags & MMC_PM_KEEP_POWER)) {
		omap_hsmmc_disable_irq(host);
		OMAP_HSMMC_WRITE(host->base, HCTL,
				OMAP_HSMMC_READ(host->base, HCTL) & ~SDBP);
	}
	host->tuning_done = 0;

	if (host->dbclk)
		clk_disable_unprepare(host->dbclk);

	del_timer_sync(&host->timer);

	pm_runtime_put_sync(host->dev);

	/* Select sleep pin state */
	pinctrl_pm_select_sleep_state(host->dev);

	return 0;
}

/* Routine to resume the MMC device */
static int omap_hsmmc_resume(struct device *dev)
{
	struct omap_hsmmc_host *host = dev_get_drvdata(dev);

	if (!host)
		return 0;

	/* Select default pin state */
	pinctrl_pm_select_default_state(host->dev);
	host->pinctrl_default_state = true;

	pm_runtime_get_sync(host->dev);

	if (host->dbclk)
		clk_prepare_enable(host->dbclk);

	if (!(host->mmc->pm_flags & MMC_PM_KEEP_POWER))
		omap_hsmmc_conf_bus_power(host);

	omap_hsmmc_protect_card(host);

	pm_runtime_mark_last_busy(host->dev);
	pm_runtime_put_autosuspend(host->dev);
	return 0;
}

#else
#define omap_hsmmc_prepare	NULL
#define omap_hsmmc_complete	NULL
#define omap_hsmmc_suspend	NULL
#define omap_hsmmc_resume	NULL
#endif

static int omap_hsmmc_runtime_suspend(struct device *dev)
{
	struct omap_hsmmc_host *host;

	host = platform_get_drvdata(to_platform_device(dev));
	if (host->tuning_done)
		omap_hsmmc_restore_dll(host);

	omap_hsmmc_context_save(host);
	dev_dbg(dev, "(mmc%d) disabled\n", host->mmc->index);

	/* Optionally let pins go into sleep state */
	pinctrl_pm_select_sleep_state(host->dev);

	return 0;
}

static int omap_hsmmc_runtime_resume(struct device *dev)
{
	struct omap_hsmmc_host *host;
	int ret;

	host = platform_get_drvdata(to_platform_device(dev));

	if (!IS_ERR(host->pinctrl_state)) {
		ret = pinctrl_select_state(host->pinctrl, host->pinctrl_state);
		if (ret)
			dev_err(mmc_dev(host->mmc),
				"failed to activate pinctrl state\n");
	}

	omap_hsmmc_context_restore(host);

	if (host->tuning_done)
		omap_hsmmc_restore_dll(host);

	dev_dbg(dev, "(mmc%d) enabled\n", host->mmc->index);

	return 0;
}

static struct dev_pm_ops omap_hsmmc_dev_pm_ops = {
	.suspend	= omap_hsmmc_suspend,
	.resume		= omap_hsmmc_resume,
	.prepare	= omap_hsmmc_prepare,
	.complete	= omap_hsmmc_complete,
	.runtime_suspend = omap_hsmmc_runtime_suspend,
	.runtime_resume = omap_hsmmc_runtime_resume,
};

static struct platform_driver omap_hsmmc_driver = {
	.probe		= omap_hsmmc_probe,
	.remove		= omap_hsmmc_remove,
	.driver		= {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &omap_hsmmc_dev_pm_ops,
		.of_match_table = of_match_ptr(omap_mmc_of_match),
	},
};

module_platform_driver(omap_hsmmc_driver);
MODULE_DESCRIPTION("OMAP High Speed Multimedia Card driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
