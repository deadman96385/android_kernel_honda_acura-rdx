/*
 * Definitions of eMMC health data functions (Micron CMD56)
 *
 * Copyright (c) 2015 DENSO INTERNATIONAL AMERICA, INC. ALL RIGHTS RESERVED.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include "denso_health.h"

/*
 * CMD56 Health Report Command Parameter (32 bits)
 * RRRRRRRR 22222222 11111111 CCCCCCCM
 * 0     : Mode:  1=Read, 0=Write
 * 1-7   : Command Index: HEALTH_CMD_*
 * 8-15  : Arg 1 
 * 16-23 : Arg 2
 * 24-31 : Reserved
 */
/*                                                               7654321 */
#define HEALTH_CMD_BAD_BLOCK_COUNTERS               0x08    /* 0b0001000 */ 
#define HEALTH_CMD_BAD_BLOCK_INFO                   0x40    /* 0b1000000 */ /* WIP - Addr out of range for all die/tbl args [0..3] */
#define HEALTH_CMD_TOTAL_BLOCK_TABLES               0x41    /* 0b1000001 */
#define HEALTH_CMD_BLOCK_ERASURES_RETRIEVE          0x42    /* 0b1000010 */
#define HEALTH_CMD_BLOCK_ADDRTYPE_RETRIEVE          0x43    /* 0b1000011 */
#define HEALTH_MODE_READ                            1
#define HEALTH_MODE_WRITE                           0
#define HEALTH_BLOCK_SIZE                           512
#define MIN(x,y) ((x)<(y)?(x):(y))
#define MAX(x,y) ((x)>(y)?(x):(y))

static void bind_mmc_structs(struct mmc_request *mrq, struct mmc_command *cmd, 
                            struct mmc_data *data)
{
    mrq->cmd = cmd;
    /* cmd->mrq = mrq;    assigned in core layer */
    if(data) {
        data->mrq = mrq;
        mrq->data = data;
    }
}

static void init_mmc_structs(struct mmc_request *mrq, struct mmc_command *cmd,
                            struct mmc_data *data)
{
    memset(mrq, 0, sizeof(*mrq));
    memset(cmd, 0, sizeof(*cmd));
    memset(data, 0, sizeof(*data));
    data->blksz = HEALTH_BLOCK_SIZE;
    data->blocks = 1;
    data->flags = MMC_DATA_READ;
    bind_mmc_structs(mrq, cmd, data);
}

static size_t copy_block_erasure_data(u8 *dest, u8 *src, size_t max) 
{
    u8 *p = dest;
    u8 *data_type;
    u16 addr1, addr2, type, count;
    int i;
    
    data_type = src + HEALTH_BLOCK_SIZE;
    for(i = 0; i <= 254; i += 2) {
        addr1 = (src[i] << 8) | src[i + 1];
        addr2 = (data_type[i] << 8) | data_type[i + 1];
        if(addr1 == addr2) {
            count = (src[i + 256] << 8) | src[i + 257];
            type = (data_type[i + 256] << 8) | data_type[i + 257];
            if(!addr1 && !count)
                break;
            p += scnprintf(p, max - (p - dest), "0x%04x %c %u\n", 
                            addr1, type ? 'S' : 'M', count);
        } else { 
            break;
        }
    }

    return (p - dest);
}

static void dump_mmc_data(struct mmc_host *mmc, struct mmc_data *data)
{
    if(data) {
        dev_dbg(mmc_dev(mmc), "mmc_data@%p: timeout_ns=%u, "
                        "timeout_clks=%u, blksz=%u, blocks=%u, "
                        "flags=%u, error=%u, xfered=%u, "
                        "stop=0x%p, mrq=0x%p\n",
                        data, data->timeout_ns, data->timeout_clks, 
                        data->blksz, data->blocks, data->flags, 
                        data->error, data->bytes_xfered, 
                        data->stop, data->mrq);
    }
}

static void dump_mmc_command(struct mmc_host *mmc, struct mmc_command *cmd)
{
    if(cmd) {
        dev_dbg(mmc_dev(mmc), "mmc_command@%p: opcode=0x%x (%u), "
                "arg=0x%x, flags=0x%x, data=0x%p, retries=%u, "
                "error=%u, mrq=0x%p\n",
                cmd, cmd->opcode, cmd->opcode, cmd->arg, 
                cmd->flags, cmd->data, cmd->retries, 
                cmd->error, cmd->mrq);
    }
}

static void dump_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
    if(mrq) {
        dev_dbg(mmc_dev(mmc), 
                "mmc_request@%p: sbc=0x%p, cmd=0x%p, "
                "data=0x%p, stop=0x%p, done=0x%p, host=0x%p\n",
                mrq, mrq->sbc, mrq->cmd, mrq->data, mrq->stop, 
                mrq->done, mrq->host);
        dump_mmc_command(mmc, mrq->cmd);
        dump_mmc_data(mmc, mrq->data);
    }
}

static int exec_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq, int setblklen)
{
    struct mmc_command cmd;
    int rc = 0;

    dev_dbg(mmc_dev(mmc), "+%s()\n", __func__);
    if(setblklen) {
        memset(&cmd, 0, sizeof(cmd));
        cmd.opcode = MMC_SET_BLOCKLEN;
        cmd.arg = 512;
        cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
        cmd.cmd_timeout_ms = 2000;
#ifdef DEBUG
        dump_mmc_command(mmc, &cmd);
#endif
        rc = mmc_wait_for_cmd(mmc, &cmd, 3);
    }
    if(!rc) {
        if(mrq->data)
            mrq->data->bytes_xfered = 0;
        mrq->data->timeout_ns = 3000000000; /* Micron TN-FC-37 note 2 pg. 4 */
#ifdef DEBUG
        dump_mmc_request(mmc, mrq);
#endif
        mmc_wait_for_req(mmc, mrq);
        dev_dbg(mmc_dev(mmc), ".%s() request completed.\n", 
                __func__);
        rc = mrq->cmd->error;
        if(!rc && mrq->data)
            rc =  mrq->data->error;
        if(!rc) {
            cmd.opcode = MMC_SEND_STATUS;
            cmd.arg = 1 << 16; /* RCA bit */
            rc = mmc_wait_for_cmd(mmc, &cmd, 3);
            if(!rc) 
                rc = R1_STATUS(cmd.resp[0]);
            else 
                dev_warn(mmc_dev(mmc), 
                        "!%s() cannot SEND_STATUS\n", 
                        __func__);
        } else {
            dev_warn(mmc_dev(mmc), 
                    "!%s() '%s' .cmd=%d, rc=%d\n", 
                    __func__, mmc_hostname(mmc), 
                    mrq->cmd->opcode, rc);
        }
    } else {
        dev_warn(mmc_dev(mmc), "!%s() could not SET_BLOCKLEN=%u\n",
                 __func__, cmd.arg);
    }
    dev_dbg(mmc_dev(mmc), "-%s() = %d\n", __func__, rc);
    return rc;
}

/**
 * Retrieve the device's bad block counters and store in the
 * given denso_mmc_health.
 *
 * Returns 0 if successful; -ENOMEM if work buffers cannot be allocated, or 
 * -EREMOTEIO if any error occurs while trying to communicate with the device.
 */
int denso_get_bad_block_counters(struct mmc_host *host, struct denso_mmc_health *report)
{
    struct mmc_card *card = host->card;
    int rc = 0;
    struct mmc_request  mrq;
    struct mmc_command  cmd;
    struct mmc_data     data;
    struct scatterlist  sg;
    u8 *buf;
    
    dev_dbg(mmc_dev(host), "+%s()\n", __func__);
    buf = kzalloc(HEALTH_BLOCK_SIZE, GFP_KERNEL);
    if(buf) {
        init_mmc_structs(&mrq, &cmd, &data);
        memset(&sg, 0, sizeof(sg));
        sg_init_one(&sg, buf, HEALTH_BLOCK_SIZE);
        data.sg = &sg;
        data.sg_len = 1;
        mmc_set_data_timeout(&data, card);

        cmd.opcode = MMC_GEN_CMD;
        cmd.arg = (HEALTH_CMD_BAD_BLOCK_COUNTERS << 1) | HEALTH_MODE_READ;
        cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC; 
        mmc_claim_host(host);
        rc = exec_mmc_request(host, &mrq, TRUE);
        mmc_release_host(host);
        if (!rc) {
            report->initial_bad_block_count = (buf[0] << 8) | buf[1];
            report->runtime_bad_block_count = (buf[2] << 8) | buf[3];
            report->remaining_spare_block_count = (buf[4] << 8) | buf[5];
            dev_dbg(mmc_dev(host), ".%s() ibbc=%u, rbbc=%u, rsbc=%u\n", 
                    __func__,
                    report->initial_bad_block_count,
                    report->runtime_bad_block_count,
                    report->remaining_spare_block_count);
        } else {
            dev_warn(mmc_dev(host), "!%s() cannot get data from device (%d)\n", __func__, rc);
            rc = -EREMOTEIO;
        }       
        kfree(buf);
    } else {
        rc = -ENOMEM;
        dev_warn(mmc_dev(host), "!%s() cannot kzalloc\n", __func__);
    }
    dev_dbg(mmc_dev(host), "-%s() = %d\n", __func__, rc);

    return rc;
}

/*
 * Retrieve eMMC block erasure data and write a report of it to  
 * a buffer.
 *
 * The amount of block erasure data (and hence the size of the report) 
 * varies by device.  This function allocates sufficient storage to 
 * contain the results, which will generally be greater than the actual
 * report size.  The address of the allocated storage is stored in dest, 
 * and the amount of memory allocated in allocated.  The actual size of 
 * the report is reflected in copied.
 *
 * Returns 0 if successful; -EFBIG if 
 * the report size exceeds buffer allocation; -EREMOTEIO if 
 * the device's internal table(s) cannot be read; or 
 * -ENOMEM  if buffers cannot be allocated.
 */
int denso_report_block_erasure_data(struct mmc_host *host, char **dest, size_t *allocated, size_t *copied)
{
    struct mmc_card *card = host->card;
    struct mmc_request  mrq;
    struct mmc_command  cmd;
    struct mmc_data     data;
    struct scatterlist  sg, sg_type;
    unsigned int num_tables = 0;
    u8 *buf, *buf_type;
    int rc = 0;
    size_t n;
    
    dev_dbg(mmc_dev(host), "+%s(mmc%d)\n", 
            __func__, host->index);
    *dest = NULL;
    *allocated = 0;
    *copied = 0;
    buf = kzalloc(HEALTH_BLOCK_SIZE * 2, GFP_KERNEL);
    if(buf) {
        buf_type = buf + HEALTH_BLOCK_SIZE;
        init_mmc_structs(&mrq, &cmd, &data);
        memset(&sg, 0, sizeof(sg));
        memset(&sg_type, 0, sizeof(sg_type));
        sg_init_one(&sg, buf, HEALTH_BLOCK_SIZE);
        sg_init_one(&sg_type, buf_type, HEALTH_BLOCK_SIZE);
        data.sg_len = 1;
        data.sg = &sg;
        mmc_set_data_timeout(&data, card);
        /* step 1: get total number of block type tables */
        cmd.opcode = MMC_GEN_CMD;
        cmd.arg = (HEALTH_CMD_TOTAL_BLOCK_TABLES << 1) | HEALTH_MODE_READ;
        cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC; 
        cmd.cmd_timeout_ms = 6000;   /* Micron TN-FC-37 note 2 pg. 4 */
        mmc_claim_host(host);
        rc = exec_mmc_request(host, &mrq, TRUE);
        if(!rc) {
            num_tables = buf[0];
            *allocated = 1024 * num_tables; /* @TODO could fit in 768*num but a bit close */
            *dest = kzalloc(*allocated, GFP_KERNEL);
            dev_dbg(mmc_dev(host), ".%s() num_tables = %u"
                                "alloc=%u, dest=0x%p\n",
                                __func__, num_tables,
                                *allocated, *dest);
            if(*dest) {
                char *p = *dest;
                int i;
                /* step 2A: for each table get block erasure counts */
                for(i = 0; i < num_tables && !rc; i++) {
                    cmd.arg = (i << 8) | (HEALTH_CMD_BLOCK_ERASURES_RETRIEVE << 1) | HEALTH_MODE_READ;
                    data.sg = &sg;
                    rc = exec_mmc_request(host, &mrq, TRUE);
                    if(!rc) {
                        dev_dbg(mmc_dev(host), ".%s() erasure table %u retrieved. bytes xfered=%u\n",
                                __func__, i, data.bytes_xfered);
                        /* step 2B: ...and block types */
                        cmd.arg = (i << 8) | (HEALTH_CMD_BLOCK_ADDRTYPE_RETRIEVE << 1) | HEALTH_MODE_READ;
                        data.sg = &sg_type;
                        rc = exec_mmc_request(host, &mrq, FALSE);
                        if(!rc) {
                            dev_dbg(mmc_dev(host), ".%s() type table %u retrieved. bytes xfered=%u\n",
                                    __func__, i, data.bytes_xfered);
                            /* step 3: write to report */
                            n = copy_block_erasure_data(p, buf, *allocated - *copied);
                            if(n) {
                                *copied += n;
                                p += n;
                                memset(buf, 0, HEALTH_BLOCK_SIZE * 2);
                            } else {
                                rc = -EFBIG;
                                dev_warn(mmc_dev(host), 
                                        "!%s() max %u reached, truncating at %u of %u tables\n",
                                        __func__, *copied, i, num_tables);
                            }
                        } else {
                            rc = -EREMOTEIO;
                            dev_warn(mmc_dev(host), "!%s() cannot retrieve type table %u\n",
                                     __func__, i);
                        }
                    } else {
                        rc = -EREMOTEIO;
                        dev_warn(mmc_dev(host), "!%s() cannot retrieve erasure table %u\n",
                                    __func__, i);
                    }
                }
            } else {
                rc = -ENOMEM;
                dev_warn(mmc_dev(host), "!%s() cannot kzalloc %u bytes for report\n",
                                __func__, *allocated);
                *allocated = 0;
            }
        } else {
            rc = -EREMOTEIO;
            dev_warn(mmc_dev(host), "!%s() cannot retrieve number of block tables\n",
                    __func__);
        }
        mmc_release_host(host);
        kfree(buf);
    } else {
        rc = -ENOMEM;
        dev_dbg(mmc_dev(host), "!%s() cannot kzalloc raw buffer\n", __func__);
    }
    dev_dbg(mmc_dev(host), "-%s(mmc%d, 0x%p, %u, %u) = %d\n", 
            __func__, host->index, *dest, *allocated, *copied, rc);
    return rc;
}

#if 0 /* WIP awaiting Micron support */
/*
 * Return 1 if all chars in src[0..len] == val; 0 otherwise.
 */
static int isall(u8 *src, u8 val, size_t len) {

    size_t i;
    for(i = 0; i < len; i++) 
        if(src[i] != val)
            return 0;
    return 1;
}

static char type_to_char(u8 type)
{
    char s;
    switch(type) {
        case 0x01:
        case 0x10:
            s = 'E';
            break;
        case 0x2:
        case 0x11:
            s = 'P';
            break;
        default:
            s = '?';
            break;
    }

    return s;
}

int denso_log_bad_block_details(struct mmc_host *mmc, u8 die, u8 table)
{
    struct mmc_card *card = mmc->card;
    struct mmc_request  mrq;
    struct mmc_command  cmd;
    struct mmc_data     data;
    struct scatterlist  sg;
    u8 *buf;
    int rc = 0;
    
    dev_dbg(mmc_dev(mmc), "+%s(%u, %u)\n", __func__, die, table);
    buf = kzalloc(HEALTH_BLOCK_SIZE, GFP_KERNEL);
    if(buf) {
        init_mmc_structs(&mrq, &cmd, &data);
        memset(&sg, 0, sizeof(sg));
        sg_init_one(&sg, buf, HEALTH_BLOCK_SIZE);
        data.sg = &sg;
        data.sg_len = 1;
        mmc_set_data_timeout(&data, card);
        cmd.opcode = MMC_GEN_CMD;
        cmd.arg = (table << 16) | (die << 8) | (HEALTH_CMD_BAD_BLOCK_INFO << 1) | HEALTH_MODE_READ;
        cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC; 
        mmc_claim_host(mmc);
        rc = exec_mmc_request(mmc, &mrq, TRUE);
        mmc_release_host(mmc);
        if (!rc) {
            /*
            * [MSB:LSB]
            * [1:0] = block address
            * [3:2] = failed page address record
            * [4] = fail type (01h or 10h = erase, 02h or 11h = pgm)
            * [5] = plane # where bad block located
            * [7:6] = reserved, don't care.
            */
            unsigned int offset = 0;
            unsigned int count = 0;
            int done = 0;
            for(done = isall(buf, 0, 8); !done; done = isall(buf + offset, 0, 8)) {
                u16 blk_addr = (buf[offset + 1] << 8) | buf[offset];
                u16 pg_addr = (buf[offset + 3] << 8) | buf[offset + 2];
                u8 type = buf[offset + 4];
                u8 plane = buf[offset + 5];
                dev_info(mmc_dev(mmc), 
                        ".%s() off=%u die=%u, tbl=%u, blk=%u, pg=%u, fail=%c, p=%u\n",
                        __func__, offset, die, table,
                        blk_addr, pg_addr, type_to_char(type), plane);
                offset = ++count * 8;
            }
            dev_info(mmc_dev(mmc), ".%s(%u, %u) logged %u detail records.\n",
                            __func__, die, table, count);
        } else {
            dev_warn(mmc_dev(mmc), 
                    "!%s() cannot get bad block details for die=%u, table=%u. rc = %d\n",
                     __func__, die, table, rc);
        }
        kfree(buf);
    } else {
        rc = -ENOMEM;
        dev_dbg(mmc_dev(mmc), "!%s() cannot kzalloc\n", __func__);
    }
    dev_dbg(mmc_dev(mmc), "-%s(%u, %u) = %d\n", __func__, die, table, rc);
    return rc;
}
#endif
