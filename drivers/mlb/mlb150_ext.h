/*
 * mlb150_ext.h
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

#ifndef _MLB150_EXT_H
#define _MLB150_EXT_H

#include <linux/list.h>
#include "mlb150.h"

#define MLB_MAX_SYNC_DEVICES	7
#define MLB_MAX_ISOC_DEVICES	4
#define MLB_MAX_ASYNC_DEVICES	1

/* 2^FCNT_VALUE = frames per sub-buffer */
#ifdef MLB_USE_MFE
/*
 * With max FCNT value (6) it is not possible to use 5.1 with 24bit samples,
 * because the buffer depth in CDT overflows: it is a 12bit value.
 */
#define FCNT_VALUE 5
#else
#define FCNT_VALUE 0
#endif

/* return the buffer depth for the given bytes-per-frame */
#define SYNC_BUFFER_DEP(bpf) (4 * (1 << FCNT_VALUE) * (bpf))

#define SYNC_BUFFER_DEP_MONO16   SYNC_BUFFER_DEP(2)
#define SYNC_BUFFER_DEP_MONO24   SYNC_BUFFER_DEP(3)
#define SYNC_BUFFER_DEP_STEREO16 SYNC_BUFFER_DEP(4)
#define SYNC_BUFFER_DEP_STEREO24 SYNC_BUFFER_DEP(6)
#define SYNC_BUFFER_DEP_DOLBY16  SYNC_BUFFER_DEP(12)
#define SYNC_BUFFER_DEP_DOLBY24  SYNC_BUFFER_DEP(18)

#define SYNC_MIN_FRAME_SIZE (2) /* mono, 16bit sample */
#define SYNC_DMA_MIN_SIZE       SYNC_BUFFER_DEP(SYNC_MIN_FRAME_SIZE) /* mono, 16bit sample */
#define SYNC_DMA_MAX_SIZE       (0x1fff + 1) /* system memory buffer size in ADT */

static inline unsigned sync_max_dma_size(unsigned bpf)
{
	return (SYNC_DMA_MAX_SIZE / SYNC_BUFFER_DEP(bpf)) *
		SYNC_BUFFER_DEP(bpf);
}

static inline bool valid_sync_buf_size(unsigned buf_size, unsigned bpf)
{
	unsigned dbr_size = SYNC_BUFFER_DEP(bpf);

	return  buf_size >= dbr_size &&
		buf_size <= sync_max_dma_size(bpf) &&
		(buf_size % dbr_size) == 0;
}

#define BUF_RING_NODES 20

enum channelmode {
	MLB_RDONLY,
	MLB_WRONLY,
	MLB_RDWR,
	MLB_CHANNEL_UNDEFINED = -1
};

static inline bool is_reading(enum channelmode mode)
{
	return mode == MLB_RDONLY || mode == MLB_RDWR;
}

static inline bool is_writing(enum channelmode mode)
{
	return mode == MLB_WRONLY || mode == MLB_RDWR;
}

enum mlb150_channel_type {
	MLB150_CTYPE_CTRL,
	MLB150_CTYPE_ASYNC,
	MLB150_CTYPE_ISOC,
	MLB150_CTYPE_SYNC,
};

#define DMABUF_FREE (0) /* this buffer should be on free list */
#define DMABUF_PINGPONG BIT(0) /* 0 - ping buffer, 1 - pong */
#define DMABUF_ERR      BIT(1) /* this buffer encountered a DMA error (TODO) */
#define DMABUF_POISON   BIT(2) /* this buffer is poisonous. Don't eat it */
#define DMABUF_UNDERRUN BIT(3) /* this buffer is to mitigate underrun on sync */

struct mlb150_dmabuf {
	struct list_head head;
	void *virt;
	dma_addr_t phys;
	unsigned long flags;
};
static inline void poison_dmab(struct mlb150_dmabuf *dmab, int poison, unsigned size)
{
	dmab->flags |= DMABUF_POISON;
	memset(dmab->virt, poison, size);
}

struct dev_max_audio_constraints {
	u8 bytes_per_ch;
	u8 num_ch;
};
extern const struct dev_max_audio_constraints
	mlb150_max_sync_dev_constraints[MLB_MAX_SYNC_DEVICES];

u32 syncsound_get_num_devices(void);
unsigned int mlb150_ext_get_isoc_channel_count(void);

#define MLB150_MAX_CHANNEL_EXTENSIONS 16

struct mlb150_io_buffers;
struct device;
struct module;

/**
 * struct mlb150_ext - an extension of mlb150 driver
 *
 * @ctype: must be set for the required type of the channel type
 * @minor: the channel number of the specific channel type
 * @size:  number of bytes in a block of DMA buffer
 * @count: number of blocks of @size bytes in the DMA buffer
 * @mode:  the mode of transfer, if the channel is opened by the extension
 * @setup: called to setup the extension. Can return an error code to
 *	automatically unregister the extension
 * @rx: called with the list of filled buffers after receiving some packets
 * @tx: called after a completed transmission
 * @owner: the module owning the code of callbacks (mandatory).
 * @cleanup: called from mlb_channel_disable to get the borrowed dmabufs back
 *
 * If @minor is -1, the extension is added to the extensions chain of
 * the first channel and the @minor is adjusted (set to 0).
 * The @size and @count fields are setup upon successful
 * registration, before calling @setup callback.
 * The @tx callback can use mlb150_ext_get_tx_buffers to get the
 * free DMA buffers.
 * The @rx callback called with the completed DMA buffer from the interrupt
 * handler!
 */
struct mlb150_ext {
	enum mlb150_channel_type ctype;
	int minor;
	unsigned int size, count;
	enum channelmode mode;
	u32 addrs;

	int (*setup)(struct mlb150_ext *, struct device *);

	void (*rx)(struct mlb150_ext *, struct mlb150_io_buffers *);
	void (*tx)(struct mlb150_ext *);

	struct module *owner;

	void (*cleanup)(struct mlb150_ext *);
};

/**
 * struct mlb150_io_buffers - the lists of io buffers
 *
 * Only the sequential access of the `in` (input) queue is supported,
 * the dmabufs must be either consumed (and returned to mlb150 driver with
 * mlb150_ext_free_dmabuf) or put in the `out` queue (with
 * mlb150_ext_put_dmabuf)
 */
struct mlb150_io_buffers {
	struct list_head in;
	struct list_head out;
};

static inline
void mlb150_io_buffers_init(struct mlb150_io_buffers *bufs)
{
	INIT_LIST_HEAD(&bufs->in);
	INIT_LIST_HEAD(&bufs->out);
}

/**
 * mlb150_ext_get_dmabuf - get a DMA buffer from the `in` queue
 *
 * The `in` queue is the queue of ready buffers for rx handlers, and the queue
 * of empty buffers when requesting DMA buffers for transmission (tx).
 */
static inline struct mlb150_dmabuf *mlb150_ext_get_dmabuf(struct mlb150_io_buffers *bufs)
{
	struct mlb150_dmabuf *dmab;

	if (list_empty(&bufs->in))
		return NULL;

	dmab = list_first_entry(&bufs->in, struct mlb150_dmabuf, head);
	list_del(&dmab->head);
	return dmab;
}

static inline void mlb150_ext_put_dmabuf(struct mlb150_io_buffers *bufs, struct mlb150_dmabuf *dmab)
{
	list_add_tail(&dmab->head, &bufs->out);
}

void mlb150_ext_set_isoc_params(int minor, unsigned blk_size, unsigned num_blks);

/**
 * mlb150_ext_put_dmabuf - put the used DMA buffer in the `out` queue
 *
 * The `out` queue makes the DMA buffers available for read syscall and for
 * the other extensions when receiving (rx handler) or schedules the buffers
 * for transmission (tx).
 */
void mlb150_ext_put_dmabuf(struct mlb150_io_buffers *, struct mlb150_dmabuf *);

/**
 * mlb150_ext_free_dmabuf - return the DMA buffer to mlb150 driver
 */
void mlb150_ext_free_dmabuf(struct mlb150_ext *, struct mlb150_dmabuf *);

/**
 * mlb150_ext_register - registers an extension interface with mlb150 driver
 *
 * Returns an error code. Might call the @setup callback if the mlb150 driver
 * is initialized far enough.
 *
 * Will return EMFILE if there is already MLB150_MAX_CHANNEL_EXTENSIONS
 * extensions registered for the channel type
 */
int mlb150_ext_register(struct mlb150_ext *);
void mlb150_ext_unregister(struct mlb150_ext *);

/**
 * mlb150_ext_get_tx_buffers - retrieve free DMA buffers
 *
 * @bufs: the list of DMA buffers; initialized by the function
 *
 * Shall return -EAGAIN if there is no free DMA buffers.
 *
 * Use the mlb150_ext_get_dmabuf helper to get a DMA buffer,
 * mlb150_ext_put_dmabuf to schedule the buffer for TX, and
 * mlb150_ext_free_dmabuf to return the buffer, if it was not used.
 *
 * This function does not wait.
 */
int mlb150_ext_get_tx_buffers(struct mlb150_ext *, struct mlb150_io_buffers *);
/**
 * mlb150_ext_get_tx_buffer - retrieve a free DMA buffer
 *
 * @bufs: the list of DMA buffers; initialized by the function
 *
 * Shall return -EAGAIN if there is no free DMA buffers.
 *
 * Same as mlb150_ext_get_tx_buffers, but never retrieves more than one
 * buffer and never takes the last free buffer.
 *
 * This function does not wait.
 */
int mlb150_ext_get_tx_buffer(struct mlb150_ext *, struct mlb150_io_buffers *);

/**
 * mlb150_ext_put_tx_buffers - schedules all buffers in the `out` queue for
 *                             transmission
 *
 * The buffers left in the `in` queue will be returned to the list
 * of empty buffers automatically.
 *
 * Will fail with -ESHUTDOWN if the channel is not started yet, and no buffers
 * will be transferred to the output queue of the channel.
 *
 * This function does not wait.
 */
int mlb150_ext_put_tx_buffers(struct mlb150_ext *, struct mlb150_io_buffers *);

/**
 * mlb150_ext_open_channel - reserve an MLB channel
 *
 * @mode: MLB_RDONLY or MLB_WRONLY
 * @addrs: logical MLB channel number
 */
int mlb150_ext_open_channel(struct mlb150_ext *, enum channelmode mode, u32 addrs);

/**
 * mlb150_ext_setup_channel - prepare the DMA buffers and configure the MLB channel
 *
 * @bytes_per_frame: number of bytes per audio frame for synchronous channels
 * @buf_size: size of DMA buffer for synchronous channels
 */
int mlb150_ext_setup_channel(struct mlb150_ext *ext,
			     unsigned bytes_per_frame,
			     unsigned buf_size);

/**
 * mlb150_ext_start_channel - start data transmission on the channel
 */
int mlb150_ext_start_channel(struct mlb150_ext *);

/**
 * mlb150_ext_stop_channel - stop data transmission on the channel
 */
int mlb150_ext_stop_channel(struct mlb150_ext *);

/**
 * mlb150_ext_close_channel - stop transfers on the channel
 *
 * Also frees the allocated DMA buffers.
 */
int mlb150_ext_close_channel(struct mlb150_ext *);

#endif /* _MLB150_EXT_H */
