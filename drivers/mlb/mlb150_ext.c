/*
 * Copyright (C) 2012-2013 CETITEC GmbH. All Rights Reserved.
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

#include <linux/fcntl.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>

#include "mlb150_ext.h"
#include "mlb150_int.h"

static struct mlb_data *drvdata;

static DEFINE_SPINLOCK(extensions_slock);

static struct mlb150_ext *extensions[][MLB150_MAX_CHANNEL_EXTENSIONS] = {
	[MLB150_CTYPE_CTRL]  = { NULL },
	[MLB150_CTYPE_SYNC]  = { NULL },
	[MLB150_CTYPE_ASYNC] = { NULL },
	[MLB150_CTYPE_ISOC]  = { NULL },
};

const struct dev_max_audio_constraints mlb150_max_sync_dev_constraints[MLB_MAX_SYNC_DEVICES] = {
	{2, 2}, /* sync0: 16-bit stereo */
	{2, 2}, /* sync1: 16-bit stereo */
	{2, 2}, /* sync2: 16-bit stereo */
	{2, 2}, /* sync3: 16-bit stereo */
	{2, 2}, /* sync4: 16-bit stereo */
	{2, 2}, /* sync5: 16-bit stereo */
	{3, 6}  /* sync6: 24-bit 5.1    */
};

/* called in mlb150_probe() (i.e. device init) */
void ext_set_drvdata(struct mlb_data *ext_drvdata)
{
	drvdata = ext_drvdata;
}

static struct mlb150_ext **ext_get_chain(struct mlb150_ext *chain[MLB150_MAX_CHANNEL_EXTENSIONS],
					 enum mlb150_channel_type ctype)
{
	unsigned long flags;

	spin_lock_irqsave(&extensions_slock, flags);
	memcpy(chain, extensions[ctype], sizeof(extensions[0]));
	spin_unlock_irqrestore(&extensions_slock, flags);
	return chain;
}

void ext_start_rx(unsigned int minor, enum mlb150_channel_type ctype, struct mlb150_io_buffers *bufs)
{
	struct mlb150_ext *chain[ARRAY_SIZE(extensions[0])], **extp;

	for (extp = ext_get_chain(chain, ctype); *extp; ++extp)
		if (extp[0]->minor == minor && extp[0]->rx) {
			extp[0]->rx(*extp, bufs);
			/* Move the buffers in the `out` chain to `in`: this
			 * extension decided they are not interesting, but the
			 * buffers may still be interesting to the other
			 * extensions.
			 * The buffers this extension deemed interesting were
			 * processed and freed. */
			list_splice_init(&bufs->out, &bufs->in);

			if (list_empty(&bufs->in))
				break;
		}
}

void ext_start_tx(unsigned int minor, enum mlb150_channel_type ctype)
{
	struct mlb150_ext *chain[ARRAY_SIZE(extensions[0])], **extp;

	for (extp = ext_get_chain(chain, ctype); *extp; ++extp)
		if (extp[0]->minor == minor && extp[0]->tx)
			extp[0]->tx(*extp);
}

void ext_cleanup(unsigned int minor, enum mlb150_channel_type ctype)
{
	struct mlb150_ext *chain[ARRAY_SIZE(extensions[0])], **extp;

	for (extp = ext_get_chain(chain, ctype); *extp; ++extp)
		if (extp[0]->minor == minor && extp[0]->cleanup)
			extp[0]->cleanup(*extp);
}

u32 syncsound_get_num_devices(void)
{
	return number_sync_channels;
}

unsigned int mlb150_ext_get_isoc_channel_count(void)
{
	return number_isoc_channels;
}

int mlb150_ext_register(struct mlb150_ext *ext)
{
	static struct mlb150_ext no_nothing;
	int ret;
	unsigned long flags;
	struct mlb150_ext **chain, **endchain = extensions[ext->ctype] + MLB150_MAX_CHANNEL_EXTENSIONS;

	/*
	 * TODO: wait until the initialization finished and complete
	 * TODO: the registration of extensions
	 */
	if (!drvdata || !drvdata->dev)
		return -ENODEV;
	if (!ext->owner)
		return -EINVAL;

	spin_lock_irqsave(&extensions_slock, flags);

	for (chain = extensions[ext->ctype]; chain < endchain; ++chain)
		if (!*chain) {
			if (ext->minor == -1)
				ext->minor = 0;
			*chain = &no_nothing;
			break;
		}

	spin_unlock_irqrestore(&extensions_slock, flags);

	if (chain == endchain)
		return -EMFILE;

	ext->mode = MLB_CHANNEL_UNDEFINED;
	ext->size = 0; /* TODO: size of the DMA buffer */
	ext->count = 1;

	switch (ext->ctype) {
	case MLB150_CTYPE_CTRL:
		ext->minor += drvdata->minor_ctrl0;
		break;
	case MLB150_CTYPE_ASYNC:
		ext->minor += drvdata->minor_async0;
		break;
	case MLB150_CTYPE_ISOC:
		ext->minor += drvdata->minor_isoc0;
		break;
	case MLB150_CTYPE_SYNC:
		ext->minor += drvdata->minor_sync0;
		break;
	}

	/* See mlb150_ext_set_isoc_params() */
	if (ext->ctype == MLB150_CTYPE_ISOC) {
		ext->count = 0;
		ext->size = 0;
	}

	/* Note that the ->setup is called without extensions_slock */
	ret = ext->setup ? ext->setup(ext, drvdata->dev) : 0;

	if (unlikely(ret)) {
		pr_err("registration: extension %p setup failed: %d\n", ext, ret);

		spin_lock_irqsave(&extensions_slock, flags);

		for (; endchain - chain > 1; ++chain)
			*chain = chain[1];

		*chain = NULL;
		spin_unlock_irqrestore(&extensions_slock, flags);

		return ret;
	}

	spin_lock_irqsave(&extensions_slock, flags);
	*chain = ext;
	spin_unlock_irqrestore(&extensions_slock, flags);
	pr_debug("registered '%s' extension on minor %d (%p)\n",
		 MLB150_CTYPE_CTRL == ext->ctype ? "ctrl" :
		 MLB150_CTYPE_ASYNC == ext->ctype ? "async" :
		 MLB150_CTYPE_ISOC == ext->ctype ? "isoc" :
		 MLB150_CTYPE_SYNC == ext->ctype ? "sync" : "unknown",
		 ext->minor, ext);
	return ret;
}

void mlb150_ext_unregister(struct mlb150_ext *ext)
{
	unsigned long flags;
	struct mlb150_ext **chain = extensions[ext->ctype];
	struct mlb150_ext **endchain = chain + MLB150_MAX_CHANNEL_EXTENSIONS;

	spin_lock_irqsave(&extensions_slock, flags);

	for (; chain < endchain; ++chain)
		if (*chain == ext) {
			for (; endchain - chain > 1; ++chain)
				*chain = chain[1];
			*chain = NULL;
			break;
		}

	spin_unlock_irqrestore(&extensions_slock, flags);

	if (chain == endchain)
		pr_err("registration: extension %p is not registered\n", ext);
}

void mlb150_ext_set_isoc_params(int minor, unsigned blk_size, unsigned num_blks)
{
	unsigned i, found = 0;
	for (i = 0; i < MLB150_MAX_CHANNEL_EXTENSIONS; i++) {
		struct mlb150_ext *ext = extensions[MLB150_CTYPE_ISOC][i];
		if(ext && ext->minor == minor) {
			ext->size  = blk_size;
			ext->count = num_blks;
			found++;
		}
	}
	if (!found)
		pr_err("mlb150_ext_set_isoc_params(): minor %d not found!\n", minor);
	if (found > 1)
		pr_warn("mlb150_ext_set_isoc_params(): found (%u) > 1!", found);
}

void mlb150_ext_free_dmabuf(struct mlb150_ext *ext, struct mlb150_dmabuf *dmab)
{
	mlb150_free_dmabuf(drvdata, ext->minor, dmab);
}

int mlb150_ext_get_tx_buffer(struct mlb150_ext *ext, struct mlb150_io_buffers *buf)
{
	mlb150_io_buffers_init(buf);
	return mlb150_get_tx_buffer_but_last(drvdata, ext->minor, buf);
}

int mlb150_ext_get_tx_buffers(struct mlb150_ext *ext, struct mlb150_io_buffers *bufs)
{
	mlb150_io_buffers_init(bufs);
	return mlb150_get_tx_buffers(drvdata, ext->minor, bufs);
}

int mlb150_ext_put_tx_buffers(struct mlb150_ext *ext, struct mlb150_io_buffers *bufs)
{
	int err = mlb150_put_tx_buffers(drvdata, ext->minor, bufs);

	if (err)
		return err;

	mlb150_io_buffers_init(bufs);
	return 0;
}

int mlb150_ext_open_channel(struct mlb150_ext *ext, enum channelmode mode, u32 addrs)
{
	int ret = mlb150_do_open(drvdata, ext->minor, mode);

	if (!ret) {
		ext->mode = mode;
		ext->addrs = addrs;
	}

	return ret;
}

int mlb150_ext_close_channel(struct mlb150_ext *ext)
{
	enum channelmode mode = ext->mode;

	ext->mode = MLB_CHANNEL_UNDEFINED;
	return mlb150_do_release(drvdata, ext->minor, mode);
}

int mlb150_ext_setup_channel(struct mlb150_ext *ext, unsigned bytes_per_frame,
			     unsigned buf_size)
{
	return mlb150_chan_allocate_dmabufs(drvdata, ext->minor, ext->mode,
					    bytes_per_frame, buf_size);
}

int mlb150_ext_start_channel(struct mlb150_ext *ext)
{
	mlb150_chan_setaddr(drvdata, ext->minor, ext->addrs);
	return mlb150_chan_startup(drvdata, ext->minor, ext->mode);
}

int mlb150_ext_stop_channel(struct mlb150_ext *ext)
{
	return mlb150_chan_shutdown(drvdata, ext->minor, ext->mode);
}
