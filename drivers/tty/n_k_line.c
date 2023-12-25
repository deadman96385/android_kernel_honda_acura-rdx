/* generic K_LINE line discipline for Linux
 *
 * Written by Paul Fulghum paulkf@microgate.com
 * for Microgate Corporation
 *
 * Microgate and SyncLink are registered trademarks of Microgate Corporation
 *
 * Adapted from ppp.c, written by Michael Callahan <callahan@maths.ox.ac.uk>,
 *	Al Longyear <longyear@netcom.com>,
 *	Paul Mackerras <Paul.Mackerras@cs.anu.edu.au>
 *
 * Original release 01/11/99
 *
 * This code is released under the GNU General Public License (GPL)
 *
 * This module implements the tty line discipline N_K_LINE for use with
 * tty device drivers that support bit-synchronous K_LINE communications.
 *
 * All K_LINE data is frame oriented which means:
 *
 * 1. tty write calls represent one complete transmit frame of data
 *    The device driver should accept the complete frame or none of
 *    the frame (busy) in the write method. Each write call should have
 *    a byte count in the range of 2-65535 bytes (2 is min K_LINE frame
 *    with 1 addr byte and 1 ctrl byte). The max byte count of 65535
 *    should include any crc bytes required. For example, when using
 *    CCITT CRC32, 4 crc bytes are required, so the maximum size frame
 *    the application may transmit is limited to 65531 bytes. For CCITT
 *    CRC16, the maximum application frame size would be 65533.
 *
 *
 * 2. receive callbacks from the device driver represents
 *    one received frame. The device driver should bypass
 *    the tty flip buffer and call the line discipline receive
 *    callback directly to avoid fragmenting or concatenating
 *    multiple frames into a single receive callback.
 *
 *    The K_LINE line discipline queues the receive frames in separate
 *    buffers so complete receive frames can be returned by the
 *    tty read calls.
 *
 * 3. tty read calls returns an entire frame of data or nothing.
 *
 * 4. all send and receive data is considered raw. No processing
 *    or translation is performed by the line discipline, regardless
 *    of the tty flags
 *
 * 5. When line discipline is queried for the amount of receive
 *    data available (FIOC), 0 is returned if no data available,
 *    otherwise the count of the next available frame is returned.
 *    (instead of the sum of all received frame counts).
 *
 * These conventions allow the standard tty programming interface
 * to be used for synchronous K_LINE applications when used with
 * this line discipline (or another line discipline that is frame
 * oriented such as N_PPP).
 *
 * The SyncLink driver (synclink.c) implements both asynchronous
 * (using standard line discipline N_TTY) and synchronous K_LINE
 * (using N_K_LINE) communications, with the latter using the above
 * conventions.
 *
 * This implementation is very basic and does not maintain
 * any statistics. The main point is to enforce the raw data
 * and frame orientation of K_LINE communications.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define K_LINE_MAGIC 0x4b4c

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>

#undef VERSION
#define VERSION(major,minor,patch) (((((major)<<8)+(minor))<<8)+(patch))

#include <linux/poll.h>
#include <linux/in.h>
#include <linux/ioctl.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/string.h>	/* used in new tty drivers */
#include <linux/signal.h>	/* used in new tty drivers */
#include <linux/if.h>
#include <linux/bitops.h>

#include <asm/termios.h>
#include <asm/uaccess.h>

/*
 * Buffers for individual K_LINE frames
 */
#define MAX_K_LINE_FRAME_SIZE 65535
#define DEFAULT_RX_BUF_COUNT 10
#define MAX_RX_BUF_COUNT 60
#define DEFAULT_TX_BUF_COUNT 3
#define MOD_NAME "n_k_line.c"


struct n_k_line_buf {
	struct n_k_line_buf *link;
	int		  count;
	u32 	  timestamp_sec;
	u32 	  timestamp_nsec;
	char		  buf[1];
};

struct n_k_line_frame_cell {
	char		value;
	u32 	  timestamp_sec;
	u32 	  timestamp_nsec;
};

#define	N_K_LINE_BUF_SIZE	(sizeof(struct n_k_line_buf) + maxframe)
#define	N_K_LINE_FRAME_SIZE	1024

struct n_k_line_buf_list {
	struct n_k_line_buf *head;
	struct n_k_line_buf *tail;
	int		  count;
	spinlock_t	  spinlock;
};

#define N_K_LINE_START		0x00
#define N_K_LINE_FORMAT		0x80
#define N_K_LINE_LEN_MASK	0x3F
#define N_K_LINE_FMT_MASK	0xC0
#define N_K_LINE_DEVICE_ID	0x81
#define N_K_LINE_TESTER_ID	0xF6

#define N_K_LINE_FRAME_STATE_INIT		0
#define N_K_LINE_FRAME_STATE_FORMAT		1
#define N_K_LINE_FRAME_STATE_TARGET		2
#define N_K_LINE_FRAME_STATE_SOURCE		3
#define N_K_LINE_FRAME_STATE_DATA		4
#define N_K_LINE_FRAME_STATE_CHKSUM		5

#define LET_STATE_WAKE				0
#define LET_STATE_STAR_COM			2
#define LET_STATE_STOP_COM			3

/**
 * struct n_k_line - per device instance data structure
 * @magic - magic value for structure
 * @flags - miscellaneous control flags
 * @tty - ptr to TTY structure
 * @backup_tty - TTY to use if tty gets closed
 * @tbusy - reentrancy flag for tx wakeup code
 * @woke_up - FIXME: describe this field
 * @tbuf - currently transmitting tx buffer
 * @tx_buf_list - list of pending transmit frame buffers
 * @rx_buf_list - list of received frame buffers
 * @tx_free_buf_list - list unused transmit frame buffers
 * @rx_free_buf_list - list unused received frame buffers
 */
struct n_k_line {
	int			magic;
	__u32			flags;
	struct tty_struct	*tty;
	struct tty_struct	*backup_tty;
	int			tbusy;
	int			woke_up;

	int			let_state;
	int			rx_frame_state;
	int			rx_frame_counter;
	__u8			rx_frame_fmt_len;
	__u8			rx_frame_sid;
	/*ktime_t 		timestamp_rx;*/
	unsigned long 		timestamp_rx;

	struct workqueue_struct		*workq_stop;
	struct workqueue_struct		*workq_start;
	struct workqueue_struct		*workq_send;
	struct delayed_work		start_comm_resp_work;
	struct delayed_work		stop_comm_resp_work;
	struct delayed_work		send_work;
	struct n_k_line_buf_list	rx_frame_buf_list;

	struct n_k_line_buf		*tbuf;
	struct n_k_line_buf_list	tx_buf_list;
	struct n_k_line_buf_list	rx_buf_list;
	struct n_k_line_buf_list	tx_free_buf_list;
	struct n_k_line_buf_list	rx_free_buf_list;
};

/*
 * K_LINE buffer list manipulation functions
 */
static void n_k_line_buf_list_init(struct n_k_line_buf_list *list);
static void n_k_line_buf_put(struct n_k_line_buf_list *list,
			   struct n_k_line_buf *buf);
static struct n_k_line_buf *n_k_line_buf_get(struct n_k_line_buf_list *list);

/* Local functions */

static struct n_k_line *n_k_line_alloc (void);

/* debug level can be set by insmod for debugging purposes */
#define DEBUG_LEVEL_INFO	0b000000100000000u
#define DEBUG_LEVEL_WARNING	0b000000010000000u
#define DEBUG_LEVEL_STATE_ENG	0b000000001000000u
#define DEBUG_LEVEL_COMM_RX	0b000000000100000u
#define DEBUG_LEVEL_FRAME_RX	0b000000000010000u
#define DEBUG_LEVEL_COMM	0b000000000001000u
#define DEBUG_LEVEL_FRAME	0b000000000000100u
#define DEBUG_LEVEL_TIME	0b000000000000010u
#define DEBUG_LEVEL_ERROR	0b000000000000001u
static int debuglevel = DEBUG_LEVEL_ERROR;

/* max frame size for memory allocations */
static int maxframe = 4096;

/* TTY callbacks */

static ssize_t n_k_line_tty_read(struct tty_struct *tty, struct file *file,
			   __u8 __user *buf, size_t nr);
static ssize_t n_k_line_tty_write(struct tty_struct *tty, struct file *file,
			    const unsigned char *buf, size_t nr);
static int n_k_line_tty_ioctl(struct tty_struct *tty, struct file *file,
			    unsigned int cmd, unsigned long arg);
static unsigned int n_k_line_tty_poll(struct tty_struct *tty, struct file *filp,
				    poll_table *wait);
static int n_k_line_tty_open(struct tty_struct *tty);
static void n_k_line_tty_close(struct tty_struct *tty);
static void n_k_line_tty_receive(struct tty_struct *tty, const __u8 *cp,
			       char *fp, int count);
static void n_k_line_tty_wakeup(struct tty_struct *tty);

static void n_k_line_send_frames(struct n_k_line *n_k_line, struct tty_struct *tty);

static int show_buffer(const char *mod_name, const char *fcn_name, const char *msg, int l_num, const char *in_buf, int number);

#define bset(p,b)	((p)[(b) >> 5] |= (1 << ((b) & 0x1f)))

#define tty2n_k_line(tty)	((struct n_k_line *) ((tty)->disc_data))
#define n_k_line2tty(n_k_line)	((n_k_line)->tty)

static void flush_rx_queue(struct tty_struct *tty)
{
	struct n_k_line *n_k_line = tty2n_k_line(tty);
	struct n_k_line_buf *buf;

	if (debuglevel & DEBUG_LEVEL_INFO)
		printk("%s(%d) flush_rx_queue\n", MOD_NAME, __LINE__);

	while ((buf = n_k_line_buf_get(&n_k_line->rx_buf_list)))
		n_k_line_buf_put(&n_k_line->rx_free_buf_list, buf);

	while ((buf = n_k_line_buf_get(&n_k_line->rx_frame_buf_list)))
		n_k_line_buf_put(&n_k_line->rx_free_buf_list, buf);

	n_k_line->rx_frame_state = N_K_LINE_FRAME_STATE_INIT;
	n_k_line->let_state = LET_STATE_WAKE;
	n_k_line->rx_frame_counter = 0;
}

static void flush_tx_queue(struct tty_struct *tty)
{
	struct n_k_line *n_k_line = tty2n_k_line(tty);
	struct n_k_line_buf *buf;
	unsigned long flags;

	if (debuglevel & DEBUG_LEVEL_INFO)
		printk("%s(%d) flush_tx_queue\n", MOD_NAME, __LINE__);


	while ((buf = n_k_line_buf_get(&n_k_line->tx_buf_list)))
		n_k_line_buf_put(&n_k_line->tx_free_buf_list, buf);

 	spin_lock_irqsave(&n_k_line->tx_buf_list.spinlock, flags);
	if (n_k_line->tbuf) {
		n_k_line_buf_put(&n_k_line->tx_free_buf_list, n_k_line->tbuf);
		n_k_line->tbuf = NULL;
	}
	spin_unlock_irqrestore(&n_k_line->tx_buf_list.spinlock, flags);
}

static void n_kline_send_work(struct work_struct *work)
{
	register struct n_k_line *n_k_line;
	struct tty_struct *tty;
	//struct n_k_line_buf *tbuf;

	if (debuglevel & DEBUG_LEVEL_INFO)
		printk("%s(%d)send() called\n", MOD_NAME, __LINE__);

	n_k_line = container_of(work, struct n_k_line, send_work.work);
	
	/* Verify pointers */
	if (!n_k_line){
		if (debuglevel & DEBUG_LEVEL_ERROR)
			printk("%s(%d)start_comm_resp() !n_k_line\n", MOD_NAME, __LINE__);
		return;
	}
	
	tty = n_k_line->tty;
	n_k_line_send_frames(n_k_line,n_k_line->tty);

	if (debuglevel & DEBUG_LEVEL_INFO)
		printk("%s(%d)send() exit\n", MOD_NAME, __LINE__);

}

static void n_kline_start_comm_resp_work(struct work_struct *work)
{
	register struct n_k_line *n_k_line;
	struct tty_struct *tty;
	struct n_k_line_buf *tbuf;
	char values[200] = {0x83, 0xF6, 0x81, 0xC1, 0xE9, 0x8F, 0x33, 0x00};
	int count = 7;

	if (debuglevel & DEBUG_LEVEL_INFO)
		printk("%s(%d)start_comm_resp() called\n", MOD_NAME, __LINE__);

	n_k_line = container_of(work, struct n_k_line, start_comm_resp_work.work);
	
	/* Verify pointers */
	if (!n_k_line){
		if (debuglevel & DEBUG_LEVEL_ERROR)
			printk("%s(%d)start_comm_resp() !n_k_line\n", MOD_NAME, __LINE__);
		return;
	}
	
	tty = n_k_line->tty;

	for (;;) {

		tbuf = n_k_line_buf_get(&n_k_line->tx_free_buf_list);
		if (tbuf){
			if (debuglevel & DEBUG_LEVEL_WARNING)
				printk("%s(%d)start_comm_resp() waiting for buffer\n", MOD_NAME, __LINE__);
			break;
		}
	}
	memcpy(tbuf->buf, values, count);
	tbuf->count = count;

	n_k_line_buf_put(&n_k_line->tx_buf_list,tbuf);
	n_k_line_send_frames(n_k_line,n_k_line->tty);

	if (debuglevel & DEBUG_LEVEL_FRAME)
		show_buffer(MOD_NAME, "TXA FULL", "FRAME", __LINE__, tbuf->buf, tbuf->count);
}

static void n_kline_stop_comm_resp_work(struct work_struct *work)
{
	register struct n_k_line *n_k_line;
	struct tty_struct *tty;
	struct n_k_line_buf *tbuf;
	char values[200] = {0x81, 0xF6, 0x81, 0xC2, 0xBA, 0x00};
	int count = 5;

	if (debuglevel & DEBUG_LEVEL_INFO)
		printk("%s(%d)stop_comm_resp() called\n", MOD_NAME, __LINE__);

	n_k_line = container_of(work, struct n_k_line, stop_comm_resp_work.work);

	/* Verify pointers */
	if (!n_k_line){
		if (debuglevel & DEBUG_LEVEL_ERROR)
			printk("%s(%d)stop_comm_resp() !n_k_line\n", MOD_NAME, __LINE__);
		return;
	}

	tty = n_k_line->tty;

	for (;;) {

		tbuf = n_k_line_buf_get(&n_k_line->tx_free_buf_list);
		if (tbuf){
			if (debuglevel & DEBUG_LEVEL_WARNING)
				printk("%s(%d)stop_comm_resp() waiting for buffer\n", MOD_NAME, __LINE__);
			break;
		}
	}
	memcpy(tbuf->buf, values, count);
	tbuf->count = count;

	n_k_line_buf_put(&n_k_line->tx_buf_list,tbuf);
	n_k_line_send_frames(n_k_line,n_k_line->tty);

	if (debuglevel & DEBUG_LEVEL_FRAME)
		show_buffer(MOD_NAME, "TXA FULL", "FRAME", __LINE__, tbuf->buf, tbuf->count);
}

static struct tty_ldisc_ops n_k_line_ldisc = {
	.owner		= THIS_MODULE,
	.magic		= TTY_LDISC_MAGIC,
	.name		= "k_line",
	.open		= n_k_line_tty_open,
	.close		= n_k_line_tty_close,
	.read		= n_k_line_tty_read,
	.write		= n_k_line_tty_write,
	.ioctl		= n_k_line_tty_ioctl,
	.poll		= n_k_line_tty_poll,
	.receive_buf	= n_k_line_tty_receive,
	.write_wakeup	= n_k_line_tty_wakeup,
	.flush_buffer   = flush_rx_queue,
};

/**
 * n_k_line_release - release an n_k_line per device line discipline info structure
 * @n_k_line - per device line discipline info structure
 */
static void n_k_line_release(struct n_k_line *n_k_line)
{
	struct tty_struct *tty = n_k_line2tty (n_k_line);
	struct n_k_line_buf *buf;

	if (debuglevel & DEBUG_LEVEL_INFO)
		printk("%s(%d)n_k_line_release() called\n",MOD_NAME,__LINE__);

	/* Ensure that the n_k_lined process is not hanging on select()/poll() */
	wake_up_interruptible (&tty->read_wait);
	wake_up_interruptible (&tty->write_wait);

	if (tty->disc_data == n_k_line)
		tty->disc_data = NULL;	/* Break the tty->n_k_line link */

	/* Release transmit and receive buffers */
	for(;;) {
		buf = n_k_line_buf_get(&n_k_line->rx_free_buf_list);
		if (buf) {
			kfree(buf);
		} else
			break;
	}
	for(;;) {
		buf = n_k_line_buf_get(&n_k_line->tx_free_buf_list);
		if (buf) {
			kfree(buf);
		} else
			break;
	}
	for(;;) {
		buf = n_k_line_buf_get(&n_k_line->rx_buf_list);
		if (buf) {
			kfree(buf);
		} else
			break;
	}
	for(;;) {
		buf = n_k_line_buf_get(&n_k_line->tx_buf_list);
		if (buf) {
			kfree(buf);
		} else
			break;
	}
	for(;;) {
		buf = n_k_line_buf_get(&n_k_line->rx_frame_buf_list);
		if (buf) {
			kfree(buf);
		} else
			break;
	}
	kfree(n_k_line->tbuf);
	kfree(n_k_line);

}	/* end of n_k_line_release() */

/**
 * n_k_line_tty_close - line discipline close
 * @tty - pointer to tty info structure
 *
 * Called when the line discipline is changed to something
 * else, the tty is closed, or the tty detects a hangup.
 */
static void n_k_line_tty_close(struct tty_struct *tty)
{
	struct n_k_line *n_k_line = tty2n_k_line (tty);

	if (debuglevel & DEBUG_LEVEL_INFO)
		printk("%s(%d)n_k_line_tty_close() called\n",MOD_NAME,__LINE__);

	/*flush_work(&n_k_line->start_comm_resp_work);*/
	/*flush_work(&n_k_line->stop_comm_resp_work);*/
	cancel_delayed_work_sync(&n_k_line->stop_comm_resp_work);
	cancel_delayed_work_sync(&n_k_line->start_comm_resp_work);
	cancel_delayed_work_sync(&n_k_line->send_work);

	if (n_k_line != NULL) {
		if (n_k_line->magic != K_LINE_MAGIC) {
			printk (KERN_WARNING"n_k_line: trying to close unopened tty!\n");
			return;
		}
#if defined(TTY_NO_WRITE_SPLIT)
		clear_bit(TTY_NO_WRITE_SPLIT,&tty->flags);
#endif
		tty->disc_data = NULL;
		if (tty == n_k_line->backup_tty)
			n_k_line->backup_tty = NULL;
		if (tty != n_k_line->tty)
			return;
		if (n_k_line->backup_tty) {
			n_k_line->tty = n_k_line->backup_tty;
		} else {
			n_k_line_release (n_k_line);
		}
	}

	if (debuglevel & DEBUG_LEVEL_INFO)
		printk("%s(%d)n_k_line_tty_close() success\n",MOD_NAME,__LINE__);

}	/* end of n_k_line_tty_close() */

/**
 * n_k_line_tty_open - called when line discipline changed to n_k_line
 * @tty - pointer to tty info structure
 *
 * Returns 0 if success, otherwise error code
 */
static int n_k_line_tty_open (struct tty_struct *tty)
{
	struct n_k_line *n_k_line = tty2n_k_line (tty);

	if (debuglevel & DEBUG_LEVEL_INFO)
		printk("%s(%d)n_k_line_tty_open() called (device=%s)\n",
		MOD_NAME,__LINE__,
		tty->name);

	/* There should not be an existing table for this slot. */
	if (n_k_line) {
		printk (KERN_ERR"n_k_line_tty_open:tty already associated!\n" );
		return -EEXIST;
	}

	n_k_line = n_k_line_alloc();
	if (!n_k_line) {
		printk (KERN_ERR "n_k_line_alloc failed\n");
		return -ENFILE;
	}

	tty->disc_data = n_k_line;
	n_k_line->tty    = tty;
	tty->receive_room = 65536;

#if defined(TTY_NO_WRITE_SPLIT)
	/* change tty_io write() to not split large writes into 8K chunks */
	set_bit(TTY_NO_WRITE_SPLIT,&tty->flags);
#endif

	/* flush receive data from driver */
	tty_driver_flush_buffer(tty);

	/*INIT_WORK(&n_k_line->start_comm_resp_work, n_kline_start_comm_resp_work);
	INIT_WORK(&n_k_line->stop_comm_resp_work, n_kline_stop_comm_resp_work);*/

	n_k_line->workq_start = create_singlethread_workqueue("n_kline_start_comm_resp_work");
	n_k_line->workq_stop = create_singlethread_workqueue("n_kline_stop_comm_resp_work");
	n_k_line->workq_send = create_singlethread_workqueue("n_kline_send_work");

	INIT_DELAYED_WORK(&n_k_line->start_comm_resp_work, n_kline_start_comm_resp_work);
	INIT_DELAYED_WORK(&n_k_line->stop_comm_resp_work, n_kline_stop_comm_resp_work);
	INIT_DELAYED_WORK(&n_k_line->send_work, n_kline_send_work);

	n_k_line->rx_frame_state = N_K_LINE_FRAME_STATE_INIT;
	n_k_line->let_state = LET_STATE_WAKE;
	n_k_line->rx_frame_counter = 0;

	if (debuglevel & DEBUG_LEVEL_INFO)
		printk("%s(%d)n_k_line_tty_open() success\n",MOD_NAME,__LINE__);

	return 0;

}	/* end of n_tty_k_line_open() */

/**
 * n_k_line_send_frames - send frames on pending send buffer list
 * @n_k_line - pointer to ldisc instance data
 * @tty - pointer to tty instance data
 *
 * Send frames on pending send buffer list until the driver does not accept a
 * frame (busy) this function is called after adding a frame to the send buffer
 * list and by the tty wakeup callback.
 */
static void n_k_line_send_frames(struct n_k_line *n_k_line, struct tty_struct *tty)
{
	register int actual;
	unsigned long flags;
	struct n_k_line_buf *tbuf;

	if (debuglevel & DEBUG_LEVEL_INFO)
		printk("%s(%d)n_k_line_send_frames() called\n",MOD_NAME,__LINE__);
 check_again:

 	spin_lock_irqsave(&n_k_line->tx_buf_list.spinlock, flags);
	if (n_k_line->tbusy) {
		n_k_line->woke_up = 1;
 		spin_unlock_irqrestore(&n_k_line->tx_buf_list.spinlock, flags);
		return;
	}
	n_k_line->tbusy = 1;
	n_k_line->woke_up = 0;
	spin_unlock_irqrestore(&n_k_line->tx_buf_list.spinlock, flags);

	/* get current transmit buffer or get new transmit */
	/* buffer from list of pending transmit buffers */

	tbuf = n_k_line->tbuf;
	if (!tbuf)
		tbuf = n_k_line_buf_get(&n_k_line->tx_buf_list);

	while (tbuf) {
		if (debuglevel & DEBUG_LEVEL_INFO)
			printk("%s(%d)sending frame %p, count=%d\n",
				MOD_NAME,__LINE__,tbuf,tbuf->count);

		/* Send the next block of data to device */
		set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
		actual = tty->ops->write(tty, tbuf->buf, tbuf->count);

		/* rollback was possible and has been done */
		if (actual == -ERESTARTSYS) {
			n_k_line->tbuf = tbuf;
			break;
		}
		/* if transmit error, throw frame away by */
		/* pretending it was accepted by driver */
		if (actual < 0)
			actual = tbuf->count;

		if (actual == tbuf->count) {
			if (debuglevel & DEBUG_LEVEL_INFO)
				printk("%s(%d)frame %p completed\n",
					MOD_NAME,__LINE__,tbuf);

			/* free current transmit buffer */
			n_k_line_buf_put(&n_k_line->tx_free_buf_list, tbuf);

			/* this tx buffer is done */
			n_k_line->tbuf = NULL;

			/* wait up sleeping writers */
			wake_up_interruptible(&tty->write_wait);

			/* get next pending transmit buffer */
			tbuf = n_k_line_buf_get(&n_k_line->tx_buf_list);
		} else {
			if (debuglevel & DEBUG_LEVEL_INFO)
				printk("%s(%d)frame %p pending\n",
					MOD_NAME,__LINE__,tbuf);

			/* buffer not accepted by driver */
			/* set this buffer as pending buffer */
			n_k_line->tbuf = tbuf;
			break;
		}
	}

	if (!tbuf)
		clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);

	/* Clear the re-entry flag */
	spin_lock_irqsave(&n_k_line->tx_buf_list.spinlock, flags);
	n_k_line->tbusy = 0;
	spin_unlock_irqrestore(&n_k_line->tx_buf_list.spinlock, flags);

        if (n_k_line->woke_up)
	  goto check_again;

	if (debuglevel & DEBUG_LEVEL_INFO)
		printk("%s(%d)n_k_line_send_frames() exit\n",MOD_NAME,__LINE__);

}	/* end of n_k_line_send_frames() */

/**
 * n_k_line_tty_wakeup - Callback for transmit wakeup
 * @tty	- pointer to associated tty instance data
 *
 * Called when low level device driver can accept more send data.
 */
static void n_k_line_tty_wakeup(struct tty_struct *tty)
{
	struct n_k_line *n_k_line = tty2n_k_line(tty);

	if (debuglevel & DEBUG_LEVEL_INFO)
		printk("%s(%d)n_k_line_tty_wakeup() called\n",MOD_NAME,__LINE__);

	if (!n_k_line)
		return;

	if (tty != n_k_line->tty) {
		clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
		return;
	}

	n_k_line_send_frames (n_k_line, tty);

}	/* end of n_k_line_tty_wakeup() */


static int show_buffer(const char *mod_name, const char *fcn_name, const char *msg, int l_num, const char *in_buf, int number){
	char out_buf[200];
	int cnt;
	for (cnt = 0; cnt < number; cnt++){
		out_buf[(cnt * 5) + 0] = ' ';
		out_buf[(cnt * 5) + 1] = '0';
		out_buf[(cnt * 5) + 2] = 'x';
		out_buf[(cnt * 5) + 3] = (((in_buf[cnt] & 0xF0) >> 4) > 9)?
					(((in_buf[cnt] & 0xF0) >> 4) + 0x37):
					(((in_buf[cnt] & 0xF0) >> 4) + 0x30);
		out_buf[(cnt * 5) + 4] = ((in_buf[cnt] & 0x0F) > 9)?
					((in_buf[cnt] & 0x0F) + 0x37):
					((in_buf[cnt] & 0x0F) + 0x30);
		out_buf[(cnt * 5) + 5] = 0;
	}
	printk("%s(%d) %s %s count=%d DATA:%s\n",
		mod_name, l_num, fcn_name, msg, number, out_buf);

	return 0;
}

/**
 * n_k_line_tty_receive - Called by tty driver when receive data is available
 * @tty	- pointer to tty instance data
 * @data - pointer to received data
 * @flags - pointer to flags for data
 * @count - count of received data in bytes
 *
 * Called by tty low level driver when receive data is available. Data is
 * interpreted as one K_LINE frame.
 */
static void n_k_line_tty_receive(struct tty_struct *tty, const __u8 *data,
			       char *flags, int count)
{
	register struct n_k_line *n_k_line = tty2n_k_line (tty);
	register struct n_k_line_buf *buf;
	/*ktime_t timestamp_diff;
	unsigned long timestamp_diff;*/
	unsigned long timestamp_temp;
	int byte_cnt = 0, end_pos = 0;/*, frm_count = 0;*/
	int data_pos = 0;

	/*n_k_line->timestamp_rx = ktime_get_real();*/
	timestamp_temp = jiffies;
	
	if (debuglevel & DEBUG_LEVEL_INFO){
		printk("%s(%d)n_k_line_tty_receive() called count=%d\n",
			MOD_NAME,__LINE__, count);
	}

	/* This can happen if stuff comes in on the backup tty */
	if (!n_k_line || tty != n_k_line->tty){
		if (debuglevel & DEBUG_LEVEL_ERROR)
			printk("%s(%d) Invalid struct pointer\n",
				MOD_NAME,__LINE__);
		return;
	}
	
	n_k_line->timestamp_rx = jiffies;

	/* verify line is using K_LINE discipline */
	if (n_k_line->magic != K_LINE_MAGIC) {
		if (debuglevel & DEBUG_LEVEL_ERROR)
			printk("%s(%d) line not using K_LINE discipline\n",
				MOD_NAME,__LINE__);
		return;
	}

	if (debuglevel & DEBUG_LEVEL_COMM)
		show_buffer(MOD_NAME, "RX", "", __LINE__, data, count);

	if (count>maxframe) {
		if (debuglevel & DEBUG_LEVEL_ERROR)
			printk("%s(%d) rx count>maxframesize, data discarded\n",
			       MOD_NAME,__LINE__);
		return;
	}

	for(byte_cnt = 0;byte_cnt < count;byte_cnt++){

		/* Looking for initial state */
		if(n_k_line->rx_frame_state == N_K_LINE_FRAME_STATE_INIT){

			if (((data[byte_cnt] & N_K_LINE_FMT_MASK) == N_K_LINE_FORMAT) && ((data[byte_cnt] & N_K_LINE_LEN_MASK) > 0)){

				n_k_line->rx_frame_state = N_K_LINE_FRAME_STATE_FORMAT;
				n_k_line->rx_frame_fmt_len = (data[byte_cnt] & N_K_LINE_LEN_MASK);

				if (debuglevel & DEBUG_LEVEL_STATE_ENG)
					printk("%s(%d) FORMAT len=%d\n", MOD_NAME, __LINE__, n_k_line->rx_frame_fmt_len);
			}

			if ((byte_cnt == (count - 1)) && (n_k_line->rx_frame_state == N_K_LINE_FRAME_STATE_INIT)){
					if (((data[byte_cnt] & N_K_LINE_FMT_MASK) == N_K_LINE_FORMAT) && ((data[byte_cnt] & 0x3F) <= 0)){
						if (debuglevel & DEBUG_LEVEL_STATE_ENG)
							printk("%s(%d) bad FORMART LENGTH\n", MOD_NAME, __LINE__);
					}
					else{
						if (debuglevel & DEBUG_LEVEL_STATE_ENG)
							printk("%s(%d) bad FORMAT\n", MOD_NAME, __LINE__);
					}
					return;
			}
		}
		else if(n_k_line->rx_frame_state == N_K_LINE_FRAME_STATE_FORMAT){

			if (data[byte_cnt] == N_K_LINE_DEVICE_ID){

				n_k_line->rx_frame_state = N_K_LINE_FRAME_STATE_TARGET;

				if (debuglevel & DEBUG_LEVEL_STATE_ENG)
					printk("%s(%d) TARGET\n", MOD_NAME, __LINE__);
			}
			else{
				n_k_line->rx_frame_state = N_K_LINE_FRAME_STATE_INIT;
				if (debuglevel & DEBUG_LEVEL_STATE_ENG)
					printk("%s(%d) bad TARGET\n", MOD_NAME, __LINE__);
				return;
			}
		}
		else if(n_k_line->rx_frame_state == N_K_LINE_FRAME_STATE_TARGET){

			if (data[byte_cnt] == N_K_LINE_TESTER_ID){

				n_k_line->rx_frame_state = N_K_LINE_FRAME_STATE_SOURCE;

				if (debuglevel & DEBUG_LEVEL_STATE_ENG)
					printk("%s(%d) SOURCE\n", MOD_NAME, __LINE__);
			}
			else{
				n_k_line->rx_frame_state = N_K_LINE_FRAME_STATE_INIT;
				if (debuglevel & DEBUG_LEVEL_STATE_ENG)
					printk("%s(%d) bad SOURCE\n", MOD_NAME, __LINE__);
				return;
			}
		}
		else if(n_k_line->rx_frame_state == N_K_LINE_FRAME_STATE_SOURCE){

			data_pos = byte_cnt;

			if (debuglevel & DEBUG_LEVEL_STATE_ENG)
				printk("%s(%d) SERVICE ID 0x%02x data_pos %d\n", MOD_NAME, __LINE__, data[byte_cnt], data_pos);

			n_k_line->rx_frame_state = N_K_LINE_FRAME_STATE_DATA;
			n_k_line->rx_frame_sid = data[byte_cnt];
		}
		else if(n_k_line->rx_frame_state == N_K_LINE_FRAME_STATE_DATA){

			if ((n_k_line->rx_frame_fmt_len + 1) <= ((count - data_pos) + n_k_line->rx_frame_counter)){

				n_k_line->rx_frame_state = N_K_LINE_FRAME_STATE_CHKSUM;

				if (debuglevel & DEBUG_LEVEL_STATE_ENG)
					printk("%s(%d) CHECKSUM 0x%02x\n", MOD_NAME, __LINE__, data[count - 1]);

				if ((n_k_line->rx_frame_fmt_len + 1) < ((count - data_pos) + n_k_line->rx_frame_counter)){
					end_pos = ((count - data_pos) + n_k_line->rx_frame_counter) - (n_k_line->rx_frame_fmt_len + 1);
					if (debuglevel & DEBUG_LEVEL_ERROR)
						printk("%s(%d) EXCESS DATA %d end_pos %d\n", MOD_NAME, __LINE__, count - (byte_cnt + 1), end_pos);
				}
				break;
			}
			else
			if ((n_k_line->rx_frame_fmt_len + 1) > ((count - data_pos) + n_k_line->rx_frame_counter)){
				if (debuglevel & DEBUG_LEVEL_STATE_ENG)
					printk("%s(%d) DATA\n", MOD_NAME, __LINE__);
			}
		}
		else{
			n_k_line->rx_frame_state = N_K_LINE_FRAME_STATE_INIT;
			if (debuglevel & DEBUG_LEVEL_ERROR)
				printk("%s(%d) UNDETERMINED %d\n", MOD_NAME, __LINE__, byte_cnt);
			return;
		}
	}

	if (	(n_k_line->rx_frame_state == N_K_LINE_FRAME_STATE_DATA) ||
		(n_k_line->rx_frame_state == N_K_LINE_FRAME_STATE_CHKSUM)){

		/* get a free K_LINE buffer */
		if (n_k_line->rx_frame_counter > 0){
			buf = n_k_line_buf_get(&n_k_line->rx_frame_buf_list);
		}
		else{
			buf = n_k_line_buf_get(&n_k_line->rx_free_buf_list);
		}

		if (!buf) {
			/* no buffers in free list, attempt to allocate another rx buffer */
			/* unless the maximum count has been reached */
			if (n_k_line->rx_buf_list.count < MAX_RX_BUF_COUNT)
				buf = kmalloc(N_K_LINE_BUF_SIZE, GFP_ATOMIC);
		}

		if (!buf) {
			if (debuglevel & DEBUG_LEVEL_ERROR)
				printk("%s(%d) no more rx buffers, data discarded\n",
				       MOD_NAME,__LINE__);
			return;
		}

		/*copy received data to K_LINE buffer*/
		if (n_k_line->rx_frame_counter == 0){
			buf->buf[0] = n_k_line->rx_frame_fmt_len;
			/*n_k_line->rx_frame_counter++;*/
		}

		if (n_k_line->rx_frame_counter < (n_k_line->rx_frame_fmt_len + 1)){

			memcpy(&(buf->buf[n_k_line->rx_frame_counter + 1]),
				&(data[data_pos]), count - (data_pos + end_pos));

			n_k_line->rx_frame_counter += (count - (data_pos + end_pos));
			buf->count = n_k_line->rx_frame_counter;
		}

		/* add K_LINE buffer to list of received frames */
		if (n_k_line->rx_frame_state < N_K_LINE_FRAME_STATE_CHKSUM){
			n_k_line_buf_put(&n_k_line->rx_frame_buf_list, buf);
		}
		else
		if (n_k_line->rx_frame_state == N_K_LINE_FRAME_STATE_CHKSUM){

			/*Calculate checksum*/
			char checksum = ((N_K_LINE_FORMAT | n_k_line->rx_frame_fmt_len) +
					N_K_LINE_DEVICE_ID +
					N_K_LINE_TESTER_ID);

			if (debuglevel & DEBUG_LEVEL_FRAME)
				show_buffer(MOD_NAME, "RX FULL", "FRAME", __LINE__, buf->buf, buf->count);

			for(byte_cnt = 1;byte_cnt < buf->count;byte_cnt++)
				checksum += buf->buf[byte_cnt];

			if (checksum != data[count - (end_pos + 1)]){

				if (debuglevel & DEBUG_LEVEL_ERROR)
					printk("Checksum Rx=0x%02x Calc=0x%02x byte_cnt=%d\n", data[count - (end_pos + 1)], checksum, byte_cnt);

				flush_rx_queue(tty);
				return;
			}

			n_k_line_buf_put(&n_k_line->rx_buf_list, buf);
			n_k_line->rx_frame_state = N_K_LINE_FRAME_STATE_INIT;
			n_k_line->rx_frame_counter = 0;

			if (n_k_line->rx_frame_sid == 0x81){

				n_k_line->rx_frame_sid = 0;
				n_k_line->let_state = LET_STATE_STAR_COM;

				if (cancel_delayed_work_sync(&n_k_line->start_comm_resp_work)){
					if (debuglevel & DEBUG_LEVEL_ERROR)
						printk("%s(%d) Canceling Pending works \"start_comm\"\n", MOD_NAME,__LINE__);
				}
				queue_delayed_work(n_k_line->workq_start, &n_k_line->start_comm_resp_work, msecs_to_jiffies(25));
			}
			else
			if (n_k_line->rx_frame_sid == 0x82){

				n_k_line->rx_frame_sid = 0;
				n_k_line->let_state = LET_STATE_STOP_COM;

				if (cancel_delayed_work_sync(&n_k_line->stop_comm_resp_work)){
					if (debuglevel & DEBUG_LEVEL_ERROR)
						printk("%s(%d) Canceling Pending works \"stop_comm\"\n", MOD_NAME,__LINE__);
				}
				queue_delayed_work(n_k_line->workq_stop, &n_k_line->stop_comm_resp_work, msecs_to_jiffies(25));
			}
		}
		else if (n_k_line->rx_frame_state > N_K_LINE_FRAME_STATE_CHKSUM){
			n_k_line_buf_put(&n_k_line->rx_buf_list, buf);
			n_k_line->rx_frame_state = N_K_LINE_FRAME_STATE_INIT;

			if (debuglevel & DEBUG_LEVEL_ERROR)
				printk("%s(%d) Invalid frame_status=%d\n",
				       MOD_NAME,__LINE__, n_k_line->rx_frame_counter);

			n_k_line->rx_frame_counter = 0;
		}
	}

	/* wake up any blocked reads and perform async signalling */
	wake_up_interruptible (&tty->read_wait);

	if (n_k_line->tty->fasync != NULL)
		kill_fasync (&n_k_line->tty->fasync, SIGIO, POLL_IN);

}	/* end of n_k_line_tty_receive() */

/**
 * n_k_line_tty_read - Called to retrieve one frame of data (if available)
 * @tty - pointer to tty instance data
 * @file - pointer to open file object
 * @buf - pointer to returned data buffer
 * @nr - size of returned data buffer
 *
 * Returns the number of bytes returned or error code.
 */
static ssize_t n_k_line_tty_read(struct tty_struct *tty, struct file *file,
			   __u8 __user *buf, size_t nr)
{
	struct n_k_line *n_k_line = tty2n_k_line(tty);
	int ret = 0;
	struct n_k_line_buf *rbuf;
	DECLARE_WAITQUEUE(wait, current);

	if (debuglevel & DEBUG_LEVEL_INFO)
		printk("%s(%d)n_k_line_tty_read() called\n",MOD_NAME,__LINE__);

	/* Validate the pointers */
	if (!n_k_line)
		return -EIO;

	/* verify user access to buffer */
	if (!access_ok(VERIFY_WRITE, buf, nr)) {
		printk(KERN_WARNING "%s(%d) n_k_line_tty_read() can't verify user "
		"buffer\n", MOD_NAME, __LINE__);
		return -EFAULT;
	}

	add_wait_queue(&tty->read_wait, &wait);

	for (;;) {
		if (test_bit(TTY_OTHER_CLOSED, &tty->flags)) {
			ret = -EIO;
			break;
		}
		if (tty_hung_up_p(file))
			break;

		set_current_state(TASK_INTERRUPTIBLE);

		rbuf = n_k_line_buf_get(&n_k_line->rx_buf_list);
		/*rbuf = n_k_line_buf_get(&n_k_line->rx_frame_buf_list);*/
		if (rbuf) {
			if (rbuf->count > nr) {
				/* too large for caller's buffer */
				ret = -EOVERFLOW;
			} else {
				if (copy_to_user(buf, rbuf->buf, rbuf->count))
					ret = -EFAULT;
				else
					ret = rbuf->count;
			}

			if (n_k_line->rx_free_buf_list.count >
			    DEFAULT_RX_BUF_COUNT)
				kfree(rbuf);
			else
				n_k_line_buf_put(&n_k_line->rx_free_buf_list, rbuf);
			break;
		}

		/* no data */
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}

		schedule();

		if (signal_pending(current)) {
			ret = -EINTR;
			break;
		}
	}

	remove_wait_queue(&tty->read_wait, &wait);
	__set_current_state(TASK_RUNNING);

	return ret;

}	/* end of n_k_line_tty_read() */

/**
 * n_k_line_tty_write - write a single frame of data to device
 * @tty	- pointer to associated tty device instance data
 * @file - pointer to file object data
 * @data - pointer to transmit data (one frame)
 * @count - size of transmit frame in bytes
 *
 * Returns the number of bytes written (or error code).
 */
static ssize_t n_k_line_tty_write(struct tty_struct *tty, struct file *file,
			    const unsigned char *data, size_t count)
{
	struct n_k_line *n_k_line = tty2n_k_line (tty);
	int error = 0;
	/*ktime_t timestamp_diff;*/
	unsigned long timestamp_temp;
	unsigned int ms_diff;
	DECLARE_WAITQUEUE(wait, current);
	struct n_k_line_buf *tbuf;

	if (debuglevel & DEBUG_LEVEL_INFO)
		/*show_buffer(MOD_NAME, "Write:", "", __LINE__, data, count);*/
		printk("%s(%d)n_k_line_tty_write() called count=%Zd\n",
			MOD_NAME,__LINE__,count);

	/* Verify pointers */
	if (!n_k_line)
		return -EIO;

	if (n_k_line->magic != K_LINE_MAGIC)
		return -EIO;

	/* verify frame size */
	if (count > maxframe ) {
		if (debuglevel & DEBUG_LEVEL_ERROR)
			printk (KERN_WARNING
				"n_k_line_tty_write: truncating user packet "
				"from %lu to %d\n", (unsigned long) count,
				maxframe );
		count = maxframe;
	}

	if ((count < (data [0] + 1)) || (count > (data[0] + 1)) || (data[0] > N_K_LINE_LEN_MASK)){
		if (debuglevel & DEBUG_LEVEL_ERROR)
			printk("%s(%d) n_k_line_tty_write() wrong data length count=0x%02x len=0x%02x\n",
				MOD_NAME,__LINE__,count,data[0]);
		return -EIO;
	}

	add_wait_queue(&tty->write_wait, &wait);

	for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);

		tbuf = n_k_line_buf_get(&n_k_line->tx_free_buf_list);
		if (tbuf)
			break;

		schedule();

		n_k_line = tty2n_k_line (tty);
		if (!n_k_line || n_k_line->magic != K_LINE_MAGIC ||
		    tty != n_k_line->tty) {
			printk("n_k_line_tty_write: %p invalid after wait!\n", n_k_line);
			error = -EIO;
			break;
		}

		if (signal_pending(current)) {
			error = -EINTR;
			break;
		}
	}

	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&tty->write_wait, &wait);

	if (!error) {
		int cnt = 0;
		unsigned char checksum = 0;

		memset(tbuf, 0, N_K_LINE_BUF_SIZE);
		/* Retrieve the user's buffer */
		tbuf->buf[0] = data [0] | N_K_LINE_FORMAT;
		tbuf->buf[1] = N_K_LINE_TESTER_ID;
		tbuf->buf[2] = N_K_LINE_DEVICE_ID;

		tbuf->count = error = (data [0] + 3);

		memcpy(&(tbuf->buf[3]), &(data[1]), tbuf->count);

		for(cnt = 0;cnt < tbuf->count;cnt++)
			checksum += tbuf->buf[cnt];

		tbuf->buf[tbuf->count] = checksum;
		tbuf->count++;

		if (debuglevel & DEBUG_LEVEL_FRAME)
			show_buffer(MOD_NAME, "TX FULL", "FRAME", __LINE__, tbuf->buf, tbuf->count);

		/* Send the data */
		n_k_line_buf_put(&n_k_line->tx_buf_list,tbuf);

		timestamp_temp = jiffies;
		ms_diff = jiffies_to_msecs(timestamp_temp - n_k_line->timestamp_rx);

		if (debuglevel & DEBUG_LEVEL_TIME){
			printk("%s(%d) timestamp_rx=%lu now=%lu delta=%lu  ms_delta=%u\n",
				MOD_NAME,__LINE__, n_k_line->timestamp_rx, timestamp_temp, timestamp_temp - n_k_line->timestamp_rx, ms_diff);
		}

		if(cancel_delayed_work_sync(&n_k_line->send_work)){
			if (debuglevel & DEBUG_LEVEL_ERROR)
				printk("%s(%d) Canceling Pending works \"send\"\n", MOD_NAME,__LINE__);
		}

		queue_delayed_work(n_k_line->workq_send, &n_k_line->send_work, msecs_to_jiffies((ms_diff >= 20)? 0: (20 - ms_diff)));
		/*n_k_line_send_frames(n_k_line,tty);*/
	}

	return error;

}	/* end of n_k_line_tty_write() */

/**
 * n_k_line_tty_ioctl - process IOCTL system call for the tty device.
 * @tty - pointer to tty instance data
 * @file - pointer to open file object for device
 * @cmd - IOCTL command code
 * @arg - argument for IOCTL call (cmd dependent)
 *
 * Returns command dependent result.
 */
static int n_k_line_tty_ioctl(struct tty_struct *tty, struct file *file,
			    unsigned int cmd, unsigned long arg)
{
	struct n_k_line *n_k_line = tty2n_k_line (tty);
	int error = 0;
	int count;
	unsigned long flags;

	if (debuglevel & DEBUG_LEVEL_INFO)
		printk("%s(%d)n_k_line_tty_ioctl() called %d\n",
			MOD_NAME,__LINE__,cmd);

	/* Verify the status of the device */
	if (!n_k_line || n_k_line->magic != K_LINE_MAGIC)
		return -EBADF;

	switch (cmd) {
	case FIONREAD:
		/* report count of read data available */
		/* in next available frame (if any) */
		spin_lock_irqsave(&n_k_line->rx_buf_list.spinlock,flags);
		if (n_k_line->rx_buf_list.head)
			count = n_k_line->rx_buf_list.head->count;
		else
			count = 0;
		spin_unlock_irqrestore(&n_k_line->rx_buf_list.spinlock,flags);
		error = put_user(count, (int __user *)arg);
		break;

	case TIOCOUTQ:
		/* get the pending tx byte count in the driver */
		count = tty_chars_in_buffer(tty);
		/* add size of next output frame in queue */
		spin_lock_irqsave(&n_k_line->tx_buf_list.spinlock,flags);
		if (n_k_line->tx_buf_list.head)
			count += n_k_line->tx_buf_list.head->count;
		spin_unlock_irqrestore(&n_k_line->tx_buf_list.spinlock,flags);
		error = put_user(count, (int __user *)arg);
		break;

	case TCFLSH:
		switch (arg) {
		case TCIOFLUSH:
		case TCOFLUSH:
			flush_tx_queue(tty);
		}
		/* fall through to default */

	default:
		error = n_tty_ioctl_helper(tty, file, cmd, arg);
		break;
	}
	return error;

}	/* end of n_k_line_tty_ioctl() */

/**
 * n_k_line_tty_poll - TTY callback for poll system call
 * @tty - pointer to tty instance data
 * @filp - pointer to open file object for device
 * @poll_table - wait queue for operations
 *
 * Determine which operations (read/write) will not block and return info
 * to caller.
 * Returns a bit mask containing info on which ops will not block.
 */
static unsigned int n_k_line_tty_poll(struct tty_struct *tty, struct file *filp,
				    poll_table *wait)
{
	struct n_k_line *n_k_line = tty2n_k_line (tty);
	unsigned int mask = 0;

	if (debuglevel & DEBUG_LEVEL_INFO)
		printk("%s(%d)n_k_line_tty_poll() called\n",MOD_NAME,__LINE__);

	if (n_k_line && n_k_line->magic == K_LINE_MAGIC && tty == n_k_line->tty) {
		/* queue current process into any wait queue that */
		/* may awaken in the future (read and write) */

		poll_wait(filp, &tty->read_wait, wait);
		poll_wait(filp, &tty->write_wait, wait);

		/* set bits for operations that won't block */
		if (n_k_line->rx_buf_list.head)
			mask |= POLLIN | POLLRDNORM;	/* readable */
		if (test_bit(TTY_OTHER_CLOSED, &tty->flags))
			mask |= POLLHUP;
		if (tty_hung_up_p(filp))
			mask |= POLLHUP;
		if (!tty_is_writelocked(tty) &&
				n_k_line->tx_free_buf_list.head)
			mask |= POLLOUT | POLLWRNORM;	/* writable */
	}
	return mask;
}	/* end of n_k_line_tty_poll() */

/**
 * n_k_line_alloc - allocate an n_k_line instance data structure
 *
 * Returns a pointer to newly created structure if success, otherwise %NULL
 */
static struct n_k_line *n_k_line_alloc(void)
{
	struct n_k_line_buf *buf;
	int i;
	struct n_k_line *n_k_line = kmalloc(sizeof(*n_k_line), GFP_KERNEL);

	if (!n_k_line)
		return NULL;

	memset(n_k_line, 0, sizeof(*n_k_line));

	n_k_line_buf_list_init(&n_k_line->rx_free_buf_list);
	n_k_line_buf_list_init(&n_k_line->tx_free_buf_list);
	n_k_line_buf_list_init(&n_k_line->rx_frame_buf_list);
	n_k_line_buf_list_init(&n_k_line->rx_buf_list);
	n_k_line_buf_list_init(&n_k_line->tx_buf_list);

	/* allocate free rx buffer list */
	for(i=0;i<DEFAULT_RX_BUF_COUNT;i++) {
		buf = kmalloc(N_K_LINE_BUF_SIZE, GFP_KERNEL);
		if (buf)
			n_k_line_buf_put(&n_k_line->rx_free_buf_list,buf);
		else if (debuglevel & DEBUG_LEVEL_INFO)
			printk("%s(%d)n_k_line_alloc(), kalloc() failed for rx buffer %d\n",MOD_NAME,__LINE__, i);
	}

	/* allocate free tx buffer list */
	for(i=0;i<DEFAULT_TX_BUF_COUNT;i++) {
		buf = kmalloc(N_K_LINE_BUF_SIZE, GFP_KERNEL);
		if (buf)
			n_k_line_buf_put(&n_k_line->tx_free_buf_list,buf);
		else if (debuglevel & DEBUG_LEVEL_INFO)
			printk("%s(%d)n_k_line_alloc(), kalloc() failed for tx buffer %d\n",MOD_NAME,__LINE__, i);
	}

	/* Initialize the control block */
	n_k_line->magic  = K_LINE_MAGIC;
	n_k_line->flags  = 0;

	return n_k_line;

}	/* end of n_k_line_alloc() */

/**
 * n_k_line_buf_list_init - initialize specified K_LINE buffer list
 * @list - pointer to buffer list
 */
static void n_k_line_buf_list_init(struct n_k_line_buf_list *list)
{
	memset(list, 0, sizeof(*list));
	spin_lock_init(&list->spinlock);
}	/* end of n_k_line_buf_list_init() */

/**
 * n_k_line_buf_put - add specified K_LINE buffer to tail of specified list
 * @list - pointer to buffer list
 * @buf	- pointer to buffer
 */
static void n_k_line_buf_put(struct n_k_line_buf_list *list,
			   struct n_k_line_buf *buf)
{
	unsigned long flags;
	spin_lock_irqsave(&list->spinlock,flags);

	buf->link=NULL;
	if (list->tail)
		list->tail->link = buf;
	else
		list->head = buf;
	list->tail = buf;
	(list->count)++;

	spin_unlock_irqrestore(&list->spinlock,flags);

}	/* end of n_k_line_buf_put() */

/**
 * n_k_line_buf_get - remove and return an K_LINE buffer from list
 * @list - pointer to K_LINE buffer list
 *
 * Remove and return an K_LINE buffer from the head of the specified K_LINE buffer
 * list.
 * Returns a pointer to K_LINE buffer if available, otherwise %NULL.
 */
static struct n_k_line_buf* n_k_line_buf_get(struct n_k_line_buf_list *list)
{
	unsigned long flags;
	struct n_k_line_buf *buf;
	spin_lock_irqsave(&list->spinlock,flags);

	buf = list->head;
	if (buf) {
		list->head = buf->link;
		(list->count)--;
	}
	if (!list->head)
		list->tail = NULL;

	spin_unlock_irqrestore(&list->spinlock,flags);
	return buf;

}	/* end of n_k_line_buf_get() */

static char k_line_banner[] __initdata =
	KERN_INFO "K_LINE line discipline maxframe=%u\n";
static char k_line_register_ok[] __initdata =
	KERN_INFO "N_K_LINE line discipline registered.\n";
static char k_line_register_fail[] __initdata =
	KERN_ERR "error registering line discipline: %d\n";
static char k_line_init_fail[] __initdata =
	KERN_INFO "N_K_LINE: init failure %d\n";

static int __init n_k_line_init(void)
{
	int status;

	/* range check maxframe arg */
	if (maxframe < 4096)
		maxframe = 4096;
	else if (maxframe > 65535)
		maxframe = 65535;

	printk(k_line_banner, maxframe);

	status = tty_register_ldisc(N_K_LINE, &n_k_line_ldisc);
	if (!status)
		printk(k_line_register_ok);
	else
		printk(k_line_register_fail, status);

	if (status)
		printk(k_line_init_fail, status);
	return status;

}	/* end of init_module() */

static char k_line_unregister_ok[] __exitdata =
	KERN_INFO "N_K_LINE: line discipline unregistered\n";
static char k_line_unregister_fail[] __exitdata =
	KERN_ERR "N_K_LINE: can't unregister line discipline (err = %d)\n";

static void __exit n_k_line_exit(void)
{
	/* Release tty registration of line discipline */
	int status = tty_unregister_ldisc(N_K_LINE);

	if (status)
		printk(k_line_unregister_fail, status);
	else
		printk(k_line_unregister_ok);
}

module_init(n_k_line_init);
module_exit(n_k_line_exit);

MODULE_LICENSE("GPL");
module_param(debuglevel, int, 0);
module_param(maxframe, int, 0);
MODULE_ALIAS_LDISC(N_K_LINE);
