/*
 * n_kwp.c - implements the N_KWP line discipline for KWP2000 Protocol.
 *
 * Copyright (C) 2014 - 2016 Honda R&D Americas, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_ldisc.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/list.h>

#define DRIVER_VERSION "v1.0"
#define DRIVER_NAME "n_kwp"

#define KMB_H_LENGTH 			3
#define KMB_DATA_LENGTH 		63
#define KMB_CS_LENGTH 			1
#define KMB_MAX_FRAME_SIZE 	(KMB_H_LENGTH+KMB_DATA_LENGTH+KMB_CS_LENGTH)

#define KMB_MAX_NR_QUEUES 		1

#define KMB_ADJUST_RECV_LENGTH		1
#define KMB_ADJUST_SEND_LENGTH		3

#define KWP_FMT_ADDR_MODE		0x80
#define KWP_FMT_LENGTH_MASK		0x3f
#define KWP_PHY_ADDR_LET		0xf6
#define KWP_PHY_ADDR_AHU		0x81

#define KWP_SID_REQ_START_COMMUNICATION 0x81
#define KWP_SID_REQ_STOP_COMMUNICATION  0x82
#define KWP_SID_RES_START_COMMUNICATION 0xc1
#define KWP_SID_RES_STOP_COMMUNICATION  0xc2

#define KWP_START_COMMUNICATION 	0
#define KWP_STOP_COMMUNICATION  	1

unsigned char kwp_sid[][2] = {
	[KWP_START_COMMUNICATION] = {
		KWP_SID_REQ_START_COMMUNICATION,
		KWP_SID_RES_START_COMMUNICATION,
	},
	[KWP_STOP_COMMUNICATION] = {
		KWP_SID_REQ_STOP_COMMUNICATION,
		KWP_SID_RES_STOP_COMMUNICATION,
	},
};

/* 25ms <= P2 <= 50ms */
#define P2_MIN 25
#define P2_MAX 50
/* 55ms <= P3 <= 5000ms */
#define P3_MIN 55
#define P3_MAX 5000
/* 5ms <= P4 <= 20ms */
#define P4_MIN 5
#define P4_MAX 20

/*
 * Format Type-2:
 *  - Data Length is contained in Format Byte.
 *  - The format with Target/Source Address specification.
 *  - AUDIO LET is using this Format Type-2.
 */
struct kmb_h {
	unsigned char fmt;
	unsigned char tgt;
	unsigned char src;
	unsigned char sid;
};

struct timestamp {
	unsigned long start_of_request_jiffies;
	unsigned long end_of_request_jiffies;
	unsigned long start_of_response_jiffies;
	unsigned long end_of_response_jiffies;
};

struct period_timeout {
	unsigned long p2;
	unsigned long p3;
	unsigned long p4;
};

struct kmb {
	struct list_head list;
	struct timestamp ts;
	unsigned char frame[KMB_MAX_FRAME_SIZE];
};

struct kmb_list {
	unsigned int refcount;
	struct mutex lock;
	struct list_head list_head;
};

#define KMB_STATE_SHIFT		8
struct kmb_control {
	unsigned int state;
	/* KMB STATE */
#define KMB_STATE_FMT  		0x00000000
#define KMB_STATE_TGT  		0x00000001
#define KMB_STATE_SRC  		0x00000002
#define KMB_STATE_SID  		0x00000003
#define KMB_STATE_DATA 		0x00000004
#define KMB_STATE_CS   		0x00000005
#define KMB_STATE_DONE 		0x00000006
#define KMB_STATE_ERR  		0x00000007
	unsigned char index;
	unsigned char length;
	unsigned int err;

	unsigned int (*process)(struct kmb_control *ctrl, struct kmb *kmb,
				unsigned char c);
	void (*clear)(struct kmb_control *ctrl);
};

struct n_kwp_data {
	struct tty_struct *tty;
	struct kmb_control ctrl[2];
#define KMB_RX	0
#define KMB_TX	1

	struct kmb_list list[4];
#define KMB_RX_LIST	0
#define KMB_TX_LIST	1
#define KMB_WAIT_LIST	2
#define KMB_FREE_LIST	3

	unsigned int total_count;
	unsigned int count;

	bool p2_time_readjusted;
	bool p3_time_readjusted;

	struct mutex atomic_read_lock;
	struct mutex output_lock;
	struct period_timeout timeout;

	bool connected;
	unsigned int mode;
#define TERMINATED_MODE 0
#define INITIALIZE_MODE 1
#define DIAGNOSTIC_MODE 2

	int (*enqueue)(struct kmb *kmb, struct kmb_list *list);
	struct kmb *(*dequeue)(struct kmb_list *list);
	int (*is_empty)(struct kmb_list *list);
};

#define KMB_FMT 0
#define KMB_TGT 1
#define KMB_SRC 2
#define KMB_SID 3
#define KMB_SOD KMB_SID
#define KMB_DATA 4

#define GET_KMB_FMT(kmb) ((kmb)->frame[KMB_FMT])
#define SET_KMB_FMT(kmb, fmt) ((kmb)->frame[KMB_FMT]=(fmt))
#define GET_KMB_FMT_MODE(kmb) ((kmb)->frame[KMB_FMT]>>6)
#define GET_KMB_FMT_LENGTH(kmb) (((kmb)->frame[KMB_FMT])&(KWP_FMT_LENGTH_MASK))
#define GET_KMB_FMT_SID(kmb) ((kmb)->frame[KMB_SID])
#define GET_KMB_H_D_SIZE(kmb) ((KMB_H_LENGTH)+GET_KMB_FMT_LENGTH((kmb)))
#define GET_KMB_FRAME_SIZE(kmb) (GET_KMB_H_D_SIZE((kmb))+(KMB_CS_LENGTH))

#define KMB_CS(kmb) ((KMB_H_LENGTH)+GET_KMB_FMT_LENGTH((kmb)))
#define KMB_EOD(kmb) (KMB_CS((kmb))-(1))

#define KWPIOC_GETMODE _IOR('K', 0, unsigned int)
#define KWPIOC_SETMODE _IOW('K', 0, unsigned int)
#define KWPIOC_RESET   _IO('K', 0)

#define KWP_DEBUG_KMB         0x00000001
#define KWP_DEBUG_KMB2        0x00000002
#define KWP_DEBUG_TIMESTAMP   0x00000004
#define KWP_DEBUG_TIMESTAMP2  0x00000008

static unsigned int kwp_debug = 0;

module_param_named(debug, kwp_debug, int, 0600);

#define KWP_INFO(fmt, args...)					\
	do {							\
		pr_info("%s: " fmt, DRIVER_NAME, ##args);	\
	} while (0)

#define KWP_DEBUG(fmt, args...)					\
	do {							\
		pr_debug("%s: " fmt, DRIVER_NAME, ##args);	\
	} while (0)

#define KWP_ERR(fmt, args...)					\
	do {							\
		pr_err("%s: " fmt, DRIVER_NAME, ##args);	\
	} while (0)

void dump_kmb(struct n_kwp_data *kwp, struct kmb *kmb, char *dir)
{
	char *str, buf[256];
	int i;

        for (str = buf, i = 0; i < GET_KMB_FMT_LENGTH(kmb) + 4; i++) {
                str += sprintf(str, "%02X ", (kmb)->frame[i]);
        }

	KWP_DEBUG("%s, %s, %s\n", ((kwp)->mode == INITIALIZE_MODE)
		  ? "INIT":"DIAG", dir, buf);
}

#define KWP_DBG_KMB(fmt, args...)		\
	do {					\
		if (kwp_debug & KWP_DEBUG_KMB)	\
			dump_kmb(fmt, ##args);	\
	} while (0)
#define KWP_DBG_KMB2(fmt, args...)		\
	do {					\
		if (kwp_debug & KWP_DEBUG_KMB2)	\
			dump_kmb(fmt, ##args);	\
	} while (0)
#define KWP_DBG_TS(fmt, args...)			\
	do {						\
		if (kwp_debug & KWP_DEBUG_TIMESTAMP)	\
			KWP_DEBUG(fmt, ##args);		\
	} while (0)
#define KWP_DBG_TS2(fmt, args...)			\
	do {						\
		if (kwp_debug & KWP_DEBUG_TIMESTAMP2)	\
			KWP_DEBUG(fmt, ##args);		\
	} while (0)

/*
 * pos |connected| mode               |
 * ----------------------------------------------------------------
 * i   | false   | TERMINATED_MODE    | default = stop communication
 *-----------------------------------------------------------------
 * rx  | false   | <INITIALIZE_MODE>  | ready start
 * tx  | <true>  | <DIAGNOSTIC_MODE>  | start communication
 *-----------------------------------------------------------------
 * rx  | true    | DIAGNOSTIC_MODE    | communication
 * tx  | true    | DIAGNOSTIC_MODE    | communication
 *-----------------------------------------------------------------
 * rx  | true    | <INITIALIZE_MODE>  | ready stop
 * tx  | <false> | TERMINATED_MODE    | stop communication
 * ----------------------------------------------------------------
 */

static inline unsigned int get_p2_max(struct n_kwp_data *kwp)
{
	return (kwp->p2_time_readjusted == true) ? P3_MAX : P2_MAX;
}

static inline unsigned int get_p3_min(struct n_kwp_data *kwp)
{
	return (kwp->p3_time_readjusted == true) ? 0 : P3_MIN;
}

static inline int is_start_communication(struct kmb *kmb, int dir)
{
	struct kmb_h *h = (struct kmb_h *)kmb->frame;

	return (h->sid == kwp_sid[KWP_START_COMMUNICATION][dir]) ? 1 : 0;
}

static inline int is_stop_communication(struct kmb *kmb, int dir)
{
	struct kmb_h *h = (struct kmb_h *)kmb->frame;

	return (h->sid == kwp_sid[KWP_STOP_COMMUNICATION][dir])	? 1 : 0;
}

static inline int is_communication_service(struct kmb *kmb, int dir)
{
	return (is_start_communication(kmb, dir)
		|| is_stop_communication(kmb, dir));
}

static unsigned char checksum(unsigned char *data, int size)
{
	unsigned char csum = 0;
	while (size--) {
		csum += *data;
		data++;
	}

	return csum;
}

static inline void kmb_frame_init(struct kmb *kmb)
{
	memset(kmb->frame, 0, KMB_MAX_FRAME_SIZE);
}

static inline void kmb_swap_byte(struct kmb *kmb, unsigned char a,
				 unsigned char b)
{
	unsigned char tmp = kmb->frame[a];

	kmb->frame[a] = kmb->frame[b];
	kmb->frame[b] = tmp;
}

static struct kmb *kmb_kalloc(void)
{
	struct kmb *kmb;

	kmb = kzalloc(sizeof(struct kmb), GFP_KERNEL);
	if (!kmb)
		goto __nomem;

	INIT_LIST_HEAD(&kmb->list);
__nomem:
	return kmb;
}

static void kmb_kfree(struct kmb *kmb)
{
	kfree(kmb);
}

static int kmb_list_enqueue(struct kmb *kmb, struct kmb_list *list)
{
	mutex_lock(&list->lock);
	list_add_tail(&kmb->list, &list->list_head);
	list->refcount++;
	mutex_unlock(&list->lock);

	return 0;
}

static struct kmb *kmb_list_dequeue(struct kmb_list *list)
{
	struct kmb *kmb;

	mutex_lock(&list->lock);
	kmb = list_first_entry(&list->list_head, struct kmb, list);
	list_del(&kmb->list);
	list->refcount--;
	mutex_unlock(&list->lock);

	return kmb;
}

static int kmb_list_empty(struct kmb_list *list)
{
	int ret;

	mutex_lock(&list->lock);
	ret = list_empty(&list->list_head);
	mutex_unlock(&list->lock);

	return ret;
}

static void kmb_ctrl_clear(struct kmb_control *ctrl)
{
	/* control clear */
	ctrl->state = KMB_STATE_FMT;
	ctrl->index = 0;
	ctrl->length = 0;
	ctrl->err = 0;
}

static unsigned int kmb_ctrl_decode_format_type2(struct kmb_control *ctrl,
						 struct kmb *kmb,
						 unsigned char c)
{
	switch(ctrl->state) {
	case KMB_STATE_FMT:
		if (KWP_FMT_ADDR_MODE & c) {
			/*
			 * Only record the start_of_request if this really is the
			 * start of a request, else we get invalid P3 errors.
			 */
			kmb->ts.start_of_request_jiffies = jiffies;
			ctrl->length = c & KWP_FMT_LENGTH_MASK;
			ctrl->state = KMB_STATE_TGT;
		} else {
			ctrl->state = KMB_STATE_ERR;
			ctrl->err = KMB_STATE_FMT;
		}
		break;
	case KMB_STATE_TGT:
		if(KWP_PHY_ADDR_AHU == c) {
			ctrl->state = KMB_STATE_SRC;
		} else {
			ctrl->state = KMB_STATE_ERR;
			ctrl->err = KMB_STATE_TGT;
		}
		break;
	case KMB_STATE_SRC:
		if(KWP_PHY_ADDR_LET == c) {
			ctrl->state = KMB_STATE_SID;
		} else {
			ctrl->state = KMB_STATE_ERR;
			ctrl->err = KMB_STATE_SRC;
		}
		break;
	case KMB_STATE_SID:
		if (ctrl->index == KMB_EOD(kmb))
			ctrl->state = KMB_STATE_CS;
		else
			ctrl->state = KMB_STATE_DATA;

		break;
	case KMB_STATE_DATA:
		if (ctrl->index == KMB_EOD(kmb))
			ctrl->state = KMB_STATE_CS;
		break;
	case KMB_STATE_CS:
		kmb->ts.end_of_request_jiffies = jiffies;
		if (checksum(kmb->frame, GET_KMB_H_D_SIZE(kmb)) == c) {
			ctrl->state = KMB_STATE_DONE;
		} else {
			ctrl->state = KMB_STATE_ERR;
			ctrl->err = KMB_STATE_CS;
		}
		break;
	}

	kmb->frame[ctrl->index] = c;
	ctrl->index++;

	return ctrl->state;
}

static unsigned int kmb_ctrl_encode_format_type2(struct kmb_control *ctrl,
						 struct kmb *kmb,
						 unsigned char c)
{
	switch(ctrl->state) {
	case KMB_STATE_FMT:
		kmb->ts.start_of_response_jiffies = jiffies;
		kmb->frame[ctrl->index] = c | KWP_FMT_ADDR_MODE;
		ctrl->length = c & KWP_FMT_LENGTH_MASK;
		ctrl->index = KMB_SID;
		ctrl->state = KMB_STATE_SID;
		break;
	case KMB_STATE_SID:
		kmb->frame[ctrl->index] = c;
		if (ctrl->index == KMB_EOD(kmb))
			ctrl->state = KMB_STATE_CS;
		else
			ctrl->state = KMB_STATE_DATA;
		ctrl->index++;
		break;
	case KMB_STATE_DATA:
		kmb->frame[ctrl->index] = c;
		if (ctrl->index == KMB_EOD(kmb))
			ctrl->state = KMB_STATE_CS;
		ctrl->index++;
		break;
	}

	if (ctrl->state == KMB_STATE_CS) {
		kmb->frame[ctrl->index] =
			checksum(kmb->frame, GET_KMB_H_D_SIZE(kmb));
		ctrl->state = KMB_STATE_DONE;
	}

	return ctrl->state;
}

static void set_mode(struct n_kwp_data *kwp, unsigned int mode)
{
	switch (mode) {
	case INITIALIZE_MODE:
		break;
	case DIAGNOSTIC_MODE:
		KWP_INFO("Communication link established\n");
		kwp->connected = true;
		break;
	case TERMINATED_MODE:
		if (kwp->connected)
			KWP_INFO("Communication link terminated\n");
		kwp->connected = false;
		kwp->count = 0;
		break;
	default:
		mode = INITIALIZE_MODE;
		break;
	}
	kwp->mode = mode;
}

static inline int rx_list_empty(struct n_kwp_data *kwp)
{
	return kwp->is_empty(&kwp->list[KMB_RX_LIST]);
}

static inline int tx_list_empty(struct n_kwp_data *kwp)
{
	return kwp->is_empty(&kwp->list[KMB_TX_LIST]);
}

static inline int free_list_empty(struct n_kwp_data *kwp)
{
	return kwp->is_empty(&kwp->list[KMB_FREE_LIST]);
}

static inline int wait_list_empty(struct n_kwp_data *kwp)
{
	return kwp->is_empty(&kwp->list[KMB_WAIT_LIST]);
}

static int response_timeout_p2(struct n_kwp_data *kwp, struct kmb *kmb)
{
	struct period_timeout *timeout = &kwp->timeout;
	struct timestamp *ts = &kmb->ts;
	unsigned int cur_p2_max = get_p2_max(kwp);
	int ret = 0;

	/* Response timeout (P2 specified time violation) */
	timeout->p2 =
		ts->start_of_response_jiffies - ts->end_of_request_jiffies;

	KWP_DBG_TS("entry P2=%ums, min=%ums, max=%ums\n",
		   jiffies_to_msecs(timeout->p2), P2_MIN, cur_p2_max);

	if (jiffies_to_msecs(timeout->p2) > cur_p2_max) {
		KWP_ERR("Response timeout (P2 specified time violation)\n");
		KWP_ERR("P2=%ums, e_o_req=%ums, s_o_res=%ums,"
			"min=%ums, max=%ums\n",
			jiffies_to_msecs(timeout->p2),
			jiffies_to_msecs(ts->end_of_request_jiffies),
			jiffies_to_msecs(ts->start_of_response_jiffies),
			P2_MIN, cur_p2_max);
		if (kwp->connected) {
			kwp->p2_time_readjusted = true;
			kwp->p3_time_readjusted = true;
		}
		ret = 1;
	} else {
		if (jiffies_to_msecs(timeout->p2) < P2_MIN) {
			unsigned long p2_min_jiffies
				= ts->end_of_request_jiffies
				+ msecs_to_jiffies(P2_MIN);
			KWP_DBG_TS("waiting until P2 min time(%ums) "
				   "is reached\n", P2_MIN);
			while (1) {
				if (time_after_eq(jiffies, p2_min_jiffies))
					break;
			}
			timeout->p2 = jiffies - ts->end_of_request_jiffies;
		}
		KWP_DBG_TS("P2=%ums, e_o_req=%ums, s_o_res=%ums\n",
			   jiffies_to_msecs(timeout->p2),
			   jiffies_to_msecs(ts->end_of_request_jiffies),
			   jiffies_to_msecs(ts->start_of_response_jiffies));

		kwp->p2_time_readjusted = false;
	}

	if (cur_p2_max > P2_MAX)
		kwp->p3_time_readjusted = true;

	return ret;
}

static ssize_t kwp_send_msg(struct n_kwp_data *kwp)
{
	struct period_timeout *timeout = &kwp->timeout;
	struct kmb_control *ctrl = &kwp->ctrl[KMB_TX];
	struct kmb *kmb;
	struct timestamp *ts;
	int sent;

	kmb = kwp->dequeue(&kwp->list[KMB_TX_LIST]);
	ts = &kmb->ts;
	ts->end_of_response_jiffies = jiffies;

	if (response_timeout_p2(kwp, kmb)) {
		KWP_ERR("Response ignore: for P2, sid=0x%02x\n",
			GET_KMB_FMT_SID(kmb));
		sent = GET_KMB_FRAME_SIZE(kmb) - KMB_ADJUST_SEND_LENGTH;
	} else {
		KWP_DBG_KMB(kwp, kmb, "Tx-");
		sent = kwp->tty->ops->write(kwp->tty, kmb->frame,
					    GET_KMB_FRAME_SIZE(kmb));
		if (sent != GET_KMB_FRAME_SIZE(kmb)) {
			KWP_ERR("tty write sent %d out of %d\n", sent, GET_KMB_FRAME_SIZE(kmb));
		}
		sent -= KMB_ADJUST_SEND_LENGTH;
		/*
		 * Only start P3 counter if we actually sent a response. Don't hold LET
		 * to P3 timing when we didn't actually respond.
		 */
		timeout->p3 = ts->end_of_response_jiffies;
	}

	KWP_DBG_TS("s_o_req=%ums, e_o_req=%ums, s_o_res=%ums, e_o_res=%ums\n",
		   jiffies_to_msecs(ts->start_of_request_jiffies),
		   jiffies_to_msecs(ts->end_of_request_jiffies),
		   jiffies_to_msecs(ts->start_of_response_jiffies),
		   jiffies_to_msecs(ts->end_of_response_jiffies));

	if (is_communication_service(kmb, KMB_TX_LIST)) {
		if (kwp->mode == INITIALIZE_MODE) {
			if (is_start_communication(kmb, KMB_TX_LIST))
				set_mode(kwp, DIAGNOSTIC_MODE);
			else
				set_mode(kwp, TERMINATED_MODE);
		}
	}

	kwp->count++;
	kwp->total_count++;

	if (!kwp->connected)
		kwp->count = 0;

	ctrl->clear(ctrl);
	kmb_frame_init(kmb);
	memset(&kmb->ts, 0, sizeof(struct timestamp));

	kwp->enqueue(kmb, &kwp->list[KMB_FREE_LIST]);

	return sent;
}

static ssize_t kwp_send(struct tty_struct *tty)
{
	struct n_kwp_data *kwp = tty->disc_data;
	struct kmb *kmb;
	int space, sent, size;

	mutex_lock(&kwp->output_lock);
	sent = 0;

	kmb = kwp->dequeue(&kwp->list[KMB_TX_LIST]);
	size = GET_KMB_FRAME_SIZE(kmb);
	kwp->enqueue(kmb, &kwp->list[KMB_TX_LIST]);
	space = tty_write_room(tty);

	if (space >= size)
		sent = kwp_send_msg(kwp);
	else {
		set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
		KWP_INFO("delay write, size %d, space %d\n", size, space);
	}

	mutex_unlock(&kwp->output_lock);

	return sent;
}

static unsigned char response_frame[][7] = {
	/* START COMMUNICATION */
	[KWP_START_COMMUNICATION] = {
		0x83, 0xf6, 0x81, 0xc1, 0xe9, 0x8f, 0x33,
	},
	/* STOP COMMUNICATION */
	[KWP_STOP_COMMUNICATION] = {
		0x81, 0xf6, 0x81, 0xc2, 0xba,
	}
};

static void send_communication_service(struct n_kwp_data *kwp, struct kmb *kmb)
{
	struct kmb_h *h = (struct kmb_h *)kmb->frame;
	int cmd = 0;

	switch (h->sid) {
	case KWP_SID_REQ_START_COMMUNICATION:
		cmd = KWP_START_COMMUNICATION;
		break;
	case KWP_SID_REQ_STOP_COMMUNICATION:
		cmd = KWP_STOP_COMMUNICATION;
		break;
	}
	kmb->ts.start_of_response_jiffies = jiffies;
	kmb_frame_init(kmb);
	memcpy(kmb->frame, &response_frame[cmd], sizeof(response_frame[cmd]));
	kwp->enqueue(kmb, &kwp->list[KMB_TX_LIST]);
	kwp_send(kwp->tty);
}

static int kwp_read_msg(struct tty_struct *tty, unsigned char __user *buf,
			  size_t nr)
{
	struct n_kwp_data *kwp = tty->disc_data;
	struct kmb_control *ctrl = &kwp->ctrl[KMB_RX];
	struct kmb *kmb = NULL;
	unsigned char *data = NULL;
	int rcvd = 0;

	kmb = kwp->dequeue(&kwp->list[KMB_RX_LIST]);

	if (!ctrl->length) {
		ctrl->length = GET_KMB_FMT_LENGTH(kmb)
			+ KMB_ADJUST_RECV_LENGTH;
		SET_KMB_FMT(kmb, GET_KMB_FMT_LENGTH(kmb));
		kmb_swap_byte(kmb, KMB_FMT, KMB_SRC);
		ctrl->index = KMB_SRC;
	}

	data = &kmb->frame[ctrl->index];

	if (ctrl->length >= nr) {
		rcvd = copy_to_user(buf, data, nr);
		if (rcvd)
			goto __copy_err;
		ctrl->index += nr;
		rcvd += nr;
		ctrl->length -= nr;
	} else {
		rcvd = copy_to_user(buf, data, ctrl->length);
		if (rcvd)
			goto __copy_err;
		ctrl->index += ctrl->length;
		rcvd += ctrl->length;
		ctrl->length -= ctrl->length;
	}

__copy_err:
	if (ctrl->length) {
		kwp->enqueue(kmb, &kwp->list[KMB_RX_LIST]);
	} else {
		ctrl->clear(ctrl);
		kmb_swap_byte(kmb, KMB_FMT, KMB_SRC);
		SET_KMB_FMT(kmb, GET_KMB_FMT_LENGTH(kmb)|KWP_FMT_ADDR_MODE);
		if (is_communication_service(kmb, KMB_RX_LIST)) {
			send_communication_service(kwp, kmb);
		} else {
			KWP_DBG_KMB2(kwp, kmb, "Rx+");
			kwp->enqueue(kmb, &kwp->list[KMB_WAIT_LIST]);
		}
	}

	return rcvd;
}

static int kwp_write_msg(struct n_kwp_data *kwp, const unsigned char *cp)
{
	struct kmb_control *ctrl = &kwp->ctrl[KMB_TX];
	struct kmb *kmb = NULL;
	int sent = 1;

	kmb = kwp->dequeue(&kwp->list[KMB_WAIT_LIST]);

	if (!ctrl->length)
		kmb_swap_byte(kmb, KMB_TGT, KMB_SRC);

        switch (ctrl->process(ctrl, kmb, *cp)) {
	case KMB_STATE_DONE:
		KWP_DBG_KMB2(kwp, kmb, "Tx+");
		ctrl->clear(ctrl);
		kwp->enqueue(kmb, &kwp->list[KMB_TX_LIST]);
		kmb = NULL;
		break;
	case KMB_STATE_ERR:
		ctrl->clear(ctrl);
		sent = 0;
		break;
	}

	if (kmb)
		kwp->enqueue(kmb, &kwp->list[KMB_WAIT_LIST]);

	return sent;
}

static int kwp_write(struct tty_struct *tty, const unsigned char *cp,
			size_t count)
{
	struct n_kwp_data *kwp = tty->disc_data;
	int n, sent = 0;

        while (count) {
		if (wait_list_empty(kwp))
			break;
                n = kwp_write_msg(kwp, cp);
		if (!n)
			break;
                cp++;
                count--;
		sent++;
        }

	return sent;
}

#define EREQ_P3		1
#define EREQ_P4		2
#define EREQ_NEED_INIT 	3
#define EREQ_DATA      	4

static void ignore_request(struct kmb_control *ctrl, struct kmb *kmb, int err)
{
	switch (err) {
	case EREQ_P3:
		KWP_ERR("Request ignore: for P3, sid=0x%02x\n",
			GET_KMB_FMT_SID(kmb));
		kmb_frame_init(kmb);
		break;
	case EREQ_P4:
		KWP_ERR("Request ignore: for P4, c=0x%02x\n",
			kmb->frame[ctrl->index-1]);
		kmb_frame_init(kmb);
		break;
	case EREQ_NEED_INIT:
		KWP_ERR("Request ignore: need to initialize "
			"communication link, sid=0x%02x\n",
			GET_KMB_FMT_SID(kmb));
		kmb_frame_init(kmb);
		break;
	case EREQ_DATA:
		switch (ctrl->err) {
		case KMB_STATE_FMT:
			KWP_DEBUG("Request ignore: Data error:(Data contents),"
				  "FMT(0x%02x)\n", kmb->frame[ctrl->index-1]);
			break;
		case KMB_STATE_TGT:
			KWP_DEBUG("Request ignore: Data error:(Data contents),"
				  "TGT(0x%02x)\n", kmb->frame[ctrl->index-1]);
			break;
		case KMB_STATE_SRC:
			KWP_DEBUG("Request ignore: Data error:(Data contents),"
				  "SRC(0x%02x)\n", kmb->frame[ctrl->index-1]);
			break;
		case KMB_STATE_CS:
			KWP_ERR("Request ignore: Data error:(Data checksum),"
				"CS(0x%02x)\n", kmb->frame[ctrl->index-1]);
			kmb_frame_init(kmb);
			break;
		}
	}
}

static int request_timeout_p3(struct n_kwp_data *kwp, struct kmb *kmb)
{
	struct period_timeout *timeout = &kwp->timeout;
	struct timestamp *ts = &kmb->ts;
	unsigned long p3;
	int ret = 0;

	if (timeout->p3)
		p3 = ts->start_of_request_jiffies - timeout->p3;
	else
		p3 = msecs_to_jiffies(get_p3_min(kwp));

	/* Request timeout (P3 specified time violation) */
	/*
	 * Differentiate between a request in less than P3 minimum, in which case we
	 * should ignore the request, and a request later than P3 maximum, in which
	 * case we treat it as the end of a communication session.
	 */
	if (jiffies_to_msecs(p3) < get_p3_min(kwp)) {
		KWP_ERR("Request timeout (P3 specified time violation) less than minimum\n");
		KWP_ERR("P3=%ums, e_o_res=%ums, s_o_req=%ums,"
			"min=%ums, max=%ums\n",
			jiffies_to_msecs(p3),
			jiffies_to_msecs(timeout->p3),
			jiffies_to_msecs(ts->start_of_request_jiffies),
			get_p3_min(kwp), P3_MAX);
		ret = -1;
	} else if (jiffies_to_msecs(p3) > P3_MAX) {
		KWP_ERR("Request timeout (P3 specified time violation) exceeds maximum\n");
		KWP_ERR("P3=%ums, e_o_res=%ums, s_o_req=%ums,"
			"min=%ums, max=%ums\n",
			jiffies_to_msecs(p3),
			jiffies_to_msecs(timeout->p3),
			jiffies_to_msecs(ts->start_of_request_jiffies),
			get_p3_min(kwp), P3_MAX);
		ret = 1;
	} else {
		KWP_DBG_TS("P3=%ums, e_o_res=%ums, s_o_req=%ums,"
			   "min=%ums, max=%ums\n",
			   jiffies_to_msecs(p3),
			   jiffies_to_msecs(timeout->p3),
			   jiffies_to_msecs(ts->start_of_request_jiffies),
			   get_p3_min(kwp), P3_MAX);
	}

	kwp->p3_time_readjusted = false;

	return ret;
}

static int request_timeout_p4(struct n_kwp_data *kwp, unsigned char c)
{
	struct period_timeout *timeout = &kwp->timeout;
	int ret;

	if (timeout->p4 && (jiffies_to_msecs(jiffies - timeout->p4) > P4_MAX)) {
		KWP_ERR("Request timeout byte interval error "
			"(P4 specified time violation), p4=%ums, 0x%02x\n",
			jiffies_to_msecs(jiffies - timeout->p4), c);
		ret = 1;
	} else {
		KWP_DBG_TS2("P4=%ums, c=%02x\n",
			    jiffies_to_msecs(jiffies - timeout->p4), c);
		ret = 0;
	}

	return ret;
}

static unsigned int kwp_recv_msg(struct n_kwp_data *kwp,
				   const unsigned char *cp)
{
	struct kmb_control *ctrl = &kwp->ctrl[KMB_RX];
	struct kmb *kmb = NULL;
	unsigned int ret = 0;

	kmb = kwp->dequeue(&kwp->list[KMB_FREE_LIST]);

	switch (ctrl->process(ctrl, kmb, *cp)) {
	case KMB_STATE_TGT:
		kwp->timeout.p4 = kmb->ts.start_of_request_jiffies;
	case KMB_STATE_SRC:
	case KMB_STATE_SID:
	case KMB_STATE_DATA:
	case KMB_STATE_CS:
		if (request_timeout_p4(kwp, *cp)) {
			ignore_request(ctrl, kmb, EREQ_P4);
			/*
			 * Since we are already ignoring this request,
			 * P4 timing isn't relevant anymore.
			 */
			kwp->timeout.p4 = 0;
			/* set P3 time again. */
			if (kwp->connected)
				kwp->timeout.p3 = 0;
			else
				set_mode(kwp, TERMINATED_MODE);
			ctrl->clear(ctrl);
		}
		kwp->timeout.p4 = jiffies;
		break;
	case KMB_STATE_DONE:
		KWP_DBG_KMB(kwp, kmb, "Rx-");

		if (kwp->connected) {
			int p3_state;

			p3_state = request_timeout_p3(kwp, kmb);
			if (p3_state) {
				ignore_request(ctrl, kmb, EREQ_P3);
				kwp->timeout.p4 = 0; /* P4 timing no longer relevant. */
                                /*
				 * Figure 12 in ISO 14230-2 says to stop communication
				 * if P3 is exceeded, not less than minimum.
				 */
				if (p3_state > 0)
					set_mode(kwp, TERMINATED_MODE);
				ctrl->clear(ctrl);
				break;
			}
		}
		if (!is_communication_service(kmb, KMB_RX_LIST) &&
		    !kwp->connected) {
			ignore_request(ctrl, kmb, EREQ_NEED_INIT);
			ctrl->clear(ctrl);
		} else {
			if (is_communication_service(kmb, KMB_RX_LIST))
				set_mode(kwp, INITIALIZE_MODE);
			ctrl->clear(ctrl);
			kwp->enqueue(kmb, &kwp->list[KMB_RX_LIST]);
			if (waitqueue_active(&kwp->tty->read_wait))
				wake_up_interruptible(&kwp->tty->read_wait);
			kmb = NULL;
		}
		kwp->timeout.p4 = 0; /* Request complete, P4 timing not relevant now. */
		break;
	case KMB_STATE_ERR:
		ignore_request(ctrl, kmb, EREQ_DATA);
		kwp->timeout.p4 = 0; /* P4 timing no longer relevant. */
		/* set P3 time again. */
		if (kwp->connected)
			kwp->timeout.p3 = 0;
		else
			set_mode(kwp, TERMINATED_MODE);

		if (ctrl->err == KMB_STATE_TGT)
			ret = (KMB_STATE_TGT << KMB_STATE_SHIFT);
		else if (ctrl->err == KMB_STATE_SRC)
			ret = ((KMB_STATE_SRC << KMB_STATE_SHIFT)
			       | kmb->frame[ctrl->index-2]);

		ctrl->clear(ctrl);
		break;
	default:
		break;
	}

	if (kmb)
		kwp->enqueue(kmb, &kwp->list[KMB_FREE_LIST]);

	return ret;
}

static int kwp_recv_buf(struct tty_struct *tty, const unsigned char *cp,
		       char *fp, int count)
{
	struct n_kwp_data *kwp = tty->disc_data;
	int rcvd = 0;
	unsigned int n;

	down_read(&tty->termios_rwsem);

	while (count) {
		if (free_list_empty(kwp))
			break;
		n = kwp_recv_msg(kwp, cp);
		if (n) {
			if ((n >> KMB_STATE_SHIFT) == KMB_STATE_TGT) {
				continue;
			} else if ((n >> KMB_STATE_SHIFT) == KMB_STATE_SRC) {
				kwp_recv_msg(kwp, (unsigned char *)&n);
				continue;
			}
		}
#if 0
		if (fp)
			fp++;
#endif
		cp++;
		count--;
		rcvd++;
	}
	up_read(&tty->termios_rwsem);

	return rcvd;
}

/*
 * ============================================
 *  callback functions for struct tty_ldisc_ops
 * ============================================
 */

static int n_kwp_open(struct tty_struct *tty)
{
	struct n_kwp_data *kwp;
	struct kmb *kmb;
	int i;

	kwp = kzalloc(sizeof(struct n_kwp_data), GFP_KERNEL);
	if (!kwp)
		goto __kwp_nomem;

	kwp->ctrl[KMB_RX].process = kmb_ctrl_decode_format_type2;
	kwp->ctrl[KMB_RX].clear = kmb_ctrl_clear;
	kwp->ctrl[KMB_TX].process = kmb_ctrl_encode_format_type2;
	kwp->ctrl[KMB_TX].clear = kmb_ctrl_clear;
	mutex_init(&kwp->atomic_read_lock);
	mutex_init(&kwp->output_lock);

	kwp->enqueue = kmb_list_enqueue;
	kwp->dequeue = kmb_list_dequeue;
	kwp->is_empty = kmb_list_empty;

	for (i=0; i<ARRAY_SIZE(kwp->list); i++) {
		INIT_LIST_HEAD(&kwp->list[i].list_head);
		mutex_init(&kwp->list[i].lock);
	}

	kmb = kmb_kalloc();
	if (!kmb)
		goto __kmb_nomem;

	kwp->enqueue(kmb, &kwp->list[KMB_FREE_LIST]);

	kwp->tty = tty;

	tty->disc_data = kwp;
	tty->receive_room = N_TTY_BUF_SIZE;

	return 0;

__kmb_nomem:
	kfree(kwp);
__kwp_nomem:
	return -ENOMEM;
}

static void n_kwp_close(struct tty_struct *tty)
{
	struct n_kwp_data *kwp = tty->disc_data;
	struct kmb *kmb;
	int i=0;

	for (i=0; i<ARRAY_SIZE(kwp->list); i++) {
		if (!kwp->is_empty(&kwp->list[i])) {
			kmb = kwp->dequeue(&kwp->list[i]);
			kmb_kfree(kmb);
			break;
		}
	}

	if (kwp)
		kfree(kwp);

	tty->disc_data = NULL;
}

static void n_kwp_flush_buffer(struct tty_struct *tty)
{
	struct n_kwp_data *kwp = tty->disc_data;
	struct kmb *kmb = NULL;
	int i;

	if (waitqueue_active(&kwp->tty->read_wait))
		wake_up_interruptible(&tty->read_wait);

	for (i=0; i<ARRAY_SIZE(kwp->list); i++) {
		if (!kwp->is_empty(&kwp->list[i])) {
			kmb = kwp->dequeue(&kwp->list[i]);
			kwp->enqueue(kmb, &kwp->list[KMB_FREE_LIST]);
			break;
		}
	}
}

static ssize_t n_kwp_read(struct tty_struct * tty, struct file * file,
			  unsigned char __user *buf, size_t nr)
{
	struct n_kwp_data *kwp = tty->disc_data;
	ssize_t ret = 0;
	DECLARE_WAITQUEUE(wait, current);

	if (file->f_flags & O_NONBLOCK) {
		if (!mutex_trylock(&kwp->atomic_read_lock))
			return -EAGAIN;
	} else {
		if (mutex_lock_interruptible(&kwp->atomic_read_lock))
			return -ERESTARTSYS;
	}
	down_read(&tty->termios_rwsem);
	add_wait_queue(&tty->read_wait, &wait);
	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);

		if (rx_list_empty(kwp)) {
			if (file->f_flags & O_NONBLOCK) {
				ret = -EAGAIN;
				break;
			}
			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
				break;
			}
			up_read(&tty->termios_rwsem);
			schedule();
			down_read(&tty->termios_rwsem);
			continue;
		}
		__set_current_state(TASK_RUNNING);
		ret = kwp_read_msg(tty, buf, nr);
		if (ret)
			break;
	}
	up_read(&tty->termios_rwsem);
	remove_wait_queue(&tty->read_wait, &wait);
	mutex_unlock(&kwp->atomic_read_lock);
	__set_current_state(TASK_RUNNING);

	return ret;
}

static ssize_t n_kwp_write(struct tty_struct *tty, struct file *file,
                           const unsigned char *buf, size_t nr)
{
	struct n_kwp_data *kwp = tty->disc_data;
	DECLARE_WAITQUEUE(wait, current);
	ssize_t n, ret = 0;

	down_read(&tty->termios_rwsem);
	add_wait_queue(&tty->write_wait, &wait);
	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}

		n = kwp_write(tty, buf, nr);
		if ((n < nr) || (n == nr && tx_list_empty(kwp))) {
			ret = n;
			goto break_out;
		}

		if (!tx_list_empty(kwp)) {
			ret = kwp_send(tty);
			nr -= ret;
			if (!nr)
				break;
		}
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}
		up_read(&tty->termios_rwsem);
		schedule();
		down_read(&tty->termios_rwsem);
	}
break_out:
	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&tty->write_wait, &wait);
	up_read(&tty->termios_rwsem);

	return ret;
}

static unsigned int n_kwp_poll(struct tty_struct *tty, struct file *file,
			       poll_table *wait)
{
	struct n_kwp_data *kwp = tty->disc_data;
	unsigned int mask = 0;

	poll_wait(file, &tty->read_wait, wait);

	if (!rx_list_empty(kwp))
		mask |= POLLIN|POLLRDNORM;  /* readable */

	return mask;
}

static int n_kwp_ioctl(struct tty_struct *tty, struct file *file,
		       unsigned int cmd, unsigned long arg)
{
	struct n_kwp_data *kwp = tty->disc_data;
	unsigned int mode;
	int ret = 0;

	switch (cmd) {
	case KWPIOC_GETMODE:
		mode = kwp->mode;
		ret = __put_user(mode, (unsigned int __user *)arg);
		break;
	case KWPIOC_SETMODE:
		ret = __get_user(mode, (unsigned int __user *)arg);
		switch (mode) {
		case INITIALIZE_MODE:
			set_mode(kwp, mode);
			break;
		case DIAGNOSTIC_MODE:
			set_mode(kwp, mode);
			break;
		case TERMINATED_MODE:
			set_mode(kwp, mode);
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	case KWPIOC_RESET:
		/* TODO: is this necessary? */
		break;
	default:
		ret = n_tty_ioctl_helper(tty, file, cmd, arg);
		break;
	}

	return ret;
}

static void n_kwp_write_wakeup(struct tty_struct *tty)
{
	struct n_kwp_data *kwp = tty->disc_data;

	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);

	if (!tx_list_empty(kwp))
		kwp_send(tty);
	else
		set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
}

static void n_kwp_receive_buf(struct tty_struct *tty, const unsigned char *cp,
			      char *fp, int count)
{
	kwp_recv_buf(tty, cp, fp, count);
}

static int n_kwp_receive_buf2(struct tty_struct *tty, const unsigned char *cp,
			      char *fp, int count)
{
	return kwp_recv_buf(tty, cp, fp, count);
}

static struct tty_ldisc_ops tty_ldisc_N_KWP = {
	.magic        = TTY_LDISC_MAGIC,
	.owner        = THIS_MODULE,
	.num          = N_KWP,
	.name         = DRIVER_NAME,
	/* The following routines are called from above. */
	.open         = n_kwp_open,
	.close        = n_kwp_close,
	.flush_buffer = n_kwp_flush_buffer,
	.read         = n_kwp_read,
	.write        = n_kwp_write,
	.poll         = n_kwp_poll,
	.ioctl        = n_kwp_ioctl,
	/* The following routines are called from below. */
	.receive_buf  = n_kwp_receive_buf,
	.write_wakeup = n_kwp_write_wakeup,
	.receive_buf2 = n_kwp_receive_buf2,
};

static int __init n_kwp_init(void)
{
	KWP_INFO("kwp ldisc(%d) registered, debug=0x%02x, %s\n",
		 tty_ldisc_N_KWP.num, kwp_debug, DRIVER_VERSION);
	return tty_register_ldisc(N_KWP, &tty_ldisc_N_KWP);
}

static void __exit n_kwp_exit(void)
{
	KWP_INFO("kwp ldisc(%d) unregistered\n", tty_ldisc_N_KWP.num);
	tty_unregister_ldisc(N_KWP);
}

module_init(n_kwp_init);
module_exit(n_kwp_exit);

MODULE_LICENSE("GPL");
