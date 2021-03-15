/*
 * Copyright (c) 2019-2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <zephyr.h>
#include <arch/cpu.h>
#include <sys/byteorder.h>
#include <logging/log.h>
#include <sys/util.h>
#include <drivers/ipm.h>

#include <openamp/open_amp.h>
#include <metal/sys.h>
#include <metal/device.h>
#include <metal/alloc.h>

#include <ipc/rpmsg_service.h>

#include <net/buf.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/hci.h>
#include <bluetooth/buf.h>
#include <bluetooth/hci_raw.h>

#define LOG_LEVEL LOG_LEVEL_INFO
#define LOG_MODULE_NAME hci_rpmsg
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static int endpoint_id;

static K_THREAD_STACK_DEFINE(tx_thread_stack, CONFIG_BT_HCI_TX_STACK_SIZE);
static struct k_thread tx_thread_data;
static K_FIFO_DEFINE(tx_queue);

#define HCI_RPMSG_CMD 0x01
#define HCI_RPMSG_ACL 0x02
#define HCI_RPMSG_SCO 0x03
#define HCI_RPMSG_EVT 0x04
#define HCI_RPMSG_ISO 0x05

static struct net_buf *hci_rpmsg_cmd_recv(uint8_t *data, size_t remaining)
{
	struct bt_hci_cmd_hdr *hdr = (void *)data;
	struct net_buf *buf;

	if (remaining < sizeof(*hdr)) {
		LOG_ERR("Not enought data for command header");
		return NULL;
	}

	buf = bt_buf_get_tx(BT_BUF_CMD, K_NO_WAIT, hdr, sizeof(*hdr));
	if (buf) {
		data += sizeof(*hdr);
		remaining -= sizeof(*hdr);
	} else {
		LOG_ERR("No available command buffers!");
		return NULL;
	}

	if (remaining != hdr->param_len) {
		LOG_ERR("Command payload length is not correct");
		net_buf_unref(buf);
		return NULL;
	}

	LOG_DBG("len %u", hdr->param_len);
	net_buf_add_mem(buf, data, remaining);

	return buf;
}

static struct net_buf *hci_rpmsg_acl_recv(uint8_t *data, size_t remaining)
{
	struct bt_hci_acl_hdr *hdr = (void *)data;
	struct net_buf *buf;

	if (remaining < sizeof(*hdr)) {
		LOG_ERR("Not enought data for ACL header");
		return NULL;
	}

	buf = bt_buf_get_tx(BT_BUF_ACL_OUT, K_NO_WAIT, hdr, sizeof(*hdr));
	if (buf) {
		data += sizeof(*hdr);
		remaining -= sizeof(*hdr);
	} else {
		LOG_ERR("No available ACL buffers!");
		return NULL;
	}

	if (remaining != sys_le16_to_cpu(hdr->len)) {
		LOG_ERR("ACL payload length is not correct");
		net_buf_unref(buf);
		return NULL;
	}

	LOG_DBG("len %u", remaining);
	net_buf_add_mem(buf, data, remaining);

	return buf;
}

static struct net_buf *hci_rpmsg_iso_recv(uint8_t *data, size_t remaining)
{
	struct bt_hci_iso_hdr *hdr = (void *)data;
	struct net_buf *buf;

	if (remaining < sizeof(*hdr)) {
		LOG_ERR("Not enough data for ISO header");
		return NULL;
	}

	buf = bt_buf_get_tx(BT_BUF_ISO_OUT, K_NO_WAIT, hdr, sizeof(*hdr));
	if (buf) {
		data += sizeof(*hdr);
		remaining -= sizeof(*hdr);
	} else {
		LOG_ERR("No available ISO buffers!");
		return NULL;
	}

	if (remaining != sys_le16_to_cpu(hdr->len)) {
		LOG_ERR("ISO payload length is not correct");
		net_buf_unref(buf);
		return NULL;
	}

	LOG_DBG("len %zu", remaining);
	net_buf_add_mem(buf, data, remaining);

	return buf;
}

static void hci_rpmsg_rx(uint8_t *data, size_t len)
{
	uint8_t pkt_indicator;
	struct net_buf *buf = NULL;
	size_t remaining = len;

	LOG_HEXDUMP_DBG(data, len, "RPMSG data:");

	pkt_indicator = *data++;
	remaining -= sizeof(pkt_indicator);

	switch (pkt_indicator) {
	case HCI_RPMSG_CMD:
		buf = hci_rpmsg_cmd_recv(data, remaining);
		break;

	case HCI_RPMSG_ACL:
		buf = hci_rpmsg_acl_recv(data, remaining);
		break;

	case HCI_RPMSG_ISO:
		buf = hci_rpmsg_iso_recv(data, remaining);
		break;

	default:
		LOG_ERR("Unknown HCI type %u", pkt_indicator);
		return;
	}

	if (buf) {
		net_buf_put(&tx_queue, buf);

		LOG_HEXDUMP_DBG(buf->data, buf->len, "Final net buffer:");
	}
}

static void tx_thread(void *p1, void *p2, void *p3)
{
	while (1) {
		struct net_buf *buf;
		int err;

		/* Wait until a buffer is available */
		buf = net_buf_get(&tx_queue, K_FOREVER);
		/* Pass buffer to the stack */
		err = bt_send(buf);
		if (err) {
			LOG_ERR("Unable to send (err %d)", err);
			net_buf_unref(buf);
		}

		/* Give other threads a chance to run if tx_queue keeps getting
		 * new data all the time.
		 */
		k_yield();
	}
}

static int hci_rpmsg_send(struct net_buf *buf)
{
	uint8_t pkt_indicator;

	LOG_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf),
		buf->len);

	LOG_HEXDUMP_DBG(buf->data, buf->len, "Controller buffer:");

	switch (bt_buf_get_type(buf)) {
	case BT_BUF_ACL_IN:
		pkt_indicator = HCI_RPMSG_ACL;
		break;
	case BT_BUF_EVT:
		pkt_indicator = HCI_RPMSG_EVT;
		break;
	case BT_BUF_ISO_IN:
		pkt_indicator = HCI_RPMSG_ISO;
		break;
	default:
		LOG_ERR("Unknown type %u", bt_buf_get_type(buf));
		net_buf_unref(buf);
		return -EINVAL;
	}
	net_buf_push_u8(buf, pkt_indicator);

	LOG_HEXDUMP_DBG(buf->data, buf->len, "Final HCI buffer:");
	rpmsg_service_send(endpoint_id, buf->data, buf->len);

	net_buf_unref(buf);

	return 0;
}

#if defined(CONFIG_BT_CTLR_ASSERT_HANDLER)
void bt_ctlr_assert_handle(char *file, uint32_t line)
{
	LOG_ERR("Controller assert in: %s at %d", file, line);
}
#endif /* CONFIG_BT_CTLR_ASSERT_HANDLER */

int endpoint_cb(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src,
		void *priv)
{
	LOG_INF("Received message of %u bytes.", len);
	hci_rpmsg_rx((uint8_t *) data, len);

	return RPMSG_SUCCESS;
}

#if 1

#if 1
#include <mpsl.h>
#include <mpsl_timeslot.h>

#define TIMESLOT_REQUEST_DISTANCE_US (1000000)
#define TIMESLOT_LENGTH_US           (200)

static bool request_in_cb = false;

/* Timeslot requests */
static mpsl_timeslot_request_t timeslot_request_earliest = {
	.request_type = MPSL_TIMESLOT_REQ_TYPE_EARLIEST,
	.params.earliest.hfclk = MPSL_TIMESLOT_HFCLK_CFG_NO_GUARANTEE,
	.params.earliest.priority = MPSL_TIMESLOT_PRIORITY_NORMAL,
	.params.earliest.length_us = TIMESLOT_LENGTH_US,
	.params.earliest.timeout_us = 1000000
};

static mpsl_timeslot_signal_return_param_t signal_callback_return_param;

static mpsl_timeslot_signal_return_param_t *mpsl_timeslot_callback(
	mpsl_timeslot_session_id_t session_id,
	uint32_t signal_type)
{
	(void) session_id; /* unused parameter */

	mpsl_timeslot_signal_return_param_t *p_ret_val = NULL;

	switch (signal_type) {
	case MPSL_TIMESLOT_SIGNAL_START:
		printk("MPSL_TIMESLOT_SIGNAL_START\n");
		if (request_in_cb) {
			// /* Request new timeslot when callback returns */
			// signal_callback_return_param.params.request.p_next =
			// 	&timeslot_request_normal;
			// signal_callback_return_param.callback_action =
			// 	MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST;
		} else {
			/* No return action, so timeslot will be ended */
			signal_callback_return_param.params.request.p_next =
				NULL;
			signal_callback_return_param.callback_action =
				MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
		}

		p_ret_val = &signal_callback_return_param;
		break;
	case MPSL_TIMESLOT_SIGNAL_SESSION_IDLE:
		printk("MPSL_TIMESLOT_SIGNAL_SESSION_IDLE\n");
		break;
	case MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED:
		printk("MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED\n");
		break;
	default:
		printk("unexpected signal: %u\n", signal_type);
		break;
	}
	return p_ret_val;
}
#endif

#include <tinycbor/cbor.h>
#include <nrf_rpc_cbor.h>

#define MAX_ENCODED_LEN 16

enum rpc_command {
	MATH_COMMAND_INC = 0x01,
};


/* Defines a group that contains functions implemented in this
 * sample. Second parameter have to be the same in both remote
 * and local side.
 */
NRF_RPC_GROUP_DEFINE(math_group, "sample_math", NULL, NULL, NULL);


static void remote_inc_handler(CborValue *value, void* handler_data)
{
        int err;
        int input = 0;
        int output;
        struct nrf_rpc_cbor_ctx ctx;

        /* Parsing the input */

        if (cbor_value_is_integer(value)) {
                cbor_value_get_int(value, &input);
        }

        nrf_rpc_cbor_decoding_done(value);

        /* Actual hard work is done in below line */

#if 1
	mpsl_timeslot_session_id_t session_id = 0xFFu;
	err = mpsl_timeslot_session_open(
					mpsl_timeslot_callback,
					&session_id);
	if (err)
	{
		printk("[Err] mpsl_timeslot_session_open\n");
	}


	err = mpsl_timeslot_request(
					session_id,
					&timeslot_request_earliest);
	if (err)
	{
		printk("[Err] mpsl_timeslot_request\n");
	}
#endif

        output = input + 1;

        /* Encoding and sending the response */

        NRF_RPC_CBOR_ALLOC(ctx, MAX_ENCODED_LEN);

        cbor_encode_int(&ctx.encoder, output);

        err = nrf_rpc_cbor_rsp(&ctx);

        if (err < 0) {
                // fatal_error(err);
        }
}

NRF_RPC_CBOR_CMD_DECODER(math_group, remote_inc_handler,
                         MATH_COMMAND_INC, remote_inc_handler, NULL);
#endif

static void err_handler(const struct nrf_rpc_err_report *report)
{
	printk("nRF RPC error %d ocurred. See nRF RPC logs for more details.",
	       report->code);
	k_oops();
}


static int serialization_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	int err;

	printk("Init begin [NET]\n");

	err = nrf_rpc_init(err_handler);
	if (err) {
		return -NRF_EINVAL;
	}

	printk("Init done [NET]\n");

	return 0;
}


SYS_INIT(serialization_init, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);

void main(void)
{
	int err;

	/* incoming events and data from the controller */
	static K_FIFO_DEFINE(rx_queue);

	LOG_DBG("Start");

	/* Enable the raw interface, this will in turn open the HCI driver */
	bt_enable_raw(&rx_queue);

	/* Spawn the TX thread and start feeding commands and data to the
	 * controller
	 */
	k_thread_create(&tx_thread_data, tx_thread_stack,
			K_THREAD_STACK_SIZEOF(tx_thread_stack), tx_thread,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	k_thread_name_set(&tx_thread_data, "HCI rpmsg TX");

	while (1) {
		struct net_buf *buf;

		buf = net_buf_get(&rx_queue, K_FOREVER);
		err = hci_rpmsg_send(buf);
		if (err) {
			LOG_ERR("Failed to send (err %d)", err);
		}
	}
}

/* Make sure we register endpoint before RPMsg Service is initialized. */
int register_endpoint(const struct device *arg)
{
	int status;

	status = rpmsg_service_register_endpoint("nrf_bt_hci", endpoint_cb);

	if (status < 0) {
		LOG_ERR("Registering endpoint failed with %d", status);
		return status;
	}

	endpoint_id = status;

	return 0;
}

SYS_INIT(register_endpoint, POST_KERNEL, CONFIG_RPMSG_SERVICE_EP_REG_PRIORITY);
