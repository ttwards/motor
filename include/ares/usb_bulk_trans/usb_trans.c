#include "zephyr/toolchain.h"
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/logging/log.h>
#include <ares/usb_bulk_trans/usb_trans.h>

#include <usb_descriptor.h>

LOG_MODULE_REGISTER(usbd_bulk, LOG_LEVEL_INF);

#define HEART_BEAT_DELAY 2

// ---- 配置参数 ----
#define ARES_USB_VID  0x1209 // 测试用VID
#define ARES_USB_PID  0x0001 // 产品PID
#define BULK_EP_OUT_1 0x01   // OUT端点地址
#define BULK_EP_IN_1  0x81   // IN端点地址
#define BULK_EP_OUT_2 0x02   // OUT端点地址
#define BULK_EP_IN_2  0x82   // IN端点地址

#define CFG_EP_IN1_IDX  0x0
#define CFG_EP_OUT1_IDX 0x1
#define CFG_EP_IN2_IDX  0x2
#define CFG_EP_OUT2_IDX 0x3

// 根据USB速度选择最大包大小
#if defined(CONFIG_USB_DEVICE_HS)
#define BULK_MPS  512 // 高速USB
#define USB_SPEED USBD_SPEED_HS
#else
#define BULK_MPS  64 // 全速USB
#define USB_SPEED USBD_SPEED_FS
#endif

K_HEAP_DEFINE(ares_usb_heap, 12 * BULK_MPS); // USB堆栈

struct usb_msg {
	uint8_t *buf; // 接收数据缓冲区
	size_t len;   // 数据长度
	bool free;
};

#define FUNC_REPL_SIZE 10

#define MAX_PACK_COUNT 8
#define MAX_FUNC_COUNT MAX_PACK_COUNT

uint8_t func_tx_bckup_cnt = 0;
K_FIFO_DEFINE(func_tx_bckup_fifo);

static void usb_trans_heart_beat(struct k_timer *timer);
K_TIMER_DEFINE(heart_beat_timer, usb_trans_heart_beat, NULL);

bool online = false;
uint32_t last_heart_beat = 0;
uint32_t last_receive = 0;

static uint8_t current_ep = 0x81;

static int8_t func_cnt = 0;
static func_mapping_t func_table[MAX_PACK_COUNT];

static int8_t sync_pack_cnt = 0;
static usb_sync_pack_t sync_pack[MAX_PACK_COUNT];

K_MSGQ_DEFINE(usb_rx_msgq, sizeof(struct usb_msg), 4, 4); // 存储指针
K_MSGQ_DEFINE(usb_tx_msgq, sizeof(struct usb_msg), 4, 4); // 存储指针

static void ares_rx_handler(struct k_work *item);
static void ares_tx_handler(struct k_work *item);

static void ares_usb_thread_entry(void);

K_THREAD_DEFINE(ares_usb_tid, 768, ares_usb_thread_entry, NULL, NULL, NULL, 0, 0, 0);

K_SEM_DEFINE(ares_usb_sem, 0, 1); // USB信号量

K_THREAD_STACK_DEFINE(ares_usb_stack, 1536);
// static void usb_rx_thread_entry(void);
// K_THREAD_DEFINE(usb_rx_tid, 1024, usb_rx_thread_entry, NULL, NULL, NULL, K_PRIO_COOP(7), 0, 0);

static void error_handle(uint16_t req_id, uint16_t error)
{
	uint8_t *buf = ALLOC_MEM(ERROR_FRAME_LENGTH);
	GET_16BITS(buf, ERROR_HEAD_IDX) = ERROR_FRAME_HEAD;
	GET_8BITS(buf, ERROR_ID_IDX) = req_id;
	GET_16BITS(buf, ERROR_CODE_IDX) = error;
	if (error != HEART_BEAT) {
		// LOG_ERR("Error code: %x, Request ID: %x", error, req_id);
	} else {
		last_heart_beat = k_cycle_get_32();
	}
	struct usb_msg msg = {
		.buf = buf,
		.len = ERROR_FRAME_LENGTH,
		.free = true,
	};
	k_msgq_put(&usb_tx_msgq, &msg, K_NO_WAIT);
	k_sem_give(&ares_usb_sem);
}

static bool proceed_tx_bck(uint8_t ID)
{
	// 创建一个临时队列
	struct k_fifo temp_fifo;
	k_fifo_init(&temp_fifo);
	bool found = false;

	// 临时存储所有数据项
	uint8_t *item;
	while ((item = k_fifo_get(&func_tx_bckup_fifo, K_NO_WAIT)) != NULL) {
		if (GET_8BITS(item, REPL_REQ_ID_IDX + 4) == ID && !found) {
			// 找到匹配的项目
			struct usb_msg msg = {
				.buf = item + SYNC_FRAME_LENGTH_OFFSET,
				.len = FUNC_REPL_SIZE,
				.free = true,
			};
			k_msgq_put(&usb_tx_msgq, &msg, K_NO_WAIT);
			k_sem_give(&ares_usb_sem);

			found = true;
			continue;
		} else {
			// 保存到临时队列
			k_fifo_put(&temp_fifo, item);
		}
	}

	// 将所有项目放回原始队列
	while ((item = k_fifo_get(&temp_fifo, K_NO_WAIT)) != NULL) {
		k_fifo_put(&func_tx_bckup_fifo, item);
	}

	return found;
}

static usb_sync_pack_t *find_pack(uint16_t ID)
{
	for (int i = 0; i < sync_pack_cnt; i++) {
		if (ID == sync_pack[i].ID) {
			return &sync_pack[i];
		}
	}
	return NULL;
}

static func_mapping_t *find_func(uint16_t ID)
{
	for (int i = 0; i < func_cnt; i++) {
		if (ID == func_table[i].id) {
			return &func_table[i];
		}
	}
	return NULL;
}

static void usb_trans_heart_beat(struct k_timer *timer)
{
	if (online) {
		if (k_cyc_to_ms_near32(k_cycle_get_32() - last_heart_beat) <= HEART_BEAT_DELAY &&
		    k_cyc_to_ms_near32(k_cycle_get_32() - last_receive) >= HEART_BEAT_DELAY) {
			LOG_ERR("Connection lost for %d ms.",
				k_cyc_to_ms_near32(k_cycle_get_32() - last_receive));
			online = false;
			last_heart_beat = k_cycle_get_32();
		}
	}
	error_handle(HEARTBEAT_ID, HEART_BEAT);
	last_heart_beat = k_cycle_get_32();
}

static void parse_sync(uint8_t data[], usb_sync_pack_t *pack, uint8_t len)
{
	LOG_HEXDUMP_DBG(data, len, "SYNC frame received");
	int data_len;
	data_len = len - 8;
	if (pack->cb != NULL) {
		pack->cb(SYNC_PACK_STATUS_WRITE);
	}
	memcpy(pack->data, data + SYNC_DATA_IDX, data_len);
	return;
}

static void parse_func(uint8_t data[], int8_t len)
{
	func_mapping_t *map = find_func(GET_16BITS(data, FUNC_ID_IDX));
	LOG_HEXDUMP_DBG(data, len, "FUNC frame received");
	if (map == NULL) {
		LOG_ERR("Cannot find corresponding ID in func mappings.");
		// error_handle(0, UNKNOWN_FUNC);
		return;
	}
	LOG_DBG("Received Arguments: %x %x %x", GET_32BITS(data, FUNC_ARG1_IDX),
		GET_32BITS(data, FUNC_ARG2_IDX), GET_32BITS(data, FUNC_ARG3_IDX));
	uint32_t ret = map->cb(GET_32BITS(data, FUNC_ARG1_IDX), GET_32BITS(data, FUNC_ARG2_IDX),
			       GET_32BITS(data, FUNC_ARG3_IDX));

	if (func_tx_bckup_cnt == MAX_FUNC_COUNT) {
		uint8_t *rm_frame = k_fifo_get(&func_tx_bckup_fifo, K_NO_WAIT);
		if (rm_frame != NULL) {
			func_tx_bckup_cnt--;
			FREE_MEM(rm_frame);
		} else {
			LOG_ERR("Failed to free memory for FUNC frame.");
			error_handle(GET_16BITS(data, FUNC_REQ_IDX), CLEANED_PACK);
			return;
		}
	}

	uint8_t *repl_frame = ALLOC_MEM(FUNC_REPL_SIZE + 4);
	repl_frame += 4;

	GET_16BITS(repl_frame, REPL_HEAD_IDX) = REPL_FRAME_HEAD;
	GET_16BITS(repl_frame, REPL_FUNC_ID_IDX) = GET_16BITS(data, FUNC_ID_IDX);
	GET_32BITS(repl_frame, REPL_RET_IDX) = (uint32_t)ret;
	GET_8BITS(repl_frame, REPL_REQ_ID_IDX) = GET_16BITS(data, FUNC_REQ_IDX);

	k_fifo_put(&func_tx_bckup_fifo, repl_frame - 4);

	func_tx_bckup_cnt++;

	struct usb_msg msg = {
		.buf = repl_frame,
		.len = FUNC_REPL_SIZE,
		.free = false,
	};
	ret = k_msgq_put(&usb_tx_msgq, &msg, K_NO_WAIT);
	k_sem_give(&ares_usb_sem);

	if (ret != 0) {
		// LOG_ERR("Failed to put FUNC frame into FIFO. (%d)", ret);
		FREE_MEM(repl_frame - 4);
		return;
	}

	return;
}

static void parse_error(uint8_t data[], int8_t len)
{
	uint8_t req_id = GET_8BITS(data, ERROR_ID_IDX);
	if (req_id == HEARTBEAT_ID) {
		return;
	}
	if (!proceed_tx_bck(req_id)) {
		usb_sync_pack_t *pack = find_pack(sys_be16_to_cpu(GET_16BITS(data, ERROR_ID_IDX)));
		if (pack == NULL) {
			LOG_ERR("Cannot find corresponding ID from error "
				"frame.");
			error_handle(req_id, CLEANED_PACK);
			return;
		}
		if (pack->cb != NULL) {
			pack->cb(SYNC_PACK_STATUS_READ);
		}
		usb_trans_sync_flush(pack);
	}
	LOG_INF("Error frame received: id =%d; code=%d", req_id, GET_16BITS(data, ERROR_CODE_IDX));
	return;
}

void usb_trans_sync_flush(usb_sync_pack_t *pack)
{
	if (pack == NULL) {
		LOG_ERR("Sync pack is NULL.");
		return;
	}
	memcpy(pack->buf + SYNC_DATA_IDX, pack->data, pack->len);
	GET_16BITS(pack->buf, SYNC_HEAD_IDX) = SYNC_FRAME_HEAD;
	GET_16BITS(pack->buf, SYNC_ID_IDX) = pack->ID;

	LOG_HEXDUMP_DBG(pack->buf, pack->len + SYNC_FRAME_LENGTH_OFFSET, "Sync data to send:");

	struct usb_msg msg = {
		.buf = pack->buf,
		.len = pack->len + SYNC_FRAME_LENGTH_OFFSET,
		.free = false,
	};
	int ret = k_msgq_put(&usb_tx_msgq, &msg, K_NO_WAIT);
	k_sem_give(&ares_usb_sem);

	if (ret != 0) {
		// LOG_ERR("Failed to put SYNC frame into FIFO. (%d)", ret);
		return;
	}
}

usb_sync_pack_t *usb_trans_sync_add(usb_trans_data_t *data, uint16_t ID, size_t len,
				    usb_trans_cb_t cb)
{
	if (sync_pack_cnt >= MAX_PACK_COUNT) {
		LOG_ERR("Sync pack count exceeds maximum.");
		return NULL;
	}
	sync_pack[sync_pack_cnt].data = data;
	sync_pack[sync_pack_cnt].ID = sys_cpu_to_be16(ID);
	sync_pack[sync_pack_cnt].len = len;
	sync_pack[sync_pack_cnt].cb = cb;
	sync_pack[sync_pack_cnt].buf = ALLOC_MEM(len + SYNC_FRAME_LENGTH_OFFSET);
	if (sync_pack[sync_pack_cnt].buf == NULL) {
		LOG_ERR("Failed to allocate memory for SYNC frame.");
		return NULL;
	}
	sync_pack_cnt++;
	if (cb) {
		cb(SYNC_PACK_STATUS_READ);
	}
	usb_trans_sync_flush(&sync_pack[sync_pack_cnt - 1]);

	return &sync_pack[sync_pack_cnt - 1];
}

void usb_trans_func_add(uint16_t header, usb_trans_func_t cb)
{
	if (cb == NULL) {
		LOG_ERR("Callback function is NULL.");
		return;
	}
	if (func_cnt >= MAX_PACK_COUNT) {
		LOG_ERR("Func count exceeds maximum.");
		return;
	}
	func_table[func_cnt].id = sys_cpu_to_be16(header);
	func_table[func_cnt].cb = cb;
	func_cnt++;
}

void usb_trans_func_remove(uint16_t header)
{
	for (int i = 0; i < func_cnt; i++) {
		if (func_table[i].id == header) {
			func_cnt--;
			func_table[i] = func_table[func_cnt];
			return;
		}
	}
	LOG_ERR("Cannot find corresponding ID from func table.");
	return;
}

static void ares_bulk_out_cb(uint8_t ep, enum usb_dc_ep_cb_status_code ep_status)
{
	uint32_t bytes_to_read;

	usb_dc_ep_read(ep, NULL, 0, &bytes_to_read);

	uint8_t *buffer = ALLOC_MEM(bytes_to_read);

	if (!buffer) {
		uint8_t buf[bytes_to_read];
		usb_dc_ep_read(ep, buf, bytes_to_read, NULL);
		usb_dc_ep_enable(ep);
		return;
	}

	usb_dc_ep_read(ep, buffer, bytes_to_read, NULL);

	struct usb_msg msg = {
		.buf = buffer,
		.len = bytes_to_read,
		.free = true,
	};

	k_msgq_put(&usb_rx_msgq, &msg, K_NO_WAIT);
	usb_dc_ep_enable(ep);

	last_receive = k_cycle_get_32();

	k_sem_give(&ares_usb_sem);
}

static void ares_bulk_in_cb(uint8_t ep, enum usb_dc_ep_cb_status_code ep_status)
{
	usb_dc_ep_enable(ep);
	return;
}

static void ares_bulk_send(uint8_t ep, struct usb_msg *msg)
{
	LOG_DBG("ep 0x%x", ep);
	// LOG_INF("msg buf %p, len %d", msg->buf, msg->len);
	LOG_HEXDUMP_DBG(msg->buf, msg->len, "USB TX");

	if (online) {
		if (usb_dc_ep_write(ep, msg->buf, msg->len, NULL) == -EAGAIN) {
			online = false;
		}
	}

	if (msg->free) {
		FREE_MEM(msg->buf);
	}
}

static void ares_tx_handler(struct k_work *item)
{
	ARG_UNUSED(item);
	struct usb_msg msg;
	while (k_msgq_num_used_get(&usb_tx_msgq) > 0) {
		k_msgq_get(&usb_tx_msgq, &msg, K_NO_WAIT);

		if (msg.len > 0) {
			ares_bulk_send(current_ep, &msg);
			current_ep = (current_ep == BULK_EP_IN_1) ? BULK_EP_IN_2 : BULK_EP_IN_1;
		} else {
			LOG_ERR("Invalid message length: %d", msg.len);
			FREE_MEM(msg.buf);
			continue;
		}
	}
}

static void ares_rx_handler(struct k_work *item)
{
	ARG_UNUSED(item);
	struct usb_msg msg;
	while (k_msgq_num_used_get(&usb_rx_msgq) > 0) {
		k_msgq_get(&usb_rx_msgq, &msg, K_NO_WAIT);

		switch (GET_16BITS(msg.buf, 0)) {
		case FUNC_FRAME_HEAD:
			if (msg.len != FUNC_FRAME_LENGTH) {
				LOG_ERR("FUNC frame length mismatch: %d vs %d", FUNC_FRAME_LENGTH,
					msg.len);
				error_handle(GET_16BITS(msg.buf, FUNC_ID_IDX), UNKNOWN_TAIL);
				goto free;
			}
			parse_func(msg.buf, msg.len);
			break;
		case SYNC_FRAME_HEAD:
			for (int i = 0; i < sync_pack_cnt; i++) {
				if (sync_pack[i].ID == GET_16BITS(msg.buf, SYNC_ID_IDX)) {
					if (sync_pack[i].len + SYNC_FRAME_LENGTH_OFFSET !=
					    msg.len) {
						LOG_ERR("SYNC frame length mismatch: %d vs %d",
							sync_pack[i].len + SYNC_FRAME_LENGTH_OFFSET,
							msg.len);
						error_handle(sync_pack[i].ID, UNKNOWN_TAIL);
						goto free;
					}
					parse_sync(msg.buf, &sync_pack[i], msg.len);
					goto free;
				}
			}
			LOG_ERR("SYNC frame received: ID %x", GET_16BITS(msg.buf, SYNC_ID_IDX));
			break;
		case ERROR_FRAME_HEAD:
			if (msg.len != ERROR_FRAME_LENGTH) {
				LOG_ERR("ERROR frame length mismatch: %d vs %d", ERROR_FRAME_LENGTH,
					msg.len);
				error_handle(GET_16BITS(msg.buf, ERROR_ID_IDX), UNKNOWN_TAIL);
				goto free;
			}
			parse_error(msg.buf, msg.len);
			break;
		default:
			LOG_ERR("Unknown frame received: %x", GET_16BITS(msg.buf, 0));
			// LOG_HEXDUMP_ERR(msg.buf, msg.len, "Unknown frame");
			break;
		}
free:
		FREE_MEM(msg.buf);
	}
}

static struct usb_ep_cfg_data ares_ep_cfg[] = {
	{
		.ep_cb = ares_bulk_in_cb,
		.ep_addr = BULK_EP_IN_1,
	},
	{
		.ep_cb = ares_bulk_out_cb,
		.ep_addr = BULK_EP_OUT_1,
	},
	{
		.ep_cb = ares_bulk_in_cb,
		.ep_addr = BULK_EP_IN_2,
	},
	{
		.ep_cb = ares_bulk_out_cb,
		.ep_addr = BULK_EP_OUT_2,
	},
};

static void cb_usb_status(struct usb_cfg_data *cfg, enum usb_dc_status_code cb_status,
			  const uint8_t *param)
{
	ARG_UNUSED(param);
	ARG_UNUSED(cfg);

	/* Check the USB status and do needed action if required */
	switch (cb_status) {
	case USB_DC_ERROR:
		LOG_INF("USB device error");
		break;
	case USB_DC_RESET:
		LOG_INF("USB device reset detected");
		break;
	case USB_DC_CONNECTED:
		LOG_INF("USB device connected");
		break;
	case USB_DC_CONFIGURED:
		LOG_INF("USB device configured");
		online = true;
		break;
	case USB_DC_DISCONNECTED:
		LOG_INF("USB device disconnected");
		online = false;
		break;
	case USB_DC_SUSPEND:
		LOG_INF("USB device suspended");
		break;
	case USB_DC_RESUME:
		LOG_INF("USB device resumed");
		break;
	case USB_DC_INTERFACE:
		ares_bulk_send(ares_ep_cfg[CFG_EP_IN1_IDX].ep_addr, 0);
		LOG_INF("USB interface configured");
		break;
	case USB_DC_SET_HALT:
		LOG_INF("Set Feature ENDPOINT_HALT");
		break;
	case USB_DC_CLEAR_HALT:
		LOG_INF("Clear Feature ENDPOINT_HALT");
		if (*param == ares_ep_cfg[CFG_EP_IN1_IDX].ep_addr) {
			ares_bulk_send(ares_ep_cfg[CFG_EP_IN1_IDX].ep_addr, 0);
		}
		break;
	case USB_DC_UNKNOWN:
	default:
		LOG_INF("USB unknown state");
		break;
	}
}

static void ares_interface_config(struct usb_desc_header *head, uint8_t bInterfaceNumber);

int ares_vendor_handle_req(struct usb_setup_packet *setup, int32_t *len, uint8_t **data)
{
	LOG_INF("Class request: bRequest 0x%x bmRequestType 0x%x len %d", setup->bRequest,
		setup->bmRequestType, *len);

	if (setup->RequestType.recipient != USB_REQTYPE_RECIPIENT_DEVICE) {
		return -ENOTSUP;
	}

	if (usb_reqtype_is_to_device(setup) && setup->bRequest == 0x5b) {
		LOG_INF("Host-to-Device, data %p", *data);
		/*
		 * Copy request data in loopback_buf buffer and reuse
		 * it later in control device-to-host transfer.
		 */
		return 0;
	}

	if ((usb_reqtype_is_to_host(setup)) && (setup->bRequest == 0x5c)) {
		LOG_INF("Device-to-Host, wLength %d, data %p", setup->wLength, *data);
		*len = setup->wLength;
		return 0;
	}

	return -ENOTSUP;
}

static void ares_interface_config(struct usb_desc_header *head, uint8_t bInterfaceNumber);

USBD_CLASS_DESCR_DEFINE(primary, 0) struct {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor if0_in1_ep;
	struct usb_ep_descriptor if0_out1_ep;
	struct usb_ep_descriptor if0_in2_ep;
	struct usb_ep_descriptor if0_out2_ep;
} __packed ares_usb_desc_if = {
	.if0 = INITIALIZER_IF(ARRAY_SIZE(ares_ep_cfg), USB_BCC_VENDOR),
	.if0_in1_ep = INITIALIZER_IF_EP(BULK_EP_IN_1, USB_DC_EP_BULK, BULK_MPS, 0),
	.if0_out1_ep = INITIALIZER_IF_EP(BULK_EP_OUT_1, USB_DC_EP_BULK, BULK_MPS, 0),
	.if0_in2_ep = INITIALIZER_IF_EP(BULK_EP_IN_2, USB_DC_EP_BULK, BULK_MPS, 0),
	.if0_out2_ep = INITIALIZER_IF_EP(BULK_EP_OUT_2, USB_DC_EP_BULK, BULK_MPS, 0),
};

USBD_DEFINE_CFG_DATA(ares_config) = {
	.usb_device_description = NULL,
	.interface_config = ares_interface_config,
	.interface_descriptor = &ares_usb_desc_if.if0,
	.cb_usb_status = cb_usb_status,
	.interface =
		{
			.class_handler = NULL,
			.custom_handler = NULL,
			.vendor_handler = ares_vendor_handle_req,
		},
	.num_endpoints = ARRAY_SIZE(ares_ep_cfg),
	.endpoint = ares_ep_cfg,
};

static void ares_interface_config(struct usb_desc_header *head, uint8_t bInterfaceNumber)
{
	ARG_UNUSED(head);

	ares_usb_desc_if.if0.bInterfaceNumber = bInterfaceNumber;
}

static void ares_usb_thread_entry(void)
{
	while (1) {
		ares_tx_handler(NULL);
		ares_rx_handler(NULL);
		k_sem_take(&ares_usb_sem, K_FOREVER);
	}
}

void ares_usb_transfer_init(void)
{
	int ret;

	// 初始化USB设备
	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}

	k_thread_start(ares_usb_tid);
	k_thread_name_set(ares_usb_tid, "ares_usb_txrx_handler");
	k_timer_start(&heart_beat_timer, K_MSEC(HEART_BEAT_DELAY), K_MSEC(HEART_BEAT_DELAY));

	LOG_INF("USB initialized successfully");
}