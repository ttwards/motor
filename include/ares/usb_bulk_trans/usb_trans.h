#ifndef usb_TRANS_H_
#define usb_TRANS_H_

#include <stdint.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/usb_device.h>

#define INITIALIZER_IF(num_ep, iface_class)                                                        \
	{                                                                                          \
		.bLength = sizeof(struct usb_if_descriptor),                                       \
		.bDescriptorType = USB_DESC_INTERFACE,                                             \
		.bInterfaceNumber = 0,                                                             \
		.bAlternateSetting = 0,                                                            \
		.bNumEndpoints = num_ep,                                                           \
		.bInterfaceClass = iface_class,                                                    \
		.bInterfaceSubClass = 0,                                                           \
		.bInterfaceProtocol = 0,                                                           \
		.iInterface = 0,                                                                   \
	}

#define INITIALIZER_IF_EP(addr, attr, mps, interval)                                               \
	{                                                                                          \
		.bLength = sizeof(struct usb_ep_descriptor),                                       \
		.bDescriptorType = USB_DESC_ENDPOINT,                                              \
		.bEndpointAddress = addr,                                                          \
		.bmAttributes = attr,                                                              \
		.wMaxPacketSize = sys_cpu_to_le16(mps),                                            \
		.bInterval = interval,                                                             \
	}

#define ALLOC_MEM(size) k_heap_aligned_alloc(&ares_usb_heap, 4, size, K_NO_WAIT)
#define FREE_MEM(buf)   k_heap_free(&ares_usb_heap, buf)

void ares_usb_transfer_init(void);

#define SYNC_FRAME_HEAD          0x5A5A
#define SYNC_HEAD_IDX            0
#define SYNC_ID_IDX              2
#define SYNC_DATA_IDX            4
#define SYNC_FRAME_LENGTH_OFFSET 4

#define FUNC_FRAME_HEAD   0xFECA
#define FUNC_HEAD_IDX     0
#define FUNC_ID_IDX       2
#define FUNC_ARG1_IDX     4
#define FUNC_ARG2_IDX     8
#define FUNC_ARG3_IDX     12
#define FUNC_REQ_IDX      16
#define FUNC_FRAME_LENGTH 18

#define ERROR_FRAME_HEAD   0xDECA
#define ERROR_HEAD_IDX     0
#define ERROR_ID_IDX       2
#define ERROR_CODE_IDX     4
#define ERROR_FRAME_LENGTH 6

#define REPL_FRAME_HEAD   0xDEC0
#define REPL_HEAD_IDX     0
#define REPL_FUNC_ID_IDX  2
#define REPL_RET_IDX      4
#define REPL_REQ_ID_IDX   8
#define REPL_FRAME_LENGTH 10

#define HEARTBEAT_ID 0xFF

#define UNKNOWN_HEAD sys_cpu_to_be16(BIT(1))
#define UNKNOWN_TAIL sys_cpu_to_be16(BIT(2))
#define UNKNOWN_FUNC sys_cpu_to_be16(BIT(3))
#define UNKNOWN_SYNC sys_cpu_to_be16(BIT(4))
#define TOOHIGH_FREQ sys_cpu_to_be16(BIT(5))
#define CLEANED_PACK sys_cpu_to_be16(BIT(6))
#define REQUEST_SYNC sys_cpu_to_be16(BIT(7))
#define HEART_BEAT   sys_cpu_to_be16(BIT(8))

#define SYNC_PACK_STATUS_READ  BIT(0)
#define SYNC_PACK_STATUS_WRITE BIT(1)

#define GET_8BITS(buf, n_byte)  (*(uint8_t *)(buf + n_byte))
#define GET_16BITS(buf, n_byte) (*(uint16_t *)(buf + n_byte))
#define GET_32BITS(buf, n_byte) (*(uint32_t *)(buf + n_byte))

typedef struct id_mapping func_mapping_t;

typedef uint8_t usb_trans_data_t;

typedef struct sync_pack usb_sync_pack_t;

typedef void (*usb_trans_cb_t)(int status);

typedef uint32_t (*usb_trans_func_t)(uint32_t arg1, uint32_t arg2, uint32_t arg3);

enum head_type {
	HEAD_TYPE_SYNC = 0,
	HEAD_TYPE_FUNC,
	HEAD_TYPE_ERROR,
	HEAD_TYPE_REPL,
	HEAD_TYPE_NONE,
};

struct sync_pack {
	uint16_t ID;
	usb_trans_data_t *data;
	uint8_t crc;
	uint8_t request_id;
	size_t len;
	usb_trans_cb_t cb;
	uint8_t *buf;
};

struct id_mapping {
	uint16_t id;
	usb_trans_func_t cb;
};

struct error_frame {
	uint16_t head;
	uint8_t request_id;
	uint8_t reserved;
	uint16_t error_code;
	uint16_t tail;
};

// 定义一个上下文结构体来存储不完整的数据
typedef struct {
	uint8_t buffer[256]; // 与process_buffer大小相同
	enum head_type head; // 当前数据的类型
	uint16_t length;     // 当前缓冲区中的数据长度
} PartialDataContext;

void usb_trans_func_add(uint16_t header, usb_trans_func_t cb);

void usb_trans_func_remove(uint16_t header);

usb_sync_pack_t *usb_trans_sync_add(usb_trans_data_t *data, uint16_t ID, size_t len,
				    usb_trans_cb_t cb);

void usb_trans_sync_flush(usb_sync_pack_t *pack);

void usb_trans_call_func(uint16_t ID, void *arg1, void *arg2, void *arg3, usb_trans_cb_t cb);

#endif /* usb_TRANS_H_ */