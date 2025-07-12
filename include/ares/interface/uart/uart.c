/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>

#include "ares/interface/ares_interface.h"
#include "uart.h"
#include "zephyr/sys/ring_buffer.h"
#include <ares/protocol/ares_protocol.h>

// Define a log level for this module
#ifndef CONFIG_ARES_UART_LOG_LEVEL
#define CONFIG_ARES_UART_LOG_LEVEL LOG_LEVEL_DBG
#endif
LOG_MODULE_REGISTER(ares_uart, CONFIG_ARES_UART_LOG_LEVEL);

/* === Pipeline and Threading Configuration === */
#define ARES_UART_PROCESSING_THREAD_STACK_SIZE CONFIG_ARES_UART_THREAD_STACK_SIZE
#define ARES_UART_PROCESSING_THREAD_PRIORITY   K_PRIO_PREEMPT(5)
#define ARES_UART_INCOMING_MSGQ_MAX_MSGS       20
#define ARES_UART_RX_INACTIVE_TIMEOUT          10

NET_BUF_POOL_DEFINE(uart_net_buf_pool, 10, 64, 4, NULL);

// Message queue to hold incoming net_buf pointers from ISR to thread
K_MSGQ_DEFINE(incoming_data_msgq, sizeof(struct net_buf *), ARES_UART_INCOMING_MSGQ_MAX_MSGS, 4);

#define ARES_UART_IF_FRAME_HEAD 0x5533
K_MEM_SLAB_DEFINE(uart_rx_slab, ARES_UART_BLOCK_SIZE, 6, 4);

// 帧头常量 (Big Endian)
#define FRAME_SYNC_BYTE_1   0x55
#define FRAME_SYNC_BYTE_2   0x33
#define FRAME_RESERVED_BYTE 0x00

/**
 * @brief 初始化帧解析器
 * @param parser 指向要初始化的解析器实例的指针
 */
void frame_parser_init(frame_parser_t *parser)
{
	if (parser == NULL) {
		return;
	}
	parser->state = STATE_WAIT_SYNC_1;
	parser->payload_len = 0;
	parser->payload_received_count = 0;
}

/**
 * @brief 从 ring_buf 中处理数据，寻找并解析数据帧
 *
 * @param parser 指向解析器实例的指针
 * @param rb 指向包含原始数据的 ring buffer 的指针
 */
void frame_parser_process(struct AresInterface *interface, frame_parser_t *parser,
			  struct ring_buf *rb)
{
	uint8_t byte;

	// 只要 ring_buf 中有数据，就持续处理
	while (ring_buf_get(rb, &byte, 1) > 0) {
		LOG_DBG("Received byte: 0x%02X\n", byte);
		switch (parser->state) {
		case STATE_WAIT_SYNC_1:
			if (byte == FRAME_SYNC_BYTE_1) {
				parser->state = STATE_WAIT_SYNC_2;
				LOG_DBG("Received sync byte 1: 0x%02X\n", byte);
			}
			break;

		case STATE_WAIT_SYNC_2:
			if (byte == FRAME_SYNC_BYTE_2) {
				parser->state = STATE_WAIT_LENGTH;
				LOG_DBG("Received sync byte 2: 0x%02X\n", byte);
			} else {
				// 如果第二个字节不是 0x33，退回初始状态
				// (优化：如果这个字节恰好是0x55，可以直接进入STATE_WAIT_SYNC_2，但简单地重置更健壮)
				frame_parser_init(parser);
			}
			break;

		case STATE_WAIT_LENGTH:
			parser->payload_len = byte;
			parser->payload_received_count = 0;
			// 检查载荷长度是否有效
			if (parser->payload_len > MAX_FRAME_PAYLOAD_SIZE) {
				LOG_ERR("Invalid payload length: %u\n", parser->payload_len);
				frame_parser_init(parser); // 长度无效，重置状态机
			} else {
				parser->state = STATE_WAIT_RESERVED;
				LOG_DBG("Payload length: %u\n", parser->payload_len);
			}
			break;

		case STATE_WAIT_RESERVED:
			if (byte == FRAME_RESERVED_BYTE) {
				// 保留字节正确
				if (parser->payload_len == 0) {
					// 如果载荷长度为0，我们已经有了一个完整的帧
					struct net_buf *buf = net_buf_alloc_with_data(
						&uart_net_buf_pool, parser->payload_buffer,
						parser->payload_len, K_NO_WAIT);
					interface->protocol->api->handle(interface->protocol, buf);
					net_buf_unref(buf);
					frame_parser_init(parser); // 重置以寻找下一帧
				} else {
					// 准备接收载荷
					parser->state = STATE_RECEIVING_PAYLOAD;
				}
			} else {
				// 保留字节错误，说明这不是一个有效的帧头，重置
				LOG_ERR("Invalid reserved byte: 0x%02X\n", byte);
				frame_parser_init(parser);
			}
			break;

		case STATE_RECEIVING_PAYLOAD:
			// 将接收到的字节存入载荷缓冲区
			parser->payload_buffer[parser->payload_received_count] = byte;
			parser->payload_received_count++;

			// 检查是否已接收完所有载荷数据
			if (parser->payload_received_count >= parser->payload_len) {
				LOG_DBG("Payload received: %u\n", parser->payload_received_count);
				// 载荷接收完毕，一个完整的帧已找到
				struct net_buf *buf = net_buf_alloc_with_data(
					&uart_net_buf_pool, parser->payload_buffer,
					parser->payload_len, K_NO_WAIT);
				interface->protocol->api->handle(interface->protocol, buf);
				net_buf_unref(buf);

				// 重置状态机，准备寻找下一个帧
				frame_parser_init(parser);
			}
			break;

		default:
			// 不应该发生的状态，安全起见重置
			LOG_ERR("Invalid state: %d", parser->state);
			frame_parser_init(parser);
			break;
		}
	}
}

int ares_uart_tx_without_lock(struct AresInterface *interface, struct net_buf *buf)
{
	struct AresUartInterface *uart_if = interface->priv_data;
	uint8_t len = buf->len;
	net_buf_push_u8(buf, 0);
	net_buf_push_u8(buf, len);
	net_buf_push_be16(buf, ARES_UART_IF_FRAME_HEAD);

	int err = uart_tx(uart_if->uart_dev, buf->data, buf->len, 0);
	if (err != 0) {
		LOG_ERR("Failed to send data.");
		return err;
	}
	net_buf_unref(buf);
	return 0;
}

struct net_buf *ares_uart_interface_alloc_buf(struct AresInterface *interface)
{
	struct net_buf *buf = net_buf_alloc(&uart_net_buf_pool, K_NO_WAIT);
	return buf;
}

// async serial callback
static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
	struct AresUartInterface *uart_if = user_data;
	int err;

	switch (evt->type) {
	case UART_TX_DONE:
		// LOG_INF("Tx sent %d bytes\n", evt->data.tx.len);
		break;

	case UART_TX_ABORTED:
		// LOG_INF("Tx aborted. Is the serial fast enough?\n");
		break;

	case UART_RX_RDY: {
		LOG_DBG("Received data %d bytes\n", evt->data.rx.len);

		if (evt->data.rx.len > 0) {
			ring_buf_put(&uart_if->uart_rb, evt->data.rx.buf + evt->data.rx.offset,
				     evt->data.rx.len);
			k_sem_give(&uart_if->sem);
		}

		break;
	}

	case UART_RX_BUF_REQUEST: {
		uint8_t *buf;

		err = k_mem_slab_alloc(&uart_rx_slab, (void **)&buf, K_NO_WAIT);
		if (err != 0) {
			LOG_ERR("Failed to allocate memory for RX buffer.");
			return;
		}

		err = uart_rx_buf_rsp(uart_if->uart_dev, buf, ARES_UART_BLOCK_SIZE);
		if (err != 0) {
			LOG_ERR("Failed to provide new buffer.");
			k_mem_slab_free(&uart_rx_slab, (void **)&buf);
			return;
		}
		break;
	}

	case UART_RX_BUF_RELEASED:
		k_mem_slab_free(&uart_rx_slab, (void *)evt->data.rx_buf.buf);
		break;

	case UART_RX_DISABLED:
		break;

	case UART_RX_STOPPED:
		break;
	}
}

void ares_uart_thread_entry(void *p1, void *p2, void *p3)
{
	struct AresInterface *interface = p1;
	struct AresUartInterface *uart_if = interface->priv_data;

	while (1) {
		k_sem_take(&uart_if->sem, K_FOREVER);
		frame_parser_process(interface, &uart_if->parser, &uart_if->uart_rb);
	}
}

void ares_uart_init_dev(struct AresInterface *interface, const struct device *uart_dev)
{
	struct AresUartInterface *uart_if = interface->priv_data;
	uart_if->uart_dev = (struct device *)uart_dev;
}

// 初始化函数
int ares_uart_init(struct AresInterface *interface)
{
	struct AresUartInterface *uart_if = interface->priv_data;

	if (!device_is_ready(uart_if->uart_dev)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}

	int err = 0;

	ring_buf_init(&uart_if->uart_rb, sizeof(uart_if->uart_rb_buf), uart_if->uart_rb_buf);
	k_sem_init(&uart_if->sem, 0, 1);

	err = uart_callback_set(uart_if->uart_dev, uart_callback, (void *)uart_if);

	// allocate buffer and start rx
	uint8_t *buf;
	err = k_mem_slab_alloc(&uart_rx_slab, (void **)&buf, K_NO_WAIT);
	__ASSERT(err == 0, "Failed to alloc slab");
	err = uart_rx_enable(uart_if->uart_dev, buf, ARES_UART_BLOCK_SIZE,
			     ARES_UART_RX_INACTIVE_TIMEOUT);
	__ASSERT(err == 0, "Failed to enable rx");

	k_thread_create(&uart_if->thread, uart_if->thread_stack,
			ARES_UART_PROCESSING_THREAD_STACK_SIZE, ares_uart_thread_entry,
			(void *)interface, NULL, NULL, ARES_UART_PROCESSING_THREAD_PRIORITY, 0,
			K_MSEC(100));
	__ASSERT(uart_if->thread != NULL, "Failed to create thread");
	k_thread_start(&uart_if->thread);

	return 0;
}