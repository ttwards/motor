/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <zephyr/drivers/uart.h>

#include "ares/interface/ares_interface.h"
#include "uart.h"
#include <ares/protocol/ares_protocol.h>

// Define a log level for this module
#ifndef CONFIG_ARES_UART_LOG_LEVEL
#define CONFIG_ARES_UART_LOG_LEVEL LOG_LEVEL_INF
#endif
LOG_MODULE_REGISTER(ares_uart, CONFIG_ARES_UART_LOG_LEVEL);

/* === Pipeline and Threading Configuration === */
#define ARES_UART_PROCESSING_THREAD_STACK_SIZE CONFIG_ARES_UART_THREAD_STACK_SIZE
#define ARES_UART_PROCESSING_THREAD_PRIORITY   K_PRIO_PREEMPT(5)
#define ARES_UART_INCOMING_MSGQ_MAX_MSGS       20
#define ARES_UART_RX_INACTIVE_TIMEOUT          10

// Message queue to hold incoming net_buf pointers from ISR to thread
K_MSGQ_DEFINE(incoming_data_msgq, sizeof(struct net_buf *), ARES_UART_INCOMING_MSGQ_MAX_MSGS, 4);

// Processing thread
K_THREAD_STACK_DEFINE(processing_thread_stack_area, ARES_UART_PROCESSING_THREAD_STACK_SIZE);
struct k_thread processing_thread_data;

#define ARES_UART_IF_FRAME_HEAD 0x5533
#define ARES_UART_BLOCK_SIZE    70
K_MEM_SLAB_DEFINE(uart_rx_slab, ARES_UART_BLOCK_SIZE, 6, 4);

NET_BUF_POOL_DEFINE(uart_tx_pool, 10, 64, 4, NULL);

void uart_tx_without_lock(struct AresInterface *interface, struct net_buf *buf)
{
	struct AresUartInterface *uart_if = interface->priv_data;
	uint8_t len = buf->len;
	net_buf_push_be16(buf, ARES_UART_IF_FRAME_HEAD);
	net_buf_push_u8(buf, len);
	net_buf_push_u8(buf, 0);
	uart_tx(uart_if->uart_dev, buf->data, buf->len + 4, 0);
	net_buf_unref(buf);
}

struct net_buf *uart_mem_alloc(struct AresInterface *interface)
{
	struct net_buf *buf = net_buf_alloc(&uart_tx_pool, K_NO_WAIT);
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

		ring_buf_put(&uart_if->uart_rb, evt->data.rx.buf + evt->data.rx.offset,
			     evt->data.rx.len);

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
		uint32_t data_len = ring_buf_get(&uart_if->uart_rb, uart_if->uart_parse_buf,
						 sizeof(uart_if->uart_parse_buf));
		struct net_buf *net_buffer = net_buf_alloc_with_data(
			&uart_tx_pool, uart_if->uart_parse_buf, data_len, K_NO_WAIT);

		while (data_len > 0) {
			uint16_t peak = net_buf_pull_be16(net_buffer);
			data_len -= 2;
			if (peak == ARES_UART_IF_FRAME_HEAD) {
				uint8_t len = net_buf_pull_u8(net_buffer);
				data_len -= 1;
				if (data_len >= len + 1) {
					struct net_buf *tmp_buf = net_buf_alloc_with_data(
						&uart_tx_pool, net_buffer->data + 1, len,
						K_NO_WAIT);
					interface->protocol->api->handle(interface->protocol,
									 tmp_buf);
				} else {
					ring_buf_put(&uart_if->incomplete_frame_rb, peak, 2);
					ring_buf_put(&uart_if->incomplete_frame_rb, len, 1);
					ring_buf_put(&uart_if->incomplete_frame_rb,
						     net_buffer->data, data_len);
					len = data_len - 1;
				}
				data_len -= len + 1;
			} else if (!ring_buf_is_empty(&uart_if->incomplete_frame_rb)) {
				uint8_t *buf;
				err = k_mem_slab_alloc(&uart_rx_slab, (void **)&buf, K_NO_WAIT);
				if (err != 0) {
					LOG_ERR("Failed to allocate memory for RX buffer.");
					return;
				}
			}
		}
	}
}

// 初始化函数
int uart_trans_init(struct AresInterface *interface)
{
	struct AresUartInterface *uart_if = interface->priv_data;

	if (!device_is_ready(uart_if->uart_dev)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}

	int err = 0;

	ring_buf_init(&uart_if->uart_rb, sizeof(uart_if->uart_rb_buf), 0);
	k_sem_init(&uart_if->sem, 0, 1);

	err = uart_callback_set(uart_if->uart_dev, uart_callback, (void *)uart_if);

	// allocate buffer and start rx
	uint8_t *buf;
	err = k_mem_slab_alloc(&uart_rx_slab, (void **)&buf, K_NO_WAIT);
	__ASSERT(err == 0, "Failed to alloc slab");
	err = uart_rx_enable(uart_if->uart_dev, buf, ARES_UART_BLOCK_SIZE,
			     ARES_UART_RX_INACTIVE_TIMEOUT);
	__ASSERT(err == 0, "Failed to enable rx");

	uart_if->thread =
		k_thread_create(&processing_thread_data, processing_thread_stack_area,
				ARES_UART_PROCESSING_THREAD_STACK_SIZE, ares_uart_thread_entry,
				(void *)uart_if, NULL, ARES_UART_PROCESSING_THREAD_PRIORITY, 0, 0);
	__ASSERT(uart_if->thread != NULL, "Failed to create thread");
	k_thread_start(uart_if->thread);

	return 0;
}