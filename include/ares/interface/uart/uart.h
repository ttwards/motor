/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARES_UART_H__
#define ARES_UART_H__

#include "zephyr/sys/ring_buffer.h"
#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <ares/interface/ares_interface.h>

int ares_uart_init(struct AresInterface *interface);
int ares_uart_write(struct AresInterface *interface, struct net_buf *buf);
struct net_buf *ares_uart_interface_alloc_buf(struct AresInterface *interface);
struct net_buf *ares_uart_interface_alloc_buf_with_data(struct AresInterface *interface, void *data,
						   size_t len);
int ares_uart_write_with_lock(struct AresInterface *interface, struct net_buf *buf,
			      struct k_mutex *mutex);

#define ARES_UART_PROCESSING_THREAD_STACK_SIZE CONFIG_ARES_UART_THREAD_STACK_SIZE

struct AresUartInterface {
	struct AresInterface *interface;

	atomic_t state;

	struct ring_buf uart_rb;
	uint8_t uart_rb_buf[256];

	struct ring_buf incomplete_frame_rb;
	uint8_t incomplete_frame_rb_buf[70];

	uint8_t uart_parse_buf[70];

	const struct device *uart_dev;

	struct k_sem sem;

	struct k_thread *thread;
	uint8_t thread_stack[ARES_UART_PROCESSING_THREAD_STACK_SIZE];
};

#define ARES_UART_INTERFACE_DEFINE(Interface_name)                                                 \
	struct AresInterfaceAPI ares_uart_interface_api = {                                        \
		.init = ares_uart_init,                                                            \
		.send = ares_uart_write,                                                           \
		.send_with_lock = ares_uart_write_with_lock,                                       \
		.alloc_buf = ares_interface_alloc_buf,                                             \
		.alloc_buf_with_data = ares_interface_alloc_buf_with_data,                         \
	};                                                                                         \
	struct AresUartInterface Internal_##Interface_name = {NULL};                               \
	struct AresInterface Interface_name = {                                                    \
		.name = #Interface_name,                                                           \
		.api = &ares_uart_interface_api,                                                   \
		.protocol = NULL,                                                                  \
		.priv_data = &Internal_##Interface_name,                                             \
	};

#endif // ARES_UART_H__