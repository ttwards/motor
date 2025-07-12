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
#include "ares/interface/ares_interface.h"

int ares_uart_init(struct AresInterface *interface);
int ares_uart_tx_without_lock(struct AresInterface *interface, struct net_buf *buf);
struct net_buf *ares_uart_interface_alloc_buf(struct AresInterface *interface);

void ares_uart_init_dev(struct AresInterface *interface, const struct device *uart_dev);

#define ARES_UART_PROCESSING_THREAD_STACK_SIZE CONFIG_ARES_UART_THREAD_STACK_SIZE

// 解析器状态机
typedef enum {
	STATE_WAIT_SYNC_1,      // 等待第一个同步字节 0x55
	STATE_WAIT_SYNC_2,      // 已收到 0x55, 等待 0x33
	STATE_WAIT_LENGTH,      // 已收到帧头, 等待长度字节
	STATE_WAIT_RESERVED,    // 已收到长度, 等待保留字节 0x00
	STATE_RECEIVING_PAYLOAD // 正在接收载荷数据
} frame_parser_state_t;

#define ARES_UART_BLOCK_SIZE    72
#define MAX_FRAME_PAYLOAD_SIZE ARES_UART_BLOCK_SIZE

// 帧解析器结构体
typedef struct {
	frame_parser_state_t state;                     // 当前状态
	uint8_t payload_len;                            // 当前帧期望的载荷长度
	uint16_t payload_received_count;                // 当前已接收的载荷字节数
	uint8_t payload_buffer[MAX_FRAME_PAYLOAD_SIZE]; // 用于组装载荷的缓冲区
} frame_parser_t;

struct AresUartInterface {
	struct AresInterface *interface;

	atomic_t state;

	struct ring_buf uart_rb;
	uint8_t uart_rb_buf[4 * ARES_UART_BLOCK_SIZE];

	struct device *uart_dev;

	struct k_sem sem;

	struct k_thread thread;
	k_thread_stack_t thread_stack[ARES_UART_PROCESSING_THREAD_STACK_SIZE];

	frame_parser_t parser;
};

#define ARES_UART_INTERFACE_DEFINE(Interface_name)                                                 \
	struct AresInterfaceAPI ares_uart_interface_api = {                                        \
		.init = ares_uart_init,                                                            \
		.send = ares_uart_tx_without_lock,                                                           \
		.alloc_buf = ares_uart_interface_alloc_buf,                                             \
	};                                                                                         \
	struct AresUartInterface Internal_##Interface_name = {NULL};                               \
	struct AresInterface Interface_name = {                                                    \
		.name = #Interface_name,                                                           \
		.api = &ares_uart_interface_api,                                                   \
		.protocol = NULL,                                                                  \
		.priv_data = &Internal_##Interface_name,                                             \
	};

#endif // ARES_UART_H__