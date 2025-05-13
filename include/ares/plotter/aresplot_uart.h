#ifndef ARESPLOT_UART_H
#define ARESPLOT_UART_H

#include "zephyr/kernel/thread.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>

// --- 用户需要实现的硬件/系统相关回调函数 ---
// --- User-implemented hardware/system-specific callback functions ---

struct aresplot_uart_params {
	struct device *uart_dev;
	uint16_t freq;
	struct k_timer timer;
	struct k_thread tid;
};

typedef struct aresplot_uart_params aresplot_uart_t;

/**
 * @brief 发送一个数据包 (通常是一个完整的Aresplot帧) 到通信接口 (例如 UART DMA,
 * USB packet) Sends a data packet (usually a complete Aresplot frame) to the
 * communication interface (e.g., UART DMA, USB packet).
 * @param data 指向要发送数据的指针 Pointer to the data to send.
 * @param length 要发送数据的长度 Length of the data to send.
 * @note 用户需要确保此函数是非阻塞的，或者在RTOS环境中适当地处理阻塞。
 * The user needs to ensure this function is non-blocking or handles blocking
 * appropriately in an RTOS environment. 如果发送操作是异步的
 * (例如DMA)，此函数启动传输后即可返回。 If the send operation is asynchronous
 * (e.g., DMA), this function can return after initiating the transfer.
 */
void aresplot_user_send_packet(const uint8_t *data, uint16_t length);

/**
 * @brief 获取当前系统时间戳 (例如，毫秒)
 * Gets the current system timestamp (e.g., in milliseconds).
 * @return 当前时间戳 Current timestamp.
 */
uint32_t aresplot_user_get_tick_ms(void);

void aresplot_user_critical_enter(void);

void aresplot_user_critical_exit(void);

#endif // ARESPLOT_UART_H