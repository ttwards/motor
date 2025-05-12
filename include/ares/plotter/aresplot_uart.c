#include "aresplot_uart.h"
#include "aresplot_protocol.h"
#include <zephyr/toolchain.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(aresplot_uart, LOG_LEVEL_INF);

#define BUF_SIZE       64
#define UART_SLAB_SIZE 4

static aresplot_uart_t aresplot_instance;
static K_MEM_SLAB_DEFINE(aresplot_uart_slab, BUF_SIZE, UART_SLAB_SIZE, 4);

static K_SEM_DEFINE(aresplot_sem, 0, 1);

static struct k_timer aresplot_uart_timer;

#define ARESPLOT_UART_STACK_SIZE 1536
#define ARESPLOT_UART_PRIORITY   7

static K_THREAD_STACK_DEFINE(aresplot_stack_area, ARESPLOT_UART_STACK_SIZE);
// --- 用户需要实现的硬件/系统相关回调函数 ---
// --- User-implemented hardware/system-specific callback functions ---

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
static bool freq_too_high = false;
void aresplot_user_send_packet(const uint8_t *data, uint16_t length)
{
	struct device *uart_dev = aresplot_instance.uart_dev;
	int err = uart_tx(uart_dev, data, length, SYS_FOREVER_US);
	// LOG_HEXDUMP_INF(data, length, "Packet");
	if (err && !freq_too_high) {
		LOG_ERR("Failed to send packet: %d, is the frequency too high?", err);
		freq_too_high = true;
	}
}

/**
 * @brief 获取当前系统时间戳 (例如，毫秒)
 * Gets the current system timestamp (e.g., in milliseconds).
 * @return 当前时间戳 Current timestamp.
 */
uint32_t aresplot_user_get_tick_ms(void)
{
	return k_uptime_get();
}

static void aresplot_uart_timer_handler(struct k_timer *timer)
{
	k_sem_give(&aresplot_sem);
}

static void aresplot_uart_callback(const struct device *dev, struct uart_event *evt,
				   void *user_data)
{
	ARG_UNUSED(user_data);
	int err;
	uint8_t *buf = NULL;

	switch (evt->type) {
	case UART_RX_RDY: {
		uint8_t *p = &(evt->data.rx.buf[evt->data.rx.offset]);
		uint16_t len = evt->data.rx.len;

		LOG_DBG("Received %d bytes", len);
		aresplot_rx_feed_packet(p, len);

		break;
	}

	case UART_RX_BUF_REQUEST: {
		LOG_DBG("UART_RX_BUF_REQUEST");
		err = k_mem_slab_alloc(&aresplot_uart_slab, (void **)&buf, K_NO_WAIT);
		if (err == 0 && buf != NULL && ((uintptr_t)buf & 0x3) == 0) {
			err = uart_rx_buf_rsp(dev, buf, BUF_SIZE);
			if (err) {
				LOG_ERR("Failed to set RX buffer: %d", err);
				k_mem_slab_free(&aresplot_uart_slab, (void **)&buf);
			}
		} else {
			LOG_ERR("Failed to allocate RX buffer: %d", err);
		}
		break;
	}

	case UART_RX_BUF_RELEASED: {
		LOG_DBG("UART_RX_BUF_RELEASED");
		void *released_buf = evt->data.rx_buf.buf;
		if (released_buf != NULL) {
			k_mem_slab_free(&aresplot_uart_slab, released_buf);
		}
		break;
	}

	case UART_RX_DISABLED:
		LOG_DBG("UART RX disabled");
		break;

	case UART_RX_STOPPED: {
		LOG_DBG("UART RX stopped");
		err = k_mem_slab_alloc(&aresplot_uart_slab, (void **)&buf, K_NO_WAIT);
		if (err == 0 && buf != NULL && ((uintptr_t)buf & 0x3) == 0) {
			memset(buf, 0, BUF_SIZE);
			err = uart_rx_enable(dev, buf, BUF_SIZE, 100);
			if (err) {
				LOG_ERR("Failed to enable RX: %d", err);
				k_mem_slab_free(&aresplot_uart_slab, (void **)&buf);
			}
		} else {
			LOG_ERR("Failed to allocate RX buffer: %d", err);
		}
		break;
	}

	case UART_TX_DONE:
		LOG_DBG("UART TX done");
		break;

	case UART_TX_ABORTED:
		// LOG_ERR("UART TX aborted");
		break;

	default:
		LOG_WRN("Unhandled UART event: %d", evt->type);
		break;
	}
}

static void aresplot_tick(void *arg1, void *arg2, void *arg3)
{
	while (!k_sem_take(&aresplot_sem, K_FOREVER)) {
		aresplot_service_tick();
	}
}

void aresplot_uart_init(const struct device *uart_dev, uint16_t freq)
{
	int err;
	uint8_t *buf = NULL;

	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART device not ready");
		return;
	}

	aresplot_instance.uart_dev = (struct device *)uart_dev;
	aresplot_instance.freq = freq;

	// 配置 UART
	const struct uart_config config = {.baudrate = 2500000,
					   .parity = UART_CFG_PARITY_NONE,
					   .stop_bits = UART_CFG_STOP_BITS_1,
					   .data_bits = UART_CFG_DATA_BITS_8,
					   .flow_ctrl = UART_CFG_FLOW_CTRL_NONE};

	err = uart_configure(uart_dev, &config);
	if (err) {
		LOG_ERR("Failed to configure UART: %d", err);
		return;
	}

	// 设置回调
	err = uart_callback_set(uart_dev, aresplot_uart_callback, &aresplot_instance);
	if (err) {
		LOG_ERR("Failed to set callback: %d", err);
		return;
	}

	// 分配并启用接收缓冲区
	err = k_mem_slab_alloc(&aresplot_uart_slab, (void **)&buf, K_NO_WAIT);
	if (err) {
		LOG_ERR("Failed to allocate RX buffer: %d", err);
		return;
	}

	uart_rx_disable(uart_dev);
	memset(buf, 0, BUF_SIZE);
	err = uart_rx_enable(uart_dev, buf, BUF_SIZE, 100);
	if (err) {
		LOG_ERR("Failed to enable RX: %d", err);
		k_mem_slab_free(&aresplot_uart_slab, (void **)&buf);
		return;
	}

	// 初始化协议
	aresplot_init(1e3 / aresplot_instance.freq);

	// 创建服务线程
	k_thread_create(&aresplot_instance.tid, aresplot_stack_area,
			K_THREAD_STACK_SIZEOF(aresplot_stack_area), aresplot_tick, NULL, NULL, NULL,
			ARESPLOT_UART_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&aresplot_instance.tid, "aresplot_uart");

	k_timer_init(&aresplot_uart_timer, aresplot_uart_timer_handler, NULL);
	k_timer_start(&aresplot_uart_timer, K_NO_WAIT, K_USEC(1e6 / aresplot_instance.freq));

	LOG_INF("Aresplot UART initialized");
}

void aresplot_user_critical_enter(void)
{
	return;
}

void aresplot_user_critical_exit(void)
{
	return;
}
