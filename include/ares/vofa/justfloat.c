#include <stdint.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include "justfloat.h"
#include "zephyr/kernel/thread.h"
#include <string.h>
#include <stdlib.h>

// LOG_MODULE_REGISTER(vofa, LOG_LEVEL_DBG);

static void jf_send_float(struct JFData *data)
{
	const struct device *uart_dev = data->uart_dev;

	for (int i = 0; i < data->channel; i++) {
		if (data->fdata[i] == data->fdata[data->channel]) {
			data->fdata[i] = 1e+6;
		}
	}

	uart_tx(uart_dev, (const uint8_t *)data->fdata, data->channel * 4 + 4, SYS_FOREVER_US);
}

/* JustFloat Feedback to console*/
static K_THREAD_STACK_DEFINE(jf_stack_area, 768); // 定义线程栈
static struct k_thread jf_thread_data;

static void jf_feedback(void *arg1, void *arg2, void *arg3)
{
	int delay = (int)arg1;
	struct JFData *data = (struct JFData *)arg2;

	while (1) {
		k_msleep(delay);
		jf_send_float(data);
	}
}

struct JFData *jf_send_init(const struct device *uart_dev, int delay, int channel)
{
	struct JFData *data = (struct JFData *)malloc(sizeof(struct JFData));
	data->fdata = (float *)malloc(sizeof(float) * (channel + 1));
	data->channel = channel;
	if (!device_is_ready(uart_dev)) {
		return NULL;
	}
	data->uart_dev = (struct device *)uart_dev;
	memcpy(&(data->fdata[channel]), tail, 4 * sizeof(uint8_t));
	/* Start JustFloat thread*/
	k_thread_create(&jf_thread_data, jf_stack_area, K_THREAD_STACK_SIZEOF(jf_stack_area),
			jf_feedback, (void *)delay, data, NULL, 1, 0, K_NO_WAIT);
	return data;
}