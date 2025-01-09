#include <stdint.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include "justfloat.h"
#include "zephyr/device.h"
#include "zephyr/kernel/thread.h"
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

// LOG_MODULE_REGISTER(vofa, LOG_LEVEL_DBG);

static void jf_send_float(struct JFData *data) {
    const struct device *uart_dev = data->uart_dev;
    for (int i = 0; i < CH_COUNT; i++) {
        unsigned char p[4];
        memcpy(p, &data->fdata[i], sizeof(float));
        uart_poll_out(uart_dev, p[0]);
        uart_poll_out(uart_dev, p[1]);
        uart_poll_out(uart_dev, p[2]);
        uart_poll_out(uart_dev, p[3]);
    }
    unsigned char *p = data->tail;
    uart_poll_out(uart_dev, p[0]);
    uart_poll_out(uart_dev, p[1]);
    uart_poll_out(uart_dev, p[2]);
    uart_poll_out(uart_dev, p[3]);
}

/* JustFloat Feedback to console*/
static K_THREAD_STACK_DEFINE(jf_stack_area, 2048); // 定义线程栈
static struct k_thread jf_thread_data;

void jf_feedback(void *arg1, void *arg2, void *arg3) {
    int            delay = (int)arg1;
    struct JFData *data  = (struct JFData *)arg2;

    while (1) {
        k_msleep(delay);
        jf_send_float(data);
    }
}

void jf_send_init(const struct device *uart_dev, struct JFData *data, int delay) {
    if (!device_is_ready(uart_dev)) {
        return;
    }
    data->uart_dev = (struct device *)uart_dev;
    memcpy(data->tail, tail, 4 * sizeof(uint8_t));
    /* Start JustFloat thread*/
    k_thread_create(&jf_thread_data, jf_stack_area, K_THREAD_STACK_SIZEOF(jf_stack_area),
                    jf_feedback, (void *)delay, data, NULL, 1, 0, K_NO_WAIT);
}