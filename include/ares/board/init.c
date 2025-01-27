/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "zephyr/toolchain.h"
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/logging/log.h>
#include <zephyr/debug/thread_analyzer.h>

#ifdef CONFIG_BOARD_DM_MC02
#define XT30_1_NODE DT_NODELABEL(power1)
#define XT30_2_NODE DT_NODELABEL(power2)
#define STRIP_NODE  DT_ALIAS(led_strip)

static const struct gpio_dt_spec pwr1 = GPIO_DT_SPEC_GET(XT30_1_NODE, gpios);
static const struct gpio_dt_spec pwr2 = GPIO_DT_SPEC_GET(XT30_2_NODE, gpios);

const struct device *strip = DEVICE_DT_GET(STRIP_NODE);

void pwr_init(void)
{
	gpio_pin_configure_dt(&pwr1, GPIO_OUTPUT_HIGH);
	gpio_pin_configure_dt(&pwr2, GPIO_OUTPUT_HIGH);
}

#define PWR_INIT pwr_init();
#define LED_INIT

#endif /* CONFIG_BOARD_DM_MC02 */

#ifdef CONFIG_BOARD_ROBOMASTER_BOARD_A

#define XT30_1_NODE DT_NODELABEL(power1)
#define XT30_2_NODE DT_NODELABEL(power2)
#define XT30_3_NODE DT_NODELABEL(power3)
#define XT30_4_NODE DT_NODELABEL(power4)

static const struct gpio_dt_spec pwr1 = GPIO_DT_SPEC_GET(XT30_1_NODE, gpios);
static const struct gpio_dt_spec pwr2 = GPIO_DT_SPEC_GET(XT30_2_NODE, gpios);
static const struct gpio_dt_spec pwr3 = GPIO_DT_SPEC_GET(XT30_3_NODE, gpios);
static const struct gpio_dt_spec pwr4 = GPIO_DT_SPEC_GET(XT30_4_NODE, gpios);

void pwr_init(void)
{
	gpio_pin_configure_dt(&pwr1, GPIO_OUTPUT_HIGH);
	gpio_pin_configure_dt(&pwr2, GPIO_OUTPUT_HIGH);
	gpio_pin_configure_dt(&pwr3, GPIO_OUTPUT_HIGH);
	gpio_pin_configure_dt(&pwr4, GPIO_OUTPUT_HIGH);
}

// #define LED_GREEN DT_ALIAS(led8)
// #define LED_RED DT_ALIAS(led9)

// const struct device *led_green = DEVICE_DT_GET(LED_GREEN);
// const struct device *led_red = DEVICE_DT_GET(LED_RED);

#define PWR_INIT pwr_init();
#define LED_INIT

#endif /* CONFIG_BOARD_ROBOMASTER_BOARD_A */

#ifdef CONFIG_BOARD_ROBOMASTER_BOARD_C

// #define LED_BLUE  DT_ALIAS(led0)
// #define LED_GREEN DT_ALIAS(led1)
// #define LED_RED   DT_ALIAS(led2)

// const struct device *led_blue = DEVICE_DT_GET(LED_BLUE);
// const struct device *led_green = DEVICE_DT_GET(LED_GREEN);
// const struct device *led_red = DEVICE_DT_GET(LED_RED);

#define PWR_INIT
#define LED_INIT

#endif /* CONFIG_BOARD_ROBOMASTER_BOARD_C */

#define RGB(_r, _g, _b) ((struct led_rgb){.r = (_r), .g = (_g), .b = (_b)})

void led_set_rgb(struct led_rgb *color)
{
#ifdef CONFIG_BOARD_DM_MC02
	led_strip_update_rgb(strip, color, 2);
#endif /* CONFIG_BOARD_DM_MC02 */

#ifdef CONFIG_BOARD_ROBOMASTER_BOARD_A
	// led_set_brightness(led_green, 1, color->g);
	// led_set_brightness(led_red, 1, color->r);
#endif /* CONFIG_BOARD_ROBOMASTER_BOARD_A */

#ifdef CONFIG_BOARD_ROBOMASTER_BOARD_C
	// led_set_brightness(led_blue, 1, color->b);
	// led_set_brightness(led_green, 1, color->g);
	// led_set_brightness(led_red, 1, color->r);
#endif /* CONFIG_BOARD_ROBOMASTER_BOARD_C */
}

void led_serivce_func(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct k_thread_runtime_stats idle_stats_old, idle_stats_new;
	uint64_t idle_cycles_diff;
	uint64_t total_cycles_diff;
	struct led_rgb color;

	while (1) {
		// 获取统计数据
		k_thread_runtime_stats_all_get(&idle_stats_old);
		k_msleep(1000);
		k_thread_runtime_stats_all_get(&idle_stats_new);

		// 计算CPU使用率 (0-100)
		idle_cycles_diff = idle_stats_new.idle_cycles - idle_stats_old.idle_cycles;
		total_cycles_diff =
			idle_stats_new.execution_cycles - idle_stats_old.execution_cycles;
		float cpu_usage = 100.0f * (1.0f - ((float)idle_cycles_diff / total_cycles_diff));

		// 映射到RGB值 (红色表示高负载，绿色表示低负载)
		uint8_t red = (uint8_t)((cpu_usage / 100.0f) * 0x0f);
		uint8_t green = (uint8_t)((1.0f - cpu_usage / 100.0f) * 0x0f);

		color = RGB(red, green, 0);
		led_set_rgb(&color);

		printk("CPU: %.1f%%, RGB: %02x%02x00\n", (double)cpu_usage, red, green);
	}
}

// static K_THREAD_STACK_DEFINE(led_serivce_stack, 1024); // 定义线程栈
// static struct k_thread led_service_thread;
// void led_init(void)
// {
// 	// struct led_rgb color = RGB(0x00, 0x0f, 0x0f);
// 	// led_set_rgb(&color);
// 	// k_sleep(K_MSEC(1000));
// 	// k_thread_create(&led_service_thread, led_serivce_stack,
// 	// 		K_THREAD_STACK_SIZEOF(led_serivce_stack), led_serivce_func, NULL, NULL,
// 	// 		NULL, -1, 0, K_NO_WAIT);
// }

void board_init(void)
{
	PWR_INIT
	LED_INIT
	printk("Board init done.\n");
}