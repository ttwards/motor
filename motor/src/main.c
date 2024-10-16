/*
 * Copyright (c) 2021-2022 Henrik Brix Andersen <henrik@brixandersen.dk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/kernel.h>
#include "TEMP_MACROS.h"
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

/*DEBUG*/
/* change this to any other UART peripheral if desired */
// #define LOG_MODULE_NAME main
// LOG_MODULE_REGISTER(LOG_MODULE_NAME,LOG_LEVEL_INF);

#define CAN_SEND_STACK_SIZE 1024
#define CAN_SEND_PRIORITY 5

#define CAN_RECV_STACK_SIZE 768
#define CAN_RECV_PRIORITY 3

#define FEEDBACK_STACK_SIZE 2048
#define FEEDBACK_PRIORITY 4

#define HIGH_BYTE(x) ((x) >> 8)
#define LOW_BYTE(x) ((x) & 0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)

/* Devicetree */
#define CANBUS_NODE DT_CHOSEN(zephyr_canbus)

/* Global vars */
static struct k_sem tx_queue_sem; // 将信号量声明为静态变量
int16_t motor_rpm[4] = {2560,2560,2560,2560};
const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
static const uint8_t tx_id = 0x200;
struct can_frame txframe = {0};
struct motor_status{
    uint8_t motor_id;       // 电机ID
    int16_t angle;            // 角度（单位：度或弧度）
    int16_t torque_current;   // 转矩电流（单位：安培或其他）
    int16_t temperature;      // 温度（单位：摄氏度）
    int16_t speed;            // 转速（单位：RPM或其他）
} Motors[4];
k_tid_t can_send_tid = 0;
k_tid_t can_recv_tid = 0;
k_tid_t feedback_tid = 0;

struct k_event recv_full_event;


/* CAN RX Filter*/
static const struct can_filter filter20x = {
	.id = 0x200,
	.mask = 0x3F0,
	.flags = 0
};

/* CAN TX/RX callback functions */
static void can_tx_callback(const struct device *can_dev, int error, void *user_data)
{
	struct k_sem *tx_queue_sem = user_data;

	k_sem_give(tx_queue_sem);
}

static void can_rx_callback(const struct device *can_dev, struct can_frame *frame, void *user_data) {
    struct motor_status *motor_anal_data = (struct motor_status *)user_data;
	struct can_frame rx_frame = *frame;
	int id = (rx_frame.id & 0xF) - 1;
	motor_anal_data[id].angle = COMBINE_HL8(rx_frame.data[0], rx_frame.data[1]);
	motor_anal_data[id].speed = COMBINE_HL8(rx_frame.data[2], rx_frame.data[3]);
	motor_anal_data[id].torque_current = COMBINE_HL8(rx_frame.data[4], rx_frame.data[5]);
	motor_anal_data[id].temperature = rx_frame.data[6];
	k_event_post(&recv_full_event, 1);
}

/* CAN Frame packaging function */
inline static void can_calc(uint8_t id, int16_t* speeds) {
	txframe.id = 0x200;
	txframe.dlc = 8;
	int8_t data[8] = {0};
	for(int i = 0; i < 4; i++) {
		data[i * 2] = HIGH_BYTE(speeds[i]);
		data[i * 2 + 1] = LOW_BYTE(speeds[i]);
	}
	memcpy(txframe.data, data, sizeof(data));
}

/* CAN Send Thread*/
K_THREAD_STACK_DEFINE(can_send_stack_area, CAN_SEND_STACK_SIZE); // 定义线程栈
extern void can_send_entry(void *arg1, void *arg2, void *arg3) { // 添加参数名称
	while(1) {
		uint8_t err = 0x1;
		if (k_sem_take(&tx_queue_sem, K_NO_WAIT) == 0)
			err = can_send(can_dev, &txframe, K_NO_WAIT, can_tx_callback, &tx_queue_sem);
		// k_msleep(100);
	}
}

/* CAN Recv Thread*/
K_THREAD_STACK_DEFINE(can_recv_stack_area, CAN_RECV_STACK_SIZE); // 定义线程栈
extern void can_recv_entry(void *arg1, void *arg2, void *arg3) { // 添加参数名称
	while (true) {
		k_event_wait(&recv_full_event, 0x1, true, K_FOREVER);
		k_event_clear(&recv_full_event, 0x1);
		for(int i = 0; i < TOTAL_MOTORS; i++) {
			// if(motor_rpm[i] < 1222) {
			// 	motor_rpm[i]++;
			// } else {
			// 	motor_rpm[i] = 0;
			// }
			motor_rpm[i] = 9999;
		}
		can_calc(0x200, motor_rpm);
	}
	
}

/* CAN Feedback to console*/
K_THREAD_STACK_DEFINE(feedback_stack_area, FEEDBACK_STACK_SIZE); // 定义线程栈
void console_feedback(void *arg1, void *arg2, void *arg3) {
	while (1) {
		thread_analyzer_print(NULL);
		// printk("rx data:  ");
		// printk("angle:%.1f deg\t", (double)(Motors[0].angle) / 8191 * 360);
		// printk("rpm:%d\t", Motors[0].speed);
		// printk("current:%d\t", Motors[0].torque_current);
		// printk("temp:%d\n",Motors[0].temperature);
		k_msleep(50);
	}
}

int main(void)
{
	int err = 0;

	/* Sem Init */
	k_sem_init(&tx_queue_sem, 1, 3); // 初始化信号量

	/* Ensure the frame is not empty*/
	can_calc(tx_id, motor_rpm);

	/* Receive Event Init*/
	k_event_init(&recv_full_event);

	/* CAN Device init.*/
	err = device_is_ready(can_dev);	
	if (err != 0)
		printk("CAN device not ready");
	// for(;;) printk("Fuck USART!");
	err = can_start(can_dev);
	if (err != 0)
		printk("Error starting CAN controller (err %d)", err);

	err = can_add_rx_filter(can_dev, can_rx_callback, Motors, &filter20x);
	if (err != 0)
		printk("Error adding CAN filter (err %d)", err);

	/* Start CAN Sending thread*/	
	struct k_thread can_send_thread_data;
	can_send_tid = k_thread_create(&can_send_thread_data, can_send_stack_area, // 修改为 can_send_stack_area
                                K_THREAD_STACK_SIZEOF(can_send_stack_area), can_send_entry,
								NULL, NULL, NULL,
								CAN_SEND_PRIORITY, 0, K_NO_WAIT);
	
	/* Start CAN Receive & Calculate thread*/
	struct k_thread can_recv_thread_data;
	can_recv_tid = k_thread_create(&can_recv_thread_data, can_recv_stack_area, // 修改为 can_send_stack_area
                                K_THREAD_STACK_SIZEOF(can_recv_stack_area), can_recv_entry,
								NULL, NULL, NULL,
								CAN_RECV_PRIORITY, 0, K_NO_WAIT);

	/* Start Feedback thread*/
	struct k_thread feedback_thread_data;
	feedback_tid = k_thread_create(&feedback_thread_data, feedback_stack_area, // 修改为 can_send_stack_area
                                K_THREAD_STACK_SIZEOF(feedback_stack_area), console_feedback,
								NULL, NULL, NULL,
								FEEDBACK_PRIORITY, 0, K_NO_WAIT);
	while (1) {
        k_sleep(K_SECONDS(1));
    }
}
