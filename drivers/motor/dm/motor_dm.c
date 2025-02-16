/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "motor_dm.h"
#include "syscalls/kernel.h"
#include "zephyr/drivers/can.h"
#include "zephyr/drivers/motor.h"
#include "zephyr/drivers/pid.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/util.h"

#define DT_DRV_COMPAT dm_motor

LOG_MODULE_REGISTER(motor_dm, CONFIG_MOTOR_LOG_LEVEL);

/**
 * @brief Converts an unsigned integer to a float, given range and number of bits.
 *
 * This function takes an unsigned integer and maps it to a float value within
 * the specified range [x_min, x_max] based on the number of bits.
 *
 * @param x_int The unsigned integer value to be converted.
 * @param x_min The minimum value of the target float range.
 * @param x_max The maximum value of the target float range.
 * @param bits The number of bits representing the unsigned integer.
 * @return The corresponding float value within the range [x_min, x_max].
 */

static inline float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/// converts unsigned int to float, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief Converts a float to an unsigned int, given range and number of bits.
 *
 * This function takes a floating-point number and converts it to an unsigned
 * integer representation based on the specified range and number of bits.
 *
 * @param x The floating-point number to convert.
 * @param x_min The minimum value of the range.
 * @param x_max The maximum value of the range.
 * @param bits The number of bits for the unsigned integer representation.
 *
 * @return The unsigned integer representation of the floating-point number.
 */
static inline int float_to_uint(float x, float x_min, float x_max, int bits)
{
	/// Converts a float to an unsigned int, given range and number of bits
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static int get_can_id(const struct device *dev)
{
	const struct dm_motor_config *cfg = dev->config;
	for (int i = 0; i < CAN_COUNT; i++) {
		if (can_devices[i] == cfg->common.phy) {
			return i;
		}
	}
	return -1;
}

void can_tx_callback(const struct device *can_dev, int error, void *user_data)
{
	struct k_sem *queue_sem = user_data;
	if (!error) {
		k_sem_give(queue_sem);
	}
}

int dm_init(const struct device *dev)
{
	const struct dm_motor_config *cfg = dev->config;
	int can_id = get_can_id(dev);
	k_sem_init(&tx_queue_sem[can_id], 3, 3); // 初始化信号量
	if (!device_is_ready(cfg->common.phy)) {
		return -1;
	}
	if (k_work_busy_get(&dm_init_work) != 0) {
		return 0;
	}
	k_work_queue_init(&dm_work_queue);

	dm_tx_timer.expiry_fn = dm_isr_init_handler;
	k_timer_start(&dm_tx_timer, K_MSEC(500), K_MSEC(1));
	k_timer_user_data_set(&dm_tx_timer, &dm_init_work);
	return 0;
}

int dm_send_queued(struct tx_frame *frame, struct k_msgq *msgq)
{
	int err = k_sem_take(frame->sem, K_NO_WAIT);
	if (err == 0) {
		err = can_send(frame->can_dev, &frame->frame, K_NO_WAIT, can_tx_callback,
			       frame->sem);
		if (err) {
			LOG_ERR("TX queue full, will be put into msgq: %d", err);
		}
	} else if (err < 0) {
		// LOG_ERR("CAN hardware TX queue is full. (err %d)", err);
		err = k_msgq_put(msgq, frame, K_NO_WAIT);
		if (err) {
			LOG_ERR("Failed to put CAN frame into TX queue: %d", err);
		}
	}
	return err;
}

int dm_queue_proceed(struct k_msgq *msgq)
{
	struct tx_frame frame;
	int err = 0;
	bool give_up = false;
	while (!k_msgq_get(msgq, &frame, K_NO_WAIT)) {
		err = k_sem_take(frame.sem, K_NO_WAIT);
		if (err == 0) {
			err = can_send(frame.can_dev, &(frame.frame), K_MSEC(1), can_tx_callback,
				       frame.sem);
			if (err) {
				LOG_ERR("Failed to send CAN frame: %d", err);
			}
			k_msgq_purge(msgq);
		} else {
			if (give_up) {
				k_msgq_purge(msgq);
				break;
			}
			k_sleep(K_USEC(300));
			give_up = true;
			continue;
		}
	}
	return err;
}

void dm_motor_control(const struct device *dev, enum motor_cmd cmd)
{
	struct dm_motor_data *data = dev->data;
	const struct dm_motor_config *cfg = dev->config;

	struct can_frame frame;
	frame.id = cfg->common.tx_id + data->tx_offset;
	frame.flags = 0;
	frame.dlc = 8;

	int err = 0;
	int can_id = get_can_id(dev);

	switch (cmd) {
	case ENABLE_MOTOR:
		data->online = true;
		memcpy(frame.data, enable_frame, 8);
		if (k_sem_take(&tx_queue_sem[can_id], K_NO_WAIT) == 0) {
			err = can_send(cfg->common.phy, &frame, K_NO_WAIT, can_tx_callback,
				       &tx_queue_sem[can_id]);
		}
		break;
	case DISABLE_MOTOR:
		data->online = false;
		memcpy(frame.data, disable_frame, 8);
		if (k_sem_take(&tx_queue_sem[can_id], K_NO_WAIT) == 0) {
			err = can_send(cfg->common.phy, &frame, K_NO_WAIT, can_tx_callback,
				       &tx_queue_sem[can_id]);
		}
		break;
	case SET_ZERO_OFFSET:
		memcpy(frame.data, set_zero_frame, 8);
		if (k_sem_take(&tx_queue_sem[can_id], K_NO_WAIT) == 0) {
			err = can_send(cfg->common.phy, &frame, K_NO_WAIT, can_tx_callback,
				       &tx_queue_sem[can_id]);
		}
		break;
	case CLEAR_PID:
		memset(&data->params, 0, sizeof(data->params));
		break;
	case CLEAR_ERROR:
		memcpy(frame.data, clear_error_frame, 8);
		if (k_sem_take(&tx_queue_sem[can_id], K_NO_WAIT) == 0) {
			err = can_send(cfg->common.phy, &frame, K_NO_WAIT, can_tx_callback,
				       &tx_queue_sem[can_id]);
		}
		break;
	}
	if (err != 0) {
		LOG_ERR("Failed to send CAN frame: %d", err);
	}
}

static void dm_motor_pack(const struct device *dev, struct can_frame *frame)
{
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
	uint8_t *pbuf, *vbuf;
	struct dm_motor_data *data = dev->data;
	const struct dm_motor_config *cfg = dev->config;

	frame->id = cfg->common.tx_id + data->tx_offset;
	frame->dlc = 8;
	frame->flags = 0;
	switch (data->common.mode) {
	case MIT:
		pos_tmp = float_to_uint(data->target_angle, -cfg->p_max, cfg->p_max, 16);
		vel_tmp = float_to_uint(data->target_radps, -cfg->v_max, cfg->v_max, 12);
		tor_tmp = float_to_uint(data->target_torque, -cfg->t_max, cfg->t_max, 12);
		kp_tmp = float_to_uint(data->params.k_p, 0, 500, 12);
		kd_tmp = float_to_uint(data->params.k_d, 0, 5, 12);

		frame->data[0] = (pos_tmp >> 8);
		frame->data[1] = pos_tmp;
		frame->data[2] = (vel_tmp >> 4);
		frame->data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
		frame->data[4] = kp_tmp;
		frame->data[5] = (kd_tmp >> 4);
		frame->data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
		frame->data[7] = tor_tmp;
		break;
	case PV:
		pbuf = (uint8_t *)&data->target_angle;
		vbuf = (uint8_t *)&data->target_radps;

		frame->data[0] = *pbuf;
		memcpy(frame->data, pbuf, 4);
		memcpy(frame->data + 4, vbuf, 4);
		break;
	case VO:
		vbuf = (uint8_t *)&data->target_radps;

		memcpy(frame->data, vbuf, 4);
		break;
	default:
		break;
	}
}

float dm_motor_get_angle(const struct device *dev)
{
	struct dm_motor_data *data = dev->data;
	return data->common.angle;
}

float dm_motor_get_speed(const struct device *dev)
{
	struct dm_motor_data *data = dev->data;
	return data->common.rpm;
}

float dm_motor_get_torque(const struct device *dev)
{
	struct dm_motor_data *data = dev->data;
	return data->common.torque;
}

int dm_motor_set_mode(const struct device *dev, enum motor_mode mode)
{
	struct dm_motor_data *data = dev->data;
	const struct dm_motor_config *cfg = dev->config;
	char mode_str[10];

	data->common.mode = mode;

	switch (mode) {
	case MIT:
		strcpy(mode_str, "mit");
		data->tx_offset = 0x0;
		break;
	case PV:
		strcpy(mode_str, "pv");
		data->tx_offset = 0x100;
		break;
	case VO:
		strcpy(mode_str, "vo");
		data->tx_offset = 0x200;
		break;
	case MULTILOOP:
		data->online = false;
		return -ENOSYS;
		break;
	default:
		break;
	}

	for (int i = 0; i < SIZE_OF_ARRAY(cfg->common.capabilities); i++) {
		if (cfg->common.pid_datas[i]->pid_dev == NULL) {
			break;
		}
		if (strcmp(cfg->common.capabilities[i], mode_str) == 0) {
			const struct pid_config *params = pid_get_params(cfg->common.pid_datas[i]);

			data->common.mode = mode;
			data->params.k_p = params->k_p;
			data->params.k_d = params->k_d;
			break;
		}
	}
	return 0;
}

int dm_motor_set_torque(const struct device *dev, float torque)
{
	struct dm_motor_data *data = dev->data;
	const struct dm_motor_config *cfg = dev->config;

	data->target_radps = 0;
	data->target_angle = 0;
	float torque_scaled = torque / cfg->gear_ratio;
	data->target_torque = torque_scaled;
	data->params.k_p = 0;
	data->params.k_d = 0;
	return 0;
}

int dm_motor_set_speed(const struct device *dev, float speed_rpm)
{
	struct dm_motor_data *data = dev->data;

	data->target_radps = RPM2RADPS(speed_rpm);

	return 0;
}

// user must set speed before setting angle
int dm_motor_set_angle(const struct device *dev, float angle)
{
	struct dm_motor_data *data = dev->data;

	data->target_angle = RAD2DEG * angle;

	return 0;
}

static int get_motor_id(struct can_frame *frame)
{
	for (int i = 0; i < MOTOR_COUNT; i++) {
		const struct device *dev = motor_devices[i];
		const struct dm_motor_config *cfg = (const struct dm_motor_config *)(dev->config);
		if ((cfg->common.rx_id & 0xFF) == (frame->id & 0xFF)) {
			return i;
		}
	}
	return -1;
}

static struct can_filter filters[CAN_COUNT];

static void dm_can_rx_handler(const struct device *can_dev, struct can_frame *frame,
			      void *user_data)
{
	int id = get_motor_id(frame);
	if (id == -1) {
		LOG_ERR("Unknown motor ID: %d", frame->id);
		return;
	}

	struct dm_motor_data *data = (struct dm_motor_data *)(motor_devices[id]->data);

	if (data->missed_times > 0) {
		if (data->missed_times > 5 && data->online == true) {
			data->missed_times = 0;
			LOG_ERR("Motor %d is back online", id);
		}
		data->missed_times--;
	}
	data->err = frame->data[0] >> 4;
	data->RAWangle = (frame->data[1] << 8) | (frame->data[2]);
	data->RAWrpm = (frame->data[3] << 4) | (frame->data[4] >> 4);
	data->RAWtorque = (frame->data[4] & 0xF) << 8;
	data->update = true;

	k_work_submit_to_queue(&dm_work_queue, &dm_rx_data_handle);
}

void dm_rx_data_handler(struct k_work *work)
{
	for (int i = 0; i < MOTOR_COUNT; i++) {
		struct dm_motor_data *data = (struct dm_motor_data *)(motor_devices[i]->data);
		if (!data->update) {
			continue;
		}
		const struct dm_motor_config *cfg =
			(const struct dm_motor_config *)(motor_devices[i]->config);

		float prev_angle = data->common.angle;
		data->common.angle =
			(uint_to_float(data->RAWangle, -cfg->p_max, cfg->p_max, 16))*RAD2DEG;
		data->common.rpm =
			RADPS2RPM(uint_to_float(data->RAWrpm, -cfg->v_max, cfg->v_max, 12));
		data->common.torque = uint_to_float(data->RAWtorque, -cfg->t_max, cfg->t_max, 12);

		data->delta_deg_sum += data->common.angle - prev_angle;
		if (data->delta_deg_sum > 360) {
			data->common.round_cnt++;
			data->delta_deg_sum -= 360.0f;
		} else if (data->delta_deg_sum < -360) {
			data->common.round_cnt--;
			data->delta_deg_sum += 360.0f;
		}

		data->update = false;
	}
}

void dm_tx_isr_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&dm_work_queue, &dm_tx_data_handle);
}

void dm_isr_init_handler(struct k_timer *dummy)
{
	dummy->expiry_fn = dm_tx_isr_handler;
	k_work_queue_start(&dm_work_queue, dm_work_queue_stack, CAN_SEND_STACK_SIZE,
			   CAN_SEND_PRIORITY, NULL);
	k_work_submit_to_queue(&dm_work_queue, &dm_init_work);
}

void dm_tx_data_handler(struct k_work *work)
{
	struct can_frame tx_frame;

	for (int i = 0; i < MOTOR_COUNT; i++) {
		struct dm_motor_data *data = motor_devices[i]->data;
		const struct dm_motor_config *cfg = motor_devices[i]->config;
		if (data->online && data->missed_times <= 5) {
			int can_id = get_can_id(motor_devices[i]);
			dm_motor_pack(motor_devices[i], &tx_frame);
			struct tx_frame queued_frame = {
				.can_dev = cfg->common.phy,
				.sem = &tx_queue_sem[can_id],
				.frame = tx_frame,
			};
			dm_send_queued(&queued_frame, &dm_can_tx_msgq);

			data->missed_times++;
			if (data->missed_times > 5) {
				LOG_ERR("Motor %d is not responding, missed %d times, setting it "
					"to "
					"offline...",
					i, data->missed_times);
				continue;
			}
			if (data->missed_times > 3 && data->err > 1) {
				dm_motor_control(motor_devices[i], CLEAR_ERROR);
			}
		}
	}
	dm_queue_proceed(&dm_can_tx_msgq);
}

void dm_init_handler(struct k_work *work)
{
	k_timer_stop(&dm_tx_timer);
	LOG_DBG("DM motor control thread started");

	uint32_t id_full1[CAN_COUNT] = {0};
	uint32_t id_full0[CAN_COUNT] = {0};
	for (int i = 0; i < MOTOR_COUNT; i++) {
		int can_id = 0;
		const struct dm_motor_config *cfg =
			(const struct dm_motor_config *)(motor_devices[i]->config);

		for (int j = 0; j < CAN_COUNT; j++) {
			if (can_devices[j] == cfg->common.phy) {
				can_id = j;
				break;
			}
		}
		if (filters[can_id].id == 0) {
			filters[can_id].id = cfg->common.rx_id & 0xFF;
			id_full1[can_id] = cfg->common.rx_id & 0xFF;
			id_full0[can_id] = ~(cfg->common.rx_id & 0xFF);
		}
		filters[can_id].id &= cfg->common.rx_id & 0xFF;
		id_full1[can_id] &= cfg->common.rx_id & 0xFF;
		id_full0[can_id] &= ~(cfg->common.rx_id & 0xFF);

		int err = can_start(cfg->common.phy);
		if (err) {
			LOG_ERR("Failed to start CAN device: %d", err);
		}
	}

	for (int i = 0; i < CAN_COUNT; i++) {
		const struct device *can_dev = can_devices[i];

		filters[i].mask = (id_full1[i] | id_full0[i]) & 0x7FF;
		int err = can_add_rx_filter(can_dev, dm_can_rx_handler, 0, &filters[i]);
		if (err < 0) {
			LOG_ERR("Error adding CAN filter (err %d)", err);
		}
		// If you recieved an error here, remember that 2# CAN of STM32 is in slave
		// mode and does not have an independent filter. If that is the issue, you
		// can ignore that.
	}

	k_sleep(K_MSEC(500));

	for (int i = 0; i < MOTOR_COUNT; i++) {
		dm_motor_control(motor_devices[i], ENABLE_MOTOR);
	}

	k_timer_start(&dm_tx_timer, K_NO_WAIT, K_MSEC(1));
	k_timer_user_data_set(&dm_tx_timer, &dm_tx_data_handle);
}

DT_INST_FOREACH_STATUS_OKAY(DMMOTOR_INST)