#include "motor_mi.h"
#include "zephyr/device.h"
#include "zephyr/drivers/can.h"
#include "zephyr/types.h"
#include <zephyr/drivers/pid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>

#define DT_DRV_COMPAT mi_motor

LOG_MODULE_REGISTER(motor_mi, CONFIG_MOTOR_LOG_LEVEL);

struct k_sem tx_frame_sem;

static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits) {
  uint32_t span = (1 << bits) - 1;
  float offset = x_max - x_min;
  return offset * x / span + x_min;
}

int float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  if (x > x_max)
    x = x_max;
  else if (x < x_min)
    x = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static int get_can_id(const struct device *dev) {
  const struct mi_motor_cfg *cfg = dev->config;
  for (int i = 0; i < CAN_COUNT; i++) {
    if (can_devices[i] == cfg->common.phy) {
      return i;
    }
  }
  return -1;
}

void can_tx_callback(const struct device *can_dev, int error, void *user_data) {
  struct k_sem *queue_sem = user_data;
  if (!error) {
    k_sem_give(queue_sem);
  }
}

int mi_init(const struct device *dev) {
  const struct mi_motor_cfg *cfg = dev->config;
  int can_id = get_can_id(dev);
  k_sem_init(&tx_queue_sem[can_id], 3, 3); // 初始化信号量
  if (!device_is_ready(cfg->common.phy)) {
    return -1;
  }
  if (k_work_busy_get(&mi_init_work) != 0) {
    return 0;
  }
  k_work_queue_init(&mi_work_queue);

  mi_tx_timer.expiry_fn = mi_isr_init_handler;
  k_timer_start(&mi_tx_timer, K_MSEC(600), K_MSEC(1));
  k_timer_user_data_set(&mi_tx_timer, &mi_init_work);
  return 0;
}
int mi_send_queued(struct tx_frame *frame, struct k_msgq *msgq) {
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

int mi_queue_proceed(struct k_msgq *msgq) {
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

void mi_motor_control(const struct device *dev, enum motor_cmd cmd) {
  struct mi_motor_data *data = dev->data;
  const struct mi_motor_cfg *cfg = dev->config;

  struct can_frame frame={0};
  frame.flags = CAN_FRAME_IDE;
  
  frame.dlc = 8;
  // uint16_t master_id;

  int can_id = get_can_id(dev);
  struct mi_can_id *mi_can_id = (struct mi_can_id *)&frame.id;
  mi_can_id->id = can_id;
  switch (cmd) {
  case ENABLE_MOTOR:
    data->online = true;
    mi_can_id->mi_msg_mode = Communication_Type_MotorEnable;
    break;
  case DISABLE_MOTOR:
    data->online = false;
    mi_can_id->mi_msg_mode = Communication_Type_MotorStop;
    break;
  case SET_ZERO_OFFSET:
    mi_can_id->mi_msg_mode = Communication_Type_SetPosZero;
    frame.data[0] = 0x01;
    break;

  case CLEAR_PID:

    break;
  case CLEAR_ERROR:

    break;
    LOG_ERR("Unsupport motor command: %d", cmd);
    return;
  }
  int err = 0;
  if (k_sem_take(&tx_queue_sem[can_id], K_NO_WAIT) == 0) {
    err = can_send(cfg->common.phy, &frame, K_NO_WAIT, can_tx_callback,
                   &tx_queue_sem[can_id]);
  }
  if (err != 0) {
    LOG_ERR("Failed to send CAN frame: %d", err);
  }
}

static void mi_motor_pack(const struct device *dev, struct can_frame *frame) {
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp, cur_tep;

  struct mi_motor_data *data = (struct mi_motor_data *)(dev->data);

  struct mi_can_id *mi_can_id = (struct mi_can_id *)(frame->id);

  mi_can_id->id = data->can_id;

  frame->dlc = 8;
  frame->flags = 0;
  struct can_frame *frame_follow = frame + 1;
  frame->flags = CAN_FRAME_IDE;
  frame_follow->flags = CAN_FRAME_IDE;
  struct mi_can_id *mi_can_id_fol = (struct mi_can_id *)(frame_follow->id);

  mi_can_id_fol->id = data->can_id;
  switch (data->common.mode) {
  case MIT:
    mi_can_id->mi_msg_mode = Communication_Type_MotionControl_MIT;

    pos_tmp = float_to_uint(data->target_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(data->target_radps, V_MIN, V_MAX, 16);
    kp_tmp = float_to_uint(data->params.k_d, KP_MIN, KP_MAX, 16);
    kd_tmp = float_to_uint(data->params.k_d, KD_MIN, KD_MAX, 16);
    tor_tmp = float_to_uint(data->target_torque, T_MIN, T_MAX, 16);
    mi_can_id->data = tor_tmp;
    frame->data[0] = (pos_tmp >> 8) & 0xFF;
    frame->data[1] = pos_tmp & 0xFF;
    frame->data[2] = (vel_tmp >> 8) & 0xFF;
    frame->data[3] = vel_tmp & 0xFF;
    frame->data[4] = (kp_tmp >> 8) & 0xFF;
    frame->data[5] = kp_tmp & 0xFF;
    frame->data[6] = (kd_tmp >> 8) & 0xFF;
    frame->data[7] = kd_tmp & 0xFF;
    break;
  case PV:
    mi_can_id->mi_msg_mode = Communication_Type_GetSingleParameter;

    pos_tmp = float_to_uint(data->target_pos, P_MIN, P_MAX, 32);
    vel_tmp = float_to_uint(data->target_radps, V_MIN, V_MAX, 32);
    frame->data[0] = (Limit_Spd >> 8) & 0xFF;
    frame->data[1] = Limit_Spd & 0xFF;
    frame->data[2] = 0;
    frame->data[3] = 0;
    frame->data[4] = (vel_tmp >> 24) & 0xFF;
    frame->data[5] = (vel_tmp >> 16) & 0xFF;
    frame->data[6] = (vel_tmp >> 8) & 0xFF;
    frame->data[7] = vel_tmp & 0xFF;

    mi_can_id_fol->mi_msg_mode = Communication_Type_SetSingleParameter;
    frame_follow->dlc = 8;
    frame_follow->flags = 0;
    frame_follow->data[0] = (Loc_Ref >> 8) & 0xff;
    frame_follow->data[1] = Loc_Ref & 0xff;
    frame_follow->data[2] = 0;
    frame_follow->data[3] = 0;
    frame_follow->data[4] = (pos_tmp >> 24) & 0xff;
    frame_follow->data[5] = (pos_tmp >> 16) & 0xff;
    frame_follow->data[6] = (pos_tmp >> 8) & 0xff;
    frame_follow->data[7] = pos_tmp & 0xff;
    break;
  case VO:
    mi_can_id->mi_msg_mode = Communication_Type_GetSingleParameter;
    vel_tmp = float_to_uint(data->target_radps, V_MIN, V_MAX, 32);
    cur_tep = float_to_uint(data->limit_cur, 0, CUR_MAX, 32);
    frame->data[0] = (Limit_Cur >> 8) & 0xFF;
    frame->data[1] = Limit_Cur & 0xFF;
    frame->data[2] = 0;
    frame->data[3] = 0;
    frame->data[4] = (cur_tep >> 24) & 0xFF;
    frame->data[5] = (cur_tep >> 16) & 0xFF;
    frame->data[6] = (cur_tep >> 8) & 0xFF;
    frame->data[7] = cur_tep & 0xFF;

    mi_can_id_fol->mi_msg_mode = Communication_Type_SetSingleParameter;

    frame_follow->dlc = 8;
    frame_follow->flags = 0;
    frame_follow->data[0] = (Spd_Ref >> 8) & 0xff;
    frame_follow->data[1] = Spd_Ref & 0xff;
    frame_follow->data[2] = 0;
    frame_follow->data[3] = 0;
    frame_follow->data[4] = (vel_tmp >> 24) & 0xff;
    frame_follow->data[5] = (vel_tmp >> 16) & 0xff;
    frame_follow->data[6] = (vel_tmp >> 8) & 0xff;
    frame_follow->data[7] = vel_tmp & 0xff;
    break;
  default:
    break;
  }
}

float mi_motor_get_angle(const struct device *dev) {
  struct mi_motor_data *data = dev->data;
  return data->common.angle;
}

float mi_motor_get_speed(const struct device *dev) {
  struct mi_motor_data *data = dev->data;
  return data->common.rpm;
}

float mi_motor_get_torque(const struct device *dev) {
  struct mi_motor_data *data = dev->data;
  return data->common.torque;
}

int mi_motor_set_mode(const struct device *dev, enum motor_mode mode) {
  struct mi_motor_data *data = dev->data;
  const struct mi_motor_cfg *cfg = dev->config;
  char mode_str[10];

  data->common.mode = mode;

  switch (mode) {
  case MIT:
    strcpy(mode_str, "mit");

    break;
  case PV:
    strcpy(mode_str, "pv");

    break;
  case VO:
    strcpy(mode_str, "vo");

    break;
  case MULTILOOP:
    data->online = false;
    return -ENOSYS;
    break;
  default:
    LOG_DBG("Unknown motor mode: %d", mode);
    break;
  }

  for (int i = 0; i < SIZE_OF_ARRAY(cfg->common.capabilities); i++) {
    if (cfg->common.pid_datas[i]->pid_dev == NULL) {
      break;
    }
    if (strcmp(cfg->common.capabilities[i], mode_str) == 0) {
      const struct pid_config *params =
          pid_get_params(cfg->common.pid_datas[i]);

      data->common.mode = mode;
      data->params.k_p = params->k_p;
      data->params.k_d = params->k_d;
      break;
    }
  }
  return 0;
}

int mi_motor_set_torque(const struct device *dev, float torque) {
  struct mi_motor_data *data = dev->data;

  data->target_torque = torque;
  return 0;
}

int mi_motor_set_speed(const struct device *dev, float speed_rpm) {
  struct mi_motor_data *data = dev->data;

  data->target_radps = RPM2RADPS(speed_rpm);

  return 0;
}

// user must set speed before setting angle
int mi_motor_set_angle(const struct device *dev, float angle) {
  struct mi_motor_data *data = dev->data;

  data->target_pos = RAD2DEG * angle;

  return 0;
}

static int get_motor_id(struct can_frame *frame) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    const struct device *dev = motor_devices[i];
    const struct mi_motor_cfg *cfg = (const struct mi_motor_cfg *)(dev->config);
    if ((cfg->common.rx_id & 0xFF) == (frame->id & 0xFF)) {
      return i;
    }
  }
  return -1;
}

static struct can_filter filters[CAN_COUNT];

static void mi_can_rx_handler(const struct device *can_dev,
                              struct can_frame *frame, void *user_data) {
  int id = get_motor_id(frame);
  if (id == -1) {
    LOG_ERR("Unknown motor ID: %d", frame->id);
    return;
  }

  struct mi_motor_data *data =
      (struct mi_motor_data *)(motor_devices[id]->data);

  if (data->missed_times > 0) {
    if (data->missed_times > 5 && data->online == true) {
      data->missed_times = 0;
      LOG_ERR("Motor %d is back online", id);
    }
    data->missed_times--;
  }
  struct mi_can_id *can_id = (struct mi_can_id *)(frame->id);
  if (can_id->mi_msg_mode == Communication_Type_MotorFeedback) {
    data->err = ((can_id->data) >> 8) & 0x1f;
    data->update = true;
    data->RAWangle = (frame->data[0] << 8) | (frame->data[1]);
    data->RAWrpm = (frame->data[2] << 8) | (frame->data[3]);
    data->RAWtorque = (frame->data[4] << 8) | (frame->data[5]);
    data->RAWtemp = (frame->data[6] << 8) | (frame->data[7]);
  } else if (can_id->mi_msg_mode == Communication_Type_GetID) {
    data->can_id = can_id->data;
  }

  k_work_submit_to_queue(&mi_work_queue, &mi_rx_data_handle);
}

void mi_rx_data_handler(struct k_work *work) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    struct mi_motor_data *data =
        (struct mi_motor_data *)(motor_devices[i]->data);
    if (!data->update) {
      continue;
    }

    float prev_angle = data->common.angle;
    data->common.angle =
        (uint16_to_float(data->RAWangle, (double)P_MIN, (double)P_MAX, 16)) *
        RAD2DEG;
    data->common.rpm = RADPS2RPM(
        uint16_to_float(data->RAWrpm, (double)V_MIN, (double)V_MAX, 16));
    data->common.torque =
        uint16_to_float(data->RAWtorque, (double)T_MIN, (double)T_MAX, 16);
    data->common.temperature = ((float)(data->RAWtemp)) / 10;
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

void mi_tx_isr_handler(struct k_timer *dummy) {
  k_work_submit_to_queue(&mi_work_queue, &mi_tx_data_handle);
}

void mi_isr_init_handler(struct k_timer *dummy) {
  dummy->expiry_fn = mi_tx_isr_handler;
  k_work_queue_start(&mi_work_queue, mi_work_queue_stack, CAN_SEND_STACK_SIZE,
                     CAN_SEND_PRIORITY, NULL);
  k_work_submit_to_queue(&mi_work_queue, &mi_init_work);
}

void mi_tx_data_handler(struct k_work *work) {
  struct can_frame tx_frame[2]={0};
tx_frame[0].flags = CAN_FRAME_IDE;
tx_frame[1].flags = CAN_FRAME_IDE;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    struct mi_motor_data *data = motor_devices[i]->data;
    const struct mi_motor_cfg *cfg = motor_devices[i]->config;
    if (data->online && data->missed_times <= 5) {
      int can_id = get_can_id(motor_devices[i]);

      mi_motor_pack(motor_devices[i], &tx_frame[0]);
      struct tx_frame queued_frame = {
          .can_dev = cfg->common.phy,
          .sem = &tx_queue_sem[can_id],
          .frame = tx_frame[0],
      };

      mi_send_queued(&queued_frame, &mi_can_tx_msgq);
      if ((data->common.mode == PV) || (data->common.mode == VO)) {
        mi_motor_pack(motor_devices[i], &tx_frame[1]);
        struct tx_frame queued_frame = {
            .can_dev = cfg->common.phy,
            .sem = &tx_queue_sem[can_id],
            .frame = tx_frame[1],
        };

        mi_send_queued(&queued_frame, &mi_can_tx_msgq);
      }
      data->missed_times++;
      if (data->missed_times > 5) {
        LOG_ERR("Motor %d is not responding, missed %d times, setting it "
                "to "
                "offline...",
                i, data->missed_times);
        continue;
      }
      if (data->missed_times > 3 && data->err > 1) {
        mi_motor_control(motor_devices[i], CLEAR_ERROR);
      }
    }
  }
  mi_queue_proceed(&mi_can_tx_msgq);
}

void mi_init_handler(struct k_work *work) {
  k_timer_stop(&mi_tx_timer);
  LOG_DBG("mi motor control thread started");

  uint32_t id_full1[CAN_COUNT] = {0};
  uint32_t id_full0[CAN_COUNT] = {0};
  for (int i = 0; i < MOTOR_COUNT; i++) {
    int can_id = 0;
    const struct mi_motor_cfg *cfg =
        (const struct mi_motor_cfg *)(motor_devices[i]->config);

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
    int err = can_add_rx_filter(can_dev, mi_can_rx_handler, 0, &filters[i]);
    if (err < 0) {
      LOG_ERR("Error adding CAN filter (err %d)", err);
    }
    // If you recieved an error here, remember that 2# CAN of STM32 is in slave
    // mode and does not have an independent filter. If that is the issue, you
    // can ignore that.
  }

  k_sleep(K_MSEC(500));

  for (int i = 0; i < MOTOR_COUNT; i++) {
    mi_motor_control(motor_devices[i], ENABLE_MOTOR);
  }

  k_timer_start(&mi_tx_timer, K_NO_WAIT, K_MSEC(2));
  k_timer_user_data_set(&mi_tx_timer, &mi_tx_data_handle);
}

DT_INST_FOREACH_STATUS_OKAY(MIMOTOR_INST)
