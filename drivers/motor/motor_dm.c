#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "motor_dm.h"

#define DT_DRV_COMPAT dm_motor

DT_INST_FOREACH_STATUS_OKAY(DMMOTOR_INST)