# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_MOTOR_M3508 motor_m3508.c)
zephyr_library_sources_ifdef(CONFIG_MOTOR_M3508 motor_dji.h)