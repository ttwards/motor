# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_MOTOR_DJI motor_dji.c)
zephyr_library_sources_ifdef(CONFIG_MOTOR_DM motor_dm.c)