# Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
# SPDX-License-Identifier: Apache-2.0

# FIXME: SHADOW_VARS: Remove this once we have enabled -Wshadow globally.
# add_compile_options($<TARGET_PROPERTY:compiler,warning_shadow_variables>)

# add_definitions(-D__ZEPHYR_SUPERVISOR__)

add_subdirectory_ifdef(CONFIG_MOTOR_DJI motor)
add_subdirectory_ifdef(CONFIG_MOTOR_DJI pid)

add_subdirectory_ifdef(CONFIG_MOTOR_MI motor)
add_subdirectory_ifdef(CONFIG_MOTOR_MI pid)

add_subdirectory_ifdef(CONFIG_MOTOR_DM motor)
add_subdirectory_ifdef(CONFIG_MOTOR_DM pid)

add_subdirectory_ifdef(CONFIG_ARES_SBUS transfer)

add_subdirectory_ifdef(CONFIG_CHASSIS chassis)

add_subdirectory_ifdef(CONFIG_WHEEL wheel)

if(CONFIG_IMU_PWM_TEMP_CTRL AND NOT CONFIG_MOTOR_DJI AND NOT CONFIG_MOTOR_MI AND NOT CONFIG_MOTOR_DM)
    add_subdirectory(pid)
endif()