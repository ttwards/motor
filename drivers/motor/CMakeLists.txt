# Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_MOTOR_DJI dji)

add_subdirectory_ifdef(CONFIG_MOTOR_MI mi)

add_subdirectory_ifdef(CONFIG_MOTOR_DM dm)

add_subdirectory_ifdef(CONFIG_MOTOR_COMMON common)
