# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0
#
# This CMake file is picked by the Zephyr build system because it is defined
# as the module CMake entry point (see zephyr/module.yml).


# This is needed so that custom driver classes using system calls are taken into
# account
# zephyr_syscall_include_directories(include)

# zephyr_include_directories(include)

# add_subdirectory(drivers)
# add_subdirectory(lib)

set(ENV{ZEPHYR_BASE} "${CMAKE_CURRENT_SOURCE_DIR}/../zephyr")
list(APPEND BOARD_ROOT .)

# zephyr_library()
zephyr_include_directories(include)
zephyr_syscall_include_directories(include)
add_subdirectory(drivers)
# zephyr_syscall_header(${CMAKE_CURRENT_SOURCE_DIR}/include/zephyr/drivers/motor.h)
# zephyr_library_sources(
#     drivers/motor/motor_m3508.c
#     drivers/motor/motor_dji.h
#     )