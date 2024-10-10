# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${CMAKE_SOURCE_DIR}/include/zephyr/drivers/motor.h)
zephyr_library()

zephyr_library_sources_ifdef()