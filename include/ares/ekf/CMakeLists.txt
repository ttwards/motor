# Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

file(GLOB EKF_SOURCES
	"${CMAKE_CURRENT_SOURCE_DIR}/*.c"
)

zephyr_library_sources(${EKF_SOURCES})

zephyr_library_compile_options(
    -O3
    -fno-strict-overflow
    -fno-common
    -ffunction-sections
    -fdata-sections
    -ffreestanding
    -fno-builtin
)