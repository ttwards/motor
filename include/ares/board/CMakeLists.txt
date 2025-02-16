# Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

file(GLOB INIT_SOURCES "*.c")
zephyr_library_sources(${INIT_SOURCES})