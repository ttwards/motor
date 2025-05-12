# Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

set(PLOTTER_SOURCES
	aresplot_protocol.c
	aresplot_uart.c
)

zephyr_library_sources(${PLOTTER_SOURCES})