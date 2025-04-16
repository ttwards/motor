# Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

set(USB_BULK_TRANS_SOURCES
	usb_trans.c
)

zephyr_library_sources(${USB_BULK_TRANS_SOURCES})