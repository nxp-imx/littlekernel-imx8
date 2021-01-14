# Copyright 2019 NXP
# Use of this source code is governed by a MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS += \
	$(LOCAL_DIR)/spdif_hw.c \
	$(LOCAL_DIR)/spdif.c

MODULE_DEFINES += FSL_SDK_DISABLE_IRQ=1 \
	FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL=1

MODULE_DEPS += platform/imx8/drivers/spdif_cs

include make/module.mk
