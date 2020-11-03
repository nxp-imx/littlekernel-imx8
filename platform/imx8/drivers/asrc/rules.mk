# Copyright 2020 NXP
# Use of this source code is governed by a MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS += \
	$(LOCAL_DIR)/asrc_hw.c \
	$(LOCAL_DIR)/asrc.c

MODULE_DEFINES += FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL=1

include make/module.mk
