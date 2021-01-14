# Copyright 2018 NXP
# Use of this source code is governed by a MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS += \
	$(LOCAL_DIR)/sai_hw.c \
	$(LOCAL_DIR)/sai.c

#
# Uncomment this to active SAI debug
#
#GLOBAL_DEFINES += \
#	IMX_SAI_AUTODETECT

include make/module.mk
