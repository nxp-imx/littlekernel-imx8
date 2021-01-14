# Copyright 2019-2020 NXP
# Use of this source code is governed by a MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_DEPS += dev/dma

MODULE_SRCS += \
	$(LOCAL_DIR)/sdma_hw.c \
	$(LOCAL_DIR)/sdma.c

MODULE_DEFINES += FSL_SDK_DISABLE_IRQ=1 \
	FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL=1

ifeq ($(IMX_ENABLE_SDMA_OCRAM), true)
MODULE_CFLAGS := -mcmodel=large
$(info using OCRAM for SDMA descriptors)
MODULE_DEFINES += \
	IMX_SDMA_USE_OCRAM=1 \
	IMX_SDMA_NUM=$(IMX_SDMA_NUM_INSTANCES)
else
$(info using DDR allocation for SDMA descriptors)
endif

# memset() crashes when memory is not aligned to 16 bytes.
# BD (buffer descriptors) is 12-byte long. When a BD is accessed as an array
# item BD is aligned to a 12-byte boundary causing a 16-byte misalign.
# It occurs in SDMA_ConfigBufferDescriptor() and SDMA_SubmitTransfer()
MODULE_CFLAGS += \
        -mstrict-align

include make/module.mk
