LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS += \
    $(LOCAL_DIR)/rpc.c \

MODULE_DEPS += dev/ivshmem/services/rpc

include make/module.mk
