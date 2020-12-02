LOCAL_DIR := $(GET_LOCAL_DIR)
MODULES_INCLUDES += $(LOCAL_DIR)/include
MODULE := $(LOCAL_DIR)

MODULE_SRCS += \
	$(LOCAL_DIR)/ivshmem-binary.c \
	$(LOCAL_DIR)/event-manager.c

MODULE_DEPS += dev/ivshmem

include make/module.mk
