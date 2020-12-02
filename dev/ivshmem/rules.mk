LOCAL_DIR := $(GET_LOCAL_DIR)

GLOBAL_INCLUDES += \
	$(LOCAL_DIR)/include

MODULE := $(LOCAL_DIR)

MODULE_DEPS += dev/jailhouse lib/cbuf lib/iovec

MODULE_SRCS += \
	$(LOCAL_DIR)/ivshmem.c \
	$(LOCAL_DIR)/ivshmem-pipe.c \
	$(LOCAL_DIR)/ivshmem-services.c \
	$(LOCAL_DIR)/ivshmem-endpoint.c

ifeq ($(IMX_IVSHMEM_STATS), true)
GLOBAL_DEFINES += IVSHMEM_MONITOR
endif

include make/module.mk
