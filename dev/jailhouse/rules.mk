LOCAL_DIR := $(GET_LOCAL_DIR)

GLOBAL_INCLUDES += \
	$(LOCAL_DIR)/include \
	$(LOCAL_DIR)/include/arch/arm-common \
	$(LOCAL_DIR)/include/arch/$(ARCH)


MODULE := $(LOCAL_DIR)

MODULE_SRCS += \
	$(LOCAL_DIR)/empty.c \

include make/module.mk
