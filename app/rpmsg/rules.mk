LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS += \
    $(LOCAL_DIR)/rpmsg.c \
    $(LOCAL_DIR)/em.c

include make/module.mk
