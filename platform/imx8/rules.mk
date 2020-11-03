
# Copyright (c) 2017, Google, Inc. All rights reserved
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files
# (the "Software"), to deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge,
# publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

ARCH := arm64
ARM_CPU := cortex-a53
WITH_SMP := 1

GLOBAL_INCLUDES += \
	$(LOCAL_DIR)/common/include \
	$(LOCAL_DIR)/common/include/core \
	$(LOCAL_DIR)/drivers/include \
	$(LOCAL_DIR)/soc/$(PLATFORM_SOC)/include \

MODULE_SRCS := \
	$(LOCAL_DIR)/debug.c \
	$(LOCAL_DIR)/platform.c \
	$(LOCAL_DIR)/power.c

MODULE_SRCS += \
	$(LOCAL_DIR)/drivers/fuse_check.c

MODULES += \
		   lib/dpc

MODULE_DEPS += \
	$(LOCAL_DIR)/drivers/gpr

MODULE_DEPS += \
	$(LOCAL_DIR)/drivers/i2c

MODULE_DEPS += \
	$(LOCAL_DIR)/drivers/sai

MODULE_DEPS += \
	$(LOCAL_DIR)/drivers/pdm

MODULE_DEPS += \
	$(LOCAL_DIR)/drivers/uart

MODULE_DEPS += \
	$(LOCAL_DIR)/drivers/spdif

MODULE_DEPS += \
	$(LOCAL_DIR)/drivers/gpio

MODULE_DEPS += \
	$(LOCAL_DIR)/drivers/gpt

MODULE_DEPS += \
	$(LOCAL_DIR)/drivers/sdma

MODULE_DEPS += \
	$(LOCAL_DIR)/drivers/clock

ifeq ($(IMX_ENABLE_ASRC_HW), y)
IMX_USE_ASRC_HW := 1
MODULE_DEPS += \
	$(LOCAL_DIR)/drivers/asrc
else
IMX_USE_ASRC_HW := 0
endif

-include $(LOCAL_DIR)/soc/$(PLATFORM_SOC)/rules.mk

MEMBASE ?= 0xC0000000
MEMSIZE ?= 0x20000000
KERNEL_LOAD_OFFSET ?= 0x010000

ifeq ($(IMX_ENABLE_SDMA_OCRAM), true)
IMX_USE_OCRAM := 1
SRAM_PBASE ?= 0x900000
SRAM_VBASE ?= 0xFFFFFFFF28000000
SRAM_SIZE ?= 0x20000

GENERATED += \
	$(BUILDDIR)/ocram.ld.S

$(BUILDDIR)/ocram.ld: $(LOCAL_DIR)/ocram.ld.S
	@echo generating $@
	@$(MKDIR)
	$(NOECHO)sed "\
		s/%SRAM_PBASE%/$(SRAM_PBASE)/;\
		s/%SRAM_VBASE%/$(SRAM_VBASE)/;\
		s/%SRAM_SIZE%/$(SRAM_SIZE)/;\
		s/%MEMBASE%/$(MEMBASE)/;\
		s/%MEMSIZE%/$(MEMSIZE)/;\
		s/%KERNEL_BASE%/$(KERNEL_BASE)/;\
		s/%KERNEL_LOAD_OFFSET%/$(KERNEL_LOAD_OFFSET)/" < $< > $@.tmp
	@$(call TESTANDREPLACEFILE,$@.tmp,$@)
else
IMX_USE_OCRAM := 0
endif

ifeq ($(IMX_PERMISSIVE_MODE), true)
GLOBAL_DEFINES += \
	ENABLE_PERMISSIVE_MODE
endif

MODULE_DEPS += \
	dev/psci \
	dev/interrupt/arm_gic \
	dev/timer/arm_generic \

MODULE_DEPS += lib/debuglog
MODULE_DEPS += lib/appargs

GLOBAL_DEFINES += \
	ENABLE_KERNEL_LL_DEBUG=0 \
	SOC_$(PLATFORM_SOC)=1 \
	CONFIG_CONSOLE_TTY_BASE=$(CONFIG_CONSOLE_TTY_BASE) \
	PLATFORM_SUPPORTS_PANIC_SHELL=1 \
	MEMBASE=$(MEMBASE) \
	MEMSIZE=$(MEMSIZE) \
	SRAM_SIZE=$(SRAM_SIZE) \
	SRAM_PBASE=$(SRAM_PBASE) \
	SRAM_VBASE=$(SRAM_VBASE) \
	IMX_USE_OCRAM=${IMX_USE_OCRAM} \
	MMU_WITH_TRAMPOLINE=1

ifeq ($(RELEASE),)
GLOBAL_DEFINES += \
	WITH_KERNEL_EVLOG=1 \

MODULE_DEPS += \
	lib/evlog
endif

#
# FSL imported drivers options
#
GLOBAL_DEFINES += \
	SDK_DEBUGCONSOLE=0

LINKER_SCRIPT += \
	$(BUILDDIR)/system-onesegment.ld

ifeq ($(IMX_ENABLE_SDMA_OCRAM), true)
EXTRA_LINKER_SCRIPTS += $(BUILDDIR)/ocram.ld
endif

WITH_CPP_SUPPORT=true
WITH_DEV_INTERRUPT_ARM_GIC=true
WITH_DEV_INTERRUPT_ARM_GIC_V3=true
WITH_DEBUG_LINEBUFFER=true

include make/module.mk
