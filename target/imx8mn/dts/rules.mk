# Copyright 2019-2021 NXP
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

PLATFORM := imx8
PLATFORM_SOC := imx8mn
IMX_ENABLE_ASRC_HW := y

CONFIG_CONSOLE_TTY_BASE := 0x30A60000

GLOBAL_DEFINES += \
	UART_IRQ=61

MODULE_SRCS := \
	$(LOCAL_DIR)/target.c \
	$(LOCAL_DIR)/board.c

MODULE_DEPS := lib/appargs

DTSS := $(notdir $(wildcard $(LOCAL_DIR)/*.dts))
DTBS := $(addprefix $(BUILDDIR)/,$(patsubst %.dts,%.dtb,$(DTSS)))

GENERATED += $(DTBS)
EXTRA_BUILDDEPS += $(DTBS)

DTS_INCDIR := $(LOCAL_DIR)/dts_include

$(BUILDDIR)/%.dtb: $(LOCAL_DIR)/%.dts
	@echo generating $@
	$(NOECHO)$(CC) -E -nostdinc -I $(DTS_INCDIR) -undef, -x assembler-with-cpp $< > $@.tmp
	$(NOECHO)dtc -I dts -O dtb $@.tmp -o $@

dtbs: $(DTBS)
include make/module.mk
