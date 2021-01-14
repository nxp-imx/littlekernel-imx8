# copy this and makefile to your external root directory and customize
# according to how you want to use a split repository

# the top level directory that all paths are relative to

LOCAL_DIR := imx8
LKMAKEROOT := ..

# paths relative to LKMAKEROOT where additional modules should be searched
LKINC := $(LOCAL_DIR)

# the path relative to LKMAKEROOT where the main lk repository lives
LKROOT := lk

# set the directory relative to LKMAKEROOT where output will go
BUILDROOT ?= $(LOCAL_DIR)

# set the default project if no args are passed
DEFAULT_PROJECT ?= imx8mm-dts

export ARCH_arm64_TOOLCHAIN_PREFIX ?= /opt/toolchains/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-elf/bin/aarch64-elf-
