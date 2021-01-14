LOCAL_DIR := $(GET_LOCAL_DIR)

TARGET := imx8mn/dts

MODULES += \
	app/i2c \
	app/gpt \
	app/sai_tx \
	app/pdm_play \
	app/audio_mixer \
	app/hifiberry \
	app/spdif \
	lib/dpc \
	lib/klog \
	lib/version

MODULES += \
        dev/ivshmem/services/console \
        dev/ivshmem/services/binary \
        dev/ivshmem/services/rpc \

MODULES += \
	dev/pca6416 \
	dev/dac/wm8524 \
	dev/dac/pcm512x \
	dev/adc/pcm186x

GLOBAL_DEFINES += \
	IMX_SAI_PERMISSIVE=1 \
	IMX_SAI_WARMUP_NR_PERIODS=1 \
	IMX_SAI_WARMUP_NR_PERIODS_FRAC=4

include project/virtual/test.mk

