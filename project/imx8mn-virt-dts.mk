include project/imx8mn-dts.mk
GLOBAL_DEFINES += APPARGS_FDT_OFFSET=0x1000

MODULES +=\
	dev/ivshmem/services/console \
	dev/ivshmem/services/binary \
	dev/ivshmem/services/rpc

