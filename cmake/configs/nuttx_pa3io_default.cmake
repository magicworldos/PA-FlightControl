
px4_nuttx_configure(HWCLASS m3 CONFIG nsh)

set(config_module_list
	drivers/boards/pa3io
	drivers/stm32
	lib/mixer
	lib/rc
	modules/px4iofirmware
	platforms/common
)
