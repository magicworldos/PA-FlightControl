include(cmake/configs/posix_sitl_default.cmake)

add_definitions(
	-DCONFIG_HIL_MODE
)

set(config_sitl_rcS_dir
	posix-configs/SITL/init/test
	)
