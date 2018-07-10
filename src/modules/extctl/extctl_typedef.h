/*
 * extctl_typedef.h
 *
 *  Created on: Jun 7, 2018
 *      Author: lidq
 */

#ifndef SRC_MODULES_EXTCTL_EXTCTL_TYPEDEF_H_
#define SRC_MODULES_EXTCTL_EXTCTL_TYPEDEF_H_

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <float.h>
#include <string.h>
#include <pthread.h>
#include <poll.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_module.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <termios.h>
#include <semaphore.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <semaphore.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <setjmp.h>
#include <sys/syscall.h>
#include <signal.h>
#include <fcntl.h>
#include <dirent.h>
#include <poll.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/commander_state.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/extctl_sp.h>
#include <uORB/topics/ext_vehicle_pos.h>
#include <uORB/topics/ext_vehicle_sp.h>
#include <uORB/topics/ext_rc.h>
#include <uORB/topics/ext_sys_status.h>
#include <uORB/topics/ext_cmd.h>

#ifdef __PX4_POSIX
#define CONFIG_PTHREAD_STACK_DEFAULT	(2048)
#define UNIX_DOMAIN 					"/tmp/UNIX.domain"
#else
#define DEV_NAME						"/dev/ttyUSB2"
#endif

#define DEV_BAUDRATE					(B115200)
#define DEV_RATE_BASE					(1000 * 1000)
#define DEV_RATE_READ					(DEV_RATE_BASE / 30)
#define DEV_RATE_POS					(DEV_RATE_BASE / 10)
#define DEV_RATE_RC						(DEV_RATE_BASE / 10)
#define DEV_RATE_SP						(DEV_RATE_BASE / 10)
#define DEV_RATE_STATUS					(DEV_RATE_BASE / 5)

enum EXTCTL_DATA_TYPE
{
	DATA_TYPE_POS = 0,
	DATA_TYPE_SP,
	DATA_TYPE_RC,
	DATA_TYPE_CMD,
	DATA_TYPE_STATUS,
	DATA_TYPE_END,
};

#endif /* SRC_MODULES_EXTCTL_EXTCTL_TYPEDEF_H_ */
