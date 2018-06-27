/*
 * extctl_typedef.h
 *
 *  Created on: Jun 7, 2018
 *      Author: lidq
 */

#ifndef SRC_DRIVERS_IOBOARD_IOBOARD_TYPEDEF_H_
#define SRC_DRIVERS_IOBOARD_IOBOARD_TYPEDEF_H_

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
#include <px4_getopt.h>

#include <uORB/topics/pwm_output.h>

#ifdef __PX4_POSIX
#define CONFIG_PTHREAD_STACK_DEFAULT	(2048)
#endif

#define DEV_NAME			"/dev/ttyAMA0"

#define DEV_BAUDRATE		(B115200)
#define DEV_RATE_BASE		(1000 * 1000)
#define DEV_RATE_W			(DEV_RATE_BASE / 100)
#define DEV_RATE_R			(DEV_RATE_BASE / 100)

#define FRM_HEAD_0			0X55
#define FRM_HEAD_1			0XAA
#define FRM_FOOT_0			0XA5
#define FRM_FOOT_1			0X5A

#define PAR_HEAD			0
#define PAR_LEN				1
#define PAR_END				2

#define SIZE_BUFF			(0x800)

typedef struct s_buff
{
	int16_t head;
	int16_t tail;
	int16_t size;
	uint8_t buff[SIZE_BUFF];
	uint32_t total_len;
	uint32_t over;
	uint32_t user_buf_over;
} s_buff;

typedef struct pwm_out_s
{
	uint16_t num_outputs;
	uint16_t pwm[16];

} pwm_out_s;

typedef struct rc_input_s
{
	bool rc_failsafe;
	bool rc_lost;
	uint32_t channel_count;
	uint16_t values[18];
} rc_input_s;

typedef struct gps_s
{
	uint64_t timestamp;
	uint64_t time_utc_usec;
	int32_t lat;
	int32_t lon;
	int32_t alt;
	int32_t alt_ellipsoid;
	float s_variance_m_s;
	float c_variance_rad;
	float eph;
	float epv;
	float hdop;
	float vdop;
	int32_t noise_per_ms;
	int32_t jamming_indicator;
	float vel_m_s;
	float vel_n_m_s;
	float vel_e_m_s;
	float vel_d_m_s;
	float cog_rad;
	int32_t timestamp_time_relative;
	uint8_t fix_type;
	bool vel_ned_valid;
	uint8_t satellites_used;
	uint8_t _padding0[5];
} gps_s;

typedef struct battery_s
{
	float vcc;
} battery_s;

enum data_type
{
	DATA_TYPE_PWM_OUTPUT = 0,
	DATA_TYPE_RC_INPUT,
	DATA_TYPE_GPS,
	DATA_TYPE_BATTERY,
	DATA_TYPE_END,
};

int ioboard_send_data_buff(void *data, int data_type, int data_len);

#endif /* SRC_MODULES_EXTCTL_EXTCTL_TYPEDEF_H_ */
