/*
 * Extern_control.h
 *
 *  Created on: Apr 21, 2018
 *      Author: lidq
 */

#ifndef SRC_DRIVERS_Extern_CONTROL_EXTERN_CONTROL_H_
#define SRC_DRIVERS_Extern_CONTROL_EXTERN_CONTROL_H_

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
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
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/extctl_sp.h>

#define DEV_NAME			"/dev/ttyS2"
#define DEV_BAUDRATE		(B115200)
#define DEV_RW_RATE			(100)
#define DEV_RW_RATE_RC		(5)
#define DEV_RW_USLEEP		(1000 * 1000 / DEV_RW_RATE)
#define DEV_RW_USLEEP_RC	(1000 * 1000 / DEV_RW_RATE_RC)

#define FRM_HEAD_0		0X55
#define FRM_HEAD_1		0XAA
#define FRM_FOOT_0		0XA5
#define FRM_FOOT_1		0X5A

#define PAR_HEAD		0
#define PAR_LEN			1
#define PAR_END			2

#define SIZE_BUFF		(0x200)

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

typedef struct vehicle_pos_s
{
	//position local
	struct
	{
		float x;
		float y;
		float z;
	};
	struct
	{
		float vx;
		float vy;
		float vz;
	};
	//position global
	struct
	{
		double lat;
		double lon;
		float alt;
	};
	struct
	{
		float vel_n;
		float vel_e;
		float vel_d;
	};
} vehicle_pos_s;

typedef struct vehicle_sp_s
{
	bool run_pos_control;
	bool run_alt_control;
	float yaw;
	struct
	{
		float sp_x;
		float sp_y;
		float sp_z;
	};
	struct
	{
		float vel_sp_x;
		float vel_sp_y;
		float vel_sp_z;
	};
} vehicle_sp_s;

typedef struct rc_s
{
	bool rc_failsafe;
	bool rc_lost;
	uint32_t channel_count;
	uint16_t values[18];
} rc_s;

enum data_type
{
	DATA_TYPE_POS = 0,
	DATA_TYPE_SP,
	DATA_TYPE_RC,
	DATA_TYPE_END,
};

int extctl_main(int argc, char *argv[]);

extern void bzero(void *s, int n);

static int frame_pos(int len_data);

static int frame_data(char *frame, int len_frame, char *data, int type, int len_data);

static int send_data_pos(vehicle_pos_s *pos);

static int send_data_sp(vehicle_sp_s *sp);

static int send_data_rc(rc_s *rc);

static int send_frame_data(char *frame, int len);

static int start(int argc, char *argv[]);

static int stop(void);

int frame_count(s_buff *lb);

int frame_parse(void);

void frame_read_data(void);

static int task_main_read(int argc, char* argv[]);

static int task_main_write(int argc, char* argv[]);

static int task_main_write_rc(int argc, char* argv[]);

static uint16_t crc16_value(uint8_t *buff, uint8_t len);

static int crc16_check(uint8_t *buff, uint8_t len, uint16_t crc16);

static int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);

#endif /* SRC_DRIVERS_Extern_CONTROL_EXTERN_CONTROL_H_ */
