/*
 * hil_main.h
 *
 *  Created on: Jun 19, 2018
 *      Author: lidq
 */

#ifndef SRC_MODULES_HIL_HIL_MAIN_H_
#define SRC_MODULES_HIL_HIL_MAIN_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <float.h>
#include <sys/time.h>
#include <px4_tasks.h>
#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>
#include <lib/geo/geo.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

#include "matrix.h"

#ifdef __PX4_POSIX
#define CONFIG_PTHREAD_STACK_DEFAULT	(2048)
#endif

#define home_lat		40.5397970
#define home_lon		121.5037566
#define home_alt		50.0

#define MAX_ANGLE		(60.0 * M_PI / 180.0)		//角度限幅 弧度/s
#define F_OMEGA			(28.0)						//转桨拉力系数
#define M_KG			(1.8)						//质量 kg
#define G_MS2			(9.80665)					//重力加速度 米/s^2
#define I_X				(0.04348)					//x轴转动惯量
#define I_Y				(0.04348)					//y轴转动惯量
#define I_Z				(6.667)						//z轴转动惯量

#define M 3
#define N 3
#define AT(i, j, n) (i * n + j)

struct quat
{
	float w;
	float x;
	float y;
	float z;
};

static void TransMatrix_R_vb_set_value(s_Matrix *R_vb, double angle_x, double angle_y, double angle_z);

static void TransMatrix_R_Q_set_value(s_Matrix *R_vb, double w, double x, double y, double z);

static void F_body_from_omega(double *omega_val, double *f_body_x, double *f_body_y, double *f_body_z);

static void Acc_global_from_F(double *f_global, double *acc_x, double *acc_y, double *acc_z);

static void hil_init(void);

static void hil_zero(void);

static void hil_maxmin(double *val, double max, double min);

static void hil_cal(double theta_t);

static void hil_angle2q(double *xyz, struct quat *q);

static void hil_pub_att(void);

static void hil_pub_local_pos(void);

static void hil_pub_global_pos(void);

static void hil_pub_gps(void);

static int hil_task_main(int argc, char *argv[]);

static int start(int argc, char *argv[]);

int hil_main(int argc, char *argv[]);

#endif /* SRC_MODULES_HIL_HIL_MAIN_H_ */
