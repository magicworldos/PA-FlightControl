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
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

#include "matrix.h"

#define MAX_ANGLE_RATE	(90.0 * M_PI / 180.0)	//角速度限幅	弧度/s^2
#define MAX_ANGLE		(60.0 * M_PI / 180.0)	//角度限幅 弧度/s
#define MAX_ACC_BODY	(60)					//机体加速度限幅 m/s^2
#define MAX_VEL_BODY_XY	(35)					//机体速度限幅水平方向 m/s
#define MAX_VEL_BODY_Z	(5)						//机体速度限幅垂直方向 m/s
#define MIN_MID_ZERO	(0.01)					//中位归0限幅
#define ACC_MID			(0.45)					//控制量升力中位值(0.0~1.0)

#define Ka_x			(3.6)					//旋转角加速度系数x
#define Ka_y			(3.6)					//旋转角加速度系数y
#define Ka_z			(0.2)					//旋转角加速度系数z

#define Aair			(5)					//空气阻尼加速度xy

#define Kacc_x			(16.0)					//机体加速度系数x
#define Kacc_y			(16.0)					//机体加速度系数y
#define Kacc_z			(6.0)					//机体加速度系数z

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

static void TransMatrix_R_vb_set_value(s_Matrix *R_vb, double theta);

static void AngularAcc_body_from_omega(double *omega, double *a0, double *a1, double *a2);

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
