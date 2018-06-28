/*
 * typedef.h
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#ifndef __TYPEDEF_H
#define __TYPEDEF_H

#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_i2c.h>
#include <system_stm32f10x.h>
#include <stm32f10x_flash.h>
#include <math.h>
#include <misc.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

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

#endif
