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
#include <stm32f10x_adc.h>
#include <math.h>
#include <misc.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define NO_MKTIME

typedef uint64_t hrt_abstime;
typedef hrt_abstime gps_abstime;

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

typedef struct battery_s
{
	float vcc;
} battery_s;

enum data_type
{
	DATA_TYPE_PWM_OUTPUT = 0,
	DATA_TYPE_RC_INPUT,
	DATA_TYPE_BATTERY,
	DATA_TYPE_END,
};

#endif
