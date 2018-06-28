/*
 * pwmout.h
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#ifndef __pwmout_H
#define __pwmout_H

#include <typedef.h>

#define TIM_PRESCALER		(72)
#define TIM_PERIOD			(2000 -1)
#define TIM_PULSE			(1000 - 1)
#define PWM_FAILSAFE		(800)
#define PWM_NUMCOUNTS		(16)

void pwmout_init(void);

void pwmout_gpio_config(void);

void pwmout_mode_config(void);

void pwmout_set_failsafe(void);

void pwmout_set_value(void);

#endif
