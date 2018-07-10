/*
 * pwmout.c
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#include <pwmout.h>

uint16_t _pwmout[PWM_NUMCOUNTS] = { 0 };

void pwmout_init()
{
	pwmout_gpio_config();

	pwmout_mode_config();

	pwmout_set_value();
}

void pwmout_gpio_config()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

#ifdef __MINI_BOARD_MODE_
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
#else
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7;
#endif
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void pwmout_mode_config()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_TimeBaseStructure.TIM_Period = TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler = TIM_PRESCALER;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM_PULSE;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

#ifdef __MINI_BOARD_MODE_
#else
	//OUT1
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	//OUT2
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	//OUT3
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	//OUT4
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
#endif
	//OUT5
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	//OUT6
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	//OUT7
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	//OUT8
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	pwmout_set_failsafe();

#ifdef __MINI_BOARD_MODE_
#else
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
#endif
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);

	pwmout_set_failsafe();
}

void pwmout_set_failsafe()
{
	for (int i = 0; i < PWM_NUMCOUNTS; i++)
	{
		_pwmout[i] = PWM_FAILSAFE;
	}
	pwmout_set_value();
}

void pwmout_set_value()
{
#ifdef __MINI_BOARD_MODE_
	TIM_SetCompare1(TIM3, _pwmout[1]);
	TIM_SetCompare2(TIM3, _pwmout[3]);
	TIM_SetCompare3(TIM3, _pwmout[2]);
	TIM_SetCompare4(TIM3, _pwmout[0]);
#else
	TIM_SetCompare1(TIM2, _pwmout[0]);
	TIM_SetCompare2(TIM2, _pwmout[1]);
	TIM_SetCompare3(TIM2, _pwmout[2]);
	TIM_SetCompare4(TIM2, _pwmout[3]);
	TIM_SetCompare1(TIM3, _pwmout[4]);
	TIM_SetCompare2(TIM3, _pwmout[5]);
	TIM_SetCompare3(TIM3, _pwmout[6]);
	TIM_SetCompare4(TIM3, _pwmout[7]);
#endif

}
