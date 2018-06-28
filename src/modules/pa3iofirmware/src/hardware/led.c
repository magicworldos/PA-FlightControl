/*
 * led_light.c
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#include <led.h>

extern u32 timer_tick;

void led_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void led0_on()
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
}

void led0_off()
{
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

void led0_blink(u32 usecs)
{
	if (timer_tick % usecs < usecs / 2)
	{
		led0_on();
	}
	else
	{
		led0_off();
	}
}
