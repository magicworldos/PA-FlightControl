/*
 * uart.c
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#include <uart1.h>

extern s_buff _recv;

void uart1_init(void)
{
	uart1_gpio_configuration();
	uart1_configuration();
}

void uart1_gpio_configuration(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void uart1_configuration(void)
{
	USART_InitTypeDef usart1_init_struct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	USART_Cmd(USART1, ENABLE);
	usart1_init_struct.USART_BaudRate = USART1_BAUDRATE;
	usart1_init_struct.USART_WordLength = USART_WordLength_8b;
	usart1_init_struct.USART_StopBits = USART_StopBits_1;
	usart1_init_struct.USART_Parity = USART_Parity_No;
	usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &usart1_init_struct);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART1_IRQn);
}

u8 uart1_putchar(u8 ch)
{
	USART_SendData(USART1, (u8) ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
	{
	}
	return ch;
}

void uart1_write(u8* buf, u8 len)
{
	for (u8 i = 0; i < len; i++)
	{
		uart1_putchar(*buf++);
	}
}

void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		while ((USART1->SR & USART_SR_RXNE) == 0)
		{
		}
		_recv.buff[_recv.head] = USART_ReceiveData(USART1);
		_recv.head = (_recv.head + 1) % _recv.size;
		if (_recv.head == _recv.tail)
		{
			_recv.over++;
		}
	}
}

