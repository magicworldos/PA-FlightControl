/*
 * uart.c
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#include <uart2.h>

static s_buff _recv2;

void uart2_init(void)
{
	_recv2.head = 0;
	_recv2.tail = 0;
	_recv2.size = SIZE_BUFF;
	memset(_recv2.buff, 0x00, SIZE_BUFF);

	uart2_gpio_configuration();
	uart2_configuration();
}

void uart2_gpio_configuration(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void uart2_configuration(void)
{
	USART_InitTypeDef usart2_init_struct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	USART_Cmd(USART2, ENABLE);
	usart2_init_struct.USART_BaudRate = USART2_BAUDRATE;
	usart2_init_struct.USART_WordLength = USART_WordLength_8b;
	usart2_init_struct.USART_StopBits = USART_StopBits_1;
	usart2_init_struct.USART_Parity = USART_Parity_No;
	usart2_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart2_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2, &usart2_init_struct);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART2_IRQn);
}

u8 uart2_putchar(u8 ch)
{
	USART_SendData(USART2, (u8) ch);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
	{
	}
	return ch;
}

void uart2_write(char* buf, int len)
{
	for (int i = 0; i < len; i++)
	{
		uart2_putchar(*buf++);
	}
}

int uart2_buff_count(s_buff *lb)
{
	int16_t n = lb->head - lb->tail;
	if (n < 0)
	{
		n += lb->size;
	}
	return n;
}

int uart2_read(char* buf, int len)
{
	int cnt = uart2_buff_count(&_recv2);
	for (int i = 0; i < cnt; i++)
	{
		buf[i] = _recv2.buff[_recv2.tail];
		_recv2.tail = (_recv2.tail + 1) % _recv2.size;
	}
	return cnt;
}

void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		while ((USART2->SR & USART_SR_RXNE) == 0)
		{
		}
		_recv2.buff[_recv2.head] = USART_ReceiveData(USART2);
		_recv2.head = (_recv2.head + 1) % _recv2.size;
		if (_recv2.head == _recv2.tail)
		{
			_recv2.over++;
		}
	}
}

