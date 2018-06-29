/*
 * uart.h
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#ifndef SRC_UART2_H_
#define SRC_UART2_H_

#include <typedef.h>
#include <protocol.h>

#define USART2_BAUDRATE	(100000)

void uart2_init(void);

void uart2_gpio_configuration(void);

void uart2_configuration(void);

u8 uart2_putchar(u8 ch);

void uart2_write(char* buf, int len);

int uart2_buff_count(void);

int uart2_read(char* buf, int len);

#endif /* SRC_UART_H_ */
