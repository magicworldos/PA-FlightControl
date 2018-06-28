/*
 * uart.h
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#ifndef SRC_UART_H_
#define SRC_UART_H_

#include <typedef.h>

void uart1_init(void);

void uart1_gpio_configuration(void);

void uart1_configuration(void);

u8 uart1_putchar(u8 ch);

void uart1_write(u8* buf, u8 len);

#endif /* SRC_UART_H_ */
