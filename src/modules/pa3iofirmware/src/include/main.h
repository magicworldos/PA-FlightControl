/*
 * main.h
 *
 *  Created on: Jun 28, 2018
 *      Author: lidq
 */

#ifndef SRC_MODULES_PA3IOFIRMWARE_SRC_INCLUDE_MAIN_H_
#define SRC_MODULES_PA3IOFIRMWARE_SRC_INCLUDE_MAIN_H_

#include <typedef.h>
#include <led.h>
#include <timer.h>
#include <protocol.h>
#include <uart1.h>
#include <uart2.h>
#include <pwmout.h>
#include <sbus.h>

int main(int argc, char* argv[]);

static void handle_protocol(void);

static void read_rc(void);

static void handle_pwmout(void *data);

static void send_battery(void);

#endif /* SRC_MODULES_PA3IOFIRMWARE_SRC_INCLUDE_MAIN_H_ */
