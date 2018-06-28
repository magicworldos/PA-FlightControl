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
#include <pwmout.h>

int main(int argc, char* argv[]);

static void handle_protocol(void);

static void handle_pwmout(void *data);

#endif /* SRC_MODULES_PA3IOFIRMWARE_SRC_INCLUDE_MAIN_H_ */
