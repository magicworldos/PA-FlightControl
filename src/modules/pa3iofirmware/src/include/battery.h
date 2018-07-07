/*
 * battery.h
 *
 *  Created on: Jul 7, 2018
 *      Author: lidq
 */

#ifndef SRC_MODULES_PA3IOFIRMWARE_SRC_INCLUDE_BATTERY_H_
#define SRC_MODULES_PA3IOFIRMWARE_SRC_INCLUDE_BATTERY_H_


#include <typedef.h>

void battery_init(void);

float battery_read(void);


#endif /* SRC_MODULES_PA3IOFIRMWARE_SRC_INCLUDE_BATTERY_H_ */
