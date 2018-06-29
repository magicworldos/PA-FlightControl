/*
 * sbus.h
 *
 *  Created on: Jun 29, 2018
 *      Author: lidq
 */

#ifndef SBUS_H_
#define SBUS_H_

#include <typedef.h>
#include <uart2.h>

#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f
#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f

#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f

#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f

#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

int sbus_read(uint16_t *rc_values, uint8_t *rc_flag);

int sbus_parse(char *buff, int len, uint16_t *val, uint8_t *flag, int *result);

#endif /* SBUS_H_ */
