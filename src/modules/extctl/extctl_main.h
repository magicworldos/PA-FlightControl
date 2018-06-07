/*
 * Extern_control.h
 *
 *  Created on: Apr 21, 2018
 *      Author: lidq
 */

#ifndef SRC_DRIVERS_Extern_CONTROL_EXTERN_CONTROL_H_
#define SRC_DRIVERS_Extern_CONTROL_EXTERN_CONTROL_H_

#include "extctl_typedef.h"

#include "extctl_sp.h"
#include "extctl_pos.h"
#include "extctl_rc.h"
#include "extctl_cmd.h"
#include "extctl_land.h"

int extctl_main(int argc, char *argv[]);

static int start(int argc, char *argv[]);

static int extctl_read(int argc, char *argv[]);

extern void bzero(void *s, int n);

static int frame_pos(int len_data);

static int frame_mk_data(char *frame, int len_frame, char *data, int type, int len_data);

static int send_frame_write(char *frame, int len);

static int stop(void);

static int frame_count(s_buff *lb);

static int frame_parse(void);

static void frame_read_data(void);

static uint16_t crc16_value(uint8_t *buff, uint8_t len);

static int crc16_check(uint8_t *buff, uint8_t len, uint16_t crc16);

static int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);

#endif /* SRC_DRIVERS_Extern_CONTROL_EXTERN_CONTROL_H_ */
