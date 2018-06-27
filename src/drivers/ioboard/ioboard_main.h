/*
 * ioborad.h
 *
 *  Created on: Jun 27, 2018
 *      Author: lidq
 */

#ifndef SRC_DRIVERS_IOBOARD_IOBOARD_MAIN_H_
#define SRC_DRIVERS_IOBOARD_IOBOARD_MAIN_H_

#include "ioboard_typedef.h"

int ioboard_main(int argc, char *argv[]);

static int start(int argc, char *argv[]);

static int stop(void);

static int ioboard_write(int argc, char *argv[]);

static int ioboard_read(void);

static int frame_pos(int len_data);

static int frame_mk_data(char *frame, int len_frame, char *data, int type, int len_data);

static int send_frame_write(char *frame, int len);

static int frame_count(s_buff *lb);

static int frame_parse(void);

static void frame_read_data(void);

static uint16_t crc16_value(uint8_t *buff, uint8_t len);

static int crc16_check(uint8_t *buff, uint8_t len, uint16_t crc16);

static int ioboard_set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);

#endif /* SRC_DRIVERS_IOBOARD_IOBOARD_MAIN_H_ */
