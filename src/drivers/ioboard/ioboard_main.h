/*
 * ioborad.h
 *
 *  Created on: Jun 27, 2018
 *      Author: lidq
 */

#ifndef SRC_DRIVERS_IOBOARD_IOBOARD_MAIN_H_
#define SRC_DRIVERS_IOBOARD_IOBOARD_MAIN_H_

#include "ioboard_typedef.h"
#include "ioboard_protocol.h"

int ioboard_main(int argc, char *argv[]);

static int start(int argc, char *argv[]);

static int stop(void);

static int ioboard_write(int argc, char *argv[]);

static int ioboard_read(void);

static int ioboard_handle_rc(void *data);

static int ioboard_handle_battery(void *data);

static int ioboard_set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);

#endif /* SRC_DRIVERS_IOBOARD_IOBOARD_MAIN_H_ */
