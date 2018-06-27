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
#include "extctl_socket.h"
#include "extctl_status.h"

int extctl_main(int argc, char *argv[]);

static int start(int argc, char *argv[]);

static int stop(void);

static int extctl_read(int argc, char *argv[]);

#ifdef __PX4_POSIX
#else
static int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);
#endif

#endif /* SRC_DRIVERS_Extern_CONTROL_EXTERN_CONTROL_H_ */
