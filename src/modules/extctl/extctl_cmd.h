/*
 * extctl_cmd.h
 *
 *  Created on: Jun 7, 2018
 *      Author: lidq
 */

#ifndef SRC_MODULES_EXTCTL_EXTCTL_CMD_H_
#define SRC_MODULES_EXTCTL_EXTCTL_CMD_H_

#include "extctl_typedef.h"

int extctl_cmd_init(void);

int extctl_cmd_handle(void *data);

int extctl_cmd_send(void);

#endif /* SRC_MODULES_EXTCTL_EXTCTL_CMD_H_ */
