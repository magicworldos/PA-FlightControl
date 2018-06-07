/*
 * extctl_sp.h
 *
 *  Created on: Jun 7, 2018
 *      Author: lidq
 */

#ifndef SRC_MODULES_EXTCTL_EXTCTL_SP_H_
#define SRC_MODULES_EXTCTL_EXTCTL_SP_H_

#include "extctl_typedef.h"

int extctl_sp_init(void);

int extctl_sp_handle(void *data);

int extctl_sp_send(void);

#endif /* SRC_MODULES_EXTCTL_EXTCTL_SP_H_ */
