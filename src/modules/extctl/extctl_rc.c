/*
 * extctl_rc.c
 *
 *  Created on: Jun 7, 2018
 *      Author: lidq
 */

#include "extctl_rc.h"

extern bool _extctl_should_exit;

int extctl_rc_handle(void *data)
{
	rc_s *rc = data;
	if (rc == NULL)
	{
		return -1;
	}
	
	return 0;
}

int extctl_rc_send(void)
{
	int rc_sub = orb_subscribe(ORB_ID(input_rc));
	struct input_rc_s in_rc;
	rc_s rc = { 0 };
	
	while (!_extctl_should_exit)
	{
		bool updated = false;
		orb_check(rc_sub, &updated);
		if (updated)
		{
			orb_copy(ORB_ID(input_rc), rc_sub, &in_rc);
			memset(&rc, 0, sizeof(rc_s));
			rc.rc_failsafe = in_rc.rc_failsafe;
			rc.rc_lost = in_rc.rc_lost;
			rc.channel_count = in_rc.channel_count;
			for (uint32_t i = 0; i < in_rc.channel_count; i++)
			{
				rc.values[i] = in_rc.values[i];
			}
			
			extctl_protocal_write(&rc, DATA_TYPE_RC, sizeof(rc_s));
		}
		usleep(DEV_RATE_RC);
	}
	
	return 0;
}
