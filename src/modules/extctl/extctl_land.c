/*
 * extctl_land.c
 *
 *  Created on: Jun 7, 2018
 *      Author: lidq
 */

#include "extctl_land.h"

//#define __EXTCTL_DEBUG_
extern bool _extctl_should_exit;

extern orb_advert_t _extctl_mavlink_log_pub;

int extctl_land_handle(void *data)
{
	cmd_s *cmd = data;
	if (cmd == NULL)
	{
		return -1;
	}
	
	return 0;
}

int extctl_land_send(void)
{
	int land_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	struct vehicle_land_detected_s veh_land;
	land_s land = { 0 };
	
	while (!_extctl_should_exit)
	{
		bool updated = false;
		orb_check(land_sub, &updated);
		if (updated)
		{
			orb_copy(ORB_ID(vehicle_land_detected), land_sub, &veh_land);
			land.landed = veh_land.landed;
			send_data_buff(&land, DATA_TYPE_LAND, sizeof(land_s));
		}
		usleep(DEV_RATE_LAND);
	}
	
	return 0;
}

