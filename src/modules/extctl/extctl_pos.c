/*
 * extctl_pos.c
 *
 *  Created on: Jun 7, 2018
 *      Author: lidq
 */

#include "extctl_pos.h"

extern bool _extctl_should_exit;

int extctl_pos_handle(void *data)
{
	vehicle_pos_s *pos = data;
	if (pos == NULL)
	{
		return -1;
	}

	return 0;
}

int extctl_pos_send(void)
{
	int pos_sub_local = orb_subscribe(ORB_ID(vehicle_local_position));
	int pos_sub_global = orb_subscribe(ORB_ID(vehicle_global_position));

	struct vehicle_local_position_s pos_local;
	struct vehicle_global_position_s pos_global;

	vehicle_pos_s pos = { 0 };

	while (!_extctl_should_exit)
	{
		uint8_t status_pos = 0;
		bool updated = false;

		orb_check(pos_sub_local, &updated);
		if (updated)
		{
			orb_copy(ORB_ID(vehicle_local_position), pos_sub_local, &pos_local);

			pos.x = pos_local.x;
			pos.y = pos_local.y;
			pos.z = pos_local.z;

			pos.vx = pos_local.vx;
			pos.vy = pos_local.vy;
			pos.vz = pos_local.vz;

			status_pos |= (1 << 0);
		}

		orb_check(pos_sub_global, &updated);
		if (updated)
		{
			orb_copy(ORB_ID(vehicle_global_position), pos_sub_global, &pos_global);

			pos.lat = pos_global.lat;
			pos.lon = pos_global.lon;
			pos.alt = pos_global.alt;

			pos.vel_n = pos_global.vel_n;
			pos.vel_e = pos_global.vel_e;
			pos.vel_d = pos_global.vel_d;

			status_pos |= (1 << 1);
		}

		if (status_pos)
		{
			send_data_buff(&pos, DATA_TYPE_POS, sizeof(vehicle_pos_s));
		}
		usleep(DEV_RATE_POS);
	}
	return 0;
}
