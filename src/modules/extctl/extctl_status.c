/*
 * extctl_status.c
 *
 *  Created on: Jun 13, 2018
 *      Author: lidq
 */

#include "extctl_status.h"

extern bool _extctl_should_exit;

int extctl_status_handle(void *data)
{
	return 0;
}

int extctl_status_send(void)
{
	int cmd_state_sub = orb_subscribe(ORB_ID(commander_state));
	int vec_state_sub = orb_subscribe(ORB_ID(vehicle_status));
	int armed_state_sub = orb_subscribe(ORB_ID(actuator_armed));
	int land_state_sub = orb_subscribe(ORB_ID(vehicle_land_detected));

	struct commander_state_s cmd_state;
	struct vehicle_status_s vec_state;
	struct actuator_armed_s arm_state;
	struct vehicle_land_detected_s land_state;

	sys_status_s sys_status = { 0 };

	while (!_extctl_should_exit)
	{
		uint8_t status = 0;
		bool updated = false;

		orb_check(cmd_state_sub, &updated);
		if (updated)
		{
			orb_copy(ORB_ID(commander_state), cmd_state_sub, &cmd_state);
			sys_status.main_state = cmd_state.main_state;
			status |= (1 << 0);
		}

		orb_check(vec_state_sub, &updated);
		if (updated)
		{
			orb_copy(ORB_ID(vehicle_status), vec_state_sub, &vec_state);
			sys_status.nav_state = vec_state.nav_state;
			status |= (1 << 1);
		}

		orb_check(armed_state_sub, &updated);
		if (updated)
		{
			orb_copy(ORB_ID(actuator_armed), armed_state_sub, &arm_state);
			sys_status.armed = arm_state.armed;
			status |= (1 << 2);
		}

		orb_check(land_state_sub, &updated);
		if (updated)
		{
			orb_copy(ORB_ID(vehicle_land_detected), land_state_sub, &land_state);
			sys_status.landed = land_state.landed;
			status |= (1 << 3);
		}

		if (status)
		{
			send_data_buff(&sys_status, DATA_TYPE_STATUS, sizeof(sys_status_s));
		}
		usleep(DEV_RATE_STATUS);
	}

	return 0;
}

