/*
 * extctl_cmd.c
 *
 *  Created on: Jun 7, 2018
 *      Author: lidq
 */

#include "extctl_cmd.h"

//#define __EXTCTL_DEBUG_

extern orb_advert_t _extctl_mavlink_log_pub;

static struct vehicle_command_s _vehicle_command = { 0 };
static orb_advert_t _cmd_pub = NULL;

int extctl_cmd_init(void)
{
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	struct vehicle_status_s state;
	bool updated = false;
	do
	{
		updated = false;
		if (orb_check(state_sub, &updated))
		{
			orb_copy(ORB_ID(vehicle_status), state_sub, &state);
		}
		usleep(100 * 1000);
	}
	while (!updated);

	if (_cmd_pub == NULL)
	{
		_cmd_pub = orb_advertise(ORB_ID(vehicle_command), &_vehicle_command);
	}

	_vehicle_command.target_system = 1;//state.system_id;
	_vehicle_command.target_component = 1;//state.component_id;

	return 0;
}

int extctl_cmd_handle(void *data)
{
	cmd_s *cmd = data;
	if (cmd == NULL)
	{
		return -1;
	}

	_vehicle_command.command = cmd->command;
	_vehicle_command.param1 = cmd->param1;
	_vehicle_command.param2 = cmd->param2;
	_vehicle_command.param3 = cmd->param3;
	_vehicle_command.param4 = cmd->param4;
	_vehicle_command.param5 = cmd->param5;
	_vehicle_command.param6 = cmd->param6;
	_vehicle_command.param7 = cmd->param7;

	orb_publish(ORB_ID(vehicle_command), _cmd_pub, &_vehicle_command);

#ifdef __EXTCTL_DEBUG_
	mavlink_log_info(&_extctl_mavlink_log_pub, "[extctl] cmd %d %d %d %4.2f %4.2f", _vehicle_command.target_system, _vehicle_command.target_component, _vehicle_command.command, (double )_vehicle_command.param1, (double )_vehicle_command.param2);
#endif

	return 0;
}

int extctl_cmd_send(void)
{
	return 0;
}

