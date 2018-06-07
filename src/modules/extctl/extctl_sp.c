/*
 * extctl_sp.c
 *
 *  Created on: Jun 7, 2018
 *      Author: lidq
 */

#include "extctl_sp.h"

//#define __EXTCTL_DEBUG_

extern bool _extctl_should_exit;
extern orb_advert_t _extctl_mavlink_log_pub;

static struct extctl_sp_s _orb_sp = { 0 };
static int _orb_class_instance = -1;
static orb_advert_t _orb_sp_topic = NULL;

int extctl_sp_init(void)
{
	_orb_sp_topic = orb_advertise_multi(ORB_ID(extctl_sp), &_orb_sp, &_orb_class_instance, ORB_PRIO_HIGH);
	orb_publish(ORB_ID(extctl_sp), _orb_sp_topic, &_orb_sp);

	return 0;
}

int extctl_sp_handle(void *data)
{
	vehicle_sp_s *sp = data;
	if (sp == NULL)
	{
		return -1;
	}

	_orb_sp.run_pos_control = sp->run_pos_control;
	_orb_sp.run_alt_control = sp->run_alt_control;
	_orb_sp.run_yaw_control = sp->run_yaw_control;

	_orb_sp.sp_yaw = sp->sp_yaw;

	_orb_sp.sp_x = sp->sp_x;
	_orb_sp.sp_y = sp->sp_y;
	_orb_sp.sp_z = sp->sp_z;

	_orb_sp.vel_sp_x = sp->vel_sp_x;
	_orb_sp.vel_sp_y = sp->vel_sp_y;
	_orb_sp.vel_sp_z = sp->vel_sp_z;

	orb_publish(ORB_ID(extctl_sp), _orb_sp_topic, &_orb_sp);

#ifdef __EXTCTL_DEBUG_
	mavlink_log_info(&_extctl_mavlink_log_pub, "[extctl] sp %4.2f %4.2f %4.2f", (double )_orb_sp.sp_x, (double )_orb_sp.sp_y, (double )_orb_sp.sp_z);
#endif

	return 0;
}

int extctl_sp_send(void)
{
	return 0;
}
