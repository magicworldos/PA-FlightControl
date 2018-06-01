/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file status_display.h
 * Status Display decouple the LED and tune form the original commander
 *
 * @author Simone Guscetti <simone@px4.io>
 */

#pragma once

#include "subscriber_handler.h"

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>

namespace status
{

class StatusDisplay
{
public:
	
	StatusDisplay(const events::SubscriberHandler &subscriber_handler);

	/** regularily called to handle state updates */
	void process();

protected:
	/**
	 * check for topic updates
	 * @return true if one or more topic got updated
	 */
	bool check_for_updates();

	/**
	 * handle LED logic changes & call publish()
	 */
	void set_leds();

	/** publish LED control */
	void publish();

	// TODO: review if there is a better variant that allocate this in the memory
	struct battery_status_s _battery_status = { };
	struct cpuload_s _cpu_load = { };
	struct vehicle_status_s _vehicle_status = { };
	struct vehicle_status_flags_s _vehicle_status_flags = { };
	struct vehicle_attitude_s _vehicle_attitude = { };

	struct led_control_s _led_control = { };

private:
	bool _old_gps_lock_valid = false;
	bool _old_home_position_valid = false;
	bool _low_battery = false;
	bool _critical_battery = false;
	int _old_nav_state = -1;
	int _old_battery_status_warning = -1;
	orb_advert_t _led_control_pub = nullptr;
	const events::SubscriberHandler &_subscriber_handler;
};

} /* status */
