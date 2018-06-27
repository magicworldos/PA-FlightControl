/*
 * UORBPwm.cpp
 *
 *  Created on: Jun 27, 2018
 *      Author: lidq
 */

#include "uorb_pwm.h"

using namespace linux_pwm_out;

UORBPwm::UORBPwm(int max_num_outputs) :
			_max_num_outputs(max_num_outputs)
{
	_pwm_topic = orb_advertise(ORB_ID(pwm_output), &_pwm_output);
}

UORBPwm::~UORBPwm()
{

}

int UORBPwm::init()
{
	return 0;
}

int UORBPwm::send_output_pwm(const uint16_t* pwm, int num_outputs)
{
	if (num_outputs > pwm_output_s::MAX_NUM_OUTPUT)
	{
		num_outputs = pwm_output_s::MAX_NUM_OUTPUT;
	}

	_pwm_output.num_outputs = num_outputs;

	for (int i = 0; i < _pwm_output.num_outputs; ++i)
	{
		_pwm_output.pwm[i] = pwm[i];
	}

	orb_publish(ORB_ID(pwm_output), _pwm_topic, &_pwm_output);

	return 0;
}
