/*
 * UORBPwm.h
 *
 *  Created on: Jun 27, 2018
 *      Author: lidq
 */

#ifndef SRC_DRIVERS_LINUX_PWM_OUT_UORB_PWM_H_
#define SRC_DRIVERS_LINUX_PWM_OUT_UORB_PWM_H_

#include <uORB/topics/pwm_output.h>
#include "common.h"

namespace linux_pwm_out
{

class UORBPwm: public PWMOutBase
{
public:
	UORBPwm(int max_num_outputs);
	virtual ~UORBPwm();

	virtual int init();

	virtual int send_output_pwm(const uint16_t *pwm, int num_outputs);

private:
	int _max_num_outputs;
	orb_advert_t _pwm_topic;
	pwm_output_s _pwm_output = { 0 };
};

}
#endif /* SRC_DRIVERS_LINUX_PWM_OUT_UORB_PWM_H_ */
