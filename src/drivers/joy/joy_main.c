/*
 * joy_main.c
 *
 *  Created on: Jul 4, 2018
 *      Author: lidq
 */

#include "joy_main.h"

static bool _should_exit = false;
static int _fd = -1;
static char _dev_name[128] = DEV_NAME;

static struct input_rc_s _orb_rc = { 0 };
static int _orb_rc_instance = -1;
static orb_advert_t _orb_rc_topic = NULL;

static float values[16] = { 0 };
static float values_curr[16] = { 0 };
static float values_last[16] = { 0 };
static int rc_nums = 0;

int joy_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = NULL;
	char *verb = NULL;
	if (argc >= 2)
	{
		verb = argv[1];
	}
	else
	{
		return -1;
	}

	while ((ch = px4_getopt(argc, argv, "d:s:", &myoptind, &myoptarg)) != EOF)
	{
		switch (ch)
		{
			case 'd':
				strcpy(_dev_name, myoptarg);
				break;

			default:
				break;
		}
	}

	if (strcmp(verb, "start") == 0)
	{
		start(argc, argv);
		return OK;
	}

	if (strcmp(verb, "stop") == 0)
	{
		stop();
		return OK;
	}

	return -1;
}

int start(int argc, char *argv[])
{
	px4_main_t entry_point = (px4_main_t) joy_run;
	int task_id = px4_task_spawn_cmd("joy", SCHED_DEFAULT, SCHED_PRIORITY_FAST_DRIVER, CONFIG_PTHREAD_STACK_DEFAULT, entry_point, (char * const *) argv);
	return task_id;
}

int stop(void)
{
	_should_exit = true;
	usleep(200 * 1000);

	return OK;
}

int joy_run(int argc, char *argv[])
{
	_fd = open(_dev_name, O_RDONLY | O_NONBLOCK);
	if (_fd < 0)
	{
		warnx("can not open dev %s.", _dev_name);
		return -1;
	}

	_orb_rc_topic = orb_advertise_multi(ORB_ID(input_rc), &_orb_rc, &_orb_rc_instance, ORB_PRIO_DEFAULT);

	uint8_t buff[DEV_BUFF_SIZE] = { 0 };
	for (int i = 0; i < 16; i++)
	{
		values[i] = RC_PWM_MIN;
		values_curr[i] = RC_PWM_MIN;
		values_last[i] = RC_PWM_MIN;
	}

	while (!_should_exit)
	{
		int len = read(_fd, buff, DEV_BUFF_SIZE);

		if (len >= 8)
		{
			parse_joy(buff, len);
		}

		publish_rc();

		usleep(DEV_RATE_R);
	}
	return 0;
}

void parse_joy(uint8_t *buff, int len)
{
	if (buff[2] == 0x00)
	{
		values[0] = RC_PWM_MIN;
	}
	else if (buff[2] == 0x7f)
	{
		values[0] = RC_PWM_MID;
	}
	else if (buff[2] == 0xff)
	{
		values[0] = RC_PWM_MAX;
	}

	if (buff[3] == 0x00)
	{
		values[1] = RC_PWM_MAX;
	}
	else if (buff[3] == 0x7f)
	{
		values[1] = RC_PWM_MID;
	}
	else if (buff[3] == 0xff)
	{
		values[1] = RC_PWM_MIN;
	}

	if ((buff[5] >> 4) & 0x1)
	{
		values[2] = RC_PWM_MIN;
	}
	else if ((buff[5] >> 7) & 0x1)
	{
		values[2] = RC_PWM_MAX;
	}
	else
	{
		values[2] = RC_PWM_MID;
	}

	if ((buff[5] >> 5) & 0x1)
	{
		values[3] = RC_PWM_MAX;
	}
	else if ((buff[5] >> 6) & 0x1)
	{
		values[3] = RC_PWM_MIN;
	}
	else
	{
		values[3] = RC_PWM_MID;
	}

	if ((buff[6] >> 0) & 0x1)
	{
		values[4] = RC_PWM_MIN;
	}
	else if ((buff[6] >> 2) & 0x1)
	{
		values[4] = RC_PWM_MAX;
	}

	if ((buff[6] >> 1) & 0x1)
	{
		values[5] = RC_PWM_MIN;
	}
	else if ((buff[6] >> 3) & 0x1)
	{
		values[5] = RC_PWM_MAX;
	}

	for (int i = 0; i < RC_NUMS; i++)
	{
		if (i < 4)
		{
			values_curr[i] = values[i] * RC_PWM_SOFT + values_last[i] * (1.0f - RC_PWM_SOFT);
		}
		else
		{
			values_curr[i] = values[i];
		}
		values_last[i] = values_curr[i];
	}

}

void publish_rc(void)
{
	_orb_rc.timestamp_last_signal = hrt_absolute_time();
	_orb_rc.channel_count = rc_nums;
	_orb_rc.rssi = 100;
	_orb_rc.rc_failsafe = 0;
	_orb_rc.rc_lost = 0;
	_orb_rc.rc_lost_frame_count = 0;
	_orb_rc.rc_total_frame_count = 100;
	_orb_rc.rc_ppm_frame_length = 0;
	_orb_rc.input_source = RC_INPUT_SOURCE_PX4IO_SBUS;

	for (int i = 0; i < RC_NUMS; i++)
	{
		_orb_rc.values[i] = values_curr[i];
		printf("%5.0f ", (double) values_curr[i]);
	}
	printf("\n");

	orb_publish(ORB_ID(input_rc), _orb_rc_topic, &_orb_rc);
}
