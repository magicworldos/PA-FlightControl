#include <main.h>

extern uint16_t _pwmout[PWM_NUMCOUNTS];

static char _pro_buff[SIZE_BUFF] = { 0 };
static int _pro_len = 0;
static int _pro_type = 0;
static gps_drv_s _gps;

int main(int argc, char* argv[])
{
	SystemInit();

	led_init();

	pwmout_init();

	uart1_init();

	uart2_init();

	protocal_init();

	timer_init();

	timer_start();

	uint32_t t = 0;

	while (1)
	{
		//10 ms
		if (t % 10 == 0)
		{
			handle_protocol();
			read_gps();
		}
		//10 ms
		if (t % 100 == 0)
		{
			handle_rc();
			send_rc();
		}
		//1s
		if (t % 1000 == 575)
		{
			send_battery();
		}

		timer_delay_ms(1);

		t++;
	}

	return 0;
}

void handle_protocol(void)
{
	void (*p_handle)(void *) = NULL;

	if (protocal_read(_pro_buff, &_pro_len, &_pro_type))
	{
		switch (_pro_type)
		{
			case DATA_TYPE_PWM_OUTPUT:
				p_handle = &handle_pwmout;
				break;

			case DATA_TYPE_GPS:
				p_handle = &handle_gps;
				break;

			default:
				break;
		}

		if (p_handle != NULL)
		{
			p_handle(_pro_buff);
		}
	}
}

void handle_gps(void *data)
{
	gps_drv_s *gps = data;
	if (gps == NULL)
	{
		return;
	}
	if (gps->len > 0)
	{
		led0_blink(10);
		uart2_write(gps->data, gps->len);
	}
}

void handle_rc(void)
{

}

void handle_pwmout(void *data)
{
	pwm_out_s *pwm = data;
	if (pwm == NULL)
	{
		return;
	}

	if (pwm->num_outputs > PWM_NUMCOUNTS)
	{
		pwm->num_outputs = PWM_NUMCOUNTS;
	}

	for (int i = 0; i < pwm->num_outputs; i++)
	{
		_pwmout[i] = pwm->pwm[i];
	}
	pwmout_set_value();
}

void read_gps(void)
{
	_gps.len = uart2_read(_gps.data, sizeof(_gps.data));
	if (_gps.len > 0)
	{
		protocal_write(&_gps, DATA_TYPE_GPS, sizeof(gps_drv_s));
	}
}

static void send_rc(void)
{
//	rc_input_s rc = { 0 };
//	rc.rc_failsafe = false;
//	rc.rc_lost = false;
//	rc.channel_count = 18;
//	for (int i = 0; i < rc.channel_count; i++)
//	{
//		rc.values[i] = 800;
//	}
//	protocal_write(&rc, DATA_TYPE_RC_INPUT, sizeof(rc_input_s));
}

static void send_battery(void)
{
//	battery_s battery = { 0 };
//	battery.vcc = 2.4;
//
//	protocal_write(&battery, DATA_TYPE_BATTERY, sizeof(battery_s));
}
