#include <main.h>

extern uint16_t _pwmout[PWM_NUMCOUNTS];

static rc_input_s _rc = { 0 };
static uint16_t _rc_values[16] = { 0 };
static uint8_t _rc_flag = 0;

static char _pro_buff[SIZE_BUFF] = { 0 };
static int _pro_len = 0;
static int _pro_type = 0;

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

			read_rc();
		}
		//10 ms
		if (t % 100 == 0)
		{
			send_battery();
		}

		led0_blink(100);
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

			default:
				break;
		}

		if (p_handle != NULL)
		{
			p_handle(_pro_buff);
		}
	}
}

void read_rc(void)
{
	if (sbus_read(_rc_values, &_rc_flag))
	{
		_rc.channel_count = 16;
		_rc.rc_failsafe = _rc_flag >> 4 & 1;
		_rc.rc_lost = 0;
		for (int i = 0; i < 16; i++)
		{
			_rc.values[i] = _rc_values[i];
		}

		protocal_write(&_rc, DATA_TYPE_RC_INPUT, sizeof(rc_input_s));
	}
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

static void send_battery(void)
{
//	battery_s battery = { 0 };
//	battery.vcc = 2.4;
//
//	protocal_write(&battery, DATA_TYPE_BATTERY, sizeof(battery_s));
}
