#include <main.h>

extern uint16_t _pwmout[PWM_NUMCOUNTS];

static char _pro_buff[SIZE_BUFF] = { 0 };
static int _pro_len = 0;
static int _pro_type = 0;

int main(int argc, char* argv[])
{
	SystemInit();

	led_init();

	pwmout_init();

	uart1_init();

	protocal_init();

	timer_init();

	timer_start();

	while (1)
	{
		handle_protocol();

		led0_blink(500);

		timer_delay_ms(10);
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
