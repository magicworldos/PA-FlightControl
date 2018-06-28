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

	uint32_t t = 0;

	while (1)
	{
		//10 ms
		if (t % 10 == 0)
		{
			handle_protocol();
		}
		//10 ms
		if (t % 10 == 5)
		{
			send_rc();
		}
		//100ms
		if (t % 100 == 55)
		{
			send_gps();
		}
		//1s
		if (t % 1000 == 575)
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

void send_gps(void)
{
	gps_s gps = { 0 };
	gps.timestamp = 0;
	gps.time_utc_usec = 0;
	gps.lat = 4053979;
	gps.lon = 1215037;
	gps.alt = 50;
	gps.alt_ellipsoid = 0;
	gps.s_variance_m_s = 0;
	gps.c_variance_rad = 0;
	gps.eph = 0.01;
	gps.epv = 0.01;
	gps.hdop = 0;
	gps.vdop = 0;
	gps.noise_per_ms = 0;
	gps.jamming_indicator = 0;
	gps.vel_m_s = 1.0;
	gps.vel_n_m_s = 1.2;
	gps.vel_e_m_s = 3.5;
	gps.vel_d_m_s = 2.4;
	gps.cog_rad = 0;
	gps.timestamp_time_relative = 0;
	gps.fix_type = 0;
	gps.vel_ned_valid = 0;
	gps.satellites_used = 0;
	gps._padding0[0] = 0;
	gps._padding0[1] = 0;
	gps._padding0[2] = 0;
	gps._padding0[3] = 0;
	gps._padding0[4] = 0;

	protocal_write(&gps, DATA_TYPE_GPS, sizeof(gps_s));
}

static void send_rc(void)
{
	rc_input_s rc = { 0 };
	rc.rc_failsafe = false;
	rc.rc_lost = false;
	rc.channel_count = 18;
	for (int i = 0; i < rc.channel_count; i++)
	{
		rc.values[i] = 800;
	}
	protocal_write(&rc, DATA_TYPE_RC_INPUT, sizeof(rc_input_s));
}

static void send_battery(void)
{
	battery_s battery = { 0 };
	battery.vcc = 2.4;

	protocal_write(&battery, DATA_TYPE_BATTERY, sizeof(battery_s));
}
