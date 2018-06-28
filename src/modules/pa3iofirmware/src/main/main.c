#include <typedef.h>
#include <led.h>
#include <timer.h>
#include <uart1.h>
#include <pwmout.h>

int main(int argc, char* argv[])
{
	SystemInit();

	led_init();

	pwmout_init();

	uart1_init();

	timer_init();

	timer_start();

	while (1)
	{
		led0_blink(50);

		timer_delay_ms(10);
	}

	return 0;
}
