/*------------------------------------------------------------------
 *  control.c -- here you can implement your control algorithm
 *		 and any motor clipping or whatever else
 *		 remember! motor input =  0-1000 : 125-250 us (OneShot125)
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#include "nrf_gpio.h"
#include "in4073.h"


#define DEBUG_LED

// turns off all leds except one
// gpio_pin_set TURNS OFF led
// gpio_pin_clear TURNS ON led
void switch_led(int color) {
	nrf_gpio_pin_set(GREEN);
	nrf_gpio_pin_set(BLUE);
	nrf_gpio_pin_set(RED);
	nrf_gpio_pin_set(YELLOW);
	nrf_gpio_pin_clear(color);
}

#include "in4073.h"

void update_motors(void)
{					
	#ifdef DEBUG_LED
		if (ae[0] < 256) switch_led(RED);
		else if (ae[0] >= 256 && ae[0] < 512) switch_led(YELLOW);
		else if (ae[0] >= 512 && ae[0] < 768) switch_led(GREEN);
		else switch_led(BLUE);
	#endif
		motor[0] = ae[0];
		motor[1] = ae[1];
		motor[2] = ae[2];
		motor[3] = ae[3];
}

void run_filters_and_control()
{
	// fancy stuff here
	// control loops and/or filters

	// ae[0] = xxx, ae[1] = yyy etc etc
	update_motors();
}
