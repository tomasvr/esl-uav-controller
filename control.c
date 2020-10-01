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
#include "control.h"
#include <assert.h>

// This funciton is used for debugging
// if color == -1, then all leds are turned off
// gpio_pin_set TURNS OFF led
// gpio_pin_clear TURNS ON led
void switch_led(int color) {
	nrf_gpio_pin_set(GREEN);
	nrf_gpio_pin_set(RED);
	nrf_gpio_pin_set(YELLOW);
	if (color != -1) {
		nrf_gpio_pin_clear(color);
	}
}

void increase_motor_speed(int16_t *ae, uint8_t motor){
	ae[motor] += STEP_SIZE;
	if (ae[motor] > UPPER_LIMIT) ae[motor] = UPPER_LIMIT;
	return;
}

void decrease_motor_speed(int16_t *ae, uint8_t motor){
	ae[motor] -= STEP_SIZE;
	if (ae[motor] < 0) ae[motor] = 0;
	return;
}

void set_motor_speed(int16_t *ae, uint8_t motor, uint8_t level){
	assert(motor >= 0 && 'Invaid motor number \n');
	assert(motor <= 3 && 'Invaid motor number \n');
	assert(level >= 0 && 'Invaid ae level \n');
	assert(level <= 10 && 'Invaid ae level \n');
	ae[motor] = (int16_t) (level/10)*UPPER_LIMIT;
}

void ctrl_action(){
	switch (g_current_m0_state){			//M0
		case MOTOR_UP:
			increase_motor_speed(ae, 0);
			break;
		case MOTOR_REMAIN:
			break;
		case MOTOR_DOWN:
			decrease_motor_speed(ae, 0);
			break;
		case MOTOR_LEVEL_0:
			set_motor_speed(ae, 0, 0);
			break;
		case MOTOR_LEVEL_1:
			set_motor_speed(ae, 0, 1);
			break;
		case MOTOR_LEVEL_2:
			set_motor_speed(ae, 0, 2);
			break;
		case MOTOR_LEVEL_3:
			set_motor_speed(ae, 0, 3);
			break;
		case MOTOR_LEVEL_4:
			set_motor_speed(ae, 0, 4);
			break;
		case MOTOR_LEVEL_5:
			set_motor_speed(ae, 0, 5);
			break;
		case MOTOR_LEVEL_6:
			set_motor_speed(ae, 0, 6);
			break;
		case MOTOR_LEVEL_7:
			set_motor_speed(ae, 0, 7);
			break;
		case MOTOR_LEVEL_8:
			set_motor_speed(ae, 0, 8);
			break;
		case MOTOR_LEVEL_9:
			set_motor_speed(ae, 0, 9);
			break;
		case MOTOR_LEVEL_10:
			set_motor_speed(ae, 0, 10);
			break;
		default:
			break;
	}
	switch (g_current_m1_state){			//M1
		case MOTOR_UP:
			increase_motor_speed(ae, 1);
			break;
		case MOTOR_REMAIN:
			break;
		case MOTOR_DOWN:
			decrease_motor_speed(ae, 1);
			break;
		case MOTOR_LEVEL_0:
			set_motor_speed(ae, 1, 0);
			break;
		case MOTOR_LEVEL_1:
			set_motor_speed(ae, 1, 1);
			break;
		case MOTOR_LEVEL_2:
			set_motor_speed(ae, 1, 2);
			break;
		case MOTOR_LEVEL_3:
			set_motor_speed(ae, 1, 3);
			break;
		case MOTOR_LEVEL_4:
			set_motor_speed(ae, 1, 4);
			break;
		case MOTOR_LEVEL_5:
			set_motor_speed(ae, 1, 5);
			break;
		case MOTOR_LEVEL_6:
			set_motor_speed(ae, 1, 6);
			break;
		case MOTOR_LEVEL_7:
			set_motor_speed(ae, 1, 7);
			break;
		case MOTOR_LEVEL_8:
			set_motor_speed(ae, 1, 8);
			break;
		case MOTOR_LEVEL_9:
			set_motor_speed(ae, 1, 9);
			break;
		case MOTOR_LEVEL_10:
			set_motor_speed(ae, 1, 10);
			break;
		default:
			break;
	}
	switch (g_current_m2_state){			//M2
		case MOTOR_UP:
			increase_motor_speed(ae, 2);
			break;
		case MOTOR_REMAIN:
			break;
		case MOTOR_DOWN:
			decrease_motor_speed(ae, 2);
			break;
		case MOTOR_LEVEL_0:
			set_motor_speed(ae, 2, 0);
			break;
		case MOTOR_LEVEL_1:
			set_motor_speed(ae, 2, 1);
			break;
		case MOTOR_LEVEL_2:
			set_motor_speed(ae, 2, 2);
			break;
		case MOTOR_LEVEL_3:
			set_motor_speed(ae, 2, 3);
			break;
		case MOTOR_LEVEL_4:
			set_motor_speed(ae, 2, 4);
			break;
		case MOTOR_LEVEL_5:
			set_motor_speed(ae, 2, 5);
			break;
		case MOTOR_LEVEL_6:
			set_motor_speed(ae, 2, 6);
			break;
		case MOTOR_LEVEL_7:
			set_motor_speed(ae, 2, 7);
			break;
		case MOTOR_LEVEL_8:
			set_motor_speed(ae, 2, 8);
			break;
		case MOTOR_LEVEL_9:
			set_motor_speed(ae, 2, 9);
			break;
		case MOTOR_LEVEL_10:
			set_motor_speed(ae, 2, 10);
			break;
		default:
			break;
	}
	switch (g_current_m3_state){			//M3
		case MOTOR_UP:
			increase_motor_speed(ae, 3);
			break;
		case MOTOR_REMAIN:
			break;
		case MOTOR_DOWN:
			decrease_motor_speed(ae, 3);
			break;
		case MOTOR_LEVEL_0:
			set_motor_speed(ae, 3, 0);
			break;
		case MOTOR_LEVEL_1:
			set_motor_speed(ae, 3, 1);
			break;
		case MOTOR_LEVEL_2:
			set_motor_speed(ae, 3, 2);
			break;
		case MOTOR_LEVEL_3:
			set_motor_speed(ae, 3, 3);
			break;
		case MOTOR_LEVEL_4:
			set_motor_speed(ae, 3, 4);
			break;
		case MOTOR_LEVEL_5:
			set_motor_speed(ae, 3, 5);
			break;
		case MOTOR_LEVEL_6:
			set_motor_speed(ae, 3, 6);
			break;
		case MOTOR_LEVEL_7:
			set_motor_speed(ae, 3, 7);
			break;
		case MOTOR_LEVEL_8:
			set_motor_speed(ae, 3, 8);
			break;
		case MOTOR_LEVEL_9:
			set_motor_speed(ae, 3, 9);
			break;
		case MOTOR_LEVEL_10:
			set_motor_speed(ae, 3, 10);
			break;
		default:
			break;
	}
	// reset motor intention
	g_current_m0_state = MOTOR_REMAIN;
	g_current_m1_state = MOTOR_REMAIN;
	g_current_m2_state = MOTOR_REMAIN;
	g_current_m3_state = MOTOR_REMAIN;
}

void update_motors(void){					
	// if (g_current_state != SAFE_ST, PANIC_ST) //TODO
		motor[0] = ae[0];
		motor[1] = ae[1];
		motor[2] = ae[2];
		motor[3] = ae[3];
	#ifdef DEBUG_LED
		// The 4 LEDS represent motor speed, blue = max speed, red = minimal speed
		if (motor[0] == 0) switch_led(-1);
		else if (motor[0] < 350) switch_led(RED);
		else if (motor[0] >= 350 && motor[0] < 700) switch_led(YELLOW);
		else switch_led(GREEN);
	#endif
}

void run_filters_and_control()
{
	// fancy stuff here
	// control loops and/or filters

	// ae[0] = xxx, ae[1] = yyy etc etc
	update_motors();
}

