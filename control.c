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

// void increase_motor_speed(int16_t *ae, uint8_t motor){
// 	ae[motor] += STEP_SIZE;
// 	if (ae[motor] > UPPER_LIMIT) ae[motor] = UPPER_LIMIT;
// 	return;
// }

// void decrease_motor_speed(int16_t *ae, uint8_t motor){
// 	ae[motor] -= STEP_SIZE;
// 	if (ae[motor] < 0) ae[motor] = 0;
// 	return;
// }

// void ctrl_action(){
// 	switch (g_current_m0_state){			//M0
// 		case MOTOR_UP:
// 			increase_motor_speed(ae, 0);
// 			break;
// 		case MOTOR_REMAIN:
// 			break;
// 		case MOTOR_DOWN:
// 			decrease_motor_speed(ae, 0);
// 			break;
// 		default:
// 			break;
// 	}
// 	switch (g_current_m1_state){			//M1
// 		case MOTOR_UP:
// 			increase_motor_speed(ae, 1);
// 			break;
// 		case MOTOR_REMAIN:
// 			break;
// 		case MOTOR_DOWN:
// 			decrease_motor_speed(ae, 1);
// 			break;
// 		default:
// 			break;
// 	}
// 	switch (g_current_m2_state){			//M2
// 		case MOTOR_UP:
// 			increase_motor_speed(ae, 2);
// 			break;
// 		case MOTOR_REMAIN:
// 			break;
// 		case MOTOR_DOWN:
// 			decrease_motor_speed(ae, 2);
// 			break;
// 		default:
// 			break;
// 	}
// 	switch (g_current_m3_state){			//M3
// 		case MOTOR_UP:
// 			increase_motor_speed(ae, 3);
// 			break;
// 		case MOTOR_REMAIN:
// 			break;
// 		case MOTOR_DOWN:
// 			decrease_motor_speed(ae, 3);
// 			break;
// 		default:
// 			break;
// 	}
// 	// reset motor intention
// 	g_current_m0_state = MOTOR_REMAIN;
// 	g_current_m1_state = MOTOR_REMAIN;
// 	g_current_m2_state = MOTOR_REMAIN;
// 	g_current_m3_state = MOTOR_REMAIN;
// }

#define yaw_speed_init 170



void yaw_control_init(YAW_CONTROL_T *yaw_control)
{
	yaw_control->kp = 1; //from keyboard
	yaw_control->ki = 0;
	yaw_control->err = 0;
	yaw_control->integral = 0;
	yaw_control->speed_comm = 0;
	yaw_control->speed_diff = 0;
	yaw_control->set_yaw_rate = 0; //from js
	yaw_control->actual_yaw_rate = 0; //from sensor
	yaw_control->actual_speed_plus = 0; 
	yaw_control->actual_speed_minus = 0; 
	printf("yaw_control struct initalized");
}

void yaw_control_speed_calculate(YAW_CONTROL_T *yaw_control, int16_t psi)//input js value here as set value;
{
	yaw_control->actual_yaw_rate = psi; //todo fix this (ugly hack)

	yaw_control->set_yaw_rate = 0; //interpret js value here
	yaw_control->err = yaw_control->set_yaw_rate - yaw_control->actual_yaw_rate;
	yaw_control->integral += yaw_control->err;
	yaw_control->speed_comm = yaw_speed_init;
	yaw_control->speed_diff = yaw_control->kp * yaw_control->err;
	//yaw_speed_diff = yaw_control.kp * yaw_control.err + yaw_control.ki * yaw_control.integral;
	yaw_control->actual_speed_plus = yaw_control->speed_comm + yaw_control->speed_diff;
	yaw_control->actual_speed_minus = yaw_control->speed_comm - yaw_control->speed_diff;
	//turn right M1 M3 + M2 M4 -
	//turn left M1 M3 - M2 M4 +
}

void increase_p_value(YAW_CONTROL_T *yaw_control) {
	if (yaw_control->kp < YAW_P_UPPER_LIMIT) {
		yaw_control->kp += YAW_P_STEP_SIZE;
	}
}

void decrease_p_value(YAW_CONTROL_T *yaw_control) {
	if (yaw_control->kp > YAW_P_LOWER_LIMIT) {
		yaw_control->kp -= YAW_P_STEP_SIZE;
	}
}

void clip_motors() {
	for (int i = 0; i < 4; i++) {
		if (ae[i] > 1000) 	ae[i] = 1000;
		if (ae[i] < 0)		ae[i] = 0;		
	}
}


void update_motors(void)
{					
	// if (g_current_state != SAFE_ST, PANIC_ST) //TODO
		clip_motors();
		printf("%3d %3d %3d %3d | \n",ae[0],ae[1],ae[2],ae[3]);
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

