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
		default:
			break;
	}
	// reset motor intention
	g_current_m0_state = MOTOR_REMAIN;
	g_current_m1_state = MOTOR_REMAIN;
	g_current_m2_state = MOTOR_REMAIN;
	g_current_m3_state = MOTOR_REMAIN;
}

#define yaw_speed_init 170

int16_t Integral_max; //maximum bound

int16_t Pitch_Output;
int16_t Roll_output;
int16_t Yaw_Output;

CONTROL_T Pitch_angle_control;
CONTROL_T Pitch_rate_control;

CONTROL_T Roll_angle_control;
CONTROL_T Roll_rate_control;

CONTROL_T Yaw_angle_control;
CONTROL_T Yaw_rate_control;

void control_init(CONTROL_T *Control)
{
	Pitch_angle_control->P = 0;
	Pitch_angle_control->I = 0;
	Pitch_angle_control->D = 0;

	Pitch_rate_control->P = 0;
	Pitch_rate_control->I = 0;
	Pitch_rate_control->D = 0;

	Roll_angle_control->P = 0;
	Roll_angle_control->I = 0;
	Roll_angle_control->D = 0;

	Roll_rate_control->P = 0; 
	Roll_rate_control->I = 0;
	Roll_rate_control->D = 0;

	Yaw_angle_control->P = 10; //from keyboard
	Yaw_rate_control->P = 0; 

	printf("Control struct initalized");
}

void yaw_control(CONTROL_T *Control, int16_t sr, int setpoint);//measure target
{
	CONTROL_T->Err = setpoint - sr; //target - measure
	CONTROL_T->Output = CONTROL_T->Err * Yaw_angle_control->P;
}

/*
void yaw_control_speed_calculate(CONTROL_T *yaw_control, int16_t sr, int setpoint)//input js value here as set value; int setpoint
{
	sr = sr / 10;
	setpoint = setpoint / 32768 * 10000;
	printf(" setpoint %2d", setpoint);
	yaw_control->actual_yaw_rate = sr; //todo fix this (ugly hack)
	yaw_control->set_yaw_rate = setpoint; //interpret js value here
	yaw_control->err = yaw_control->set_yaw_rate - yaw_control->actual_yaw_rate;
	yaw_control->integral += yaw_control->err;
	printf(" err %2d", yaw_control->err);
	yaw_control->speed_comm = yaw_speed_init;
	yaw_control->speed_diff = yaw_control->kp * yaw_control->err;
	//yaw_speed_diff = yaw_control.kp * yaw_control.err + yaw_control.ki * yaw_control.integral;
	yaw_control->actual_speed_plus = yaw_control->speed_comm + yaw_control->speed_diff;
	yaw_control->actual_speed_minus = yaw_control->speed_comm - yaw_control->speed_diff;
	ae[0] = yaw_control->actual_speed_plus; //turn right M1 M3 + M2 M4 -  turn left M1 M3 - M2 M4 +
	ae[1] = yaw_control->actual_speed_minus;
	ae[2] = yaw_control->actual_speed_plus;
	ae[3] = yaw_control->actual_speed_minus;
	printf(" %6d | %6d ", yaw_control->actual_speed_plus, yaw_control->actual_speed_minus);
	printf(" %2d | %2d | %2d | %2d\n ", ae[0], ae[1], ae[2], ae[3]);
}
*/

void increase_p_value(CONTROL_T *Control) {
	if (yaw_control->kp < YAW_P_UPPER_LIMIT) {
		yaw_control->kp += YAW_P_STEP_SIZE;
	}
}

void decrease_p_value(CONTROL_T *Control) {
	if (yaw_control->kp > YAW_P_LOWER_LIMIT) {
		yaw_control->kp -= YAW_P_STEP_SIZE;
	}
}


void update_motors(void)
{					
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

