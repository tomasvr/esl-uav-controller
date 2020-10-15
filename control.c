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

bool DMP = true;
bool calib_done = false;
uint8_t calib_counter = 0;
int16_t sensor_calib = 0, sensor_sum = 0;
int32_t angle_calib[3] = {0};
int32_t gyro_calib[3] = {0};
int32_t acce_calib[3] = {0};
int16_t phi_calib = 0, theta_calib = 0, psi_calib = 0;
int16_t sp_calib = 0, sq_calib = 0, sr_calib = 0;
int16_t sax_calib = 0, say_calib = 0, saz_calib = 0;

void sensor_calcu(uint8_t num)
{
	angle_calib[0] += phi; angle_calib[1] += theta; angle_calib[2] += psi; 
	gyro_calib[0] += sp; gyro_calib[1] += sq; gyro_calib[2] += sr; 
	acce_calib[0] += sax; acce_calib[1] += say; acce_calib[2] += saz;
	calib_counter++;
	if(calib_counter == num)
	{
		// printf("| SUM: %6d \n", gyro_calib[2]);
		angle_calib[0] /= num; angle_calib[1] /= num; angle_calib[2] /= num;//phi theta psi 
		gyro_calib[0] /= num; gyro_calib[1] /= num; gyro_calib[2] /= num;//sp sq sr
		acce_calib[0] /= num; acce_calib[1] /= num; acce_calib[2] /= num;//sax say saz
		// printf("| PSI_CALIB: %6d \n", gyro_calib[2]);
		calib_done = true;
		calib_counter = 0;

		phi_calib = angle_calib[0]; theta_calib = angle_calib[1]; psi_calib = angle_calib[2];
		sp_calib = gyro_calib[0]; sq_calib = gyro_calib[1]; sr_calib = gyro_calib[2];
		sax_calib = acce_calib[0]; say_calib = acce_calib[1]; saz_calib = acce_calib[2];

		angle_calib[0] = 0; angle_calib[1] = 0; angle_calib[2] = 0;
		gyro_calib[0] = 0; gyro_calib[1] = 0; gyro_calib[2] = 0;
		acce_calib[0] = 0; acce_calib[1] = 0; acce_calib[2] = 0;
	}
	else calib_done = false; // not calibrated yet
	// 	return -1;
}

void sensor_caib()
{
	sensor_calcu(100); 
	if (calib_done) 
	{	
		printf("\n CALIB DONE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
		printf("\n SR CALIB DONE, SR_CALIB: %6d\n", sr_calib);//sr
	}

	if(DMP)
	{
		dmp_set_gyro_bias(gyro_calib);
		dmp_set_accel_bias(acce_calib);
	}
}

void offset_remove()
{
	phi -= phi_calib; theta -= theta_calib; psi -= psi_calib;
	sp -= sp_calib; sq -= sq_calib; sr -= sr_calib;
	sax -= sax_calib; say -= say_calib; saz -= saz_calib;
}


/**
 * @brief      Control Loop Structure
 *
 * @author     Xinyun Xu
 */
int16_t dt = 0.004; //integral time =  control period <= 0.1-0.2 * sample period
int16_t Integral_max; //maximum bound

int16_t Pitch_Output;
int16_t Roll_Output;
int16_t Yaw_Output;
int16_t Lift = 0;

CONTROL_T Pitch_angle_control;
CONTROL_T Pitch_rate_control;

CONTROL_T Roll_angle_control;
CONTROL_T Roll_rate_control;

CONTROL_T Yaw_angle_control;
CONTROL_T Yaw_rate_control;

void control_init(CONTROL_T *Control)
{
	Roll_angle_control.P = 0;
	Roll_angle_control.I = 0;
	Roll_angle_control.D = 0;

	Roll_rate_control.P = 0; 
	Roll_rate_control.I = 0;
	Roll_rate_control.D = 0;

	Pitch_angle_control.P = 0;
	Pitch_angle_control.I = 0;
	Pitch_angle_control.D = 0;

	Pitch_rate_control.P = 0;
	Pitch_rate_control.I = 0;
	Pitch_rate_control.D = 0;

	Yaw_angle_control.P = 10; //from keyboard
	Yaw_angle_control.I = 0;

	Yaw_rate_control.P = 5; 
	Yaw_rate_control.I = 0; 

	// printf("Control struct initalized");
}	

void yaw_control_err_calcu(CONTROL_T *Control, int16_t target, int measure)//setpoint sr
{
	Control->Err = (target - measure)>>6; //target - measure
	Control->Output = Control->Err * Control->P;
	Yaw_Target = target;
	Yaw_Measure = measure;
	Yaw_Err = Yaw_rate_control.Err;
	Yaw_Output = Yaw_rate_control.Output;
}

void control_err_calcu(CONTROL_T *Control, int16_t target, int measure)
{
	Control->Err = target - measure;
	Control->Integral = Control->Err * dt;
	if(Control->Integral > Integral_max) Control->Integral = Integral_max;
	if(Control->Integral < -Integral_max) Control->Integral = -Integral_max;
	Control->Deriv = Control->Err - Control->Pre_Err;
	Control->Output = Control->P*Control->Err + Control->I*Control->Integral + Control->D*Control->Deriv;
	Control->Pre_Err = Control->Err;
}

void yaw_control()
{
	//control_err_cal(&Yaw_angle_control, TARGET_Z, psi);
	yaw_control_err_calcu(&Yaw_rate_control, 0, sr);
	// if(Yaw_Output > 5000) Yaw_Output = 5000;

}

void control()
{
	control_err_calcu(&Roll_angle_control, TARGET_X, phi);
	control_err_calcu(&Pitch_angle_control, TARGET_Y, theta);	
	// control_err_cal(&Yaw_angle_control, TARGET_Z, psi);

	control_err_calcu(&Roll_rate_control, Roll_angle_control.Output, sp);
	control_err_calcu(&Pitch_rate_control, Pitch_angle_control.Output, sq);
	// yaw_control_err_cal(&Yaw_rate_control, Yaw_rate_control.Output, sr);
	Roll_Output = Roll_rate_control.Output;
	Pitch_Output = Pitch_rate_control.Output;

	if(Roll_Output > ROLL_THRE) Roll_Output = ROLL_THRE;
	if(Roll_Output < -ROLL_THRE) Roll_Output = -ROLL_THRE;

	if(Pitch_Output > PITCH_THRE) Pitch_Output = PITCH_THRE;
	if(Pitch_Output < -PITCH_THRE) Pitch_Output = -PITCH_THRE;
}

void yaw_control_motor_output()
{
	ae[0] = SPEED_REF + Yaw_Output;
	ae[1] = SPEED_REF - Yaw_Output;
	ae[2] = SPEED_REF + Yaw_Output;
	ae[3] = SPEED_REF - Yaw_Output;
}

void control_motor_output()
{
	ae[0] = SPEED_REF + Pitch_Output;
	ae[1] = SPEED_REF - Roll_Output;
	ae[2] = SPEED_REF - Pitch_Output;
	ae[3] = SPEED_REF + Roll_Output;
	/*
	ae[0] = SPEED_REF + Pitch_Output + Yaw_Output;
	ae[1] = SPEED_REF - Roll_Output - Yaw_Output;
	ae[2] = SPEED_REF - Pitch_Output + Yaw_Output;
	ae[3] = SPEED_REF + Roll_Output - Yaw_Output;
	*/
}

void speed_limit()
{
	for(uint8_t i = 0; i < 4; i++)
	{
	if(ae[i] > 420) ae[i] = 420;
	if(ae[i] < 170) ae[i] = 170;
	}
}

/*
void yaw_control_speed_calculate(CONTROL_T *yaw_control, int16_t sr, int setpoint)//input js value here as set value; int setpoint
{
	sr = sr / 10;
	setpoint = setpoint / 32768 * 10000;
	printf(" setpoint %2d", setpoint);
	yaw_control->actual_yaw_rate = sr; //todo fix this (ugly hack)
	yaw_control->set_yaw_rate = setpoint; //interpret js value here
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
	printf("yaw_control initalized.\n");
}
*/

void increase_p_value(CONTROL_T *Control) {
	if (Control->P < YAW_P_UPPER_LIMIT) {
		Control->P += YAW_P_STEP_SIZE;

	}
}

void decrease_p_value(CONTROL_T *Control) {
	if (Control->P > YAW_P_LOWER_LIMIT) {
		Control->P -= YAW_P_STEP_SIZE;
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

