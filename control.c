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

/**
 * @brief      Control Loop Structure
 *
 * @author     Xinyun Xu
 *
 * @param      data  The data
 *
 * @return     { description_of_the_return_value }
 */
int16_t dt = 0.004; //integral time =  control period <= 0.1-0.2 * sample period
int16_t Integral_max; //maximum bound

int16_t Pitch_Output;
int16_t Roll_Output;
int16_t Yaw_Output;

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

	Yaw_rate_control.P = 3; 
	Yaw_rate_control.I = 0; 

	// printf("Control struct initalized");
}

void yaw_control_err_cal(CONTROL_T *Control, int16_t target, int measure)//setpoint sr
{
	Control->Err = (target - measure)/100; //target - measure
	Control->Output = Control->Err * Control->P;
	Yaw_Target = target;
	Yaw_Measure = measure;
	Yaw_Err = Yaw_rate_control.Err;
	Yaw_Output = Yaw_rate_control.Output;
}

void control_err_cal(CONTROL_T *Control, int16_t target, int measure)
{
	Control->Err = target - measure;
	Control->Integral = Control->Err * dt;
	if(Control->Integral > Integral_max) Control->Integral = Integral_max;
	if(Control->Integral < -Integral_max) Control->Integral = -Integral_max;
	Control->Deriv = Control->Err - Control->Pre_Err;
	Control->Output = Control->P*Control->Err + Control->I*Control->Integral + Control->D*Control->Deriv;
	Control->Pre_Err = Control->Err;
}

void speed_limit()
{
	for(uint8_t i = 0; i < 4; i++)
	{
	if(ae[i] > 350) ae[i] = 350;
	if(ae[i] < 170) ae[i] = 170;
	}
}


void clip_motors() {
	for (int i = 0; i < 4; i++) {
		if (ae[i] > 1000) 	ae[i] = 1000;
		if (ae[i] < 0)		ae[i] = 0;		
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

void sensor_calc(uint8_t num)
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

void sensor_calib()
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


#define yaw_speed_init 170

void controller_init(CONTROLLER *controller)
{
	// prinf('Controller init begin... \n');
	controller->set_point = 0;
	controller->sensor_value = 0;
	controller->err = 0;
	controller->kp = 1;
	controller->ki = 1;
	controller->integral = 0;
	controller->output = 0;
	// prinf('Controller init end. \n');
}

int16_t controller_calc(CONTROLLER *controller, int16_t set_point, int16_t sensor_value)
{
	controller->set_point = set_point;
	controller->err = controller->set_point - sensor_value;
	controller->integral += controller->err;
	controller->output = controller->kp * controller->err + controller->ki * controller->integral;
	return controller->output;
}

void increase_p_value(CONTROLLER *controller)
{
	controller->kp += CONTROLLER_P_STEP_SIZE;
	if(controller->kp > CONTROLLER__P_UPPER_LIMIT) controller->kp -= CONTROLLER_P_STEP_SIZE;
}

void decrease_p_value(CONTROLLER *controller)
{
	controller->kp -= CONTROLLER_P_STEP_SIZE;
	if(controller->kp < CONTROLLER_P_LOWER_LIMIT) controller->kp += CONTROLLER_P_STEP_SIZE;
}

// void increase_p_value(CONTROL_T *Control) {
// 	if (Control->P < YAW_P_UPPER_LIMIT) {
// 		Control->P += YAW_P_STEP_SIZE;
// 	}
// }

// void decrease_p_value(CONTROL_T *Control) {
// 	if (Control->P > YAW_P_LOWER_LIMIT) {
// 		Control->P -= YAW_P_STEP_SIZE;
// 	}
// }

void update_motors(void)
{					
	// if (g_current_state != SAFE_ST, PANIC_ST) //TODO
		clip_motors();
		//printf("%3d %3d %3d %3d | \n",ae[0],ae[1],ae[2],ae[3]);
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

