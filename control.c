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

void speed_limit()
{
	for(uint8_t i = 0; i < 4; i++)
	{
	if(ae[i] > 450) ae[i] = 450;
	if(ae[i] < 170) ae[i] = 170;
	}
}

void clip_motors() {
	for (int i = 0; i < 4; i++) {
		if (ae[i] > MAX_ALLOWED_SPEED) 	ae[i] = MAX_ALLOWED_SPEED;
		if (ae[i] < 0)		ae[i] = 0;		
	}
}

void zero_motors() {
	ae[0] = 0;
	ae[1] = 0;
	ae[2] = 0;
	ae[3] = 0;	
}

// calibraiton
bool DMP = true;
bool calib_done = false;
uint8_t calib_counter = 0;
// int16_t sensor_calib = 0:
int16_t sensor_sum = 0;
int32_t angle_calib[3] = {0};
int32_t gyro_calib[3] = {0};
int32_t acce_calib[3] = {0};
int16_t phi_calib = 0, theta_calib = 0, psi_calib = 0;
int16_t sp_calib = 0, sq_calib = 0, sr_calib = 0;
int16_t sax_calib = 0, say_calib = 0, saz_calib = 0;

void sensor_calc(uint8_t num)
{
	do
	{
		angle_calib[0] += phi; 
		angle_calib[1] += theta; 
		angle_calib[2] += psi; 
		gyro_calib[0] += sp; 
		gyro_calib[1] += sq; 
		gyro_calib[2] += sr; 
		acce_calib[0] += sax; 
		acce_calib[1] += say; 
		acce_calib[2] += saz;
		calib_counter++;

	}while(calib_counter < num);

	if(calib_counter == num)
	{
		// calculate average
		// printf("| SUM: %6d \n", gyro_calib[2]);
		angle_calib[0] /= num; 
		angle_calib[1] /= num; 
		angle_calib[2] /= num;//phi theta psi 
		gyro_calib[0] /= num; 
		gyro_calib[1] /= num; 
		gyro_calib[2] /= num;//sp sq sr
		acce_calib[0] /= num; 
		acce_calib[1] /= num; 
		acce_calib[2] /= num;//sax say saz
		// printf("| PSI_CALIB: %6d \n", gyro_calib[2]);

		// store calibrated value
		phi_calib = angle_calib[0]; 
		theta_calib = angle_calib[1]; 
		psi_calib = angle_calib[2];
		sp_calib = gyro_calib[0]; 
		sq_calib = gyro_calib[1]; 
		sr_calib = gyro_calib[2];
		sax_calib = acce_calib[0]; 
		say_calib = acce_calib[1]; 
		saz_calib = acce_calib[2];
		calib_done = true;
		calib_counter = 0;

		// reset 
		angle_calib[0] = 0; 
		angle_calib[1] = 0; 
		angle_calib[2] = 0;
		gyro_calib[0] = 0; 
		gyro_calib[1] = 0; 
		gyro_calib[2] = 0;
		acce_calib[0] = 0; 
		acce_calib[1] = 0; 
		acce_calib[2] = 0;
	}
	else calib_done = false; // not calibrated yet
	// 	return -1;
}

void sensor_calib()
{
	sensor_calc(100); 
	if (calib_done) 
	{	
		printf("\n CALIB DONE\n");
		printf("\n SR CALIB DONE, SR_CALIB: %6d\n", sr_calib);//sr
	}
	if(DMP)
	{
		dmp_set_gyro_bias(gyro_calib);
		dmp_set_accel_bias(acce_calib);
	}
}

// void offset_remove()
// {
// 	phi -= phi_calib; 
// 	theta -= theta_calib; 
// 	psi -= psi_calib;
// 	sp -= sp_calib; 
// 	sq -= sq_calib; 
// 	sr -= sr_calib;
// 	sax -= sax_calib; 
// 	say -= say_calib; 
// 	saz -= saz_calib;
// }


// #define yaw_speed_init 170

// controller
int16_t yaw_set_point = 0;
int16_t roll_set_point = 0;
int16_t pitch_set_point = 0;
int16_t Z = 0;
int16_t L = 0;
int16_t M = 0;
int16_t N = 1;

void controller_init(CONTROLLER *controller)
{
	// prinf('Controller init begin... \n');
	controller->set_point = 0;
	controller->sensor_value = 0;
	controller->err = 0;
	controller->kp_rate = 5;
	controller->kp_angle = 5;
	controller->ki = 1;
	controller->integral = 0;
	controller->output = 0;
	// prinf('Controller init end. \n');
}

void  increase_p_rate_value(CONTROLLER *controller)
{
	if (controller->kp_rate < CONTROLLER_P_UPPER_LIMIT) {
		controller->kp_rate += CONTROLLER_P_STEP_SIZE;
	}
}

void decrease_p_rate_value(CONTROLLER *controller)
{
	if (controller->kp_rate > CONTROLLER_P_LOWER_LIMIT) {
		controller->kp_rate -= CONTROLLER_P_STEP_SIZE;
	}
}

void  increase_p_angle_value(CONTROLLER *controller)
{
	if (controller->kp_angle < CONTROLLER_P_UPPER_LIMIT) {
		controller->kp_angle += CONTROLLER_P_STEP_SIZE;
	}
}

void decrease_p_angle_value(CONTROLLER *controller)
{
	if (controller->kp_angle > CONTROLLER_P_LOWER_LIMIT) {
		controller->kp_angle-= CONTROLLER_P_STEP_SIZE;
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

/*------------------------------------------------------------------
 * keybord_ctrl_action -- allowing the keyboard to do all the actions
 *------------------------------------------------------------------
 */
void keyboard_ctrl_action(){
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

int16_t yaw_control_calc(CONTROLLER *yaw_control, int16_t yaw_set_point, int16_t sr)
{
	yaw_control->set_point = yaw_set_point;
	yaw_control->err = yaw_control->set_point - sr;
	// yaw_control->integral += yaw_control->err;
	yaw_control->output = yaw_control->kp_rate * yaw_control->err;
	//printf("yaw output: %d\n", yaw_control->output);
	return yaw_control->output;
}

int16_t pitch_control_calc(CONTROLLER *pitch_control, int16_t pitch_set_point, int16_t sq)
{
	pitch_control->set_point = pitch_set_point;
	pitch_control->err = pitch_control->set_point - sq;
	// pitch_control->integral += pitch_control->err;
	pitch_control->output = pitch_control->kp_rate * pitch_control->err;
	//printf("pitch output: %d\n", pitch_control->output);
	return pitch_control->output;
}

int16_t roll_control_calc(CONTROLLER *roll_control, int16_t roll_set_point, int16_t sp)
{
	roll_control->set_point = roll_set_point;
	roll_control->err = roll_control->set_point - sp;
	// roll_control->integral += roll_control->err;
	roll_control->output = roll_control->kp_rate * roll_control->err;
	//printf("roll setpoint: %d sq: %d err: %d output: %d \n", roll_set_point, sp, roll_control->err, roll_control->output);
	return roll_control->output;
}

double sqrt(double square)
{
    double root=square/3;
    int i;
    if (square <= 0) return 0;
    for (i=0; i<32; i++)
        root = (root + square / root) / 2;
    return root;
}

void actuate(int16_t Z_needed, int16_t L_needed, int16_t M_needed, int16_t N_needed)
{
	int sqr_0 = Z_needed/(4*b) - N_needed/(4*d) + M_needed/(2*b);
	int sqr_1 = -L_needed/(2*b) + Z_needed/(4*b) + N_needed/(4*d);
	int sqr_2 = -Z_needed/(4*b) - N_needed/(4*d) + M_needed/(2*b);
	int sqr_3 = L_needed/(2*b) + Z_needed/(4*b) + N_needed/(4*d);
	// int sqr_0 = -N_needed/4;
	// printf("sqr_0 is: %d \n", sqr_0 );
	// bool isNegative = false;

	// TODO: sqr_* should be positive, check(&fix) this
	if(sqr_0 < 0)
	{
		sqr_0 = -sqr_0;
	}
	if(sqr_1 < 0)
	{
		sqr_1 = -sqr_1;
	}
	if(sqr_2 < 0)
	{
		sqr_2 = -sqr_2;
	}
	if(sqr_3 < 0)
	{
		sqr_3 = -sqr_3;
	}

	int res_0 = sqrt(sqr_0);
	int res_1 = sqrt(sqr_1);
	int res_2 = sqrt(sqr_2);
	int res_3 = sqrt(sqr_3);
	// printf("ae0 is: %d, isNegative: %B \n", res, isNegative);
	// printf("sqr1 is: %d \n", sqr_1 );
	// printf("sqr2 is: %d \n", sqr_2 );
	// printf("sqr3 is: %d \n", sqr_3 );
	ae[0] = res_0;
	ae[1] = res_1;
	ae[2] = res_2;
	ae[3] = res_3;
	// ae[1] = (int16_t) sqrt(sqr_1);
	// ae[2] = (int16_t) sqrt(sqr_2);
	// ae[3] = (int16_t) sqrt(sqr_3);
	return;
}

void update_motors(void)
{					
	// if (fcb_state != SAFE_ST, PANIC_ST) //TODO
		clip_motors();
		// printf("%3d %3d %3d %3d | \n",ae[0],ae[1],ae[2],ae[3]);
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

void calculate_motor_values(int16_t pitch, int16_t roll, int16_t yaw, uint16_t lift) { //TODO: add min throttle (around 170) and max throttle (1000)
	ae[0] = (lift << 2) + pitch - yaw;
	ae[1] = (lift << 2) - roll + yaw;
	ae[2] = (lift << 2) - pitch - yaw;
	ae[3] = (lift << 2) + roll + yaw;
}

void run_filters_and_control()
{
	// printf("roll: %d\n", roll);
	// printf("pitch: %d\n", pitch);
	// printf("yaw: %d\n", yaw);
	// printf("lift: %d\n", lift);
	// fancy stuff here
	// control loops and/or filters
	switch(fcb_state) {
		case SAFE_ST:
			zero_motors();
			break;
		case PANIC_ST:
			//todo
			break;
		case MANUAL_ST:
			calculate_motor_values(pitch, roll, yaw, lift);
			break;
		case CALIBRATION_ST:
			zero_motors();
			break;
		case YAWCONTROL_ST:
			//todo
			// N_needed = yaw_control_calc(yaw_control_pointer, yaw_set_point, sr-sr_calib);
			// yaw_control_calc(yaw_control_pointer, 10, 0);
			// printf('N = %d \n', yaw_control_calc(yaw_control_pointer, 10, 0));
			//actuate(100, 0, 0, yaw_control_calc(yaw_control_pointer, 60, 0)); // only N_needed in yaw control mode
			calculate_motor_values(pitch, roll, yaw_control_calc(yaw_control_pointer, yaw, (sr >> 8)*-1 ), lift); // i think sr needs *-1 (reverse sign)
			//printf("sr: %d\n", sr >> 8);
			//printf("yaw: %d\n", yaw);
			
			break;
		case FULLCONTROL_ST:

			calculate_motor_values(
				pitch_control_calc(pitch_control_pointer, pitch, (sq >> 8)), 
				roll_control_calc(roll_control_pointer, roll, (sp >> 8)), 
				yaw_control_calc(yaw_control_pointer, yaw, (sr >> 8)*-1 ),  // i think sr needs *-1 (reverse sign)
				lift);
			break;
		case UNKNOWN_ST:	
			zero_motors();
		default:
			printf("ERROR run_filters_and_control - unknown fcb_state: %d", fcb_state);
			break;
	}
	// ae[0] = xxx, ae[1] = yyy etc etc
	update_motors();
}