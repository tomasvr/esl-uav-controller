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


// initalize 
uint8_t output_shift_value = OUTPUT_SHIFT_START_VALUE;

/*This funciton is used for debugging
* if color == -1, then all leds are turned off
* gpio_pin_set TURNS OFF led
* gpio_pin_clear TURNS ON led
*/
void switch_led(int color) {
	nrf_gpio_pin_set(GREEN);
	nrf_gpio_pin_set(RED);
	nrf_gpio_pin_set(YELLOW);
	if (color != -1) nrf_gpio_pin_clear(color);
}


// variable declaration for Calibration
bool DMP = true;
bool calib_done = false;
int16_t sensor_sum 	= 0;
uint8_t calib_counter 	= 0;
int32_t angle_calib[3] 	= {0};
int32_t gyro_calib[3] 	= {0};
int32_t acce_calib[3] 	= {0};
int16_t phi_calib = 0, 	theta_calib = 0, psi_calib = 0;
int16_t sp_calib = 0, 	sq_calib = 0, 	sr_calib = 0;
int16_t sax_calib = 0, 	say_calib = 0, 	saz_calib = 0;

/*
*
*/
void sensor_calc(uint8_t num) {
	do{
		angle_calib[0] 	+= phi; 
		angle_calib[1]	+= theta; 
		angle_calib[2] 	+= psi; 
		gyro_calib[0] 	+= sp; 
		gyro_calib[1] 	+= sq; 
		gyro_calib[2] 	+= sr; 
		acce_calib[0]	+= sax; 
		acce_calib[1] 	+= say; 
		acce_calib[2] 	+= saz;
		calib_counter++;

	}while(calib_counter < num);

	if(calib_counter == num){
		// calculate average
		angle_calib[0] 	/= num; 
		angle_calib[1] 	/= num; 
		angle_calib[2] 	/= num;//phi theta psi 
		gyro_calib[0] 	/= num; 
		gyro_calib[1] 	/= num; 
		gyro_calib[2] 	/= num;//sp sq sr
		acce_calib[0] 	/= num; 
		acce_calib[1]	/= num; 
		acce_calib[2] 	/= num;//sax say saz

		// store calibrated value
		phi_calib 	= angle_calib[0]; 
		theta_calib = angle_calib[1]; 
		psi_calib 	= angle_calib[2];
		sp_calib 	= gyro_calib[0]; 
		sq_calib 	= gyro_calib[1]; 
		sr_calib 	= gyro_calib[2];
		sax_calib 	= acce_calib[0]; 
		say_calib 	= acce_calib[1]; 
		saz_calib 	= acce_calib[2];
		calib_done 	= true;
		calib_counter = 0;

		// reset 
		angle_calib[0] 	= 0; 
		angle_calib[1] 	= 0; 
		angle_calib[2] 	= 0;
		gyro_calib[0] 	= 0; 
		gyro_calib[1] 	= 0; 
		gyro_calib[2] 	= 0;
		acce_calib[0] 	= 0; 
		acce_calib[1] 	= 0; 
		acce_calib[2] 	= 0;
	}
	else calib_done = false; // not calibrated yet
}

void sensor_calib() {
	sensor_calc(100); 
	if (calib_done) printf("\n CALIB DONE!\n");
	
	if(DMP){
		dmp_set_gyro_bias(gyro_calib);
		dmp_set_accel_bias(acce_calib);
	}
}

void offset_remove() {
	phi 	-= phi_calib; 
	theta 	-= theta_calib; 
	psi 	-= psi_calib;
	sp 	-= sp_calib; 
	sq 	-= sq_calib; 
	sr 	-= sr_calib;
	sax -= sax_calib; 
	say -= say_calib; 
	saz -= saz_calib;
}

/*
 * Initialize the control struct.
 * Zehang Wu
 */
void controller_init(CONTROLLER *controller) {
	controller->set_point = 0;
	controller->sensor_value = 0;
	controller->err = 0;
	controller->kp_rate = 1;
	controller->kp_angle = 1;
	controller->ki = 1;
	controller->integral = 0;
	controller->output = 0;
}

void adjust_parameter_value(uint8_t message_byte) {
	switch(message_byte) {
		case P_RATE_YAW_INC:
				increase_p_rate_value(yaw_control_pointer);
				break;
			case P_RATE_YAW_DEC:
				decrease_p_rate_value(yaw_control_pointer);
				break;
			case P_ANGLE_PITCHROLL_INC:
				increase_p_angle_value(pitch_control_pointer);
				increase_p_angle_value(roll_control_pointer);
				break;
			case P_ANGLE_PITCHROLL_DEC:
				decrease_p_angle_value(pitch_control_pointer);
				decrease_p_angle_value(roll_control_pointer);
				break;
			case P_RATE_PITCHROLL_INC:
				increase_p_rate_value(pitch_control_pointer);
				increase_p_rate_value(roll_control_pointer);
				break;
			case P_RATE_PITCHROLL_DEC:
				decrease_p_rate_value(pitch_control_pointer);
				decrease_p_rate_value(roll_control_pointer);
				break;
			case P_SHIFT_RIGHT_VALUE_INC:
				increase_shift_value(pitch_control_pointer);
				break; 
			case P_SHIFT_RIGHT_VALUE_DEC:
				decrease_shift_value(pitch_control_pointer);
				break;				 			
			default: 
				printf("FCB: UKNOWN CHANGE P VALUE: \n", message_byte);
				break;
	}		
}




void  increase_p_rate_value(CONTROLLER *controller) {
	if (controller->kp_rate < CONTROLLER_P_UPPER_LIMIT) 
		controller->kp_rate += CONTROLLER_P_STEP_SIZE;
}

void decrease_p_rate_value(CONTROLLER *controller) {
	if (controller->kp_rate > CONTROLLER_P_LOWER_LIMIT) 
		controller->kp_rate -= CONTROLLER_P_STEP_SIZE;
}

void  increase_p_angle_value(CONTROLLER *controller) {
	if (controller->kp_angle < CONTROLLER_P_UPPER_LIMIT) 
		controller->kp_angle += CONTROLLER_P_STEP_SIZE;
}

void decrease_p_angle_value(CONTROLLER *controller) {
	if (controller->kp_angle > CONTROLLER_P_LOWER_LIMIT)
		controller->kp_angle-= CONTROLLER_P_STEP_SIZE;
}

void increase_shift_value() {
	if (output_shift_value < OUTPUT_SHIFT_UPPER_LIMIT) {
		output_shift_value += 1;
	}
}

void decrease_shift_value() {
	if (output_shift_value > OUTPUT_SHIFT_LOWER_LIMIT) {
		output_shift_value -= 1;
	}
}


/* one step calculation for yaw control loop
 * Zehang Wu
 */
int16_t yaw_control_calc(CONTROLLER *yaw_control, int16_t yaw_set_point, int16_t sr) {
	// yaw_control->set_point = yaw_set_point;
	// yaw_control->err = yaw_control->set_point - sr;
	int16_t error = (yaw_set_point - sr);
	int32_t yaw_output = error * yaw_control->kp_rate;
	yaw_control->output = yaw_control->kp_rate * error;
	return yaw_output >> (output_shift_value - 4);
}

/* one step calculation for pitch control loop
 * Zehang Wu
 */
int16_t pitch_control_calc(CONTROLLER *pitch_control, int16_t pitch_set_point, int16_t p_sq, int16_t p_theta) {
	// setpoint is in range [-8192 ... 8191] to match desired theta range
	int16_t error = (pitch_set_point - p_theta); // will be in range [-16xxx ... 16xxx]
	int32_t output_angle = (error * pitch_control->kp_angle - p_sq);
	int32_t pitch_output = output_angle * pitch_control->kp_rate;
	pitch_control->output = pitch_output;
	//printf("pitch_output: %ld\n", pitch_output >> 8);	
	return pitch_output >> output_shift_value; // divide by a lot to give sensible value
}

/* one step calculation for roll control loop
 * Zehang Wu
 */
int16_t roll_control_calc(CONTROLLER *roll_control, int16_t roll_set_point, int16_t p_sp, int16_t p_phi) {
	// int16_t output = ((roll_set_point - phi) * roll_control->kp_angle - sp) * roll_control->kp_rate;
	int16_t error = (roll_set_point - p_phi); // will be in range [-16xxx ... 16xxx]
	int32_t output_angle = (error * roll_control->kp_angle - p_sp);
	int32_t roll_output = output_angle * roll_control->kp_rate;
	roll_control->output = roll_output;
	return roll_output >> output_shift_value;
}


/*
* Limit the motors' speed in [170, 450].
* "aruthor"
*/
void clip_motors() {
	for (int i = 0; i < 4; i++) {
		if (ae[i] > MAX_ALLOWED_SPEED) 	ae[i] = MAX_ALLOWED_SPEED;
		if (ae[i] < MIN_ALLOWED_SPEED) 	ae[i] = MIN_ALLOWED_SPEED;
		if (ae[i] < 0)					ae[i] = 0;		
	}
}

/*
* Set all motor speed to 0.
* "aruthor"
*/
void zero_motors() {
	ae[0] = 0;
	ae[1] = 0;
	ae[2] = 0;
	ae[3] = 0;	
}

/* for safety */
// int16_t clip_motor_value(int16_t value) {
// 	if (value < 0) {
// 		return 0;
// 	}
// 	if (value > MAX_ALLOWED_SPEED) {
// 		return MAX_ALLOWED_SPEED;
// 	}
// 	return value;
// }

// int16_t operating_motor_bounds(int16_t value) {
// 	if (value < 0) {
// 		return 0;
// 	}
// 	if (value > MAX_ALLOWED_SPEED) {
// 		return MAX_ALLOWED_SPEED;
// 	}
// 	return value;
// }

void update_motors(void)
{					
	if (fcb_state != SAFE_ST) clip_motors();
		// printf("%3d %3d %3d %3d | \n",ae[0],ae[1],ae[2],ae[3]);
		motor[0] = ae[0];
		motor[1] = ae[1];
		motor[2] = ae[2];
		motor[3] = ae[3];

#ifdef DEBUG_LED
		// The 4 LEDS represent motor speed, blue = max speed, red = minimal speed
		if (motor[0] == 0) switch_led(-1);
		else if (motor[0] < 200) switch_led(RED);
		else if (motor[0] >= 200 && motor[0] < 400) switch_led(YELLOW);
		else switch_led(GREEN);
#endif
}

/* calculate actuator values(ae[*]) from pitch, roll, paw and lift
 * 'Author'
 */
void calculate_motor_values(int16_t pitch_final, int16_t roll_final, int16_t yaw_final, uint16_t lift_final) { //TODO: add min throttle (around 170) and max throttle (1000)
	// ae[0] = operating_motor_bounds((lift << 2) + (pitch /320 - yaw/320));
	// ae[1] = operating_motor_bounds((lift << 2) - (roll/320  + yaw/320));
	// ae[2] = operating_motor_bounds((lift << 2) - (pitch/320 - yaw/320));
	// ae[3] = operating_motor_bounds((lift << 2) + (roll/320  + yaw /320));

	// clip values
	// if (pitch_final < -MAX_DIFF_VALUE) pitch_final  = -MAX_DIFF_VALUE; 
	// if (pitch_final >  MAX_DIFF_VALUE) pitch_final  =  MAX_DIFF_VALUE; 
	// if (roll_final 	< -MAX_DIFF_VALUE) roll_final   = -MAX_DIFF_VALUE; 
	// if (roll_final 	>  MAX_DIFF_VALUE) roll_final   =  MAX_DIFF_VALUE; 
	// if (yaw_final 	< -MAX_DIFF_VALUE) yaw_final 	= -MAX_DIFF_VALUE; 
	// if (yaw_final 	>  MAX_DIFF_VALUE) yaw_final 	=  MAX_DIFF_VALUE; 

	ae[0] = BASE_LIFT + (lift_final) + pitch_final - yaw_final; //* MAX_ALLOWED_DIFF_MOTOR / 256;
	ae[1] = BASE_LIFT + (lift_final) - roll_final  + yaw_final; // * MAX_ALLOWED_DIFF_MOTOR / 256;
	ae[2] = BASE_LIFT + (lift_final) - pitch_final - yaw_final; // * MAX_ALLOWED_DIFF_MOTOR / 256;
	ae[3] = BASE_LIFT + (lift_final) + roll_final  + yaw_final; // * MAX_ALLOWED_DIFF_MOTOR / 256;
}

uint32_t calculate_time_diff (uint32_t start_time) {
	return get_time_us() - start_time;
}

int16_t clip_to_int8_values(int16_t value) {
	if (value > 127) {
		return 127;
	}
	if (value < -128) {
		return -128;
	}
	return value;
}

void run_filters_and_control() {
	uint16_t adjusted_lift = lift << 1; // translate range to [0-512]
	switch(fcb_state) {
		case SAFE_ST:
			zero_motors();
			break;
		case PANIC_ST:
			;
			uint16_t panic_lift_level = (adjusted_lift < PANIC_MODE_LIFT) ? (adjusted_lift) : PANIC_MODE_LIFT;
			calculate_motor_values(
				pitch_control_calc(pitch_control_pointer, clip_to_int8_values(0  + pitch_trim) << 6, sq, theta), 
				 roll_control_calc(roll_control_pointer,  clip_to_int8_values(0  + roll_trim)  << 6, sp, phi), 
				  0,  // i think sr needs *-1 (reverse sign)
				panic_lift_level);
			break;
		case MANUAL_ST:
			calculate_motor_values(clip_to_int8_values(pitch + pitch_trim) >> 1, clip_to_int8_values(roll + roll_trim) >> 1, clip_to_int8_values(yaw + yaw_trim) >> 1, adjusted_lift);
			break;
		case CALIBRATION_ST:
			sensor_calib();
			fcb_state = SAFE_ST;
			zero_motors();
			break;
		case YAWCONTROL_ST:
			//todo
			calculate_motor_values(pitch, roll, yaw_control_calc(yaw_control_pointer, clip_to_int8_values(yaw + yaw_trim) << 6, (sr)*-1 ), adjusted_lift); // i think sr needs *-1 (reverse sign
			break;
		case FULLCONTROL_ST:
			calculate_motor_values(
				pitch_control_calc(pitch_control_pointer, clip_to_int8_values(pitch + pitch_trim) << 6, sq, theta), 
				 roll_control_calc(roll_control_pointer,  clip_to_int8_values(roll  + roll_trim)  << 6, sp, phi), 
				  yaw_control_calc(yaw_control_pointer,   clip_to_int8_values(yaw   + yaw_trim)   << 6, sr*-1 ),  // i think sr needs *-1 (reverse sign)
				adjusted_lift);
			break;
		case UNKNOWN_ST:	
			zero_motors();
		default:
			printf("ERROR run_filters_and_control - unknown fcb_state: %d", fcb_state);
			break;
	}
	update_motors();
}