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
 * Sensor calibration process.
 * Xinyun Xu
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

/*
 * Sensor calibration num operation.
 * Xinyun Xu
 */
void sensor_calib() {
	sensor_calc(CALIBRATION_NUM); 
	if (calib_done) printf("\n CALIB DONE!\n");
}

/*
 * Sensor offset remove.
 * Xinyun Xu
 */
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
				change_p_value(yaw_control_pointer, RATE, true);
				break;
			case P_RATE_YAW_DEC:
				change_p_value(yaw_control_pointer, RATE, false);
				break;
			case P_ANGLE_PITCHROLL_INC:
				change_p_value(pitch_control_pointer, ANGLE, true);
				change_p_value(roll_control_pointer, ANGLE, true);
				break;
			case P_ANGLE_PITCHROLL_DEC:
				change_p_value(pitch_control_pointer, ANGLE, false);
				change_p_value(roll_control_pointer, ANGLE, false);
				break;
			case P_RATE_PITCHROLL_INC:
				change_p_value(pitch_control_pointer, RATE, true);
				change_p_value(roll_control_pointer, RATE, true);
				break;
			case P_RATE_PITCHROLL_DEC:
				change_p_value(pitch_control_pointer, RATE, false);
				change_p_value(roll_control_pointer, RATE, false);
				break;
			case P_SHIFT_RIGHT_VALUE_INC:
				change_shift_value(true);
				break; 
			case P_SHIFT_RIGHT_VALUE_DEC:
				change_shift_value(false);
				break;				 			
			default: 
				printf("FCB: UKNOWN CHANGE P VALUE: \n", message_byte);
				break;
	}		
}

/**
 * @brief      Change a P paramter for the P-controller
 *
 * @param      controller  The controller whose P parameter to be changed
 * @param[in]  param       The parameter to be changed (rate or angle)
 * @param[in]  increase    Wheter parameter should be increased (1) or decreased (0)
 * 
 * @author     T. van Rietbergem=n
 */
void  change_p_value(CONTROLLER *controller, P_PARAM param, bool increase) {
	if 	((param == RATE) && increase) {
		if (controller->kp_rate < CONTROLLER_P_UPPER_LIMIT) 
			controller->kp_rate += CONTROLLER_P_STEP_SIZE;
	} 
	else if ((param == RATE) && !increase) {
		if (controller->kp_rate > CONTROLLER_P_LOWER_LIMIT) 
			controller->kp_rate -= CONTROLLER_P_STEP_SIZE;
	} 
	else if ((param == ANGLE) && increase) {
		if (controller->kp_angle < CONTROLLER_P_UPPER_LIMIT) 
			controller->kp_angle += CONTROLLER_P_STEP_SIZE;
	} 
	else if ((param == ANGLE) && !increase) {
		if (controller->kp_angle > CONTROLLER_P_LOWER_LIMIT)
			controller->kp_angle-= CONTROLLER_P_STEP_SIZE;
	} 	
	else {
		printf("ERROR - UNKOWN PARAM CHANGE (change_p_value)\n");
	}	
}

/**
 * @brief      Change the shift-right paramter which decreased control output
 *
 * @param[in]  increase  Whether shift-right parameter should be increased or decreased
 * 
 * @author     T. van Rietbergen
 */
void change_shift_value(bool increase) {
	if (increase) {
		if (output_shift_value < OUTPUT_SHIFT_UPPER_LIMIT) {
			output_shift_value += 1;
		}		
	}
	else {
		if (output_shift_value > OUTPUT_SHIFT_LOWER_LIMIT) {
			output_shift_value -= 1;
		}			
	}

}

/* one step calculation for yaw control loop
 * Zehang Wu
 */
int16_t yaw_control_calc(CONTROLLER *yaw_control, int16_t yaw_set_point, int16_t sr) 
{
	int16_t error = (yaw_set_point - sr);
	int32_t yaw_output = error * yaw_control->kp_rate;
	return yaw_output >> (output_shift_value - 4);
}

/* one step calculation for pitch control loop
 * Zehang Wu
 */
int16_t pitch_control_calc(CONTROLLER *pitch_control, int16_t pitch_set_point, int16_t p_sq, int16_t p_theta) 
{
	// setpoint is in range [-8192 ... 8191] to match desired theta range
	int16_t error = (pitch_set_point - p_theta); // will be in range [-16xxx ... 16xxx]
	int32_t output_angle = (error * pitch_control->kp_angle - p_sq);
	int32_t pitch_output = output_angle * pitch_control->kp_rate;
	// printf("pitch_output: %ld\n", pitch_output >> 8);	
	return pitch_output >> output_shift_value; // divide by a lot to give sensible value
}

/* one step calculation for roll control loop
 * Zehang Wu
 */
int16_t roll_control_calc(CONTROLLER *roll_control, int16_t roll_set_point, int16_t p_sp, int16_t p_phi) {
	int16_t error = (roll_set_point - p_phi); // will be in range [-16xxx ... 16xxx]
	int32_t output_angle = (error * roll_control->kp_angle - p_sp);
	int32_t roll_output = output_angle * roll_control->kp_rate;
	return roll_output >> output_shift_value;
}


/*
* Limit the motors' speed in [170, 450].
* " "
*/
void clip_motors() {
	for (int i = 0; i < 4; i++) {
		if (ae[i] > MAX_ALLOWED_SPEED) 	ae[i] = MAX_ALLOWED_SPEED;
		if (ae[i] < MIN_ALLOWED_SPEED) 	ae[i] = MIN_ALLOWED_SPEED;
		if (ae[i] < 0)					ae[i] = 0;		
	}
}

/**
 * @brief      Zero all the engine values
 * 
 */
void zero_motors() {
	ae[0] = 0;
	ae[1] = 0;
	ae[2] = 0;
	ae[3] = 0;	
}

void update_motors(void)
{					
	if (fcb_state != SAFE_ST) clip_motors();
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

/**
 * @brief      Calculate motor values based on current state, desired attitude and sensor input
 *
 * @param[in]  p_pitch  The pitch final
 * @param[in]  p_roll   The roll final
 * @param[in]  p_yaw    The yaw final
 * @param[in]  lift_final   The lift final
 * 
 * @author     Zehang Wu
 */
void calculate_motor_values(int16_t p_pitch, int16_t p_roll, int16_t p_yaw, uint16_t p_lift) { //TODO: add min throttle (around 170) and max throttle (1000)

	// Clip values to now exceed maximum difference between values
	if (p_pitch < -MAX_DIFF_VALUE) p_pitch  = -MAX_DIFF_VALUE; 
	if (p_pitch >  MAX_DIFF_VALUE) p_pitch  =  MAX_DIFF_VALUE; 
	if (p_roll 	< -MAX_DIFF_VALUE) p_roll   = -MAX_DIFF_VALUE; 
	if (p_roll 	>  MAX_DIFF_VALUE) p_roll   =  MAX_DIFF_VALUE; 
	if (p_yaw 	< -MAX_DIFF_VALUE) p_yaw 	= -MAX_DIFF_VALUE; 
	if (p_yaw 	>  MAX_DIFF_VALUE) p_yaw 	=  MAX_DIFF_VALUE; 

	ae[0] = BASE_LIFT + (lift_final) + p_pitch - p_yaw;
	ae[1] = BASE_LIFT + (lift_final) - p_roll  + p_yaw;
	ae[2] = BASE_LIFT + (lift_final) - p_pitch - p_yaw; 
	ae[3] = BASE_LIFT + (lift_final) + p_roll  + p_yaw; 

}

/*
* Calculate the time duration(T = current_time - start_time)
* J. Cui 
*/
uint32_t calculate_time_diff (uint32_t start_time) {
	return get_time_us() - start_time;
}


/**
 * @brief      Ensure that attidute setpoint values do not exceed allowed range
 *
 * @param[in]  value  The value to be clipped
 *
 * @return     Clipped value in the range [-128, 127]
 */
int16_t clip_to_int8_values(int16_t value) {
	if (value > 127) {
		return 127;
	}
	if (value < -128) {
		return -128;
	}
	return value;
}

/**
 * @brief      Control motors based on current state, JS input and sensor input
 * 
 * @author     T. van Rietbergen
 */
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
				roll_control_calc(roll_control_pointer,   clip_to_int8_values(0  + roll_trim)  << 6, sp, phi), 
				0,
				panic_lift_level);
			break;
		case MANUAL_ST:
			calculate_motor_values(
				clip_to_int8_values(pitch + pitch_trim) >> 1, 
				clip_to_int8_values(roll + roll_trim) >> 1, 
				clip_to_int8_values(yaw + yaw_trim) >> 1, 
				adjusted_lift);
			break;
		case CALIBRATION_ST:
			zero_motors();
			sensor_calib();
			fcb_state = SAFE_ST;
			break;
		case YAWCONTROL_ST:
			calculate_motor_values(
				pitch, 
				roll, 
				yaw_control_calc(yaw_control_pointer, clip_to_int8_values(yaw + yaw_trim) << 6, (sr)*-1 ), 
				adjusted_lift); // i think sr needs *-1 (reverse sign
			break;
		case FULLCONTROL_ST:
			calculate_motor_values(
				pitch_control_calc(pitch_control_pointer, clip_to_int8_values(pitch + pitch_trim) << 6, sq, theta), 
				roll_control_calc(roll_control_pointer,   clip_to_int8_values(roll  + roll_trim)  << 6, sp, phi), 
				yaw_control_calc(yaw_control_pointer,     clip_to_int8_values(yaw   + yaw_trim)   << 6, sr*-1 ),  // sr needs *-1 (reverse sign)
				adjusted_lift);
			break;
		case UNKNOWN_ST:	
			zero_motors();
		default:
			zero_motors();
			printf("ERROR run_filters_and_control - unknown fcb_state: %d", fcb_state);
			break;
	}
	update_motors();
}