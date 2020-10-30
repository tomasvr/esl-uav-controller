/*------------------------------------------------------------------
 *  in4073.c -- test QR engines and sensors
 *
 *  reads ae[0-3] uart rx queue
 *  (q,w,e,r increment, a,s,d,f decrement)
 *
 *  prints timestamp, ae[0-3], sensors to uart tx queue
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  June 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"
#include <assert.h>

uint32_t usb_comm_last_received;
uint32_t usb_current_time;
uint32_t ctrl_loop_time;

// each packet includes 3 fragments
uint8_t FRAG_COUNT = 0;

// State variables initalization
STATE_t fcb_state = SAFE_ST;
STATE_t fcb_dest_state = UNKNOWN_ST;

// Communication variables initalization
COMM_TYPE g_current_comm_type = UNKNOWN_COMM;

// Keep track of current js_axis_type & value
JOYSTICK_AXIS_t js_axis_type;
uint8_t js_total_value;

int8_t pitch_trim = 0;
int8_t roll_trim = 0;
int8_t yaw_trim = 0;

uint32_t last_time_battery;
uint32_t current_time_battery;

// Motor variables initalization
MOTOR_CTRL g_current_m0_state = MOTOR_REMAIN;
MOTOR_CTRL g_current_m1_state = MOTOR_REMAIN;
MOTOR_CTRL g_current_m2_state = MOTOR_REMAIN;
MOTOR_CTRL g_current_m3_state = MOTOR_REMAIN;

// controller object declaration
CONTROLLER yaw_control;
CONTROLLER *yaw_control_pointer = &yaw_control;
CONTROLLER roll_control;
CONTROLLER *roll_control_pointer = &roll_control;
CONTROLLER pitch_control;
CONTROLLER *pitch_control_pointer = &pitch_control;

const char * state_to_str(STATE_t p_state) {
	switch(p_state) {
		case SAFE_ST:
			return "SAFE_ST";	
			break;
		case PANIC_ST:
			return "PANIC_ST";	
			break;
		case MANUAL_ST:
			return "MANUAL_ST";	
			break;
		case CALIBRATION_ST:
			return "CALIBRATION_ST";	
			break;
		case YAWCONTROL_ST:
			return "YAWCONTROL_ST";	
			break;
		case FULLCONTROL_ST:
			return "FULLCONTROL_ST";	
			break;
		case UNKNOWN_ST:
			return "UNKNOWN_ST";	
			break;		
	}

}

void enter_panic_mode(bool remain_off, char caller[]){
	if (fcb_state == SAFE_ST) {
		return; // if in safe mode then you do not need to go to panic mode
	}
	printf("FCB: Entered PANIC MODE"); // called by: %s", caller);
	fcb_state = PANIC_ST; // make sure system is in panic state;
	uint32_t panic_start_time = get_time_us();
	uint32_t panic_elapsed_time = get_time_us() - panic_start_time;
	while(panic_elapsed_time < PANIC_DURATION) {
		if (check_sensor_int_flag()) {
			get_dmp_data();
		}
		run_filters_and_control();
		//printf("ae0  %d\n", ae[0]);
		//printf("ae1  %d\n", ae[1]);
		//printf("ae2  %d\n", ae[2]);
		//printf("ae3  %d\n", ae[3]);
		panic_elapsed_time = get_time_us() - panic_start_time;
	}
	lift = 0; //reset motor_lift_level
	fcb_state = SAFE_ST;
	if (remain_off) { //wait for reboot
			ae[0] = 0;
			ae[1] = 0;
			ae[2] = 0;
			ae[3] = 0;
			update_motors();
			demo_done = true;
	}
}

void USB_comm_update_received() {
	usb_current_time = get_time_us();
	usb_comm_last_received = usb_current_time;
}

void check_USB_connection_alive() {
	usb_current_time = get_time_us();
	if (usb_current_time - usb_comm_last_received > USB_COMM_INTERVAL_THRESHOLD) { //TODO: check for overflow? <- Need fix when operation time > 70 mins
		enter_panic_mode(true, "usb_check failed \n");
	}
}

void store_js_axis_commands(JOYSTICK_AXIS_t joystick_axis, uint8_t js_total_value) {
	joystick_axis_stored_values[joystick_axis] = js_total_value;
}


int16_t clip_motor_value(int16_t value) { 
	if (value > 1000) {
		return 1000;
	}
	else if (value < 0) {
		return 0;
	}
	return value;
}

/* 
 * Keyboard trimming setpoint step by step.
 * Xinyun Xu
*/
void keyboard_trimming(uint8_t motor_states) {
	switch(motor_states){
		case LIFT_UP:
			if (lift + TRIM_STEP_SIZE < 255) {
				lift = lift + TRIM_STEP_SIZE;
			}
			break;
		case LIFT_DOWN:
			if (lift - TRIM_STEP_SIZE > 0) {
				lift = lift - TRIM_STEP_SIZE;
			}
			break;
		case PITCH_UP:
			pitch_trim += TRIM_STEP_SIZE;
			break;
		case PITCH_DOWN:
			pitch_trim -= TRIM_STEP_SIZE;
			break;
		case ROLL_RIGHT:
			roll_trim += TRIM_STEP_SIZE;
			break;
		case ROLL_LEFT:
			roll_trim -= TRIM_STEP_SIZE;
			break;
		case YAW_RIGHT:
			yaw_trim += TRIM_STEP_SIZE;
			break;
		case YAW_LEFT:
			yaw_trim -= TRIM_STEP_SIZE;
			break;				
	}
	printf("key trimming: %d\n", motor_states);
	printf("pitch trim:%d  roll trim: %d yaw trim %d\n", pitch_trim, roll_trim, yaw_trim);
}

/*
 * Decode messages
 * " "
 */
void messg_decode(uint8_t message_byte){

	// TODO: delete the printf() after debug
	//printf("FCB: FRAG_COUNT: %d \n", FRAG_COUNT);
	//printf("FCB: message byte: "PRINTF_BINARY_PATTERN_INT8"\n",PRINTF_BYTE_TO_BINARY_INT8(message_byte));

	/* First byte is 	start byte 				 */
	/* Second byte is 	comm_type byte 	(case 2) */ // ONLY FOR JS_AXIS_TYPE ARE THE LEFT MOST 2 BYTES FOR AXIS TYPE
	/* Third byte is 	data 			(case 1) */

	switch(FRAG_COUNT) {
		case 2: // Comm Type
			/* If a new message is received, update last received message time */
		 	USB_comm_update_received();
			//printf("message_byte: "PRINTF_BINARY_PATTERN_INT8"\n",PRINTF_BYTE_TO_BINARY_INT8(message_byte));
			g_current_comm_type = retrieve_comm_type(message_byte); //shift right to get bits at beginning of byte
			if (g_current_comm_type == JS_AXIS_COMM) {
				js_axis_type = retrieve_js_axis_type(message_byte); 
			}
			break;

		case 1: // Data
			switch(g_current_comm_type) {
				case ESC_COMM:
					enter_panic_mode(true, "ESC pressed");
				case CTRL_COMM:
					;	// C requires this semicolon here
					/* only change motors if in appropriate mode */ //todo: move this logic to a central place
					// if (fcb_state == MANUAL_ST || fcb_state == YAWCONTROL_ST || fcb_state == FULLCONTROL_ST) {
						if (fcb_state == MANUAL_ST || fcb_state == YAWCONTROL_ST || fcb_state == FULLCONTROL_ST) {
							uint8_t motor_states = retrieve_keyboard_motor_control(message_byte);
							keyboard_trimming(motor_states);
						}
						// printf("FCB: motor states: "PRINTF_BINARY_PATTERN_INT8"\n",PRINTF_BYTE_TO_BINARY_INT8(motor_states));
						else {
						printf("Cannot control keyboard motor in current mode: %d \n", fcb_state);						
						}
						break;
				case MODE_SW_COMM:
			 		fcb_dest_state = retrieve_mode(message_byte);
			 		//printf("Comm type: %d, State: %d \n", g_current_comm_type, fcb_dest_state);
					fcb_state = mode_sw_action("FCB", fcb_state, fcb_dest_state);
					fcb_dest_state = UNKNOWN_ST;					
					break;
				case JS_AXIS_COMM:
					//printf("axis: %d value: %d \n", (uint8_t)js_axis_type, message_byte);						
					switch (js_axis_type) {
						case ROLL_AXIS:
							roll  = translate_axis(message_byte);
							break;
						case PITCH_AXIS:
							pitch = translate_axis(message_byte);
							break;
						case YAW_AXIS:
							yaw   = translate_axis(message_byte);
							break;
						case LIFT_THROTTLE:
							message_byte = translate_throttle(message_byte);
							lift = message_byte;
							break;					
					}	
					store_js_axis_commands(js_axis_type, message_byte);
					break;
				case CHANGE_P_COMM:
					switch(message_byte) {
						case P_RATE_YAW_INC:
			 				increase_p_rate_value(yaw_control_pointer);
			 				printf("FCB: P RATE YAW CONTROL UP\n");
			 				break;
			 			case P_RATE_YAW_DEC:
				 			printf("FCB: P RATE YAW CONTROL DOWN\n");
			 				decrease_p_rate_value(yaw_control_pointer);
			 				break;
			 			case P_ANGLE_PITCHROLL_INC:
				 			printf("FCB: P ANGLE PITCHROLL CONTROL UP\n");
			 				increase_p_angle_value(pitch_control_pointer);
			 				increase_p_angle_value(roll_control_pointer);
			 				break;
			 			case P_ANGLE_PITCHROLL_DEC:
				 			printf("FCB: P ANGLE PITCHROLL CONTROL DOWN\n");
			 				decrease_p_angle_value(pitch_control_pointer);
			 				decrease_p_angle_value(roll_control_pointer);
			 				break;
			 			case P_RATE_PITCHROLL_INC:
				 			printf("FCB: P RATE PITCHROLL CONTROL UP\n");
			 				increase_p_rate_value(pitch_control_pointer);
			 				increase_p_rate_value(roll_control_pointer);
			 				break;
			 			case P_RATE_PITCHROLL_DEC:
				 			printf("FCB: P RATE PITCHROLL CONTROL DOWN\n");
			 				decrease_p_rate_value(pitch_control_pointer);
			 				decrease_p_rate_value(roll_control_pointer);
			 				break;
			 			case P_SHIFT_RIGHT_VALUE_INC:
				 			printf("FCB: P SHIFT VALUE UP\n");
			 				increase_shift_value(pitch_control_pointer);
			 				break; 
			 			case P_SHIFT_RIGHT_VALUE_DEC:
				 			printf("FCB: P SHIFT VALUE DOWN\n");
			 				decrease_shift_value(pitch_control_pointer);
			 				break;				 			
			 			default: 
				 			printf("FCB: UKNOWN CHANGE P VALUE: \n", message_byte);
				 			//break;
					}					
				 	break;
				case BAT_INFO_COMM:
					break;
				case SYS_LOG_COMM:
					break;
				case USB_CHECK_COMM:
			 		if (check_mode_sync(message_byte, fcb_state)) {
		 				printf("ERROR: STATE MISMATCH - PC state: %d, FCB state: %d \n", message_byte, fcb_state);
		 				enter_panic_mode(false, "State mismatch");
		 			}						
		 			break;
				default:
		    		printf("ERROR (messg_decode): UNKNOWN COMM TYPE: %d \n", g_current_comm_type);
					break;			
			}	
			/* Reset all message variables to prepare for next message */
			g_current_comm_type = UNKNOWN_COMM;
	 		break;

		default:
		    printf("ERROR (messg_decode): UNKNOWN FRAG COUNT: %d \n", FRAG_COUNT);
		    break;
	}
}

/*
 * Process the received packet.
 * J. Cui
 */
void process_packet(uint8_t c){	
	//printf("frag received: %d, FRAG_COUNT: %d \n", c, FRAG_COUNT);
	if (c == 0x55 && FRAG_COUNT == 0 && fcb_state != PANIC_ST) {
		FRAG_COUNT = 2;
		return;
	}
	if (FRAG_COUNT > 0){
		messg_decode(c);
		FRAG_COUNT--;
	} else {
		printf("process key called but FRAG_COUNT < 0, FRAG_COUNT: \n", FRAG_COUNT);
	}
}

/*
 * Check battery voltage.
 * Xinyun Xu
 */
void check_battery_volt(){
	current_time_battery = get_time_us();
	uint16_t battery_integer = bat_volt / 100;
	uint16_t battery_decimal = bat_volt & 0x00FF;
	if(current_time_battery - last_time_battery > BATTERY_CHECK_INTERVAL_THRESHOLD){
		if(bat_volt < 1150 && bat_volt > 1100){
			printf("CURRENT VOLTAGE: %d.%dV \n", battery_integer, battery_decimal);
		}
		else if(bat_volt < 1100 && bat_volt > 1050){
			printf("WARNING, CURRENT VOLTAGE: %d.%dV \n", battery_integer, battery_decimal);
			enter_panic_mode(true, "batter voltage too low");
		}
		else if(bat_volt < 1050){
			printf("CRITICAL WARNING, CURRENT VOLTAGE: %d.%dV \n", battery_integer, battery_decimal);
			enter_panic_mode(true, "batter voltage CRITICAL"); 
		}
	last_time_battery = current_time_battery;
	}
}

/*
* Print info in the terminal.
* J. Cui
*/
void print_info_testing() {
	printf("%10ld | ", get_time_us());
	printf("%3d %3d %3d %3d  | ",ae[0],ae[1],ae[2],ae[3]);
	//printf("%6d %6d %6d | ", phi, theta, psi);
	printf("%6d %6d %6d | ", sp, sq, sr);
	//printf("%4d | %4ld | %6ld   | ", bat_volt, temperature, pressure);
	printf("Py: %2d Pr: %2d Pa: %2d", yaw_control.kp_rate, pitch_control.kp_rate, pitch_control.kp_angle);
	//printf("setp: %4d sp: %4d err: %4d output: %4d ", pitch_control.set_point, sp, pitch_control.err, pitch_control.output);
	//printf("%4ld", ctrl_loop_time);
	printf(" mode: %s | \n", state_to_str(fcb_state));
}

/*------------------------------------------------------------------
 * main -- everything you need is here :)
 *------------------------------------------------------------------
 */
int main(void)
{
	uart_init();
	gpio_init();
	timers_init();
	adc_init();
	twi_init();
	imu_init(true, 100);	
	baro_init();
	spi_flash_init();
	ble_init();	
	
	demo_done = false;
	int counter = 0;

	usb_comm_last_received = get_time_us();
	
	controller_init(yaw_control_pointer);
	controller_init(roll_control_pointer);
	controller_init(pitch_control_pointer);
	// printf(" AE0 AE1 AE2 AE3  | MODE \n");
	while (!demo_done)
	{
		if (rx_queue.count) process_packet( dequeue(&rx_queue) );

		// Execute commands that need to be handled in all modes
		// if (g_current_comm_type == ESC_COMM){ // terminate program
		// 	enter_panic_mode(true, "ESC pressed");
		// }

		if (fcb_state == PANIC_ST) {
			enter_panic_mode(false, "FCB IN PANIC_ST");
		}



		if (check_timer_flag()) {
			nrf_gpio_pin_toggle(BLUE);
			check_USB_connection_alive();
			#ifdef ENABLE_BATT_CHECK
			check_battery_volt();			
			#endif
			adc_request_sample();
			read_baro();

			if (counter % 20 == 0) {
			logging();
			readout();
			}
			// printf("trim p: %2d r: %2d y: %2d", pitch_trim, roll_trim, yaw_trim);
			// printf("Py: %2d Pr: %2d Pa: %2d Ps: %2d", yaw_control.kp_rate, pitch_control.kp_rate, pitch_control.kp_angle, output_shift_value);
			// print_info_testing();
		}

		if (check_sensor_int_flag()) {
			get_dmp_data();
			ctrl_loop_time = get_time_us();
			run_filters_and_control();
			ctrl_loop_time = get_time_us() - ctrl_loop_time;
		}


		counter++;
	}
	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);
	NVIC_SystemReset();
}