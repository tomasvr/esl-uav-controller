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

/* --- PRINTF_BYTE_TO_BINARY macro's --- */
#define PRINTF_BINARY_PATTERN_INT8 "%c%c%c%c%c%c%c%c"
#define PRINTF_BYTE_TO_BINARY_INT8(i)    \
    (((i) & 0x80ll) ? '1' : '0'), \
    (((i) & 0x40ll) ? '1' : '0'), \
    (((i) & 0x20ll) ? '1' : '0'), \
    (((i) & 0x10ll) ? '1' : '0'), \
    (((i) & 0x08ll) ? '1' : '0'), \
    (((i) & 0x04ll) ? '1' : '0'), \
    (((i) & 0x02ll) ? '1' : '0'), \
    (((i) & 0x01ll) ? '1' : '0')
#define PRINTF_BINARY_PATTERN_INT16 \
    PRINTF_BINARY_PATTERN_INT8              PRINTF_BINARY_PATTERN_INT8
#define PRINTF_BYTE_TO_BINARY_INT16(i) \
    PRINTF_BYTE_TO_BINARY_INT8((i) >> 8),   PRINTF_BYTE_TO_BINARY_INT8(i)
#define PRINTF_BINARY_PATTERN_INT32 \
    PRINTF_BINARY_PATTERN_INT16             PRINTF_BINARY_PATTERN_INT16
#define PRINTF_BYTE_TO_BINARY_INT32(i) \
    PRINTF_BYTE_TO_BINARY_INT16((i) >> 16), PRINTF_BYTE_TO_BINARY_INT16(i)
#define PRINTF_BINARY_PATTERN_INT64    \
    PRINTF_BINARY_PATTERN_INT32             PRINTF_BINARY_PATTERN_INT32
#define PRINTF_BYTE_TO_BINARY_INT64(i) \
    PRINTF_BYTE_TO_BINARY_INT32((i) >> 32), PRINTF_BYTE_TO_BINARY_INT32(i)
/* --- end macros --- */


#include "in4073.h"
#include <assert.h>

#define USB_COMM_INTERVAL_THRESHOLD 2000000 // in us (1000000 = 1 second)    

uint32_t usb_comm_last_received;
uint32_t current_time;

// each packet includes 4 fragments
uint8_t FRAG_COUNT = 0;

// State variables initalization
STATE_t g_current_state = SAFE_ST;
STATE_t g_dest_state = NO_WHERE;

// Communication variables initalization
COMM_TYPE g_current_comm_type = UNKNOWN_COMM;

// Motor variables initalization
MOTOR_CTRL g_current_m0_state = MOTOR_REMAIN;
MOTOR_CTRL g_current_m1_state = MOTOR_REMAIN;
MOTOR_CTRL g_current_m2_state = MOTOR_REMAIN;
MOTOR_CTRL g_current_m3_state = MOTOR_REMAIN;

uint8_t jsvalue_left;
uint8_t jsvalue_right;
JOYSTICK_AXIS_t joystick_axis;
uint16_t js_total_value;

// controller object declaration
CONTROLLER *yaw_control;
// CONTROLLER *roll_control;
// CONTROLLER *pitch_control;

/* The return value indicates how many motors went up (+) or down (-) 
 * which is used to adjust the motor_lift_level
 */
// int8_t find_motor_state(uint8_t messg){
// 	uint8_t m_ctrl_1 = messg & 0xf0; 		
// 	uint8_t m_ctrl_2 = messg & 0x0f;
// 	int result = 0;
// 	// find motor state of motor 0 
// 	if (m_ctrl_1>>6 == 0) {				// 0000 -> M0
// 		if ((m_ctrl_1 >> 5)&1 && (m_ctrl_1 >> 4)&1) {
// 			g_current_m0_state = MOTOR_UP;
// 			result += 1;
// 		}
// 		if (((m_ctrl_1 >> 5)&1) == 1 && ((m_ctrl_1 >> 4)&1) == 0) {
// 			g_current_m0_state = MOTOR_DOWN;
// 			result -= 1;
// 		}
// 		if (((m_ctrl_1 >> 5)&1) == 0) g_current_m0_state = MOTOR_REMAIN;
// 	}
// 	if (m_ctrl_1>>6 == 2) {				// 0010 -> M2
// 		if ((m_ctrl_1 >> 5)&1 && (m_ctrl_1 >> 4)&1) {
// 			g_current_m2_state = MOTOR_UP;
// 			result += 1;
// 		}
// 		if (((m_ctrl_1 >> 5)&1) == 1 && ((m_ctrl_1 >> 4)&1) == 0) {
// 			g_current_m2_state = MOTOR_DOWN;
// 			result -= 1;
// 		}
// 		if (((m_ctrl_1 >> 5)&1) == 0) g_current_m2_state = MOTOR_REMAIN;
// 	}
// 	if (m_ctrl_2>>2 == 1) {				// 0001 -> M1
// 		if ((m_ctrl_2 >> 1)&1 && (m_ctrl_2>>0)&1) {
// 			g_current_m1_state = MOTOR_UP;
// 			result += 1;
// 		}
// 		if (((m_ctrl_2 >> 1)&1) == 1 && ((m_ctrl_2 >> 0)&1) == 0) {
// 			g_current_m1_state = MOTOR_DOWN;
// 			result -= 1;
// 		}
// 		if (((m_ctrl_2 >> 1)&1) == 0) g_current_m1_state = MOTOR_REMAIN;
// 	}
// 	if (m_ctrl_2>>2 == 3) {				// 0011 -> M3
// 		if ((m_ctrl_2 >> 1)&1 && (m_ctrl_2>>0)&1) {
// 			g_current_m3_state = MOTOR_UP;
// 			result += 1;
// 		}
// 		if (((m_ctrl_2 >> 1)&1) == 1 && ((m_ctrl_2 >> 0)&1) == 0) {
// 			g_current_m3_state = MOTOR_DOWN;
// 			result -= 1;
// 		}
// 		if (((m_ctrl_2 >> 1)&1) == 0) g_current_m3_state = MOTOR_REMAIN;
// 	} 
// 	return result;
// }

// int16_t find_min_ae()
// {
// 	int16_t min = ae[0];
// 	if(ae[1] < min) min = ae[1];
// 	if(ae[2] < min) min = ae[2];
// 	if(ae[3] < min) min = ae[3];
// 	return min;
// }

void enter_panic_mode(bool cable_detached){
	if (g_current_state == SAFE_ST) {
		return; // if in safe mode then you do not need to go to panic mode
	}
	printf("FCB: QR: Entered PANIC MODE.");
	int16_t motor_speed = motor_lift_level;
	while (motor_speed > 0) {
		motor_speed = motor_speed - 10; 
		if (motor_speed < 10) {
			motor_speed = 0;
		}
		ae[0] = motor_speed;
		ae[1] = motor_speed;
		ae[2] = motor_speed;
		ae[3] = motor_speed;
		update_motors(); //or run filters_and_control() ?
		nrf_delay_ms(100);

	}
	motor_lift_level = 0; //reset motor_lift_level
	g_current_state = SAFE_ST;
	if (cable_detached) { //wait for reboot
		while(1) {
			motor[0] = 0;
			motor[1] = 0;
			motor[2] = 0;
			motor[3] = 0;
		}
	}
}

void USB_comm_update_received() {
	current_time = get_time_us();
	usb_comm_last_received = current_time;
}

void check_USB_connection_alive() {
	current_time = get_time_us();
	if (current_time - usb_comm_last_received > USB_COMM_INTERVAL_THRESHOLD) { //TODO: check for overflow? <--(Need fix when operation time > 70 mins)
		enter_panic_mode(true);
	}
}

void store_js_axis_commands(JOYSTICK_AXIS_t joystick_axis, uint16_t js_total_value) {
	if (joystick_axis == LIFT_THROTTLE) { // Throttle axis needs seperate calculation to determine when it is all the way down
		if(js_total_value <= 32767){
			js_total_value = 32767 - js_total_value;
		}
		else {
			js_total_value = 65536 - js_total_value + 32767;
		}
	}
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

void process_js_axis_cmd(JOYSTICK_AXIS_t joystick_axis, uint16_t js_total_value) {
	// printf("FCB: JS AXIS RECEIVED - axis: %d value: %ld \n", joystick_axis, js_total_value);
	uint8_t percentage = 0; // (percentage%)
	switch(joystick_axis){

		case ROLL_AXIS:
			if(js_total_value <= 32767){ // roll counterclockwise
				percentage = (uint8_t) (100.f * js_total_value / 32767);
				ae[1] = (int16_t) clip_motor_value(motor_lift_level + MOTOR_MAX_CHANGE * percentage / 100);
				ae[3] = (int16_t) clip_motor_value(motor_lift_level - MOTOR_MAX_CHANGE * percentage / 100);
			}
			else{ // roll clockwise
				percentage = (uint8_t) (100.f * (65536-js_total_value) / 32767);
				ae[1] = (int16_t) clip_motor_value(motor_lift_level - MOTOR_MAX_CHANGE * percentage / 100);
				ae[3] = (int16_t) clip_motor_value(motor_lift_level + MOTOR_MAX_CHANGE * percentage / 100);
			}
			break;

		case PITCH_AXIS:
			if(js_total_value <= 32767){ // pitch down
				percentage = (uint8_t) (100.f * js_total_value / 32767);
				ae[0] = (int16_t) clip_motor_value(motor_lift_level - MOTOR_MAX_CHANGE * percentage / 100);
				ae[2] = (int16_t) clip_motor_value(motor_lift_level + MOTOR_MAX_CHANGE * percentage / 100);
			}
			else{ // pitch up
				percentage = (uint8_t) (100.f * (65536-js_total_value) / 32767);
				ae[0] = (int16_t) clip_motor_value(motor_lift_level + MOTOR_MAX_CHANGE * percentage / 100);
				ae[2] = (int16_t) clip_motor_value(motor_lift_level - MOTOR_MAX_CHANGE * percentage / 100);
			}
			break;

		case YAW_AXIS:
			if(js_total_value <= 32767){ // yaw counterclockwise

				percentage = (uint8_t) (100.f * js_total_value / 32767);
				ae[0] = (int16_t) clip_motor_value(motor_lift_level + MOTOR_MAX_CHANGE * percentage / 100);
				ae[2] = (int16_t) clip_motor_value(motor_lift_level + MOTOR_MAX_CHANGE * percentage / 100);		
			}
			else{ // yaw clockwise
				percentage = (uint8_t) (100.f * (65536-js_total_value) / 32767);
				ae[1] = (int16_t) clip_motor_value(motor_lift_level + MOTOR_MAX_CHANGE * percentage / 100);
				ae[3] = (int16_t) clip_motor_value(motor_lift_level + MOTOR_MAX_CHANGE * percentage / 100);
			}
			break;

		case LIFT_THROTTLE:
			if(js_total_value <= 32767){
				percentage = (uint8_t) (100.f * (32767-js_total_value) / 65535);
			}
			else{
				percentage = (uint8_t) (100.f * (65536-js_total_value+32767) / 65535);
			}
			motor_lift_level = MOTOR_UPPER_LIMIT * percentage / 100;
			ae[0] = motor_lift_level;
			ae[1] = motor_lift_level;
			ae[2] = motor_lift_level;
			ae[3] = motor_lift_level;
			break;

		default:
			enter_panic_mode(false);
			break;
	}
	// printf("%3d %3d %3d %3d | \n",ae[0],ae[1],ae[2],ae[3]);		
	return;
}




void handle_frag1(uint8_t messg) {
	switch (g_current_comm_type) {
		case CTRL_COMM:
			if (g_current_state == MANUAL_ST || g_current_state == YAWCONTROL_ST) {
		 		motor_control_counter += find_motor_state(messg);
		 		if (motor_control_counter == 4) {
		 			motor_lift_level = clip_motor_value(motor_lift_level += STEP_SIZE);
		 		}
		 		if (motor_control_counter == -4) {
		 			motor_lift_level = clip_motor_value(motor_lift_level -= STEP_SIZE);
		 		}					
	 		}
			break;
		case JS_AXIS_COMM:
 			jsvalue_left = messg;	
 			js_total_value = (jsvalue_left << 8) | jsvalue_right;
	 		store_js_axis_commands(joystick_axis, js_total_value); // in EVERY state(also in manual/control) store js values to check neutral position
	 		if (g_current_state == MANUAL_ST || g_current_state == YAWCONTROL_ST) { // if in control mode, control drone
	 			// TODO: only execute the following function in mannual mode
	 			process_js_axis_cmd(joystick_axis, js_total_value);
	 		} 
			break;
		default:
			printf("Frag 1 no comm_type specific action\n");
			break;
	}
}

/*------------------------------------------------------------------
 * messg_decode -- decode messages
 *------------------------------------------------------------------
 */
void messg_decode(uint8_t message_byte){


	//printf("FCB: FRAG_COUNT: %d \n", FRAG_COUNT);
	//printf("FCB: message byte: "PRINTF_BINARY_PATTERN_INT8"\n",PRINTF_BYTE_TO_BINARY_INT8(message_byte));

	/* First byte is 	start byte 				 */
	/* Second byte is 	comm_type byte 	(case 2) */ 
	/* Third byte is 	data 			(case 1) */

	switch(FRAG_COUNT) {
		case 2: // Comm Type
			/* If a new message is received, update last received message time */
		 	USB_comm_update_received();
	 		/* Used to determine if 'a' or 'z' was pressed */
	 		motor_control_counter = 0;

			//printf("comm_type: "PRINTF_BINARY_PATTERN_INT8"\n",PRINTF_BYTE_TO_BINARY_INT8(comm_type));
			g_current_comm_type = retrieve_comm_type(message_byte); //shift right to get bits at beginning of byte
			break;

		case 1: // Data
			switch(g_current_comm_type) {
				case CTRL_COMM:
					;	// C requires this semicolon here
					uint8_t motor_states = retrieve_keyboard_motor_control(message_byte); 
					// maybe move to comm.c
					g_current_m0_state = (motor_states & 11000000) >> 6;
					g_current_m1_state = (motor_states & 00110000) >> 4;
					g_current_m2_state = (motor_states & 00001100) >> 2;
					g_current_m3_state = (motor_states & 00000011);
					/* If 'a' or 'z' was pressed, adjust motor_lift_level */
					if (0b11111111 == motor_states) {
						motor_lift_level += STEP_SIZE;
					}
					else if (0b00000000 == motor_states) {
						motor_lift_level -= STEP_SIZE;
					}
					break;
				case MODE_SW_COMM:
			 		g_dest_state = retrieve_mode(message_byte);
			 		printf("Comm type: %d, State: %d \n", g_current_comm_type, g_dest_state);
					g_current_state = mode_sw_action("FCB", g_current_state, g_dest_state, false);
					printf("current_state: %d \n", g_current_state);
					g_dest_state = NO_WHERE;					
					break;
				case JS_AXIS_COMM:
	 				joystick_axis = retrieve_js_axis(message_byte);
					break;
				case CHANGE_P_COMM:
			 		if (message_byte == 0x01) {
			 			printf("FCB: P CONTROL UP\n");
			 			increase_p_value(yaw_control);
			 		}
			 		if (message_byte == 0x00) {
				 		printf("FCB: P CONTROL DOWN\n");
				 		decrease_p_value(yaw_control);
			 		}						
				 	break;
				case BAT_INFO_COMM:
					break;
				case SYS_LOG_COMM:
					break;
				case ESC_COMM:
					break;
				case USB_CHECK_COMM:
			 		if (check_mode_sync(message_byte, g_current_state)) {
		 				printf("ERROR: STATE MISMATCH - PC state: %d, FCB state: %d \n", message_byte, g_current_state);
		 			}						
		 			break;
				default:
		    		printf("ERROR (messg_decode): UNKNOWN COMM TYPE: %d \n", g_current_comm_type);
					break;			
			}	
			/* Reset all message variables to prepare for next message */
			g_current_comm_type = UNKNOWN_COMM;
			joystick_axis = -1; 				
	 	break;

		default:
		    printf("ERROR (messg_decode): UNKNOWN FRAG COUNT: %d \n", FRAG_COUNT);
		    break;
	}
}

/*------------------------------------------------------------------
 * process_key -- process command keys
 *------------------------------------------------------------------
 */
void process_key(uint8_t c){	
	if (c == 0x55 && FRAG_COUNT == 0 && g_current_state != PANIC_ST) { // '0x55': 0b01010101(start byte)
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

	uint32_t counter = 0;
	demo_done = false;

	usb_comm_last_received = get_time_us();
	
	motor_lift_level = 0;
	
	controller_init(yaw_control);

	printf(" AE0 AE1 AE2 AE3  | MODE \n");
	while (!demo_done)
	{
		if (rx_queue.count) process_key( dequeue(&rx_queue) );

		// check if USB connection is still alive by checking last time received
		if (counter % 100 == 0) check_USB_connection_alive(); // use counter so this doesn't happen too often

		if (check_timer_flag()) 
		{
			if (counter % 20 == 0) 
			{
				nrf_gpio_pin_toggle(BLUE);
				//printf("FCB: current state: %4d \n", g_current_state);
 			}
			adc_request_sample();
			read_baro();

			// printf("%10ld | ", get_time_us());
			printf("%3d %3d %3d %3d  | ",ae[0],ae[1],ae[2],ae[3]);
			// printf("%6d %6d %6d | ", phi, theta, psi);
			// printf("%6d %6d %6d | ", sp, sq, sr);
			// printf("%4d | %4ld | %6ld   | ", bat_volt, temperature, pressure);
			printf("%4d \n", g_current_state - 1);
			clear_timer_flag();
			//printf("%4d \n", motor_lift_level);
	}

		if (check_sensor_int_flag()) 
		{
			get_dmp_data();
			run_filters_and_control();
		}

		// Execute commands that need to be handled in all modes
		if (g_current_comm_type == ESC_COMM){ // terminate program
			demo_done = true;
			g_current_comm_type = UNKNOWN_COMM;
			enter_panic_mode(false);
		}
		if (g_current_comm_type == CTRL_COMM){
			keyboard_ctrl_action();
		}

		// Execute commands  that only need to be handled in certain mode
		if (g_current_state == PANIC_ST)
		{
			enter_panic_mode(false); //enter panic mode for any reason other than cable
		}
		// if(g_current_state == MANUAL_ST)
		// {
		// 	process_js_axis_cmd(joystick_axis, js_total_value);
		// }
		if (g_current_state == CALIBRATION_ST) 
		{
			sensor_calib();
			//offset_remove();
			g_current_state = SAFE_ST;
		}
		if (g_current_state == YAWCONTROL_ST)
		{
			N_needed = yaw_control_calc(yaw_control, yaw_set_point, sr-sr_calib);
			actuate(0, 0, 0, N_needed); // only N_needed in yay control mode
		}
		if (g_current_state == FULLCONTROL_ST)
		{
			// TODO: do full controller things
		}
		counter++;
	}
	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);
	NVIC_SystemReset();
}
