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
#define BATTERY_CHECK_INTERVAL_THRESHOLD 5000000   

uint32_t usb_comm_last_received;
uint32_t current_time;

// each packet includes 3 fragments
uint8_t FRAG_COUNT = 0;

// State variables initalization
STATE_t fcb_state = SAFE_ST;
STATE_t fcb_dest_state = UNKNOWN_ST;

// Communication variables initalization
COMM_TYPE g_current_comm_type = UNKNOWN_COMM;

// Keep track of current js_axis_type
JOYSTICK_AXIS_t js_axis_type;

// Motor variables initalization
MOTOR_CTRL g_current_m0_state = MOTOR_REMAIN;
MOTOR_CTRL g_current_m1_state = MOTOR_REMAIN;
MOTOR_CTRL g_current_m2_state = MOTOR_REMAIN;
MOTOR_CTRL g_current_m3_state = MOTOR_REMAIN;

JOYSTICK_AXIS_t joystick_axis;
uint8_t js_total_value;

// controller object declaration
CONTROLLER yaw_control;
CONTROLLER *yaw_control_pointer = & yaw_control;
// CONTROLLER *roll_control;
// CONTROLLER *pitch_control;

void enter_panic_mode(bool cable_detached){
	if (fcb_state == SAFE_ST) {
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
	fcb_state = SAFE_ST;
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

void store_js_axis_commands(JOYSTICK_AXIS_t joystick_axis, uint8_t js_total_value) {
	if (joystick_axis == LIFT_THROTTLE) { // Throttle axis needs seperate calculation to determine when it is all the way down
		if(js_total_value <= JS_AXIS_MID_VALUE){
			js_total_value = JS_AXIS_MID_VALUE - js_total_value;
		}
		else {
			js_total_value = JS_AXIS_MAX_VALUE - js_total_value + JS_AXIS_MID_VALUE;
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



void process_js_axis_cmd(JOYSTICK_AXIS_t joystick_axis, uint8_t js_total_value) {
	//printf("FCB: JS AXIS RECEIVED - axis: %d value: %d \n", joystick_axis, js_total_value);
	uint8_t percentage = 0; // (percentage%)
	switch(joystick_axis){

		case ROLL_AXIS:
			if(js_total_value <= JS_AXIS_MID_VALUE){ // roll counterclockwise
				percentage = (uint8_t) (100.f * js_total_value / JS_AXIS_MID_VALUE);
				ae[1] = (int16_t) clip_motor_value(motor_lift_level + MOTOR_MAX_CHANGE * percentage / 100);
				ae[3] = (int16_t) clip_motor_value(motor_lift_level - MOTOR_MAX_CHANGE * percentage / 100);
			}
			else{ // roll clockwise
				percentage = (uint8_t) (100.f * (JS_AXIS_MAX_VALUE-js_total_value) / JS_AXIS_MID_VALUE);
				ae[1] = (int16_t) clip_motor_value(motor_lift_level - MOTOR_MAX_CHANGE * percentage / 100);
				ae[3] = (int16_t) clip_motor_value(motor_lift_level + MOTOR_MAX_CHANGE * percentage / 100);
			}
			break;

		case PITCH_AXIS:
			if(js_total_value <= JS_AXIS_MID_VALUE){ // pitch down
				percentage = (uint8_t) (100.f * js_total_value / JS_AXIS_MID_VALUE);
				ae[0] = (int16_t) clip_motor_value(motor_lift_level - MOTOR_MAX_CHANGE * percentage / 100);
				ae[2] = (int16_t) clip_motor_value(motor_lift_level + MOTOR_MAX_CHANGE * percentage / 100);
			}
			else{ // pitch up
				percentage = (uint8_t) (100.f * (JS_AXIS_MAX_VALUE-js_total_value) / JS_AXIS_MID_VALUE);
				ae[0] = (int16_t) clip_motor_value(motor_lift_level + MOTOR_MAX_CHANGE * percentage / 100);
				ae[2] = (int16_t) clip_motor_value(motor_lift_level - MOTOR_MAX_CHANGE * percentage / 100);
			}
			break;

		case YAW_AXIS:
			if(js_total_value <= JS_AXIS_MID_VALUE){ // yaw counterclockwise

				percentage = (uint8_t) (100.f * js_total_value / JS_AXIS_MID_VALUE);
				ae[0] = (int16_t) clip_motor_value(motor_lift_level + MOTOR_MAX_CHANGE * percentage / 100);
				ae[2] = (int16_t) clip_motor_value(motor_lift_level + MOTOR_MAX_CHANGE * percentage / 100);		
			}
			else{ // yaw clockwise
				percentage = (uint8_t) (100.f * (JS_AXIS_MAX_VALUE-js_total_value) / JS_AXIS_MID_VALUE);
				ae[1] = (int16_t) clip_motor_value(motor_lift_level + MOTOR_MAX_CHANGE * percentage / 100);
				ae[3] = (int16_t) clip_motor_value(motor_lift_level + MOTOR_MAX_CHANGE * percentage / 100);
			}
			break;

		case LIFT_THROTTLE:
			if(js_total_value <= JS_AXIS_MID_VALUE){
				percentage = (uint8_t) (100.f * (JS_AXIS_MID_VALUE-js_total_value) / JS_AXIS_DIVIDE_VALUE);
			}
			else{
				percentage = (uint8_t) (100.f * (JS_AXIS_MAX_VALUE-js_total_value+JS_AXIS_MID_VALUE) / JS_AXIS_DIVIDE_VALUE);
			}
			motor_lift_level = MOTOR_UPPER_LIMIT * percentage / 100;
			//printf("FCB: percentage: %f lift_level: %d \n", percentage, motor_lift_level);
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

/*------------------------------------------------------------------
 * messg_decode -- decode messages
 *------------------------------------------------------------------
 */
void messg_decode(uint8_t message_byte){


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
				case CTRL_COMM:
					;	// C requires this semicolon here
					uint8_t motor_states = retrieve_keyboard_motor_control(message_byte); 
					//printf("FCB: motor states: "PRINTF_BINARY_PATTERN_INT8"\n",PRINTF_BYTE_TO_BINARY_INT8(motor_states));
					g_current_m0_state = (motor_states >> 6) & 3; //extract last two bytes with & 3 operator 
					g_current_m1_state = (motor_states >> 4) & 3;
					g_current_m2_state = (motor_states >> 2) & 3;
					g_current_m3_state = (motor_states)		 & 3;

					/* only change motors if in appropriate mode */ //todo: move this logic to a central place
					if (fcb_state == 	MANUAL_ST || fcb_state == YAWCONTROL_ST || fcb_state == FULLCONTROL_ST) {
						keyboard_ctrl_action();
						/* If 'a' or 'z' was pressed, adjust motor_lift_level */
						if (0b11111111 == motor_states) {
							motor_lift_level += STEP_SIZE;
						}
						else if (0b00000000 == motor_states) {
							motor_lift_level -= STEP_SIZE;
						}
					} else {
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
	 				//joystick_axis = retrieve_js_axis(message_byte);
					store_js_axis_commands(js_axis_type, message_byte);
					if (fcb_state == 	MANUAL_ST || fcb_state == YAWCONTROL_ST || fcb_state == FULLCONTROL_ST) {
						process_js_axis_cmd(js_axis_type, message_byte);
					}	 				
					break;
				case CHANGE_P_COMM:
					switch(message_byte) {
						case P_YAW_INC:
			 				increase_p_value(yaw_control_pointer);
			 				printf("FCB: P YAW CONTROL UP\n");
			 				break;
			 			case P_YAW_DEC:
				 			printf("FCB: P YAW CONTROL DOWN\n");
			 				decrease_p_value(yaw_control_pointer);
			 				break;
			 			default: 
				 			printf("FCB: UKNOWN CHANGE P VALUE: \n", message_byte);
					}					
				 	break;
				case BAT_INFO_COMM:
					break;
				case SYS_LOG_COMM:
					break;
				case ESC_COMM:
					break;
				case USB_CHECK_COMM:
			 		if (check_mode_sync(message_byte, fcb_state)) {
		 				printf("ERROR: STATE MISMATCH - PC state: %d, FCB state: %d \n", message_byte, fcb_state);
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

/*------------------------------------------------------------------
 * process_key -- process command keys
 *------------------------------------------------------------------
 */
void process_key(uint8_t c){	
	if (c == 0x55 && FRAG_COUNT == 0 && fcb_state != PANIC_ST) { // '0x55': 0b01010101(start byte)
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

uint32_t last_time_battery;
uint32_t current_time_battery;


void check_battery_volt(){
	current_time_battery = get_time_us();
	if(current_time_battery - last_time_battery > BATTERY_CHECK_INTERVAL_THRESHOLD){
		printf("current voltage: %d.%dV \n", bat_volt/100, bat_volt&0x00FF);
		if(bat_volt < 1150 && bat_volt > 1100){
			printf("CURRENT VOLTAGE: %d.%dV \n", bat_volt/100, bat_volt&0x00FF);
		}
		else if(bat_volt < 1100 && bat_volt > 1050){
			printf("WARNING, CURRENT VOLTAGE1: %d.%dV \n", bat_volt/100, bat_volt&0x00FF);
			enter_panic_mode(true);
		}
		else if(bat_volt < 1050){
			printf("WARNING, CURRENT VOLTAGE2: %d.%dV \n", bat_volt/100, bat_volt&0x00FF);
			enter_panic_mode(true); 
		}
	last_time_battery = current_time_battery;
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
	
	controller_init(yaw_control_pointer);

	printf(" AE0 AE1 AE2 AE3  | MODE \n");
	while (!demo_done)
	{
		if (rx_queue.count) process_key( dequeue(&rx_queue) );

		if (counter % 100 == 0) check_USB_connection_alive();

		check_battery_volt();//enable panic mode when connect to drone

		if (check_timer_flag()) 
		{
		
			if (counter % 20 == 0) 
			{
				nrf_gpio_pin_toggle(BLUE);
				// printf("p yaw param: %4d \n", yaw_control_pointer->kp);
				// printf("p yaw output: %4d \n", yaw_control_pointer->output);
				// printf("p yaw error: %4d \n", yaw_control_pointer->err);	
 			}
			adc_request_sample();
			read_baro();

			// printf("%10ld | ", get_time_us());
			// printf("%3d %3d %3d %3d  | ",ae[0],ae[1],ae[2],ae[3]);
			// printf("%6d %6d %6d | ", phi, theta, psi);
			// printf("%6d %6d %6d | ", sp, sq, sr);
			printf("%4d | %4ld | %6ld   | ", bat_volt, temperature, pressure);
			printf("%4d \n", fcb_state - 1);
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
		// if (g_current_comm_type == CTRL_COMM){
		// 	keyboard_ctrl_action();
		// }


		// Execute commands  that only need to be handled in certain mode
		if (fcb_state == PANIC_ST)
		{
			enter_panic_mode(false); //enter panic mode for any reason other than cable
		}
		// if(fcb_state == MANUAL_ST)
		// {
		// 	process_js_axis_cmd(joystick_axis, js_total_value);
		// }
		if (fcb_state == CALIBRATION_ST) 
		{
			sensor_calib();
			//offset_remove();
			fcb_state = SAFE_ST;
		}
		if (fcb_state == YAWCONTROL_ST)
		{
			N_needed = yaw_control_calc(yaw_control_pointer, yaw_set_point, sr-sr_calib);
			actuate(0, 0, 0, N_needed); // only N_needed in yay control mode
		}
		if (fcb_state == FULLCONTROL_ST)
		{
			// TODO: do full controller things
		}
		counter++;
	}
	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);
	NVIC_SystemReset();
}