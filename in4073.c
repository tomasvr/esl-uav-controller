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
uint32_t ctrl_loop_time;

// each packet includes 3 fragments
uint8_t FRAG_COUNT = 0;

int8_t pitch_trim = 0;
int8_t roll_trim = 0;
int8_t yaw_trim = 0;

uint32_t last_time_battery;
uint32_t current_time_battery;

// State variables initalization
STATE_t fcb_state = SAFE_ST;

// Communication variables initalization
COMM_TYPE g_current_comm_type = UNKNOWN_COMM;

// Keep track of current js_axis_type & value
JOYSTICK_AXIS_t js_axis_type;
uint8_t js_total_value;

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

/**
 * @brief      Returns a string representation of a state.
 *
 * @param[in]  p_state  The state
 *
 * @return     String representation of the state.
 * 
 * @author     T. van Rietbergen
 */
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
		default:
			return "UNKNOWN_ST";	
			break;		
	}
}

/**
 * @brief      Enters a panic mode.
 *
 * @param[in]  remain_off  Whether drone remains off after panic mode
 * @param      info        Reason for calling panic mode
 *
 * @author     T. van Rietbergen
 */
void enter_panic_mode(bool remain_off, char info[]){
	if (fcb_state == SAFE_ST) {
		return; // if in safe mode then you do not need to go to panic mode
	}
	printf("FCB: PANIC MODE called by: %s", info);
	fcb_state = PANIC_ST; // make sure system is in panic state;
	uint32_t panic_start_time = get_time_us();
	uint32_t panic_elapsed_time = get_time_us() - panic_start_time;
	while(panic_elapsed_time < PANIC_DURATION) {
		if (check_sensor_int_flag()) {
			get_dmp_data();
		}
		run_filters_and_control();
		panic_elapsed_time = get_time_us() - panic_start_time;
	}
	lift = 0; //reset motor_lift_level
	fcb_state = SAFE_ST;
	run_filters_and_control();
	if (remain_off) { 
		demo_done = true;
	}
}

/**
 * @brief      Check last time a USB check message has been received
 * 
 * @author     T. van Rietbergen
 */
void check_USB_connection_alive() {
	if (get_time_us() - usb_comm_last_received > USB_COMM_INTERVAL_THRESHOLD) { //TODO: check for overflow? <- Need fix when operation time > 70 mins
		enter_panic_mode(true, "usb_check failed \n");
	}
}

/**
 * @brief      Process keyboard trimming command
 *
 * @param[in]  trim_command  The trim command
 * 
 * @author     Xinyun Xu
 */
void process_trim_command(uint8_t message_byte) {
	/* only change motors if in appropriate mode */ 
	if (fcb_state == MANUAL_ST || fcb_state == YAWCONTROL_ST || fcb_state == FULLCONTROL_ST) {
		uint8_t trim_command = retrieve_keyboard_motor_control(message_byte);
		switch(trim_command){
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
	}
	else {
		printf("Cannot control keyboard motor in current mode: %d \n", fcb_state);						
	}
}

/**
 * @brief      Process JS command from packet
 *
 * @param[in]  message_byte  The message byte
 * 
 * @author     T. van Rietbergen
 */
void process_js_command(uint8_t message_byte) {
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
	/* Always store received JS status for checking neutral position */
	joystick_axis_stored_values[js_axis_type] = js_total_value;
}

/**
 * @brief      Process the state sync message which checks for state sync between PC and FCB
 */
void process_state_sync_message(uint8_t message_byte) {
	if (check_mode_sync(message_byte, fcb_state) != 0) {
		printf("ERROR: STATE MISMATCH - PC state: %d, FCB state: %d \n", message_byte, fcb_state);
		enter_panic_mode(false, "State mismatch");
	}		
}

/**
 * @brief      Decode message byte based on comm type and fragment count
 *
 * @param[in]  message_byte  The message byte to be processed
 * 
 * @author     T. van Rietbergem
 */
void messg_decode(uint8_t message_byte){

	/* First byte is 	start byte 				 */
	/* Second byte is 	comm_type byte 	(case 2) */ // ONLY FOR JS_AXIS_TYPE ARE THE LEFT MOST 2 BYTES FOR AXIS TYPE
	/* Third byte is 	data 			(case 1) */

	switch(FRAG_COUNT) {
		case 2: // Comm Type
			/* If a new message is received, update last received message time */
			usb_comm_last_received = get_time_us();
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
					process_trim_command(message_byte);
					break;
				case MODE_SW_COMM:
					fcb_state = mode_sw_action("FCB", fcb_state, retrieve_mode(message_byte));
					break;
				case JS_AXIS_COMM:
					process_js_command(message_byte);
					break;
				case CHANGE_P_COMM:
					adjust_parameter_value(message_byte);
				 	break;
				case STATE_SYNC_COMM:
					process_state_sync_message(message_byte);				
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

/**
 * @brief      Process the received packet.
 *
 * @param[in]  c     Received packet byte
 * 
 * @author     J. Cui
 */
void process_packet(uint8_t c){	
	if (c == 0x55 && FRAG_COUNT == 0 && fcb_state != PANIC_ST) {
		FRAG_COUNT = 2;
		return;
	}
	if (FRAG_COUNT > 0){
		messg_decode(c);
		FRAG_COUNT--;
	} else {
		printf("process key called but FRAG_COUNT < 0, FRAG_COUNT: %d \n", FRAG_COUNT);
	}
}

/*
 * Check battery voltage.
 * Xinyun Xu
 */
void check_battery_volt(){
	current_time_battery = get_time_us();
	if(current_time_battery - last_time_battery > BATTERY_CHECK_INTERVAL_THRESHOLD){
		printf("current voltage: %d.%dV \n", bat_volt/100, bat_volt&0x00FF);
		if(bat_volt < 1150 && bat_volt > 1100){
			printf("CURRENT VOLTAGE: %d.%dV \n", bat_volt/100, bat_volt&0x00FF);
		}
		else if(bat_volt < 1100 && bat_volt > 1050){
			printf("WARNING, CURRENT VOLTAGE1: %d.%dV \n", bat_volt/100, bat_volt&0x00FF);
			enter_panic_mode(true, "batter voltage too low");
		}
		else if(bat_volt < 1050){
			printf("WARNING, CURRENT VOLTAGE2: %d.%dV \n", bat_volt/100, bat_volt&0x00FF);
			enter_panic_mode(true, "batter voltage CRITICAL"); 
		}
	last_time_battery = current_time_battery;
	}
}

/*
* Print info in the terminal.
* J. Cui
*/
void print_info() {
	printf("%10ld | ", get_time_us());
	printf("%3d %3d %3d %3d  | ",ae[0],ae[1],ae[2],ae[3]);
	//printf("trim p: %2d r: %2d y: %2d", pitch_trim, roll_trim, yaw_trim);
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
	while (!demo_done)
	{
		if (rx_queue.count) process_packet( dequeue(&rx_queue) );

		if (fcb_state == PANIC_ST) {
			enter_panic_mode(false, "FCB IN PANIC_ST");
		}

		if (check_timer_flag()) {
			nrf_gpio_pin_toggle(BLUE);
			check_USB_connection_alive();
			#ifdef ENABLE_BATT_CHECK
			adc_request_sample();
			check_battery_volt();//enable panic mode when connect to drone			
			#endif
			read_baro();

			if (counter % 20 == 0) {
			logging();
			}
			print_info();
		}

		if (check_sensor_int_flag()) {
			get_dmp_data();
			run_filters_and_control();
		}
		counter++;
	}
	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);
	NVIC_SystemReset();
}