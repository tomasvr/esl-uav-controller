<<<<<<< HEAD
/*
	control.h - header file for (motor) control
*/

#ifndef CONTROL_H__
#define CONTROL_H__

#define DEBUG_LED
#define STEP_SIZE 10
#define UPPER_LIMIT 1000

#define CONTROLLER_P_STEP_SIZE 1
#define CONTROLLER_P_UPPER_LIMIT 50
#define CONTROLLER_P_LOWER_LIMIT 1 

#define MAX_ALLOWED_SPEED 1000
#define MIN_ALLOWED_SPEED 0
#define MAX_ALLOWED_DIFF_MOTOR 50

// the states that a motor has
typedef enum {
		MOTOR_REMAIN,
		MOTOR_UP,
		MOTOR_DOWN,
		MOTOR_UNKNOWN
} MOTOR_CTRL;

extern MOTOR_CTRL g_current_m0_state;
extern MOTOR_CTRL g_current_m1_state;
extern MOTOR_CTRL g_current_m2_state;
extern MOTOR_CTRL g_current_m3_state;

typedef struct // TODO: should use float?
{
	int16_t set_point;
	int16_t sensor_value;
	int16_t err;
	uint8_t kp_angle, kp_rate, ki;
	int16_t integral;
	int16_t output;

} CONTROLLER;

void increase_motor_speed(uint8_t motor);
void decrease_motor_speed(uint8_t motor);
void keyboard_ctrl_action();

// calibration
extern bool DMP;
extern bool calib_done;
extern uint8_t calib_counter;
extern int16_t sensor_sum;
extern int32_t angle_calib[3];
extern int32_t gyro_calib[3];
extern int32_t acce_calib[3];
extern int16_t phi_calib, theta_calib, psi_calib;
extern int16_t sp_calib, sq_calib, sr_calib;
extern int16_t sax_calib, say_calib, saz_calib;
void sensor_calc(uint8_t);
void sensor_calib(void);

// controller
#define b 1 
#define d 1
void controller_init(CONTROLLER *controller);
void increase_p_value(CONTROLLER *controller);
void decrease_p_value(CONTROLLER *controller);

#endif // CONTROL_H__

// uint32_t timestamp = get_time_us();
// struct info_log info_logging = {timestamp, bat_volt};

// uint8_t address_init = 0x000000;
// uint8_t address_counter = 0;
// void data_logging()
// {
// struct sensor_log sensor_logging = {phi, theta, psi, sp , sq, sr};
// if(flash_write_bytes(address_init + address_counter, (uint8_t*) &sensor_logging, 4))printf("+");
// }

// void data_logging()
// {
// 	uint8_t counter = 0;
// 	uint8_t restore_counter = 0;
// 	uint8_t time_store[4] = {0};
// 	uint32_t time_restore[1] = {0};// uint8_t time_counter = 0;// uint8_t time_restore_counter = 0;  

// 	uint8_t  battery_store[2] = {0};
// 	uint16_t battery_restore[1] = {0};// uint8_t battery_counter = 0;// uint8_t battery_restore_counter = 0;
	
// 	uint8_t  phi_store[2] = {0};
// 	uint16_t phi_restore[1] = {0};// uint8_t phi_counter = 0;// uint8_t phi_restore_counter = 0;
// 	int16_t phi_d = 0;

// 	uint8_t  theta_store[2] = {0};
// 	uint16_t theta_restore[1] = {0};// uint8_t theta_counter = 0;// uint8_t theta_restore_counter = 0;
// 	int16_t theta_d = 0;

// 	uint8_t  psi_store[2] = {0};
// 	uint16_t psi_restore[1] = {0};// uint8_t psi_counter = 0;// uint8_t psi_restore_counter = 0;
// 	int16_t psi_d = 0;

// 	uint8_t pressure_store[4] = {0};
// 	uint32_t pressure_restore[1] = {0};// uint8_t pressure_counter = 0;// uint8_t pressure_restore_counter = 0;   

// 	//time
// 	time_store[counter] = (get_time_us() >> 24); time_store[counter+1] = (get_time_us() >> 16);
// 	time_store[counter+2] = (get_time_us() >> 8); time_store[counter+3] = get_time_us();

// 	if(flash_write_bytes(address_init + address_counter, &time_store[address_counter], 4))printf(" |");
// 	if(flash_read_bytes(address_init + address_counter, &time_store[address_counter], 4))printf("|");

// 	time_restore[restore_counter] = (time_store[counter] << 24) | (time_store[counter+1] << 16) 
// 	| (time_store[counter+2] << 8) | (time_store[counter+3]);
// 	printf("|%10ld ", time_restore[restore_counter]);
// 	// time_restore_counter++; time_counter = time_counter + 4;
// 	address_counter = address_counter + 4;

// 	//Battery
// 	battery_store[counter] = bat_volt >> 8; battery_store[counter+1] = bat_volt;

// 	if(flash_write_bytes(address_init + address_counter, &battery_store[address_counter], 2))printf(" |");
// 	if(flash_read_bytes(address_init + address_counter, &battery_store[address_counter], 2))printf("|");

// 	battery_restore[restore_counter] = battery_store[counter] << 8 | battery_store[counter+1];
// 	printf("|%4d ", battery_restore[restore_counter]);
// 	// battery_restore_counter++; battery_counter = battery_counter + 2;
// 	address_counter = address_counter + 2;

// 	//phi
// 	phi_store[counter] = phi >> 8; phi_store[counter+1] = phi;

// 	if(flash_write_bytes(address_init + address_counter, &phi_store[address_counter], 2))printf(" |");
// 	if(flash_read_bytes(address_init + address_counter, &phi_store[address_counter], 2))printf("|");

// 	phi_restore[restore_counter] = phi_store[counter] << 8 | phi_store[counter + 1];
// 	if(phi_restore[restore_counter] > 32768) {
// 		phi_d = phi_restore[restore_counter] - 65536;
// 	}
// 	else
// 		phi_d = phi_restore[restore_counter];

// 	printf("|%6d ", phi_d);
// 	// phi_restore_counter++; phi_counter = phi_counter + 2;
// 	address_counter = address_counter + 2;

// 	//theta
// 	theta_store[counter] = theta >> 8; theta_store[counter+1] = theta;

// 	if(flash_write_bytes(address_init + address_counter, &theta_store[address_counter], 2))printf(" |");
// 	if(flash_read_bytes(address_init + address_counter, &theta_store[address_counter], 2))printf("|");

// 	theta_restore[restore_counter] = theta_store[counter] << 8 | theta_store[counter + 1];
// 	if(theta_restore[restore_counter] > 32768) {
// 		theta_d = theta_restore[restore_counter] - 65536;
// 	}
// 	else
// 		theta_d = theta_restore[restore_counter];

// 	printf("|%6d ", theta_d);
// 	address_counter = address_counter + 2;

// 	//psi
// 	psi_store[counter] = psi >> 8; psi_store[counter+1] = psi;

// 	if(flash_write_bytes(address_init + address_counter, &psi_store[address_counter], 2))printf(" |");
// 	if(flash_read_bytes(address_init + address_counter, &psi_store[address_counter], 2))printf("|");

// 	psi_restore[restore_counter] = psi_store[counter] << 8 | psi_store[counter + 1];
// 	if(psi_restore[restore_counter] > 32768) {
// 		psi_d = psi_restore[restore_counter] - 65536;
// 	}
// 	else
// 		psi_d = psi_restore[restore_counter];

// 	printf("|%6d ", psi_d);
// 	address_counter = address_counter + 2;

// 	//pressure
// 	pressure_store[counter] = (pressure >> 24);pressure_store[counter+1] = (pressure >> 16);
// 	pressure_store[counter+2] = (pressure >> 8);pressure_store[counter+3] = pressure;

// 	if(flash_write_bytes(address_init + address_counter, &pressure_store[address_counter], 4))printf(" |");
// 	if(flash_read_bytes(address_init + address_counter, &pressure_store[address_counter], 4))printf("|");

// 	pressure_restore[restore_counter] = (pressure_store[counter] << 24) | (pressure_store[counter+1] << 16) 
// 	| (pressure_store[counter+2] << 8) | (pressure_store[counter+3]);
// 	printf("| %6ld\n", pressure_restore[restore_counter]);
// 	//pressure_restore_counter++;pressure_counter = pressure_counter + 4;
// 	address_counter = address_counter + 4;
// }
=======
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

// TODO: delete the macros after debug
/* --- PRINTF_BYTE_TO_BINARY macro's --- */
// #define PRINTF_BINARY_PATTERN_INT8 "%c%c%c%c%c%c%c%c"
// #define PRINTF_BYTE_TO_BINARY_INT8(i)    \
//     (((i) & 0x80ll) ? '1' : '0'), \
//     (((i) & 0x40ll) ? '1' : '0'), \
//     (((i) & 0x20ll) ? '1' : '0'), \
//     (((i) & 0x10ll) ? '1' : '0'), \
//     (((i) & 0x08ll) ? '1' : '0'), \
//     (((i) & 0x04ll) ? '1' : '0'), \
//     (((i) & 0x02ll) ? '1' : '0'), \
//     (((i) & 0x01ll) ? '1' : '0')
// #define PRINTF_BINARY_PATTERN_INT16 \
//     PRINTF_BINARY_PATTERN_INT8              PRINTF_BINARY_PATTERN_INT8
// #define PRINTF_BYTE_TO_BINARY_INT16(i) \
//     PRINTF_BYTE_TO_BINARY_INT8((i) >> 8),   PRINTF_BYTE_TO_BINARY_INT8(i)
// #define PRINTF_BINARY_PATTERN_INT32 \
//     PRINTF_BINARY_PATTERN_INT16             PRINTF_BINARY_PATTERN_INT16
// #define PRINTF_BYTE_TO_BINARY_INT32(i) \
//     PRINTF_BYTE_TO_BINARY_INT16((i) >> 16), PRINTF_BYTE_TO_BINARY_INT16(i)
// #define PRINTF_BINARY_PATTERN_INT64    \
//     PRINTF_BINARY_PATTERN_INT32             PRINTF_BINARY_PATTERN_INT32
// #define PRINTF_BYTE_TO_BINARY_INT64(i) \
//     PRINTF_BYTE_TO_BINARY_INT32((i) >> 32), PRINTF_BYTE_TO_BINARY_INT32(i)
/* --- end macros --- */


#include "in4073.h"
#include <assert.h>

#define USB_COMM_INTERVAL_THRESHOLD 2000000 // in us (1000000 = 1 second) 
#define BATTERY_CHECK_INTERVAL_THRESHOLD 5000000   

uint32_t usb_comm_last_received;
uint32_t current_time;
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

void enter_panic_mode(bool cable_detached, char caller[]){
	if (fcb_state == SAFE_ST) {
		return; // if in safe mode then you do not need to go to panic mode
	}
	printf("FCB: QR: Entered PANIC MODE called by: %s", caller);
	uint16_t temp = lift;
	uint16_t motor_speed = (temp << 2);
	while (motor_speed > 0) {
		motor_speed = motor_speed - 10; 
		if (motor_speed < 10) {
			motor_speed = 0;
		}
		ae[0] = motor_speed;
		ae[1] = motor_speed;
		ae[2] = motor_speed;
		ae[3] = motor_speed;
		printf("motor speed: %d\n", motor_speed);
		update_motors(); //or run filters_and_control() ? <- enter panic mode does not need control, so update_motors() is enough
		nrf_delay_ms(100);
	}
	lift = 0; //reset motor_lift_level
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
	if (current_time - usb_comm_last_received > USB_COMM_INTERVAL_THRESHOLD) { //TODO: check for overflow? <- Need fix when operation time > 70 mins
		enter_panic_mode(true, "usb_check failed");
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

/* Translate js axis to range: 0-255 instead of 0 in the middle */
uint8_t translate_throttle(uint8_t throttle) {
	if(throttle <= JS_AXIS_MID_VALUE){
		throttle = JS_AXIS_MID_VALUE - throttle;
	}
	else {
		throttle = JS_AXIS_MAX_VALUE - throttle + JS_AXIS_MID_VALUE;
	}
	return throttle;
}

void keyboard_adjust_motors(uint8_t motor_states) {
	uint8_t step = STEP_SIZE >> 2;
	switch(motor_states){
		case LIFT_UP:
			lift += step;
			lift = clip_motor_value(lift << 2) / 4;
			break;
		case LIFT_DOWN:
			if (lift != 0) 
				lift -= step;
			lift = clip_motor_value(lift << 2) / 4;
			break;
		case PITCH_UP:
			pitch += step;
			break;
		case PITCH_DOWN:
			pitch -= step;
			break;
		case ROLL_RIGHT:
			roll += step;
			break;
		case ROLL_LEFT:
			roll -= step;
			break;
		case YAW_RIGHT:
			yaw += step;
			break;
		case YAW_LEFT:
			yaw -= step;
			break;				
	}
	printf("key adjust: %d\n", motor_states);
}

/*
 * Decode messages
 * "aruthor"
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
					demo_done = false;
					return;
				case CTRL_COMM:
					;	// C requires this semicolon here

					/* only change motors if in appropriate mode */ //todo: move this logic to a central place
					if (fcb_state == MANUAL_ST || fcb_state == YAWCONTROL_ST || fcb_state == FULLCONTROL_ST) {
						//keyboard_ctrl_action();
						/* If 'a' or 'z' was pressed, adjust motor_lift_level */
						uint8_t motor_states = retrieve_keyboard_motor_control(message_byte);
						keyboard_adjust_motors(motor_states);
						// printf("FCB: motor states: "PRINTF_BINARY_PATTERN_INT8"\n",PRINTF_BYTE_TO_BINARY_INT8(motor_states));
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
					//printf("axis: %d value: %d \n", js_axis_type, message_byte);						
					switch (js_axis_type) {
						case ROLL_AXIS:
							roll = (int8_t) message_byte;
							break;
						case PITCH_AXIS:
							pitch = (int8_t) message_byte;
							break;
						case YAW_AXIS:
							yaw = (int8_t) message_byte;
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
			 			default: 
				 			printf("FCB: UKNOWN CHANGE P VALUE: \n", message_byte);
					}					
				 	break;
				case BAT_INFO_COMM:
					break;
				case SYS_LOG_COMM:
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

/*
 * Process the received packet.
 * J. Cui
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
		printf("process key called but FRAG_COUNT < 0, FRAG_COUNT: \n", FRAG_COUNT);
	}
}

// TODO: change placement?
uint32_t last_time_battery;
uint32_t current_time_battery;

/*
 * Check battery voltage.
 * "aruthor"
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
void print_log_in_ter() {
	// printf("%10ld | ", get_time_us());
	printf("%3d %3d %3d %3d  | ",ae[0],ae[1],ae[2],ae[3]);
	//printf("%6d %6d %6d | ", phi, theta, psi);
	printf("%6d %6d %6d | ", sp, sq, sr);
	//printf("%4d | %4ld | %6ld   | ", bat_volt, temperature, pressure);
	printf("y_p_r: %2d r_p_r: %2d p_p_r: %2d", yaw_control.kp_rate, roll_control.kp_rate, pitch_control.kp_rate);
	printf("r_p_a: %2d p_p_a: %2d", roll_control.kp_angle, pitch_control.kp_angle);
	printf("setp: %4d sp: %4d err: %4d output: %4d ", roll_control.set_point, sp, roll_control.err, roll_control.output);
	printf("%4d |", fcb_state - 1);
	printf("%4d \n", ctrl_loop_time);
	clear_timer_flag();
	//printf("%4d \n", motor_lift_level);
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
	
	motor_lift_level = 0;
	
	controller_init(yaw_control_pointer);
	controller_init(roll_control_pointer);
	controller_init(pitch_control_pointer);

	// printf(" AE0 AE1 AE2 AE3  | MODE \n");
	while (!demo_done)
	{
		if (rx_queue.count) process_packet( dequeue(&rx_queue) );

		if (counter % 100 == 0) check_USB_connection_alive();
		//check_battery_volt();//enable panic mode when connect to drone

		if (check_timer_flag()) {
			if (counter % 20 == 0) {
				nrf_gpio_pin_toggle(BLUE);
 			}
			adc_request_sample();
			read_baro();
			print_log_in_ter();
		}

		if (check_sensor_int_flag()) {
			get_dmp_data();
			ctrl_loop_time = get_time_us();
			run_filters_and_control();
			ctrl_loop_time = get_time_us() - ctrl_loop_time;
		}

		// Execute commands that need to be handled in all modes
		if (g_current_comm_type == ESC_COMM){ // terminate program
			demo_done = true;
			g_current_comm_type = UNKNOWN_COMM;
			enter_panic_mode(false, "ESC pressed");
		}
		counter++;
	}
	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);
	NVIC_SystemReset();
}
>>>>>>> master
