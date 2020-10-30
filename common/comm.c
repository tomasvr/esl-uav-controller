/* 

	comm.c - contains implementation for communication between PC and FCB

*/


#include "comm.h"
#include <unistd.h>

#define BIT_LOCATION_COMM_TYPE 		8 
#define BIT_LOCATION_MODE			16 
#define BIT_LOCATION_JS_AXIS		14
#define BIT_LOCATION_MOTOR_STATES	16
#define BIT_LOCATION_PARAM			16

/* 
* Check whether pc and drone is in the same state.
* J. Cui 
*/
int check_mode_sync (uint8_t pc_state, STATE_t fcb_state){
	if (pc_state == fcb_state) {
		return 0;
	}
	return 1;
}

/* Translate js axis to range: [-127, 127] (unsigned to signed) */
int8_t translate_axis(uint8_t value) {
	int8_t signed_valued;
	if (value <= 127) {
		signed_valued = value;
		return signed_valued;
	}
	signed_valued = value - 255;
	return signed_valued;
} 

/* Translate js axis to range: 0-255 instead of 0 in the middle */
uint8_t translate_throttle(int8_t throttle) {
	if(throttle <= JS_AXIS_MID_VALUE){
		throttle = JS_AXIS_MID_VALUE - throttle;
	}
	else {
		throttle = JS_AXIS_MAX_VALUE - throttle + JS_AXIS_MID_VALUE;
	}
	return throttle;
}

/*  
* Append functions, append required bits for a message. (PC side)
* " "
*/
uint32_t append_keyboard_motor_control (uint32_t message, uint8_t motor_states){
	message |= motor_states << BIT_LOCATION_MOTOR_STATES;
	return message;
}

uint32_t append_js_axis_type (uint32_t message, JOYSTICK_AXIS_t joystick_type){ 
	uint32_t joystick_type_uint32 = joystick_type;
	message |= joystick_type_uint32 << BIT_LOCATION_JS_AXIS;
	return message;
}

uint32_t append_comm_type(uint32_t message, COMM_TYPE comm_type){
	uint32_t comm_type_uint32 = comm_type;
	message |= comm_type_uint32 << BIT_LOCATION_COMM_TYPE;
	return message;
}

uint32_t append_mode(uint32_t message, STATE_t mode){ 
	uint32_t mode_uint32 = mode; 	
	message |= mode_uint32 << BIT_LOCATION_MODE;			
	return message;
}

uint32_t append_parameter_change(uint32_t message, PID_PARAMETER_CHANGE_t parameter){ 
	uint32_t param_uint32 = parameter;
	message |= param_uint32 << BIT_LOCATION_PARAM;			
	return message;
}

/* 
* Retrieve required info from a message. (Drone side) 
* " "
*/
uint8_t retrieve_keyboard_motor_control (uint8_t message_byte){ //TODO: change message to pointer for better performancee
	//todo
	return message_byte;
}

JOYSTICK_AXIS_t retrieve_js_axis_type(uint8_t message_byte){ //TODO: change to pointers for better performancee
 	JOYSTICK_AXIS_t joystick_type = (message_byte >> 6);
	return joystick_type;
}

COMM_TYPE retrieve_comm_type(uint8_t message_byte){ //TODO: change to pointers for better performancee
	COMM_TYPE comm_type = (message_byte & 0b00111111); // the two most left bits are reserved for axis_type
	return comm_type;
}

STATE_t retrieve_mode(uint8_t message_byte){
	STATE_t state = message_byte;
	return state;
}

PID_PARAMETER_CHANGE_t retrieve_parameter_change(uint8_t message_byte){ 
	return message_byte;
}
