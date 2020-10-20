/* 

	comm.c - contains implementation for communication between PC and FCB

*/


#include "comm.h"
#include <unistd.h>

// #define BYTE_0_START 0
// #define BYTE_1_START 8
// #define BYTE_2_START 16

#define BIT_LOCATION_COMM_TYPE 		8 //TODO: use these defines instead of number in append and retrieve functions below
#define BIT_LOCATION_MODE			16 // (depending where we want to put it) //TODO: CHANGE THIS TO 12?
#define BIT_LOCATION_JS_AXIS		14
#define BIT_LOCATION_MOTOR_STATES	16


int check_mode_sync (uint8_t state, STATE_t g_current_state){
	if (state == g_current_state) {
		return 0;
	}
	return 1;
}

/* Append functions (PC side) */

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
	uint32_t mode_uint32 = mode; 	// put the enum mode into a 32 bit number
	message |= mode_uint32 << BIT_LOCATION_MODE;			// Shift the number left until it lines up with the location of the mode in the message
	return message;
}

/* Retrieve functions (FCB side) */

uint32_t retrieve_keyboard_motor_control (uint8_t message){ //TODO: change message to pointer for better performancee
	//todo
	return message;
}

JOYSTICK_AXIS_t retrieve_js_axis_type(uint8_t message){ //TODO: change to pointers for better performancee
 	JOYSTICK_AXIS_t joystick_type = (message >> 6);
	return joystick_type;
}

COMM_TYPE retrieve_comm_type(uint8_t message){ //TODO: change to pointers for better performancee
	COMM_TYPE comm_type = (message & 0b00111111); // the two most left bits are reserved for axis_type
	return comm_type;
}

STATE_t retrieve_mode(uint8_t message){
	STATE_t state = message;
	return state;
}
