/* 

	comm.c - contains implementation for communication between PC and FCB

*/


#include "comm.h"
#include <unistd.h>

#define BIT_LOCATION_COMM_TYPE 	12 //TODO: use these defines instead of number in append and retrieve functions below
#define BIT_LOCATION_STATE		8 // (depending where we want to put it) //TODO: CHANGE THIS TO 12?
#define BIT_LOCATION_JS_AXIS	8


int check_mode_sync (uint8_t state, STATE_t g_current_state){
	int mode_synced = 1; 
	STATE_t tstate;

	//printf(" state: %d\n", state);
	//printf(" g_current_state: %d\n", g_current_state);

	if (state == 0x00) tstate = SAFE_ST;				// 0000 -> SAFE_ST
	else if (state == 0x01) tstate = MANUAL_ST;			// 0001 -> MANUAL_ST
	else if (state == 0x02) tstate = CALIBRATION_ST;	// 0010 -> CALIBRATION_ST	
	else if (state == 0x03) tstate = YAWCONTROL_ST;		// 0011 -> YAWCONTROL_ST	
	else if (state == 0x04) tstate = FULLCONTROL_ST;	// 0100 -> FULLCONTROL_ST
	else if (state == 0x08) tstate = PANIC_ST;			// 1000 -> PANIC_ST
	else perror("UNKNOWN STATE IN check_mode_sync");	// STATE UNKNOWN

	if (g_current_state == tstate) mode_synced = 0;
	return mode_synced;
}

//TODO: put all these function into one using a "function template"?
uint32_t append_js_axis (uint32_t message, JOYSTICK_AXIS_t joystick_type){ //TODO: change to pointers for better performancee
	uint32_t joystick_type_uint32 = joystick_type;
	//printf("JS_AXIS is: %d\n", joystick_type);
	message |= joystick_type_uint32 << BIT_LOCATION_JS_AXIS;
	return message;
}

JOYSTICK_AXIS_t retrieve_js_axis(uint32_t message){ //TODO: change to pointers for better performancee
	//uint8_t joystick_type_uint8 = message >> BIT_LOCATION_JS_AXIS;
	uint8_t joystick_type_uint8 = message; // temporary fix
 	JOYSTICK_AXIS_t joystick_type = joystick_type_uint8;
	return joystick_type;
}

uint32_t append_comm_type(uint32_t message, COMM_TYPE comm_type){ //TODO: change to pointers for better performancee
	uint32_t comm_type_uint32 = comm_type;
	//printf("COMM_TYPE is: %d\n", comm_type);
	message |= comm_type_uint32 << BIT_LOCATION_COMM_TYPE;
	return message;
}

COMM_TYPE retrieve_comm_type(uint32_t message){ //TODO: change to pointers for better performancee
	//uint8_t comm_type_uint8 = message >> BIT_LOCATION_COMM_TYPE;
	uint8_t comm_type_uint8 = message; // temporary fix
	COMM_TYPE comm_type = comm_type_uint8;
	return comm_type;
}


uint32_t append_mode(uint32_t message, STATE_t mode){ //TODO: change to pointers for better performancee
	uint32_t mode_uint32 = mode; 	// put the enum mode into a 32 bit number
	//printf("State_t mode is: %d\n", mode);
	message |= mode_uint32 << BIT_LOCATION_STATE;			// Shift the number left until it lines up with the location of the mode in the message
	return message;
}

STATE_t retrieve_mode(uint32_t message){
	//uint8_t state_uint8 = message >> BIT_LOCATION_STATE; //todo: this step might be not necessary
	uint8_t state_uint8 = message; //temporary fix
	STATE_t state = state_uint8;
	return state;
}
