/* 

	comm.c - contains implementation for communication between PC and FCB

*/


#include "comm.h"
#include <unistd.h>
/**
 * @brief      Sends a packet from PC to FCB
 *
 * @author     Tomas van Rietbergen
 *
 * @param      data  The data
 *
 * @return     { description_of_the_return_value }
 */


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

// COMM_TYPE find_comm_type (uint8_t comm_type){
// 	COMM_TYPE current_comm_type;
// 	if 		(comm_type == 0x00) current_comm_type = CTRL_COMM;			// 0000 -> CTRL_COMM
// 	else if (comm_type == 0x10) current_comm_type = MODE_SW_COMM;		// 0001 -> MODE_SW_COMM
// 	else if (comm_type == 0x20) current_comm_type = JS_COMM;			// 0010 -> JS_COMM
// 	else if (comm_type == 0x70) current_comm_type = CHANGE_P_COMM;		// 0111 -> CHANGE_P_COMM
// 	else if (comm_type == 0x80) current_comm_type = BAT_INFO;			// 1000 -> BAT_INFO
// 	else if (comm_type == 0x90) current_comm_type = USB_CHECK_COMM;		// 1001 -> USB_CHECK_COMM
// 	else if (comm_type == 0xb0) current_comm_type = SYS_LOG;			// 1011 -> SYS_LOG
// 	else if (comm_type == 0xf0) current_comm_type = ESC_COMM;			// 1111 -> ESC
// 	else {
// 		current_comm_type = NO_COMM;
// 		printf("COMM TYPE WAS NOT RECOGNISED\n");
// 	}
// 	return current_comm_type;
// }

// STATE_t find_dest_state(uint8_t messg){
// 	STATE_t dest_state = NO_WHERE;
// 	if 		(messg == 0x00) dest_state = SAFE_ST;
// 	else if (messg == 0x80) dest_state = PANIC_ST;
// 	else if (messg == 0x10) dest_state = MANUAL_ST;
// 	else if (messg == 0x20) dest_state = CALIBRATION_ST;
// 	else if (messg == 0x30) dest_state = YAWCONTROL_ST;
// 	else if (messg == 0x40) dest_state = FULLCONTROL_ST;
// 	else 					dest_state = NO_WHERE;
// 	return dest_state;
// }

// JOYSTICK_TYPE_t find_joystick_message_type(uint8_t messg){
// 	JOYSTICK_TYPE_t joystick_type;
// 	if 		(messg == 0x00) joystick_type = ROLL_AXIS; //0000
// 	else if (messg == 0x01) joystick_type = PITCH_AXIS; //0001
// 	else if (messg == 0x02) joystick_type = YAW_AXIS;   //0010
// 	else if (messg == 0x03) joystick_type = LIFT_THROTTLE; // 0011
// 	else printf("error, joystick_type not found\n");
// 	return joystick_type;
// }
//					payload     payload     comm mode    start bit
// Example message 0000-0000   0000-0000    0000-0000    0101-0101


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
