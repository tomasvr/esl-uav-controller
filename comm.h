/*
 *	comm.h - header file for communication between PC and FCB
 */

#include "states.h"
#include <inttypes.h>

#ifndef COMM_H__
#define COMM_H__

#define BASE_MESSAGE_PACKET_BITS 0b00000000000000000000000001010101

/* Used for keyboard control */
#define LIFT_UP 	0b01010101
#define LIFT_DOWN 	0b10101010  
#define PITCH_DOWN  0b10000100
#define PITCH_UP  	0b01001000
#define ROLL_RIGHT 	0b00100001
#define ROLL_LEFT  	0b00010010
#define YAW_LEFT 	0b10011001
#define YAW_RIGHT 	0b01100110

// the command types during communication
typedef enum {
	UNKNOWN_COMM,
	CTRL_COMM, // keyboard command
	MODE_SW_COMM,
	JS_AXIS_COMM, // joystick commands
	CHANGE_P_COMM,
	BAT_INFO_COMM,
	SYS_LOG_COMM,
	ESC_COMM,
	USB_CHECK_COMM,
} COMM_TYPE;

typedef enum {
		ROLL_AXIS,
		PITCH_AXIS,
		YAW_AXIS,
		LIFT_THROTTLE,	
} JOYSTICK_AXIS_t;

typedef enum {
		P_RATE_YAW_INC,
		P_RATE_YAW_DEC,
		P_ANGLE_PITCHROLL_INC,
		P_ANGLE_PITCHROLL_DEC,
		P_RATE_PITCHROLL_INC,
		P_RATE_PITCHROLL_DEC,
} PID_PARAMETER_CHANGE_t;

int check_mode_sync (uint8_t state, STATE_t fcb_state);

uint32_t append_mode(uint32_t message, STATE_t mode);

#endif 