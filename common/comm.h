/*
 *	comm.h - header file for communication between PC and FCB
 */

#ifndef COMM_H__
#define COMM_H__

#include "configuration.h"
#include "states.h"
#include <inttypes.h>

/* Trimming command translation to byte format */
#define LIFT_UP 	0b01010101
#define LIFT_DOWN 	0b10101010  
#define PITCH_DOWN  0b10000100
#define PITCH_UP  	0b01001000
#define ROLL_RIGHT 	0b00100001
#define ROLL_LEFT  	0b00010010
#define YAW_LEFT 	0b10011001
#define YAW_RIGHT 	0b01100110

// The different communication (packet) types sent over UART
typedef enum {
	UNKNOWN_COMM,
	CTRL_COMM, // keyboard command
	MODE_SW_COMM,
	JS_AXIS_COMM, // joystick commands
	CHANGE_P_COMM,
	ESC_COMM,
	STATE_SYNC_COMM,
} COMM_TYPE;

// The different JS axis appearing in JS command packets
typedef enum {
		ROLL_AXIS,
		PITCH_AXIS,
		YAW_AXIS,
		LIFT_THROTTLE,	
} JOYSTICK_AXIS_t;

// The different types of parameter changes appearing in parameter change packetes
typedef enum {
		P_RATE_YAW_INC,
		P_RATE_YAW_DEC,
		P_ANGLE_PITCHROLL_INC,
		P_ANGLE_PITCHROLL_DEC,
		P_RATE_PITCHROLL_INC,
		P_RATE_PITCHROLL_DEC,
		P_SHIFT_RIGHT_VALUE_INC,
		P_SHIFT_RIGHT_VALUE_DEC
} PARAM_CHANGE_t;

int check_mode_sync (uint8_t state, STATE_t fcb_state);

int8_t translate_axis(uint8_t value); 
uint8_t translate_throttle(int8_t throttle);

/* Insert data in packet byte */
uint32_t append_keyboard_motor_control(uint32_t message, uint8_t mode);
uint32_t append_js_axis_type(uint32_t message, JOYSTICK_AXIS_t mode);
uint32_t append_comm_type(uint32_t message, COMM_TYPE mode);
uint32_t append_mode(uint32_t message, STATE_t mode);
uint32_t append_parameter_change(uint32_t message, PARAM_CHANGE_t mode);

/* Retrieve data from packet byte */
uint8_t 		retrieve_keyboard_motor_control (uint8_t packet_byte);
JOYSTICK_AXIS_t retrieve_js_axis_type(uint8_t packet_byte);
COMM_TYPE 		retrieve_comm_type(uint8_t packet_byte);
STATE_t 		retrieve_mode(uint8_t packet_byte);
PARAM_CHANGE_t 	retrieve_parameter_change(uint8_t packet_byte);


#endif 