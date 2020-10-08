/*

	comm.h - header file for communication between PC and FCB

*/
#include "states.h"
#include <inttypes.h>
//#include <stdint.h>

#ifndef COMM_H__
#define COMM_H__

#define BASE_MESSAGE_PACKET_BITS 0b00000000000000000000000001010101 //


// the states that our QR has
// enum STATE {
// 		SAFE_ST, 
// 		PANIC_ST,
// 		MANUAL_ST,
// 		CALIBRATION_ST,
// 		YAWCONTROL_ST,
// 		FULLCONTROL_ST,
// 		NO_WHERE
// 	};

// the command types during communication
typedef enum {
		
		CTRL_COMM, // this means keyboard command
		MODE_SW_COMM,
		JS_AXIS_COMM, // joystick commands
		CHANGE_P_COMM,
		BAT_INFO_COMM,
		SYS_LOG_COMM,
		ESC_COMM,
		USB_CHECK_COMM,
		NO_COMM
	} COMM_TYPE;

typedef enum {
		ROLL_AXIS,
		PITCH_AXIS,
		YAW_AXIS,
		LIFT_THROTTLE,	
} JOYSTICK_AXIS_t;


/**
 * @brief      Send packet from PC to FCB.
 */
// bool send_packet_to_fcb(uint8_t data);

// uint32_t append_current_mode(uint32_t messg, enum STATE g_current_state);
int check_mode_sync (uint8_t state, STATE_t g_current_state);

COMM_TYPE find_comm_type (uint8_t comm_type);

STATE_t find_dest_state(uint8_t messg);

uint32_t append_mode(uint32_t message, STATE_t mode);

#endif 