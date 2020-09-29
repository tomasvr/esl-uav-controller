/*

	comm.h - header file for communication between PC and FCB

*/
#include "in4073.h"
#include "states.h"
//#include <stdint.h>

#ifndef COMM_H__
#define COMM_H__

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
enum COMM_TYPE {
		CTRL_COMM,
		MODE_SW_COMM,
		BAT_INFO,
		SYS_LOG,
		ESC_COMM,
		NO_COMM
	};


/**
 * @brief      Send packet from PC to FCB.
 */
// bool send_packet_to_fcb(uint8_t data);

// uint32_t append_current_mode(uint32_t messg, enum STATE g_current_state);
int check_mode_sync (uint8_t state, STATE_t g_current_state);

enum COMM_TYPE find_comm_type (uint8_t comm_type);

STATE_t find_dest_state(uint8_t messg);

#endif 