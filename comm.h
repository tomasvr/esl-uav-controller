/*

	comm.h - header file for communication between PC and FCB

*/
#include "in4073.h"
//#include <stdint.h>

#ifndef COMM_H__
#define COMM_H__

// the states that our QR has
enum STATE {
		SAFE_ST, 
		PANIC_ST,
		MANUAL_ST,
		CALIBRATION_ST,
		YAWCONTROL_ST,
		FULLCONTROL_ST,
		NO_WHERE
	};

// the command types during communication
enum COMM_TYPE {
		CTRL_COMM,
		MODE_SW_COMM,
		BAT_INFO,
		SYS_LOG,
		ESC_COMM,
		JS_COMM
		NO_COMM
	};

// the stats that motor 0 has
enum M0_CRTL{
		M0_UP,
		M0_REMAIN,
		M0_DOWN,
		M0_ASCENDING,
		M0_DESCENDING
	};
// the states that motor 1 has
enum M1_CRTL{		
		M1_UP,
		M1_REMAIN,
		M1_DOWN,
		M1_ASCENDING,
		M1_DESCENDING
	};
// the states that motor 2 has
enum M2_CRTL{	
		M2_UP,
		M2_REMAIN,
		M2_DOWN,
		M2_ASCENDING,
		M2_DESCENDING
	};
// the states that motor 3 has
enum M3_CRTL{
		M3_UP,
		M3_REMAIN,
		M3_DOWN,
		M3_ASCENDING,
		M3_DESCENDING
	};

/**
 * @brief      Send packet from PC to FCB.
 */
// bool send_packet_to_fcb(uint8_t data);

// uint32_t append_current_mode(uint32_t messg, enum STATE g_current_state);
int check_mode_sync (uint8_t state, enum STATE g_current_state);

enum COMM_TYPE find_comm_type (uint8_t comm_type);

enum STATE find_dest_state(uint8_t messg);

#endif 