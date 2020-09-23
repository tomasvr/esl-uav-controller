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


int check_mode_sync (uint8_t state, enum STATE g_current_state){
	int mode_synced = 0; 

	if (state == 0x00){					// 0000 -> SAFE_ST
		enum STATE tstate = SAFE_ST;
		if (g_current_state == tstate) mode_synced = 1;
	}
	if (state == 0x01){					// 0001 -> MANUAL_ST
		enum STATE tstate = MANUAL_ST;
		if (g_current_state == tstate) mode_synced = 1;
	}
	if (state == 0x02){					// 0010 -> CALIBRATION_ST
		enum STATE tstate = CALIBRATION_ST;
		if (g_current_state == tstate) mode_synced = 1;
	}
	if (state == 0x03){					// 0011 -> YAWCONTROL_ST
		enum STATE tstate = YAWCONTROL_ST;
		if (g_current_state == tstate) mode_synced = 1;
	}
	if (state == 0x04){					// 0100 -> FULLCONTROL_ST
		enum STATE tstate = FULLCONTROL_ST;
		if (g_current_state == tstate) mode_synced = 1;
	}
	if (state == 0x08){					// 0000 -> SAFE_ST
		enum STATE tstate = PANIC_ST;
		if (g_current_state == tstate) mode_synced = 1;
	}
	
	return mode_synced;
}

enum COMM_TYPE find_comm_type (uint8_t comm_type){
	enum COMM_TYPE current_comm_type = NO_COMM;
	if (comm_type == 0x00) {			// 0000 -> CTRL_COMM
		current_comm_type = CTRL_COMM;	
	}else if(comm_type == 0x10){		// 0001 -> MODE_SW_COMM
		current_comm_type = MODE_SW_COMM;
	}else if(comm_type == 0x80){		// 1000 -> BAT_INFO
		current_comm_type = BAT_INFO;
	}else if(comm_type == 0x90){		// 1001 -> SYS_LOG
		current_comm_type = SYS_LOG;
	}else if (comm_type == 0xf0){
		current_comm_type = ESC_COMM;	// 1111 -> ESC
	}else current_comm_type = NO_COMM;
	return current_comm_type;
}