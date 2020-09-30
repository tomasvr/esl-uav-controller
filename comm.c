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


int check_mode_sync (uint8_t state, STATE_t g_current_state){
	int mode_synced = 0; 

	if (state == 0x00){					// 0000 -> SAFE_ST
		STATE_t tstate = SAFE_ST;
		if (g_current_state == tstate) mode_synced = 1;
	}
	if (state == 0x01){					// 0001 -> MANUAL_ST
		STATE_t tstate = MANUAL_ST;
		if (g_current_state == tstate) mode_synced = 1;
	}
	if (state == 0x02){					// 0010 -> CALIBRATION_ST
		STATE_t tstate = CALIBRATION_ST;
		if (g_current_state == tstate) mode_synced = 1;
	}
	if (state == 0x03){					// 0011 -> YAWCONTROL_ST
		STATE_t tstate = YAWCONTROL_ST;
		if (g_current_state == tstate) mode_synced = 1;
	}
	if (state == 0x04){					// 0100 -> FULLCONTROL_ST
		STATE_t tstate = FULLCONTROL_ST;
		if (g_current_state == tstate) mode_synced = 1;
	}
	if (state == 0x08){					// 0000 -> SAFE_ST
		STATE_t tstate = PANIC_ST;
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

STATE_t find_dest_state(uint8_t messg){
	STATE_t dest_state = NO_WHERE;
	if (messg == 0x00) dest_state = SAFE_ST;
	else if (messg == 0x80) dest_state = PANIC_ST;
	else if (messg == 0x10) dest_state = MANUAL_ST;
	else if (messg == 0x20) dest_state = CALIBRATION_ST;
	else if (messg == 0x30) dest_state = YAWCONTROL_ST;
	else if (messg == 0x40) dest_state = FULLCONTROL_ST;
	else dest_state = NO_WHERE;
	return dest_state;
}
