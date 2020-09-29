/*

	states.h - header file for state logic

	all logic regarding state transitions should go in this module

*/

#ifndef STATES_H__
#define STATES_H__

#include <stdbool.h>
#include <stdio.h>

typedef enum {
		SAFE_ST, 
		PANIC_ST,
		MANUAL_ST,
		CALIBRATION_ST,
		YAWCONTROL_ST,
		FULLCONTROL_ST,
		NO_WHERE
	} STATE_t ;

STATE_t mode_sw_action(char caller[], STATE_t g_current_state, STATE_t g_dest_state, bool ESC);

#endif