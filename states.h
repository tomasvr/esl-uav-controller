/*
 *
 *	states.h - header file for state logic
 *
 *	all logic regarding state transitions should go in this module
 *
 */

#ifndef STATES_H__
#define STATES_H__

#include <stdbool.h>
#include <stdio.h>

typedef enum {
	NO_WHERE,
	SAFE_ST, 
	PANIC_ST,
	MANUAL_ST,
	CALIBRATION_ST,
	YAWCONTROL_ST,
	FULLCONTROL_ST
} STATE_t ;

/*
 * @brief      Outputs new state based on input state 
 * 	
 * @param      caller           The caller (either TERM or FCB)
 * @param[in]  g_current_state  The current state of the caller
 * @param[in]  g_dest_state     The destination state of the caller
 * @param[in]  ESC              Indicates whethre ESC was pressed (only for TERM)
 *
 * @return     The new state that the caller should switch to
 */
STATE_t mode_sw_action(char caller[], STATE_t g_current_state, STATE_t g_dest_state, bool ESC);

#endif