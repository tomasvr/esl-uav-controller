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
#include <stdint.h>

#define NUMBER_OF_JS_AXIS 4 // there are 4 axis (pitch, roll, yaw, throttle)

// Used to check status of joystick axis before lift off
uint8_t joystick_axis_stored_values[NUMBER_OF_JS_AXIS];

typedef enum {
	SAFE_ST, 
	PANIC_ST,
	MANUAL_ST,
	CALIBRATION_ST,
	YAWCONTROL_ST,
	FULLCONTROL_ST,
	UNKNOWN_ST
} STATE_t ;

// is_allowed[state][to_state] == 1 -> is allowed to switch from 'state' to 'to_state', 0 -> not allowed
static const uint8_t is_allowed[7][7] = {

/* 	 from\to  safe    panic   calib   manual  yaw_c   full_c  unknown */
  /* safe   */ {0, 	    0,      1,      1,      1,      1,      0}, 
  /* panic  */ {0, 	    0,      0,      0,      0,      0,      0}, 
  /* manual */ {0, 	    1,      0, 	    0,      0,      0,      0}, 
  /* calib  */ {0, 	    1,      0, 	    0,      0,      0,      0}, 
  /* yaw c  */ {0, 	    1,      0, 	    0,      0,      0,      0}, 
  /* full c */ {0, 	    1,      0, 	    0,      0,      0,      0}, 
  /* unknown*/ {0, 	    0,      0,      0,      0,      0,      0}, 
};

STATE_t mode_sw_action(char caller[], STATE_t g_current_state, STATE_t g_dest_state);

#endif