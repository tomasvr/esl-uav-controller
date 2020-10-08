/* 

	states.c - contains implementation for state logic

*/



js_axis_values_zeroed() {
	int all_values_are_zero = 1;
	for (int i; i < NUMBER_OF_JS_AXIS; i++) {
		if(joystick_axis_stored_values[i] > 0) {
			all_values_are_zero = 0;
		}
	}
	return all_values_are_zero;
}

#include "states.h"

STATE_t mode_sw_action(char caller[], STATE_t g_current_state, STATE_t g_dest_state, bool ESC){
	if (ESC) {
		g_current_state = SAFE_ST;
	}
	else if (g_current_state == SAFE_ST){ 
		if (g_dest_state == PANIC_ST) { // safe mode cannot go to panic mode
			printf("%s: MODE_SWITCH_ERROR: Cannot switch to PANIC MODE while in SAFE MODE!\n", caller);
		} 
		else if (js_axis_values_zeroed() == false) {
		printf("%s: MODE_SWITCH_ERROR: please but joystick in neutral position before leaving SAFE MODE\n", caller);

		}
		else {
			g_current_state = g_dest_state;
		}
	} 
	else if (g_current_state == PANIC_ST){
		if (g_dest_state != SAFE_ST){ // panic mode always goes to safe mode
			printf("%s: MODE_SWITCH_ERROR: Cannot switch to other modes else than SAFE MODE while in PANIC MODE.\n", caller);
		} else {
			g_current_state = g_dest_state;
		}
	}

	else if (g_current_state != SAFE_ST && g_current_state != PANIC_ST){ //TODO fix this logic
		if (g_dest_state == PANIC_ST || g_dest_state == g_current_state || g_dest_state == CALIBRATION_ST || g_dest_state == YAWCONTROL_ST){
			g_current_state = g_dest_state;
		} else {
			printf("%s: MODE_SWITCH_ERROR: Cannot directly switch to other modes else than PANIC MODE in the current mode.\n", caller);
		}
	}
	return g_current_state; 
}