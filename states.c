/* 

	states.c - contains implementation for state logic

*/

#include "states.h"


int js_axis_values_zeroed() {
	int all_values_are_zero = 1;
	for (int i = 0; i < NUMBER_OF_JS_AXIS; i++) {
		if(joystick_axis_stored_values[i] > 1) { //IMPORTANT: AT STARTUP JS AXIS IS RANDOMLY 1 WHILE IN NEUTRAL, SO: > 1
			all_values_are_zero = 0;
		}
		printf(" %d: %ld ", i, joystick_axis_stored_values[i]);
	}
	return all_values_are_zero;
}


STATE_t mode_sw_action(char caller[], STATE_t state, STATE_t g_dest_state, bool ESC){
	if (ESC) {
		state = SAFE_ST;
	}
	else if (state == SAFE_ST){ 
		if (g_dest_state == PANIC_ST) { // safe mode cannot go to panic mode
			printf("%s: MODE_SWITCH_ERROR: Cannot switch to PANIC MODE while in SAFE MODE!\n", caller);
		} 
		else if (js_axis_values_zeroed() == 0) {
		printf("%s: MODE_SWITCH_ERROR: please but joystick in neutral position before leaving SAFE MODE\n", caller);

		}
		else {
			state = g_dest_state;
		}
	} 
	else if (state == PANIC_ST){
		if (g_dest_state != SAFE_ST){ // panic mode always goes to safe mode
			printf("%s: MODE_SWITCH_ERROR: Cannot switch to other modes else than SAFE MODE while in PANIC MODE.\n", caller);
		} else {
			state = g_dest_state;
		}
	}

	else if (state != SAFE_ST && state != PANIC_ST){ //TODO fix this logic
		if (g_dest_state == PANIC_ST || g_dest_state == state || g_dest_state == CALIBRATION_ST || g_dest_state == YAWCONTROL_ST){
			state = g_dest_state;
		} else {
			printf("%s: MODE_SWITCH_ERROR: Cannot directly switch to other modes else than PANIC MODE in the current mode.\n", caller);
		}
	}
	return state; 
}