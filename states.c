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


STATE_t mode_sw_action(char caller[], STATE_t state, STATE_t to_state) {

	// if (state == to_state) return state; // except for calibration state? because you want to activate calibration

	/* Check if desired switch is invalid */
	// switch(state) {
	// 	case SAFE_ST:
	// 		if (to_state == PANIC_ST) { // safe mode cannot go to panic mode
	// 			printf("%s: MODE_SWITCH_ERROR: Cannot switch to PANIC MODE while in SAFE MODE!\n", caller);
	// 			return state;
	// 		} 
	// 		if (js_axis_values_zeroed() == 0) {
	// 			printf("%s: MODE_SWITCH_ERROR: please put joystick in neutral position before leaving SAFE MODE\n", caller);
	// 			return state;
	// 		}	
	// 		break;
	// 	case PANIC_ST:
	// 			printf("%s: MODE_SWITCH_ERROR: cannot manually switch to any state from panic mode!\n", caller);
	// 			return state;			
	// 		break;
	// 	case MANUAL_ST:
	// 	case CALIBRATION_ST:
	// 	case YAWCONTROL_ST:
	// 	case FULLCONTROL_ST:
	// 		if (to_state != PANIC_ST && to_state != CALIBRATION_ST){
	// 			printf("%s: MODE_SWITCH_ERROR: cannot switch from state %d to state %d.\n", caller, state, to_state);
	// 			return state;
	// 		}
	// 		break;
	// 	default:
	// 		printf("%s: ERROR mode_sw_action - unknown state: %d\n", caller, state);
	// 		break;
	// }
	// /* Switch is okay */
	// return to_state; 
	

	if(is_allowed[state][to_state])
	{
		return to_state;
	}
	else
	{
		printf("%s: MODE_SWITCH_ERROR: cannot switch from state %d to state %d.\n", caller, state, to_state);
		return state;
	}

}