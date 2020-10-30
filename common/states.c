/* 

	states.c - contains implementation for state logic

*/

#include "states.h"

/**
 * @brief      Checks whether all JS values are 0 (or close enough)
 *
 * @return     True if all are 0 (or close enough), false otherwise
 * 
 * @author     T. van Rietbergen
 */
int js_axis_values_zeroed() {
	int all_values_are_zero = 1;
	for (int i = 0; i < NUMBER_OF_JS_AXIS; i++) {
		if(joystick_axis_stored_values[i] > 3) { // due to small deviations, sometimes JS value is slightly higher than 0 even in netural mode, hence '3'
			all_values_are_zero = 0;
		}
		printf(" %d: %d ", i, joystick_axis_stored_values[i]);
	}
	return all_values_are_zero;
}


STATE_t mode_sw_action(char caller[], STATE_t state, STATE_t to_state) {

	if(is_allowed[state][to_state])
	{
		if(to_state == PANIC_ST) {
			return to_state;
		}
		else if(js_axis_values_zeroed() == 0)
		{
			printf("%s: MODE_SWITCH_ERROR: cannot switch mode when js not at neutral position \n", caller);
			return state;
		}
		else return to_state;
	}
	else
	{
		printf("%s: MODE_SWITCH_ERROR: cannot switch from state %d to state %d.\n", caller, state, to_state);
		return state;
	}

}