/* 

	states.c - contains implementation for state logic

*/

#include "states.h"

STATE_t mode_sw_action(char caller[], STATE_t g_current_state, STATE_t g_dest_state, bool ESC){
	if (ESC) {
		g_current_state = SAFE_ST;
	}
	else if (g_current_state == SAFE_ST){ 
		if (g_dest_state == PANIC_ST) { // safe mode cannot go to panic mode
			printf("%s: Can not switch to PANIC MODE while in SAFE MODE!\n", caller);
		} else {
			g_current_state = g_dest_state;
		}
	} 
	else if (g_current_state == PANIC_ST){
		if (g_dest_state != SAFE_ST){ // panic mode always goes to safe mode
			printf("%s: can not switch to other modes else than SAFE MODE while in PANIC MODE.\n", caller);
		} else {
			g_current_state = g_dest_state;
		}
	}
	else if (g_current_state != SAFE_ST && g_current_state != PANIC_ST){
		if (g_dest_state == PANIC_ST || g_dest_state == g_current_state){
			g_current_state = g_dest_state;
		} else {
			printf("%s: Can not directly switch to other modes else than PANIC MODE in the current mode.\n", caller);
		}
	}
	return g_current_state; 
}