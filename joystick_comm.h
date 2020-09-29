#ifndef JOYSTICK_COMM_H__
#define JOYSTICK__COMM_H__

#include <inttypes.h>
#include "states.h"
#include <stdbool.h> 


#define THRESHOLD_READ 20000

uint32_t messg_encode_send_js(int *axis, int *button, STATE_t g_current_state);

#endif