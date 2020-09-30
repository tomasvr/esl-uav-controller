/*

	control.h - header file for (motor) control

*/

#ifndef CONTROL_H__
#define CONTROL_H__

#define DEBUG_LED
#define STEP_SIZE 10
#define UPPER_LIMIT 1000

// the states that a motor has
typedef enum {
		MOTOR_UP,
		MOTOR_REMAIN,
		MOTOR_DOWN
} MOTOR_CTRL;

extern MOTOR_CTRL g_current_m0_state;
extern MOTOR_CTRL g_current_m1_state;
extern MOTOR_CTRL g_current_m2_state;
extern MOTOR_CTRL g_current_m3_state;

void ctrl_action();

typedef struct {
	int16_t err;
	uint8_t kp;
	uint8_t ki;
	uint16_t integral;
	int16_t speed_comm;
	int16_t speed_diff;
	int16_t set_yaw_rate;
	int16_t actual_yaw_rate;
	int16_t actual_speed_plus;
	int16_t actual_speed_minus;
} YAW_CONTROL_T;

void yaw_control_speed_calculate();
void yaw_control_init();

#endif // CONTROL_H__
