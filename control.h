/*

	control.h - header file for (motor) control

*/

#ifndef CONTROL_H__
#define CONTROL_H__

#define DEBUG_LED
#define STEP_SIZE 10
#define UPPER_LIMIT 1000

#define CONTROLLER_P_STEP_SIZE 1
#define CONTROLLER__P_UPPER_LIMIT 50
#define CONTROLLER_P_LOWER_LIMIT 1 

// the states that a motor has
typedef enum {
		MOTOR_UP,
		MOTOR_REMAIN,
		MOTOR_DOWN,
		MOTOR_LEVEL_0,
		MOTOR_LEVEL_1,
		MOTOR_LEVEL_2,
		MOTOR_LEVEL_3,
		MOTOR_LEVEL_4,
		MOTOR_LEVEL_5,
		MOTOR_LEVEL_6,
		MOTOR_LEVEL_7,
		MOTOR_LEVEL_8,
		MOTOR_LEVEL_9,
		MOTOR_LEVEL_10
} MOTOR_CTRL;

extern MOTOR_CTRL g_current_m0_state;
extern MOTOR_CTRL g_current_m1_state;
extern MOTOR_CTRL g_current_m2_state;
extern MOTOR_CTRL g_current_m3_state;

void ctrl_action();

typedef struct // TODO: should use float?
{
	int16_t set_point;
	int16_t sensor_value;
	int16_t err;
	uint8_t kp, ki;
	int16_t integral;
	int16_t output;

} CONTROLLER;

// void yaw_control_speed_calculate(CONTROLLER*, int16_t, int);
// void yaw_control_init(CONTROLLER*);
// void increase_p_value(CONTROLLER*);
// void decrease_p_value(CONTROLLER*);

void controller_init(CONTROLLER *controller);
int16_t controller_calc(CONTROLLER *controller, int16_t set_point, int16_t sensor_value);
void increase_p_value(CONTROLLER *controller);
void decrease_p_value(CONTROLLER *controller);

#endif // CONTROL_H__
