/*

	control.h - header file for (motor) control

*/

#ifndef CONTROL_H__
#define CONTROL_H__

#define DEBUG_LED
#define STEP_SIZE 10
#define UPPER_LIMIT 1000

#define YAW_P_STEP_SIZE 1
#define YAW_P_UPPER_LIMIT 50
#define YAW_P_LOWER_LIMIT 1 

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

void sensor_calcu(uint8_t);
void sensor_caib(void);
void offset_remove(void);

typedef struct {
	uint8_t P;
	uint8_t I;
	uint8_t D;
	int16_t Err;
	int16_t Pre_Err;
	int16_t Integral;
	int16_t Deriv;
	int16_t Output;
} CONTROL_T;

#define SPEED_REF 230
//test points before js
#define TARGET_X 0
#define TARGET_Y 0
#define TARGET_Z 0

#define PITCH_THRE 1500
#define ROLL_THRE 1500

int16_t Pitch_Output;
int16_t Roll_Output;
int16_t Yaw_Target, Yaw_Measure;
int16_t Yaw_Err;
int16_t Yaw_Output;

void control_init(CONTROL_T*);
void yaw_control_err_calcu(CONTROL_T*, int16_t, int);
void control_err_calcu(CONTROL_T*, int16_t, int);
void yaw_control(void);
void control(void);
void yaw_control_motor_output(void);
void control_motor_output(void);
void speed_limit(void);

void increase_p_value(CONTROL_T*);
void decrease_p_value(CONTROL_T*);

#endif // CONTROL_H__
