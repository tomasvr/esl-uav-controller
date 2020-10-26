/*
	control.h - header file for (motor) control
*/

#ifndef CONTROL_H__
#define CONTROL_H__

#define DEBUG_LED
#define STEP_SIZE 10
#define UPPER_LIMIT 1000

#define CONTROLLER_P_STEP_SIZE 1
#define CONTROLLER_P_UPPER_LIMIT 50
#define CONTROLLER_P_LOWER_LIMIT 1 

#define MAX_ALLOWED_SPEED 1000
#define MIN_ALLOWED_SPEED 0
#define MAX_ALLOWED_DIFF_MOTOR 50

// the states that a motor has
typedef enum {
		MOTOR_REMAIN,
		MOTOR_UP,
		MOTOR_DOWN,
		MOTOR_UNKNOWN
} MOTOR_CTRL;

extern MOTOR_CTRL g_current_m0_state;
extern MOTOR_CTRL g_current_m1_state;
extern MOTOR_CTRL g_current_m2_state;
extern MOTOR_CTRL g_current_m3_state;

typedef enum {
		YAW_KP_UP,
		YAW_KP_DOWN,
} TRIM_CTRL;

extern TRIM_CTRL YAW_KP_STATE;

void keyboard_yaw_ctrl_kp(void);

typedef struct // TODO: should use float?
{
	int16_t set_point;
	int16_t sensor_value;
	int16_t err;
	uint8_t kp_angle, kp_rate, ki;
	int16_t integral;
	int16_t output;

} CONTROLLER;

void increase_motor_speed(uint8_t motor);
void decrease_motor_speed(uint8_t motor);
void keyboard_ctrl_action();

// calibration
extern bool DMP;
extern bool calib_done;
extern uint8_t calib_counter;
extern int16_t sensor_sum;
extern int32_t angle_calib[3];
extern int32_t gyro_calib[3];
extern int32_t acce_calib[3];
extern int16_t phi_calib, theta_calib, psi_calib;
extern int16_t sp_calib, sq_calib, sr_calib;
extern int16_t sax_calib, say_calib, saz_calib;
void sensor_calc(uint8_t);
void sensor_calib(void);

// controller
#define b 1 
#define d 1
void controller_init(CONTROLLER *controller);
void increase_p_value(CONTROLLER *controller);
void decrease_p_value(CONTROLLER *controller);

#endif // CONTROL_H__