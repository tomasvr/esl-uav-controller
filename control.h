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
	uint8_t kp, ki;
	int16_t integral;
	int16_t output;

} CONTROLLER;

// calibraiton
extern bool DMP;
extern bool calib_done;
extern uint8_t calib_counter;
// int16_t sensor_calib = 0:
extern int16_t sensor_sum;
extern int32_t angle_calib[3];
extern int32_t gyro_calib[3];
extern int32_t acce_calib[3];
extern int16_t phi_calib, theta_calib, psi_calib;
extern int16_t sp_calib, sq_calib, sr_calib;
extern int16_t sax_calib, say_calib, saz_calib;
void sensor_calc(uint8_t);
void sensor_calib(void);
// void offset_remove(void);

void ctrl_action();


// controller
#define b 1 
#define d 1
extern int16_t yaw_set_point;
extern int16_t roll_set_point;
extern int16_t pitch_set_point;
extern int16_t Z_needed;
extern int16_t L_needed;
extern int16_t M_needed;
extern int16_t N_needed;
void controller_init(CONTROLLER *controller);
void increase_p_value(CONTROLLER *controller);
void decrease_p_value(CONTROLLER *controller);
int16_t yaw_control_calc(CONTROLLER *yaw_control, int16_t yaw_set_point, int16_t sr);
void actuate(int16_t Z_needed, int16_t L_needed, int16_t M_needed, int16_t N_needed);

void increase_motor_speed(int16_t *ae, uint8_t motor);
void decrease_motor_speed(int16_t *ae, uint8_t motor);
void keyboard_ctrl_action();

#endif // CONTROL_H__

