/*
	control.h - header file for (motor) control
*/

#ifndef CONTROL_H__
#define CONTROL_H__

// the states that a motor has
typedef enum {
		MOTOR_REMAIN,
		MOTOR_UP,
		MOTOR_DOWN,
		MOTOR_UNKNOWN
} MOTOR_CTRL;

typedef enum {
	RATE,
	ANGLE
} P_PARAM;

extern MOTOR_CTRL g_current_m0_state;
extern MOTOR_CTRL g_current_m1_state;
extern MOTOR_CTRL g_current_m2_state;
extern MOTOR_CTRL g_current_m3_state;

typedef struct
{
	int16_t set_point;
	int16_t sensor_value;
	int16_t err;
	uint8_t kp_angle, kp_rate, ki;
	int16_t integral;
	int16_t output;

} CONTROLLER;

void adjust_parameter_value(uint8_t message_byte);
void change_p_value(CONTROLLER *controller, P_PARAM param, bool increase);
void change_shift_value(bool increase);

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
void increase_shift_value();
void decrease_shift_value();
void intialize_parameters();
#endif // CONTROL_H__