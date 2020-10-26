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

// uint32_t timestamp = get_time_us();
// struct info_log info_logging = {timestamp, bat_volt};

// uint8_t address_init = 0x000000;
// uint8_t address_counter = 0;
// void data_logging()
// {
// struct sensor_log sensor_logging = {phi, theta, psi, sp , sq, sr};
// if(flash_write_bytes(address_init + address_counter, (uint8_t*) &sensor_logging, 4))printf("+");
// }

// void data_logging()
// {
// 	uint8_t counter = 0;
// 	uint8_t restore_counter = 0;
// 	uint8_t time_store[4] = {0};
// 	uint32_t time_restore[1] = {0};// uint8_t time_counter = 0;// uint8_t time_restore_counter = 0;  

// 	uint8_t  battery_store[2] = {0};
// 	uint16_t battery_restore[1] = {0};// uint8_t battery_counter = 0;// uint8_t battery_restore_counter = 0;
	
// 	uint8_t  phi_store[2] = {0};
// 	uint16_t phi_restore[1] = {0};// uint8_t phi_counter = 0;// uint8_t phi_restore_counter = 0;
// 	int16_t phi_d = 0;

// 	uint8_t  theta_store[2] = {0};
// 	uint16_t theta_restore[1] = {0};// uint8_t theta_counter = 0;// uint8_t theta_restore_counter = 0;
// 	int16_t theta_d = 0;

// 	uint8_t  psi_store[2] = {0};
// 	uint16_t psi_restore[1] = {0};// uint8_t psi_counter = 0;// uint8_t psi_restore_counter = 0;
// 	int16_t psi_d = 0;

// 	uint8_t pressure_store[4] = {0};
// 	uint32_t pressure_restore[1] = {0};// uint8_t pressure_counter = 0;// uint8_t pressure_restore_counter = 0;   

// 	//time
// 	time_store[counter] = (get_time_us() >> 24); time_store[counter+1] = (get_time_us() >> 16);
// 	time_store[counter+2] = (get_time_us() >> 8); time_store[counter+3] = get_time_us();

// 	if(flash_write_bytes(address_init + address_counter, &time_store[address_counter], 4))printf(" |");
// 	if(flash_read_bytes(address_init + address_counter, &time_store[address_counter], 4))printf("|");

// 	time_restore[restore_counter] = (time_store[counter] << 24) | (time_store[counter+1] << 16) 
// 	| (time_store[counter+2] << 8) | (time_store[counter+3]);
// 	printf("|%10ld ", time_restore[restore_counter]);
// 	// time_restore_counter++; time_counter = time_counter + 4;
// 	address_counter = address_counter + 4;

// 	//Battery
// 	battery_store[counter] = bat_volt >> 8; battery_store[counter+1] = bat_volt;

// 	if(flash_write_bytes(address_init + address_counter, &battery_store[address_counter], 2))printf(" |");
// 	if(flash_read_bytes(address_init + address_counter, &battery_store[address_counter], 2))printf("|");

// 	battery_restore[restore_counter] = battery_store[counter] << 8 | battery_store[counter+1];
// 	printf("|%4d ", battery_restore[restore_counter]);
// 	// battery_restore_counter++; battery_counter = battery_counter + 2;
// 	address_counter = address_counter + 2;

// 	//phi
// 	phi_store[counter] = phi >> 8; phi_store[counter+1] = phi;

// 	if(flash_write_bytes(address_init + address_counter, &phi_store[address_counter], 2))printf(" |");
// 	if(flash_read_bytes(address_init + address_counter, &phi_store[address_counter], 2))printf("|");

// 	phi_restore[restore_counter] = phi_store[counter] << 8 | phi_store[counter + 1];
// 	if(phi_restore[restore_counter] > 32768) {
// 		phi_d = phi_restore[restore_counter] - 65536;
// 	}
// 	else
// 		phi_d = phi_restore[restore_counter];

// 	printf("|%6d ", phi_d);
// 	// phi_restore_counter++; phi_counter = phi_counter + 2;
// 	address_counter = address_counter + 2;

// 	//theta
// 	theta_store[counter] = theta >> 8; theta_store[counter+1] = theta;

// 	if(flash_write_bytes(address_init + address_counter, &theta_store[address_counter], 2))printf(" |");
// 	if(flash_read_bytes(address_init + address_counter, &theta_store[address_counter], 2))printf("|");

// 	theta_restore[restore_counter] = theta_store[counter] << 8 | theta_store[counter + 1];
// 	if(theta_restore[restore_counter] > 32768) {
// 		theta_d = theta_restore[restore_counter] - 65536;
// 	}
// 	else
// 		theta_d = theta_restore[restore_counter];

// 	printf("|%6d ", theta_d);
// 	address_counter = address_counter + 2;

// 	//psi
// 	psi_store[counter] = psi >> 8; psi_store[counter+1] = psi;

// 	if(flash_write_bytes(address_init + address_counter, &psi_store[address_counter], 2))printf(" |");
// 	if(flash_read_bytes(address_init + address_counter, &psi_store[address_counter], 2))printf("|");

// 	psi_restore[restore_counter] = psi_store[counter] << 8 | psi_store[counter + 1];
// 	if(psi_restore[restore_counter] > 32768) {
// 		psi_d = psi_restore[restore_counter] - 65536;
// 	}
// 	else
// 		psi_d = psi_restore[restore_counter];

// 	printf("|%6d ", psi_d);
// 	address_counter = address_counter + 2;

// 	//pressure
// 	pressure_store[counter] = (pressure >> 24);pressure_store[counter+1] = (pressure >> 16);
// 	pressure_store[counter+2] = (pressure >> 8);pressure_store[counter+3] = pressure;

// 	if(flash_write_bytes(address_init + address_counter, &pressure_store[address_counter], 4))printf(" |");
// 	if(flash_read_bytes(address_init + address_counter, &pressure_store[address_counter], 4))printf("|");

// 	pressure_restore[restore_counter] = (pressure_store[counter] << 24) | (pressure_store[counter+1] << 16) 
// 	| (pressure_store[counter+2] << 8) | (pressure_store[counter+3]);
// 	printf("| %6ld\n", pressure_restore[restore_counter]);
// 	//pressure_restore_counter++;pressure_counter = pressure_counter + 4;
// 	address_counter = address_counter + 4;
// }