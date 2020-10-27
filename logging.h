/*

	logging.h - header file for logging facility

*/

#ifndef logging_H__
#define logging_H__

#include "in4073.h"
/**
 * @brief      Writes specified data to log.
 */
// bool write_to_log(uint8_t data);

#define START_ADDRESS 0x000000
#define END_ADDRESS   0x01FFFF

struct tag_log {
	uint32_t time;
	uint16_t battery;
};//12bytes

struct js_log {
	int8_t roll_log;
	int8_t pitch_log;
	int8_t yaw_log;
	uint8_t lift_log;
};//4bytes

struct sensor_log {
	int16_t roll; //angle
	int16_t pitch;
	int16_t yaw;
	int16_t gyro_x;//rate
	int16_t gyro_y;
	int16_t gyro_z;
};//24bytes

struct motor_log {
	int16_t motor_0;
	int16_t motor_1;
	int16_t motor_2;
	int16_t motor_3;
};//16bytes

void logging(void);
void plot_info(void);

#endif // logging_H__
