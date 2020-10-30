#include "logging.h"
#include "in4073.h"



uint8_t address_init = START_ADDRESS;
uint8_t address_current = START_ADDRESS;
uint8_t address_readout = START_ADDRESS;

/* 
 * Implementation for logging facility on FCB
 * Flash logging address limit judgement
 * Xinyun Xu
*/
bool flash_stuff()
{
	if(address_current < END_ADDRESS)
		return true;
	else
		return false;
}

/* 
 * Flash readout address limit judgement
 * Xinyun Xu
*/
bool flash_read()
{
	if(address_readout < END_ADDRESS) 
		return true;
	else 
		return false;
}

/* 
 * Implementation for logging facility on FCB
 * Data logging into flash
 * Xinyun Xu
*/
void data_logging()
{
	uint32_t timestamp = get_time_us();
	struct tag_log tag_logging = {timestamp, bat_volt};//32 16
	if(flash_write_bytes(address_current, (uint8_t*) &tag_logging, 12))
	address_current += 12;

	struct js_log js_logging = {roll, pitch, yaw, lift};
	if(flash_write_bytes(address_current, (uint8_t*) &js_logging, 4))
	address_current += 4;

	struct sensor_log sensor_logging = {phi, theta, psi, sp , sq, sr};//int16_t * 6
	if(flash_write_bytes(address_current, (uint8_t*) &sensor_logging, 24))
	address_current += 24;


	struct motor_log motor_logging = {ae[0], ae[1], ae[2], ae[3]};
	if(flash_write_bytes(address_current, (uint8_t*) &motor_logging, 16))
	address_current += 4;
}

/* 
 * Readout logging data from flash
 * Xinyun Xu
*/
void data_readout()
{
	uint32_t timestamp = get_time_us();
	struct tag_log tag_logging = {timestamp, bat_volt};//32 16
	if(flash_read_bytes(address_readout, (uint8_t*) &tag_logging, 12))
	printf("log: %10ld %3d | ", tag_logging.time, tag_logging.battery);
	address_readout += 12;

	struct js_log js_logging = {roll, pitch, yaw, lift};
	if(flash_read_bytes(address_readout, (uint8_t*) &js_logging, 4))
	printf("%3d %3d %3d %3d  | ", js_logging.roll_log, js_logging.pitch_log, js_logging.yaw_log, js_logging.lift_log);
	address_readout += 4;

	struct sensor_log sensor_logging = {phi, theta, psi, sp , sq, sr};//int16_t * 6
	if(flash_read_bytes(address_readout, (uint8_t*) &sensor_logging, 24))
		printf("%3d %3d %3d | %3d %3d %3d | ", sensor_logging.roll, sensor_logging.pitch, sensor_logging.yaw, 
	sensor_logging.gyro_x , sensor_logging.gyro_y, sensor_logging.gyro_z); 
	address_readout += 24;

	struct motor_log motor_logging = {ae[0], ae[1], ae[2], ae[3]};
	if(flash_read_bytes(address_readout, (uint8_t*) &motor_logging, 16))
	printf("%3d %3d %3d %3d\n", motor_logging.motor_0, motor_logging.motor_1, motor_logging.motor_2, motor_logging.motor_3);
	address_readout += 4;
	

	
}

/* 
 * Logging Implementation
 * Xinyun Xu
*/
void logging(){
	if(flash_chip_erase() && flash_read()){
		data_logging();
	}
}

/* 
 * Readout Implementation
 * Xinyun Xu
*/
void readout(){
	if(flash_read()){
	data_readout();
	}
}
