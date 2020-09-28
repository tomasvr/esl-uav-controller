/*------------------------------------------------------------------
 *  control.c -- here you can implement your control algorithm
 *		 and any motor clipping or whatever else
 *		 remember! motor input =  0-1000 : 125-250 us (OneShot125)
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"

void update_motors(void)
{					
	motor[0] = ae[0];
	motor[1] = ae[1];
	motor[2] = ae[2];
	motor[3] = ae[3];
}

uint8_t calibration_counter = 0;
int16_t sensor_calib = 0;
int16_t sensor_calibration(int16_t sensor_ori, uint8_t num)//average
{
	int16_t sensor_temp = 0, sensor_sum = 0;
	sensor_temp = sensor_ori;
	sensor_sum += sensor_temp;
	calibration_counter++;
	if(calibration_counter == 3){
		sensor_calib = sensor_sum / num;
		calibration_counter = 0;
	}
	return sensor_calib;
}

int16_t phi_calib = 0, theta_calib = 0, psi_calib = 0;
void run_filters_and_control()
{
	sensor_calibration(phi, 3); phi_calib = sensor_calib;
	printf("| %6d ", phi_calib);

	sensor_calibration(theta, 3); theta_calib = sensor_calib;
	printf("| %6d ", theta_calib);

	sensor_calibration(psi, 3); psi_calib = sensor_calib;
	printf("| %6d ", psi_calib);
	// fancy stuff here
	// control loops and/or filters

	// ae[0] = xxx, ae[1] = yyy etc etc
	update_motors();
}

