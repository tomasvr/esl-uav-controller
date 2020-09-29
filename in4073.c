/*------------------------------------------------------------------
 *  in4073.c -- test QR engines and sensors
 *
 *  reads ae[0-3] uart rx queue
 *  (q,w,e,r increment, a,s,d,f decrement)
 *
 *  prints timestamp, ae[0-3], sensors to uart tx queue
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  June 2016
 *------------------------------------------------------------------
 */


/* --- PRINTF_BYTE_TO_BINARY macro's --- */
#define PRINTF_BINARY_PATTERN_INT8 "%c%c%c%c%c%c%c%c"
#define PRINTF_BYTE_TO_BINARY_INT8(i)    \
    (((i) & 0x80ll) ? '1' : '0'), \
    (((i) & 0x40ll) ? '1' : '0'), \
    (((i) & 0x20ll) ? '1' : '0'), \
    (((i) & 0x10ll) ? '1' : '0'), \
    (((i) & 0x08ll) ? '1' : '0'), \
    (((i) & 0x04ll) ? '1' : '0'), \
    (((i) & 0x02ll) ? '1' : '0'), \
    (((i) & 0x01ll) ? '1' : '0')

#define PRINTF_BINARY_PATTERN_INT16 \
    PRINTF_BINARY_PATTERN_INT8              PRINTF_BINARY_PATTERN_INT8
#define PRINTF_BYTE_TO_BINARY_INT16(i) \
    PRINTF_BYTE_TO_BINARY_INT8((i) >> 8),   PRINTF_BYTE_TO_BINARY_INT8(i)
#define PRINTF_BINARY_PATTERN_INT32 \
    PRINTF_BINARY_PATTERN_INT16             PRINTF_BINARY_PATTERN_INT16
#define PRINTF_BYTE_TO_BINARY_INT32(i) \
    PRINTF_BYTE_TO_BINARY_INT16((i) >> 16), PRINTF_BYTE_TO_BINARY_INT16(i)
#define PRINTF_BINARY_PATTERN_INT64    \
    PRINTF_BINARY_PATTERN_INT32             PRINTF_BINARY_PATTERN_INT32
#define PRINTF_BYTE_TO_BINARY_INT64(i) \
    PRINTF_BYTE_TO_BINARY_INT32((i) >> 32), PRINTF_BYTE_TO_BINARY_INT32(i)
/* --- end macros --- */
//printf("The value received is: "PRINTF_BINARY_PATTERN_INT8 "\n",PRINTF_BYTE_TO_BINARY_INT8(c));

#define STEP_SIZE 1
#define UPPER_LIMIT 330

#include "in4073.h"
#include <assert.h>

// each packet includes 4 fragments
int FRAG_COUNT = 0;

enum STATE g_current_state = SAFE_ST;
enum STATE g_dest_state = NO_WHERE;
enum COMM_TYPE g_current_comm_type = NO_COMM;
enum M0_CRTL g_current_m0_state = M0_REMAIN;
enum M1_CRTL g_current_m1_state = M1_REMAIN;
enum M2_CRTL g_current_m2_state = M2_REMAIN;
enum M3_CRTL g_current_m3_state = M3_REMAIN;

int find_motor_state(uint8_t messg){
	uint8_t m_ctrl_1 = messg & 0xf0; 		
	uint8_t m_ctrl_2 = messg & 0x0f;
	int result = 0;
	// find motor state of motor 0 
	if (m_ctrl_1>>6 == 0) {				// 0000 -> M0
		if ((m_ctrl_1 >> 5)&1 && (m_ctrl_1 >> 4)&1) g_current_m0_state = M0_UP;
		if (((m_ctrl_1 >> 5)&1) == 1 && ((m_ctrl_1 >> 4)&1) == 0) g_current_m0_state = M0_DOWN;
		if (((m_ctrl_1 >> 5)&1) == 0) g_current_m0_state = M0_REMAIN;
		result = 1;
	}
	if (m_ctrl_1>>6 == 2) {				// 0010 -> M2
		if ((m_ctrl_1 >> 5)&1 && (m_ctrl_1 >> 4)&1) g_current_m2_state = M2_UP;
		if (((m_ctrl_1 >> 5)&1) == 1 && ((m_ctrl_1 >> 4)&1) == 0) g_current_m2_state = M2_DOWN;
		if (((m_ctrl_1 >> 5)&1) == 0) g_current_m2_state = M2_REMAIN;
		result = 1;
	}
	if (m_ctrl_2>>2 == 1) {				// 0001 -> M1
		if ((m_ctrl_2 >> 1)&1 && (m_ctrl_2>>0)&1) g_current_m1_state = M1_UP;
		if (((m_ctrl_2 >> 1)&1) == 1 && ((m_ctrl_2 >> 0)&1) == 0) g_current_m1_state = M1_DOWN;
		if (((m_ctrl_2 >> 1)&1) == 0) g_current_m1_state = M1_REMAIN;
		result = 1;
	}
	if (m_ctrl_2>>2 == 3) {				// 0011 -> M3
		if ((m_ctrl_2 >> 1)&1 && (m_ctrl_2>>0)&1) g_current_m3_state = M3_UP;
		if (((m_ctrl_2 >> 1)&1) == 1 && ((m_ctrl_2 >> 0)&1) == 0) g_current_m3_state = M3_DOWN;
		if (((m_ctrl_2 >> 1)&1) == 0) g_current_m3_state = M3_REMAIN;
		result = 1;
	} 
	return result;
}

/*------------------------------------------------------------------
 * messg_decode -- decode messages
 *------------------------------------------------------------------
 */
void messg_decode(uint8_t messg){
	
	// printf("The %d byte is: \n", 4-FRAG_COUNT);			// print out each fragment QR receives
	// printf("   		 "PRINTF_BINARY_PATTERN_INT8 "\n",PRINTF_BYTE_TO_BINARY_INT8(messg));
	/*--------------------------------------------------------------
	 * decode the first frag, two field in this byte: 
	 * 		----------------------
	 * 		| C C C C || M M M M |                  
	 * 		----------------------
	 * first 4 bits -> command type { CTRL_COMM , MODE_SW_COMM , BAT_INFO , SYS_LOG , NO_COMM }
	 * last 4 bits 	-> mode/state { SAFE_ST , PANIC_ST , MANUAL_ST , CALIBRATION_ST , YAWCONTROL_ST , FULLCONTROL_ST }
	 *--------------------------------------------------------------
	 */
	if (FRAG_COUNT == 3){
		uint8_t comm_type = messg & 0xf0;
		uint8_t state = messg & 0x0f;

		g_current_comm_type = find_comm_type(comm_type);
		//assert( == 1 && "QR: No such command found.");

		int result = check_mode_sync(state, g_current_state);
		// assert(result == 1 && "QR: The mode in QR is not sync with PC or the action is not allowed in current mode!"); // might have to enter the panic mode?????????????????
	}

	/*--------------------------------------------------------------
	 * decode the second and the thrid frag, fields in these byte are depend on the  
	 * command type received in the previous byte.
	 *--------------------------------------------------------------
	 */
	if (FRAG_COUNT == 2 || FRAG_COUNT == 1){
		/*--------------------------------------------------------------
		 * if the command is CTRL_TYPE, two field in this byte: 
		 * 		  ----------------------		 ----------------------
		 * FRAG_2 | 0 0 0 0 || 1 1 1 1 |  FRAG_1 | 2 2 2 2 || 3 3 3 3 |                
		 * 		  ----------------------         ----------------------
		 * FRAG_2:
		 * first 4 bits -> motor 0 state { M0_UP, M0_REMAIN , M0_DOWN }
		 * last 4 bits 	-> motor 1 state { M1_UP, M1_REMAIN , M1_DOWN }
		 * FRAG_1:
		 * first 4 bits -> motor 2 state { M2_UP, M2_REMAIN , M2_DOWN }
		 * last 4 bits 	-> motor 3 state { M3_UP, M3_REMAIN , M3_DOWN }
		 *--------------------------------------------------------------
		 */
		 		
	 	if (g_current_comm_type == CTRL_COMM){
	 		int result;
	 		result = find_motor_state(messg);
	 		assert(result == 1 && "QR: Fail to find the motor state.");
	 	}
	 	
	 	/*--------------------------------------------------------------
		 * if the command is MODE_SW_TYPE, two field in this byte: 
		 * 		  ----------------------		 ----------------------
		 * FRAG_2 |DEST_STATE||  all 0 |  FRAG_1 |  all 0  ||  all 0  |                
		 * 		  ----------------------         ----------------------
		 * FRAG_2:
		 * first 4 bits -> the destination state { SAFE_ST , PANIC_ST , MANUAL_ST , CALIBRATION_ST , YAWCONTROL_ST , FULLCONTROL_ST }
		 * last 4 bits 	-> default 0
		 * FRAG_1:
		 * first 4 bits -> default 0
		 * last 4 bits 	-> default 0
		 *--------------------------------------------------------------
		 */
	 	if (g_current_comm_type == MODE_SW_COMM && FRAG_COUNT == 2){
	 		g_dest_state = find_dest_state(messg);
	 	}

	}
	return;
}

/*------------------------------------------------------------------
 * process_key -- process command keys
 *------------------------------------------------------------------
 */
void process_key(uint8_t c){	
	if (c == 0x55 && FRAG_COUNT == 0 && g_current_state != PANIC_ST) {
		FRAG_COUNT = 3;
		return;
	}
	while (FRAG_COUNT > 0){
		messg_decode(c);
		FRAG_COUNT--;
		return;
	}
	nrf_gpio_pin_toggle(RED);
	return;
}

void increase_motor_speed(int16_t *ae, uint8_t motor){
	ae[motor] += STEP_SIZE;
	if (ae[motor] > UPPER_LIMIT) ae[motor] = UPPER_LIMIT;
	return;
}

void decrease_motor_speed(int16_t *ae, uint8_t motor){
	ae[motor] -= STEP_SIZE;
	if (ae[motor] < 0) ae[motor] = 0;
	return;
}

void ctrl_action(){
	switch (g_current_m0_state){			//M0
		case M0_UP:
			increase_motor_speed(ae, 0);
			break;
		case M0_REMAIN:
			break;
		case M0_DOWN:
			decrease_motor_speed(ae, 0);
			break;
		default:
			break;
	}
	switch (g_current_m1_state){			//M1
		case M1_UP:
			increase_motor_speed(ae, 1);
			break;
		case M1_REMAIN:
			break;
		case M1_DOWN:
			decrease_motor_speed(ae, 1);
			break;
		default:
			break;
	}
	switch (g_current_m2_state){			//M2
		case M2_UP:
			increase_motor_speed(ae, 2);
			break;
		case M2_REMAIN:
			break;
		case M2_DOWN:
			decrease_motor_speed(ae, 2);
			break;
		default:
			break;
	}
	switch (g_current_m3_state){			//M3
		case M3_UP:
			increase_motor_speed(ae, 3);
			break;
		case M3_REMAIN:
			break;
		case M3_DOWN:
			decrease_motor_speed(ae, 3);
			break;
		default:
			break;
	}
}

void mode_sw_action(){
	if (g_current_state == SAFE_ST){
		if (g_dest_state == PANIC_ST) {
			printf("QR: Can not switch to PANIC MODE while in SAFE MODE!\n");
			return;
		}
		else{ // fix 1 bug?
			g_current_state = g_dest_state;
		}
		return;
	} 
	if (g_current_state == PANIC_ST){
		if (g_dest_state != SAFE_ST){
			printf("QR: an not switch to other modes else than SAFE MODE while in PANIC MODE.\n");
			return;
		}
		return;
	}
	if (g_current_state != SAFE_ST && g_current_state != PANIC_ST){
		if (g_dest_state == PANIC_ST || g_dest_state == g_current_state){
			g_current_state = g_dest_state;
			return;
		}else{
			printf("QR: Can not directly switch to other modes else than PANIC MODE in the current mode.\n");
			return;
		}
	}
}

void reset_motor_state(){
	g_current_m0_state = M0_REMAIN;
	g_current_m1_state = M1_REMAIN;
	g_current_m2_state = M2_REMAIN;
	g_current_m3_state = M3_REMAIN;
}

void execute (){
	if (g_current_comm_type == CTRL_COMM){
		ctrl_action();
		reset_motor_state();
		// it looks like I have to reset the g_current_comm_type, but with this reset a bug appears
	}

	// if(g_current_comm_type == JS_COMM){
	// 	// handled by the thread
	// }

	if (g_current_comm_type == MODE_SW_COMM && g_dest_state != NO_WHERE){
		mode_sw_action();
		g_dest_state = NO_WHERE;
		g_current_comm_type = NO_COMM;
	}

	if (g_current_comm_type == ESC_COMM){
		demo_done = true;
		g_current_comm_type = NO_COMM;
	}
}

uint8_t calibration_counter = 0;
int16_t sensor_calib = 0, sensor_sum = 0;
int16_t phi_calib = 0, theta_calib = 0, psi_calib = 0;
int16_t sensor_calibration(int16_t sensor_ori, uint8_t num)//average
{
	int16_t sensor_temp = 0;
	sensor_temp = sensor_ori;
	sensor_sum += sensor_temp;
	calibration_counter++;
	if(calibration_counter == 3){
		sensor_calib = sensor_sum / num;
		calibration_counter = 0; sensor_sum = 0;
		printf("| %6d \n", sensor_calib);
	}
	else
		printf("||\n");
	return sensor_calib;
}

/*------------------------------------------------------------------
 * main -- everything you need is here :)
 *------------------------------------------------------------------
 */
int main(void)
{
	uart_init();
	gpio_init();
	timers_init();
	adc_init();
	twi_init();
	imu_init(true, 100);	
	baro_init();
	spi_flash_init();
	ble_init();

	uint32_t counter = 0;
	demo_done = false;

	printf("    TIME   | AE0 AE1 AE2 AE3 |   PHI    THETA   PSI |     SP     SQ     SR |  BAT | TEMP | PRESSURE | MODE \n");
	while (!demo_done)
	{
		if (rx_queue.count) process_key( dequeue(&rx_queue) );
		execute();
		if (check_timer_flag()) 
		{
			if (counter++%20 == 0) nrf_gpio_pin_toggle(BLUE);

			adc_request_sample();
			read_baro();

			printf("%10ld | ", get_time_us());
			printf("%3d %3d %3d %3d | ",ae[0],ae[1],ae[2],ae[3]);
			printf("%6d %6d %6d | ", phi, theta, psi);
			printf("%6d %6d %6d | ", sp, sq, sr);
			printf("%4d | %4ld | %6ld   | ", bat_volt, temperature, pressure);
			printf("%4d \n", g_current_state);

			sensor_calibration(phi, 3); phi_calib = sensor_calib;

			clear_timer_flag();
		}

		if (check_sensor_int_flag()) 
		{
			get_dmp_data();
			run_filters_and_control();
		}
		if (g_current_state == PANIC_ST){
				printf("QR: Entered PANIC MODE.");
				int i = 800;
				while (i) {
					ae[0] = i;
					ae[1] = i;
					ae[2] = i;
					ae[3] = i;
					run_filters_and_control();
					nrf_delay_ms(1000);
					i = i - 100;
				}
			g_current_state = SAFE_ST;
		}
	}

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}
