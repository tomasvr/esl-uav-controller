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


#include "in4073.h"
#include <assert.h>

#define USB_COMM_INTERVAL_THRESHOLD 2000000 // in us (1000000 = 1 second)    

uint32_t usb_comm_last_received;
uint32_t current_time;

// each packet includes 4 fragments
int FRAG_COUNT = 0;

// State variables initalization
STATE_t g_current_state = SAFE_ST;
STATE_t g_dest_state = NO_WHERE;

// Communication variables initalization
COMM_TYPE g_current_comm_type = NO_COMM;

// Motor variables initalization
MOTOR_CTRL g_current_m0_state = MOTOR_REMAIN;
MOTOR_CTRL g_current_m1_state = MOTOR_REMAIN;
MOTOR_CTRL g_current_m2_state = MOTOR_REMAIN;
MOTOR_CTRL g_current_m3_state = MOTOR_REMAIN;

YAW_CONTROL_T yaw_control;

int find_motor_state(uint8_t messg){
	uint8_t m_ctrl_1 = messg & 0xf0; 		
	uint8_t m_ctrl_2 = messg & 0x0f;
	int result = 0;
	// find motor state of motor 0 
	if (m_ctrl_1>>6 == 0) {				// 0000 -> M0
		if ((m_ctrl_1 >> 5)&1 && (m_ctrl_1 >> 4)&1) g_current_m0_state = MOTOR_UP;
		if (((m_ctrl_1 >> 5)&1) == 1 && ((m_ctrl_1 >> 4)&1) == 0) g_current_m0_state = MOTOR_DOWN;
		if (((m_ctrl_1 >> 5)&1) == 0) g_current_m0_state = MOTOR_REMAIN;
		result = 1;
	}
	if (m_ctrl_1>>6 == 2) {				// 0010 -> M2
		if ((m_ctrl_1 >> 5)&1 && (m_ctrl_1 >> 4)&1) g_current_m2_state = MOTOR_UP;
		if (((m_ctrl_1 >> 5)&1) == 1 && ((m_ctrl_1 >> 4)&1) == 0) g_current_m2_state = MOTOR_DOWN;
		if (((m_ctrl_1 >> 5)&1) == 0) g_current_m2_state = MOTOR_REMAIN;
		result = 1;
	}
	if (m_ctrl_2>>2 == 1) {				// 0001 -> M1
		if ((m_ctrl_2 >> 1)&1 && (m_ctrl_2>>0)&1) g_current_m1_state = MOTOR_UP;
		if (((m_ctrl_2 >> 1)&1) == 1 && ((m_ctrl_2 >> 0)&1) == 0) g_current_m1_state = MOTOR_DOWN;
		if (((m_ctrl_2 >> 1)&1) == 0) g_current_m1_state = MOTOR_REMAIN;
		result = 1;
	}
	if (m_ctrl_2>>2 == 3) {				// 0011 -> M3
		if ((m_ctrl_2 >> 1)&1 && (m_ctrl_2>>0)&1) g_current_m3_state = MOTOR_UP;
		if (((m_ctrl_2 >> 1)&1) == 1 && ((m_ctrl_2 >> 0)&1) == 0) g_current_m3_state = MOTOR_DOWN;
		if (((m_ctrl_2 >> 1)&1) == 0) g_current_m3_state = MOTOR_REMAIN;
		result = 1;
	} 
	return result;
}

void enter_panic_mode(bool cable_detached){
	if (g_current_state == SAFE_ST) {
		return; // if in safe mode then you do not need to go to panic mode
	}
	printf("QR: Entered PANIC MODE.");
	int motor_speed = PANIC_MODE_MOTOR_SPEED;
	while (motor_speed >= 0) {
		ae[0] = motor_speed;
		ae[1] = motor_speed;
		ae[2] = motor_speed;
		ae[3] = motor_speed;
		update_motors(); //or run filters_and_control() ?
		nrf_delay_ms(1000);
		motor_speed = motor_speed - 100;
	}
	g_current_state = SAFE_ST;
	if (cable_detached) { //wait for reboot
		while(1) {
			motor[0] = 0;
			motor[1] = 0;
			motor[2] = 0;
			motor[3] = 0;
		}
	}
}

void USB_comm_update_received() {
	printf("USB comm check has been received!\n");
	current_time = get_time_us();
	usb_comm_last_received = current_time;
}

void check_USB_connection_alive() {
	current_time = get_time_us();
	if (current_time - usb_comm_last_received > USB_COMM_INTERVAL_THRESHOLD) { //TODO: WHAT HAPPENS ON OVERFLOW!
		enter_panic_mode(true);
	}
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
	
	//printf("  messg: "PRINTF_BINARY_PATTERN_INT8"\n",PRINTF_BYTE_TO_BINARY_INT8(messg));
	if (FRAG_COUNT == 3){
		uint8_t comm_type = messg & 0xf0; //take left most 4 bits from current byte
		uint8_t state = messg & 0x0f;     //take right most 4 bits from current byte

		//printf("  comm_type: "PRINTF_BINARY_PATTERN_INT8"\n",PRINTF_BYTE_TO_BINARY_INT8(comm_type));

		g_current_comm_type = find_comm_type(comm_type);
		//assert( == 1 && "QR: No such command found.");

	 	if (g_current_comm_type == USB_CHECK_COMM) {
	 		USB_comm_update_received();
	 	}

		int result = check_mode_sync(state, g_current_state);
		//printf("check_mode_sync result: %d\n", result);
		//assert(result == 1 && "QR: The mode in QR is not sync with PC or the action is not allowed in current mode!"); // might have to enter the panic mode?????????????????
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
		 		
	 	if (g_current_comm_type == CTRL_COMM && (g_current_state == MANUAL_ST || g_current_state == YAWCONTROL_ST)) {
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
	 	else if (g_current_comm_type == MODE_SW_COMM && FRAG_COUNT == 2){
	 		g_dest_state = find_dest_state(messg);
	 	}

	 	/* USB_comm_check message, so we don't care about the contents of frag 2 and 1 */
	 	else if (g_current_comm_type == USB_CHECK_COMM) {
	 		//do nothing
	 	}

	 	else if (g_current_comm_type == CHANGE_P_COMM && FRAG_COUNT == 2) {
		printf("  CHANGE_P_COMM message: "PRINTF_BINARY_PATTERN_INT8"\n",PRINTF_BYTE_TO_BINARY_INT8(messg));
	 		if (messg == 0x01) {
	 			printf("P CONTROL UP\n");
	 			increase_p_value(&yaw_control);
	 		}
	 		if (messg == 0x00) {
		 		printf("P CONTROL DOWN\n");
		 		decrease_p_value(&yaw_control);
	 		}
	 	}

	 	else {
	      	printf("UNKNOWN COMM TYPE OR TRASHED MESSG RECEIVED AT FCB SIDE \n");
	 	}
	}
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
	while (FRAG_COUNT > 0){ //TODO: why is there a return statement in this while loop?
		messg_decode(c);
		FRAG_COUNT--;
		return;
	}
}

void execute (){
	if (g_current_comm_type == ESC_COMM){ // terminate program
		demo_done = true;
		g_current_comm_type = NO_COMM;
	}
	if (g_current_comm_type == CTRL_COMM){
		ctrl_action();
		// it looks like I have to reset the g_current_comm_type, but with this reset a bug appears
	}

	// if(g_current_comm_type == JS_COMM){
	// 	// handled by the thread
	// }

	if (g_current_comm_type == MODE_SW_COMM && g_dest_state != NO_WHERE){
		g_current_state = mode_sw_action("FCB", g_current_state, g_dest_state, false);
		g_dest_state = NO_WHERE;
		g_current_comm_type = NO_COMM;
	}


}

uint8_t calibration_counter = 0;
int16_t sensor_calib = 0, sensor_sum = 0;
int16_t phi_calib = 0, theta_calib = 0, psi_calib = 0;
int16_t calib_return;
bool calibration_done = false;

int16_t sensor_calibration(int16_t sensor_ori, uint8_t num)//average
{
	int16_t sensor_temp = 0;
	sensor_temp = sensor_ori;
	sensor_sum += sensor_temp;
	calibration_counter++;
	if(calibration_counter == num){
		sensor_calib = sensor_sum / num;
		sensor_sum = 0;
		calibration_counter = 0;
		calibration_done = true;
		printf("| Calib done: %6d \n", sensor_calib);
		return sensor_calib;
	}
	// not calibrated yet
	return -1;
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
	usb_comm_last_received = get_time_us();

	yaw_control_init(&yaw_control);

	printf("    TIME   | AE0 AE1 AE2 AE3 |   PHI    THETA   PSI |     SP     SQ     SR |  BAT | TEMP | PRESSURE | MODE \n");
	while (!demo_done)
	{
		if (rx_queue.count) 
		{
			process_key( dequeue(&rx_queue) );
		}
		execute();

		// check if USB connection is still alive by checking last time received
		if (counter % 100 == 0) check_USB_connection_alive(); // use counter so this doesn't happen too often

		if (check_timer_flag()) 
		{
			if (counter % 20 == 0) nrf_gpio_pin_toggle(BLUE);
 
			adc_request_sample();
			read_baro();

			printf("%10ld | ", get_time_us());
			printf("%3d %3d %3d %3d | ",ae[0],ae[1],ae[2],ae[3]);
			printf("%6d %6d %6d | ", phi, theta, psi);
			printf("%6d %6d %6d | ", sp, sq, sr);
			printf("%4d | %4ld | %6ld   | ", bat_volt, temperature, pressure);
			printf("%4d \n", g_current_state);

			

			clear_timer_flag();
		}

		if (check_sensor_int_flag()) 
		{
			get_dmp_data();
			run_filters_and_control();
		}
		if (g_current_state == PANIC_ST){
			enter_panic_mode(false); //enter panic mode for any reason other than cable
		}
		if (g_current_state == CALIBRATION_ST) {
			calib_return = sensor_calibration(psi, 10); 
			if (calib_return != -1) {
				psi_calib = sensor_calib;
				printf("\n PSI CALIB DONE, PSI_CALIB: %6d\n", psi_calib);	
				g_current_state = SAFE_ST;
			}
		}
		if (g_current_state == YAWCONTROL_ST){
			if (counter % 200 == 0) {
				if (calibration_done) {
					//input: setpoint signal + psi signal
					//output: motor speed
					//setpoint = 0, yaw rate = 0
					yaw_control_speed_calculate(&yaw_control, psi);
				} else {
					printf("\n DO CALIBRATION BEFORE YAW CONTROL MODE! \n");
				}
			}

		
		}
		counter++;
	}

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}
