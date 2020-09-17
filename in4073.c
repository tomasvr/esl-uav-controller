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


#include "in4073.h"

int FRAG_COUNT = 0;
enum STATE {
		SAFE_ST, 
		PANIC_ST,
		MANUAL_ST,
		CALIBRATION_ST,
		YAWCONTROL_ST,
		FULLCONTROL_ST
	};

enum COMM_TYPE {
		CTRL_COMM,
		MODE_SW_COMM,
		BAT_INFO,
		SYS_LOG,
		NO_COMM
	};


enum M0_CRTL{
		M0_UP,
		M0_REMAIN,
		M0_DOWN
	};

enum M1_CRTL{		
		M1_UP,
		M1_REMAIN,
		M1_DOWN,
	};

enum M2_CRTL{	
		M2_UP,
		M2_REMAIN,
		M2_DOWN,
	};

enum M3_CRTL{
		M3_UP,
		M3_REMAIN,
		M3_DOWN
	};

enum STATE g_current_state = SAFE_ST;

enum M0_CRTL g_current_m0_state = M0_REMAIN;
enum M1_CRTL g_current_m1_state = M1_REMAIN;
enum M2_CRTL g_current_m2_state = M2_REMAIN;
enum M3_CRTL g_current_m3_state = M3_REMAIN;

enum COMM_TYPE g_current_comm_type = NO_COMM;


bool command_allowed (void){
	bool res = false;
	if (g_current_state != PANIC_ST)
		res = true;
	return res;
}

/*------------------------------------------------------------------
 * messg_decode -- decode messages
 *------------------------------------------------------------------
 */
void messg_decode(uint8_t messg){
	
	printf("The %d byte is: \n", 4-FRAG_COUNT);
	printf("   		 "PRINTF_BINARY_PATTERN_INT8 "\n",PRINTF_BYTE_TO_BINARY_INT8(messg));
	
	if (FRAG_COUNT == 3){
		if (!(messg & 0xf0)) {
			g_current_comm_type = CTRL_COMM;	
		} 
	}
	if (FRAG_COUNT == 2){
		if (g_current_comm_type == CTRL_COMM){
			uint8_t m0_ctrl = messg & 0xf0; 		// 0bxxxx0000
			uint8_t m1_ctrl = messg & 0x0f; 		// 0b0000xxxx
			if ((m0_ctrl >> 5)&1 && (m0_ctrl >> 4)&1) g_current_m0_state = M0_UP;
			if (((m0_ctrl >> 5)&1) == 1 && ((m0_ctrl >> 4)&1) == 0) g_current_m0_state = M0_DOWN;
			if (((m0_ctrl >> 5)&1) == 0) g_current_m0_state = M0_REMAIN;

			if ((m1_ctrl >> 1)&1 && (m1_ctrl>>0)&1) g_current_m1_state = M1_UP;
			if (((m1_ctrl >> 1)&1) == 1 && ((m1_ctrl >> 0)&1) == 0) g_current_m1_state = M1_DOWN;
			if (((m1_ctrl >> 1)&1) == 0) g_current_m1_state = M1_REMAIN;
		}
	}
	if (FRAG_COUNT == 1){
		if (g_current_comm_type == CTRL_COMM){
			uint8_t m2_ctrl = messg & 0xf0;
			uint8_t m3_ctrl = messg & 0x0f;
			if ((m2_ctrl >> 5)&1 && (m2_ctrl >> 4)&1) g_current_m2_state = M2_UP;
			if (((m2_ctrl >> 5)&1) == 1 && ((m2_ctrl >> 4)&1) == 0) g_current_m2_state = M2_DOWN;
			if (((m2_ctrl >> 5)&1) == 0) g_current_m2_state = M2_REMAIN;

			if ((m3_ctrl >> 1)&1 && (m3_ctrl>>0)&1) g_current_m3_state = M3_UP;
			if (((m3_ctrl >> 1)&1) == 1 && ((m3_ctrl >> 0)&1) == 0) g_current_m3_state = M3_DOWN;
			if (((m3_ctrl >> 1)&1) == 0) g_current_m3_state = M3_REMAIN;
		}
	}
}
/*------------------------------------------------------------------
 * process_key -- process command keys
 *------------------------------------------------------------------
 */
void process_key(uint8_t c)
{
	//printf("The value received is: "PRINTF_BINARY_PATTERN_INT8 "\n",PRINTF_BYTE_TO_BINARY_INT8(c));
	
	if (c == 0x55 && FRAG_COUNT == 0) {
		printf("Detect a starter of the packet, start receiving...\n");
		FRAG_COUNT = 3;
		return;
	}
	while (FRAG_COUNT > 0){
		messg_decode(c);
		if (FRAG_COUNT == 1) printf("Packet received. The received message --command type %d --state %d --m0_action %d --m1_action %d --m2_action %d --m3_action %d\n",
								g_current_comm_type, g_current_state, g_current_m0_state, g_current_m1_state, g_current_m2_state, g_current_m3_state);
		FRAG_COUNT--;
		return;
	}

	return;

	// switch (c) 	// control signal switch
	// {
	// 	case 'u':			// lift up	
	// 		ae[0] += 10;
	// 		ae[1] += 10;
	// 		ae[2] += 10;
	// 		ae[3] += 10;
	// 		break;
	// 	case 'd': 			// lift down
	// 		//printf("Value is %ld\n", c);
	// 		ae[0] -= 10;
	// 		if (ae[0] < 0) ae[0] = 0;
	// 		ae[1] -= 10;
	// 		if (ae[1] < 0) ae[1] = 0;
	// 		ae[2] -= 10;
	// 		if (ae[2] < 0) ae[2] = 0;
	// 		ae[3] -= 10;
	// 		if (ae[3] < 0) ae[3] = 0;
	// 		break;
	// 	case 'A':			// pitch down
	// 		//printf("Value is %ld\n", c);
	// 		ae[0] -= 10;
	// 		if (ae[0] < 0) ae[0] = 0;
	// 		ae[2] += 10;
	// 		break;
	// 	case 'B':			// pitch up
	// 		//printf("Value is %ld\n", c);
	// 		ae[0] += 10;
	// 		ae[2] -= 10;
	// 		if (ae[2] < 0) ae[2] = 0;
	// 		break;
	// 	case 'C':			// roll down
	// 		//printf("Value is %ld\n", c);
	// 		ae[1] -= 10;
	// 		if (ae[1] < 0) ae[1] = 0;
	// 		ae[3] += 10;
	// 		break;
	// 	case 'D':			// roll up
	// 		//printf("Value is %ld\n", c);
	// 		ae[1] += 10;
	// 		ae[3] -= 10;
	// 		if (ae[3] < 0) ae[3] = 0;
	// 		break;
	// 	case 'q': 			// yaw down(left)
	// 		//printf("Value is %ld\n", c);
	// 		ae[1] += 10;
	// 		ae[3] += 10;
	// 		ae[0] -= 10;
	// 		if (ae[0] < 0) ae[0] = 0;
	// 		ae[2] -= 10;
	// 		if (ae[2] < 0) ae[2] = 0;
	// 		break;
	// 	case 'w': 			// yaw up(right)
	// 		//printf("Value is %ld\n", c);
	// 		ae[0] += 10;
	// 		ae[2] += 10;
	// 		ae[1] -= 10;
	// 		if (ae[1] < 0) ae[1] = 0;
	// 		ae[3] -= 10;
	// 		if (ae[3] < 0) ae[3] = 0;
	// 		break;
	// 	case 27:
	// 		demo_done = true;
	// 		break;
	// 	case 48:
	// 		g_current_state = SAFE_ST;
	// 		break;
	// 	case 49:
	// 		g_current_state = PANIC_ST;
	// 		break;
	// 	case 50:
	// 		if (command_allowed()){
	// 			g_current_state = MANUAL_ST;
	// 		}
	// 		break;
	// 	default:
	// 		nrf_gpio_pin_toggle(RED);
	// }
}

void ctrl_action(){
	switch (g_current_m0_state){			//M0
		case M0_UP:
			ae[0] += 10;
			printf("M0 goes up.\n");
			break;
		case M0_REMAIN:
			break;
		case M0_DOWN:
			ae[0] -= 10;
			break;
		default:
			break;
	}

	switch (g_current_m1_state){			//M1
		case M1_UP:
			ae[1] += 10;
			break;
		case M1_REMAIN:
			break;
		case M1_DOWN:
			ae[1] -= 10;
			break;
		default:
			break;
	}
	switch (g_current_m2_state){			//M2
		case M2_UP:
			ae[2] += 10;
			break;
		case M2_REMAIN:
			break;
		case M2_DOWN:
			ae[2] -= 10;
			break;
		default:
			break;
	}

	switch (g_current_m3_state){			//M3
		case M3_UP:
			ae[3] += 10;
			break;
		case M3_REMAIN:
			break;
		case M3_DOWN:
			ae[3] -= 10;
			break;
		default:
			break;
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
	}
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
			clear_timer_flag();
		}

		if (check_sensor_int_flag()) 
		{
			get_dmp_data();
			run_filters_and_control();
		}
	}	

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}
