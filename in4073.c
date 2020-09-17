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
enum State {
		SAFE_ST, 
		PANIC_ST,
		MANUAL_ST,
		CALIBRATION_ST,
		YAWCONTROL_ST,
		FULLCONTROL_ST
	};

enum State g_current_state = SAFE_ST;

bool command_allowed (void){
	bool res = false;
	if (g_current_state != PANIC_ST)
		res = true;
	return res;
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
		printf("The %d byte is: \n", 4-FRAG_COUNT);
		printf("   		 "PRINTF_BINARY_PATTERN_INT8 "\n",PRINTF_BYTE_TO_BINARY_INT8(c));
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
