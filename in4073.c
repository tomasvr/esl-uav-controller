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

#include "in4073.h"

/*------------------------------------------------------------------
 * process_key -- process command keys
 *------------------------------------------------------------------
 */
void process_key(uint8_t c)
{
	switch (c)
	{
		case 'q':
			ae[0] += 10;
			break;
		case 'a':
			ae[0] -= 10;
			if (ae[0] < 0) ae[0] = 0;
			break;
		case 'w':
			ae[1] += 10;
			break;
		case 's':
			ae[1] -= 10;
			if (ae[1] < 0) ae[1] = 0;
			break;
		case 'e':
			ae[2] += 10;
			break;
		case 'd':
			ae[2] -= 10;
			if (ae[2] < 0) ae[2] = 0;
			break;
		case 'r':
			ae[3] += 10;
			break;
		case 'f':
			ae[3] -= 10;
			if (ae[3] < 0) ae[3] = 0;
			break;
		case 27:
			demo_done = true;
			break;
		default:
			nrf_gpio_pin_toggle(RED);
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

	uint8_t address_init = 0x000000;
	uint8_t address_counter = 0;

	uint8_t time_store[100] = {0};
	uint32_t time_restore[25] = {0};
	uint8_t time_counter = 0;
	uint8_t time_restore_counter = 0;  

	uint8_t  battery_store[50] = {0};
	uint16_t battery_restore[25] = {0};
	uint8_t battery_counter = 0;
	uint8_t battery_restore_counter = 0;
	
	uint8_t  phi_store[50] = {0};
	uint16_t phi_restore[25] = {0};
	uint8_t phi_counter = 0;
	uint8_t phi_restore_counter = 0;
	int16_t phi_d = 0;
	

/*
	uint8_t address_init = 0x000000;
	uint8_t pressure_store[40] = {0};
	uint32_t pressure_restore[10] = {0};
	uint8_t pressure_counter = 0;
	uint8_t pressure_restore_counter = 0;   
*/
/*
	uint8_t address_init = 0x000000;
	uint8_t **pressure_store = {0};
	uint32_t **pressure_restore = malloc(, );
*/

    flash_chip_erase();

	while (!demo_done)
	{
		if (rx_queue.count) process_key( dequeue(&rx_queue) );

		if (check_timer_flag()) 
		{
			if (counter++%20 == 0) nrf_gpio_pin_toggle(BLUE);
			nrf_gpio_pin_toggle(YELLOW);

			adc_request_sample();
			read_baro();


			printf("%10ld | ", get_time_us());
			printf("%3d %3d %3d %3d | ",ae[0],ae[1],ae[2],ae[3]);
			printf("%6d %6d %6d | ", phi, theta, psi);
			printf("%6d %6d %6d | ", sp, sq, sr);
			printf("%4d | %4ld | %6ld", bat_volt, temperature, pressure);

			//time
			time_store[time_counter] = (get_time_us() >> 24);
			time_store[time_counter+1] = (get_time_us() >> 16);
			time_store[time_counter+2] = (get_time_us() >> 8);
			time_store[time_counter+3] = get_time_us();

			if(flash_write_bytes(address_init + address_counter, &time_store[address_counter], 4))printf("|1");
			if(flash_read_bytes(address_init + address_counter, &time_store[address_counter], 4))printf("0|");

			time_restore[time_restore_counter] = (time_store[time_counter] << 24) | (time_store[time_counter+1] << 16) 
			| (time_store[time_counter+2] << 8) | (time_store[time_counter+3]);
			printf("| %10ld ", time_restore[time_restore_counter]);

			time_restore_counter++;
			time_counter = time_counter + 4;
			address_counter = address_counter + 4;

			//Battery
			battery_store[battery_counter] = bat_volt >> 8;
			battery_store[battery_counter+1] = bat_volt;

			if(flash_write_bytes(address_init + address_counter, &battery_store[address_counter], 2))printf("|1");
			if(flash_read_bytes(address_init + address_counter, &battery_store[address_counter], 2))printf("0|");
		
			battery_restore[battery_restore_counter] = battery_store[battery_counter] << 8 | battery_store[battery_counter+1];
			printf("| %4d ", battery_restore[battery_restore_counter]);

			battery_restore_counter++;
			battery_counter = battery_counter + 2;
			address_counter = address_counter + 2;


			phi_store[phi_counter] = phi >> 8;
			phi_store[phi_counter+1] = phi;

			if(flash_write_bytes(address_init + address_counter, &phi_store[address_counter], 2))printf("|1");
			if(flash_read_bytes(address_init + address_counter, &phi_store[address_counter], 2))printf("0|");
		
			phi_restore[phi_restore_counter] = phi_store[phi_counter] << 8 | phi_store[phi_counter + 1];
			
			if(phi_restore[phi_restore_counter] > 32768) {
				phi_d = phi_restore[phi_restore_counter] - 65536;
			}
			else
				phi_d = phi_restore[phi_restore_counter];

			printf("| %6d\n ", phi_d);
			phi_restore_counter++;
			phi_counter = phi_counter + 2;
			address_counter = address_counter + 2;
/*
			pressure_store[pressure_counter] = (pressure >> 24);
			pressure_store[pressure_counter+1] = (pressure >> 16);
			pressure_store[pressure_counter+2] = (pressure >> 8);
			pressure_store[pressure_counter+3] = pressure;
			//pressure_counter = pressure_counter + 4;

			if(flash_write_bytes(address_init + pressure_counter, &pressure_store[pressure_counter], 4))
			printf("Write successfully!");

			if(flash_read_bytes(address_init + pressure_counter, &pressure_store[pressure_counter], 4))
			printf("Read successfully!");

			pressure_restore[pressure_restore_counter] = (pressure_store[pressure_counter] << 24) | (pressure_store[pressure_counter+1] << 16) 
			| (pressure_store[pressure_counter+2] << 8) | (pressure_store[pressure_counter+3]);
			printf("| %6ld\n", pressure_restore[pressure_restore_counter]);

			pressure_restore_counter++;
			pressure_counter = pressure_counter + 4;
*/

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
