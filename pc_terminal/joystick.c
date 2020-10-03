/* 

	joystick.c - contains implementation joystick communication

*/

#include "joystick.h"
#include <assert.h>

bool ESC; //todo: delete this

void increase_level(int16_t level, uint8_t amount){
	assert(amount >= 0 && "Invalid amount \n");
	assert(amount <= 10 && "Invalid amount \n");
	level += amount;
	if(level > 10) level = 10;
	return;
}

void decrease_level(int16_t level, uint8_t amount){
	assert(amount >= 0 && "Invalid amount \n");
	assert(amount <= 10 && "Invalid amount \n");
	level -= amount;
	if(level < 0) level = 0;
	return;
}

uint32_t generate_messg_js(int16_t level_0, int16_t level_1, int16_t level_2, int16_t level_3){
	uint32_t messg = 0b00000000000000000010000001010101;
	switch(level_0){
		case 0:
			break;
		case 1:
			messg |= (0x1 << 20);
			break;
		case 2:
			messg |= (0x2 << 20);
			break;
		case 3:
			messg |= (0x3 << 20);
			break;
		case 4:
			messg |= (0x4 << 20);
			break;
		case 5:
			messg |= (0x5 << 20);
			break;
		case 6:
			messg |= (0x6 << 20);
			break;
		case 7:
			messg |= (0x7 << 20);
			break;
		case 8:
			messg |= (0x8 << 20);
			break;
		case 9:
			messg |= (0x9 << 20);
			break;
		case 10:
			messg |= (0xa << 20);
			break;
		default:
			break;
	}
	switch(level_1){
		case 0:
			break;
		case 1:
			messg |= (0x1 << 16);
			break;
		case 2:
			messg |= (0x2 << 16);
			break;
		case 3:
			messg |= (0x3 << 16);
			break;
		case 4:
			messg |= (0x4 << 16);
			break;
		case 5:
			messg |= (0x5 << 16);
			break;
		case 6:
			messg |= (0x6 << 16);
			break;
		case 7:
			messg |= (0x7 << 16);
			break;
		case 8:
			messg |= (0x8 << 16);
			break;
		case 9:
			messg |= (0x9 << 16);
			break;
		case 10:
			messg |= (0xa << 16);
			break;
		default:
			break;
	}
	switch(level_2){
		case 0:
			break;
		case 1:
			messg |= (0x1 << 28);
			break;
		case 2:
			messg |= (0x2 << 28);
			break;
		case 3:
			messg |= (0x3 << 28);
			break;
		case 4:
			messg |= (0x4 << 28);
			break;
		case 5:
			messg |= (0x5 << 28);
			break;
		case 6:
			messg |= (0x6 << 28);
			break;
		case 7:
			messg |= (0x7 << 8);
			break;
		case 8:
			messg |= (0x8 << 28);
			break;
		case 9:
			messg |= (0x9 << 28);
			break;
		case 10:
			messg |= (0xa << 28);
			break;
		default:
			break;
	}
	switch(level_3){
		case 0:
			break;
		case 1:
			messg |= (0x1 << 24);
			break;
		case 2:
			messg |= (0x2 << 24);
			break;
		case 3:
			messg |= (0x3 << 24);
			break;
		case 4:
			messg |= (0x4 << 24);
			break;
		case 5:
			messg |= (0x5 << 24);
			break;
		case 6:
			messg |= (0x6 << 24);
			break;
		case 7:
			messg |= (0x7 << 24);
			break;
		case 8:
			messg |= (0x8 << 24);
			break;
		case 9:
			messg |= (0x9 << 24);
			break;
		case 10:
			messg |= (0xa << 24);
			break;
		default:
			break;
	}
	return messg;
}

/**
 * @brief		encode js cmds & send to drone
 *
 * @author     	Zehang Wu
 *
 * @param      	axes:
 *				buttons:
 *				axis: reading values of js
 *				button: reading values of js buttons
 *
 * @return     	0 if no error,
 * 				-1 if there is error
 */
void messg_encode_send_js(int *axis, int *button) {
	// variable declaration
	int i;
	uint32_t messg;

	// check js values
	int16_t level_0 = 0;
	int16_t level_1 = 0;
	int16_t level_2 = 0;
	int16_t level_3 = 0;
	uint8_t amount = 0;
	// printf("Axes: ");
	for (i = 0; i < 6; i++){
		// printf("%2d:%6d ", i, axis[i]);
		if(i==0 && axis[i]!=0){
			// roll
			if(axis[i]<-THRESHOLD_READ){
				// roll counterclockwise
				amount = (uint8_t) ((-axis[i]-THRESHOLD_READ) / 30000) * 10;
				increase_level(level_1, amount);
				decrease_level(level_3, amount);
				amount = 0;
			}
			else if(axis[i]>THRESHOLD_READ){
				// roll clockwise
				amount = (uint8_t) ((axis[i]-THRESHOLD_READ) / 30000) * 10;
				decrease_level(level_1, amount);
				increase_level(level_3, amount);
				amount = 0;
			}
		}
		else if(i==1 && axis[i]!=0){
			// pitch
			if(axis[i]<-THRESHOLD_READ){
				// pitch up
				amount = (uint8_t) ((-axis[i]-THRESHOLD_READ) / 30000) * 10;
				increase_level(level_0, amount);
				decrease_level(level_2, amount);
				amount = 0;
			}
			else if(axis[i]>THRESHOLD_READ){
				// pitch down
				amount = (uint8_t) ((axis[i]-THRESHOLD_READ) / 30000) * 10;
				decrease_level(level_0, amount);
				increase_level(level_2, amount);
				amount = 0;
			}

		}
		else if(i==2 && axis[i]!=0){
			// yaw
			if(axis[i]<-THRESHOLD_READ){
				// yaw counterclockwise
				amount = (uint8_t) ((-axis[i]-THRESHOLD_READ) / 30000) * 10;
				decrease_level(level_0, amount);
				increase_level(level_1, amount);
				decrease_level(level_2, amount);
				increase_level(level_3, amount);
				amount = 0;
			}
			else if(axis[i]>THRESHOLD_READ){
				// yaw clockwise
				amount = (uint8_t) ((axis[i]-THRESHOLD_READ) / 30000) * 10;
				increase_level(level_0, amount);
				decrease_level(level_1, amount);
				increase_level(level_2, amount);
				decrease_level(level_3, amount);
				amount = 0;
			}

		}
		else if(i==3 && axis[i]!=0){
			// lift
			if(axis[i]<-THRESHOLD_READ){
				// lift up
				amount = (uint8_t) ((-axis[i]-THRESHOLD_READ) / 30000) * 10;
				increase_level(level_0, amount);
				increase_level(level_1, amount);
				increase_level(level_2, amount);
				increase_level(level_3, amount);
				amount = 0;
			}
			else if(axis[i]>THRESHOLD_READ){
				// lift down
				amount = (uint8_t) ((axis[i]-THRESHOLD_READ) / 30000) * 10;
				decrease_level(level_0, amount);
				decrease_level(level_1, amount);
				decrease_level(level_2, amount);
				decrease_level(level_3, amount);
				amount = 0;
			}

		}
		// else if(i==4 && axis[i]!=0){
		// 	// small button on js (left&right)
		// }
		// else if(i==5 && axis[i]!=0){
		// 	// small button on js (forward&backward)
		// }
	}
	// generate messg based on levels of 4 motors
	printf("level_0: %d, ", level_0);
	printf("level_1: %d, ", level_1);
	printf("level_2: %d, ", level_2);
	printf("level_3: %d, \n", level_3);
	messg = generate_messg_js(level_0, level_1, level_2, level_3);
	if (g_current_state != SAFE_ST) messg = append_mode(messg);
	rs232_putchar(messg);

	// check button values
	// printf("Buttons: ");
	for (i = 0; i < 12; i++){
		// printf("%2d:%s ", i, button[i] ? "on " : "off");
		if(i==0 && button[i]){// button 1(fire button) pressed
			messg = 0b00000000000000001111000001010101;
			messg = append_mode(messg); 
			ESC = true;
			g_current_state = mode_sw_action("TERM", g_current_state, g_dest_state, ESC);
			rs232_putchar(messg);
		}
		if(i==6 && button[i]){// button 7 pressed, drone switches to SAFE_ST
			messg = 0b00000000000000000001000001010101;
			messg = append_mode(messg); 
			g_dest_state = SAFE_ST;
			g_current_state = mode_sw_action("TERM", g_current_state, g_dest_state, ESC);
			g_dest_state = NO_WHERE;
			rs232_putchar(messg);
		}
		if(i==7 && button[i]){// button 8 pressed, drone switches to PANIC_ST
			messg = 0b00000000100000000001000001010101;
			messg = append_mode(messg); 
			g_dest_state = PANIC_ST;
			g_current_state = mode_sw_action("TERM", g_current_state, g_dest_state, ESC);
			g_dest_state = NO_WHERE;
			rs232_putchar(messg);
		}
		if(i==8 && button[i]){// button 9 pressed drone switches to MANUAL_ST
			messg = 0b00000000000100000001000001010101;
			messg = append_mode(messg); 
			g_dest_state = MANUAL_ST;
			g_current_state = mode_sw_action("TERM", g_current_state, g_dest_state, ESC);
			g_dest_state = NO_WHERE;
			rs232_putchar(messg);
		}
		// if(i==9 && button[9]){// button 10 pressed drone switches to CALIBRATION_ST
		// 	rs232_putchar(messg);
		// }
		// if(i==10 && button[10]){// button 11 pressed drone switches to YAWCONTROL_ST
		// 	rs232_putchar(messg);
		// }
		// if(i==11 && button[11]){// button 12 pressed drone switches to FULLCONTROL_ST
		// 	rs232_putchar(messg);
		// }
	}
}