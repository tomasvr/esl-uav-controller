/* 

	joystick.c - contains implementation joystick communication

*/

#include "joystick.h"

bool ESC; //todo: delete this

void messg_encode_send_js(int *axis, int *button) {
	// variable declaration
	int i;
	uint32_t messg;

	// check js values
	// printf("Axes: ");
	for (i = 0; i < 6; i++){
		// printf("%2d:%6d ", i, axis[i]);
		if(i==0 && axis[i]!=0){
			// roll
			if(axis[i]<-THRESHOLD_READ){
				// roll counterclockwise
				messg = 0b10001111000001100000001001010101;
				if (g_current_state != SAFE_ST) messg = append_current_mode(messg);
				rs232_putchar(messg);
			}
			else if(axis[i]>THRESHOLD_READ){
				// roll clockwise
				messg = 0b10001110000001110000001001010101;
				if (g_current_state != SAFE_ST) messg = append_current_mode(messg);
				rs232_putchar(messg);
			}
		}
		else if(i==1 && axis[i]!=0){
			// pitch
			if(axis[i]<-THRESHOLD_READ){
				// pitch up
				messg = 0b10101100001101000000001001010101;
				if (g_current_state != SAFE_ST) messg = append_current_mode(messg);
				rs232_putchar(messg);
			}
			else if(axis[i]>THRESHOLD_READ){
				// pitch down
				messg = 0b10111100001001000000001001010101;
				if (g_current_state != SAFE_ST) messg = append_current_mode(messg);
				rs232_putchar(messg);
			}

		}
		else if(i==2 && axis[i]!=0){
			// yaw
			if(axis[i]<-THRESHOLD_READ){
				// yaw counterclockwise
				messg = 0b10101111001001110000001001010101;
				if (g_current_state != SAFE_ST) messg = append_current_mode(messg);
				rs232_putchar(messg);
			}
			else if(axis[i]>THRESHOLD_READ){
				// yaw clockwise
				messg = 0b10111110001101100000001001010101;
				if (g_current_state != SAFE_ST) messg = append_current_mode(messg);
				rs232_putchar(messg);
			}

		}
		else if(i==3 && axis[i]!=0){
			// lift
			if(axis[i]<-THRESHOLD_READ){
				// lift up
				messg = 0b10111111001101110000001001010101;
				if (g_current_state != SAFE_ST) messg = append_current_mode(messg); 
				rs232_putchar(messg);
			}
			else if(axis[i]>THRESHOLD_READ){
				// lift down
				messg = 0b10101110001001100000001001010101;
				if (g_current_state != SAFE_ST) messg = append_current_mode(messg);
				rs232_putchar(messg);
			}

		}
		// else if(i==4 && axis[i]!=0){
		// 	// small button on js (left&right)
		// }
		// else if(i==5 && axis[i]!=0){
		// 	// small button on js (forward&backward)
		// }
	}

	// check button values
	// printf("Buttons: ");
	for (i = 0; i < 12; i++){
		// printf("%2d:%s ", i, button[i] ? "on " : "off");
		if(i==0 && button[i]){// button 1(fire button) pressed
			messg = 0b00000000000000001111000001010101;
			messg = append_current_mode(messg); 
			ESC = true;
			g_current_state = mode_sw_action("TERM", g_current_state, g_dest_state, ESC);
			rs232_putchar(messg);
		}
		if(i==6 && button[i]){// button 7 pressed, drone switches to SAFE_ST
			messg = 0b00000000000000000001000001010101;
			messg = append_current_mode(messg); 
			g_dest_state = SAFE_ST;
			g_current_state = mode_sw_action("TERM", g_current_state, g_dest_state, ESC);
			g_dest_state = NO_WHERE;
			rs232_putchar(messg);
		}
		if(i==7 && button[i]){// keyboard 8 pressed, drone switches to PANIC_ST
			messg = 0b00000000100000000001000001010101;
			messg = append_current_mode(messg); 
			g_dest_state = PANIC_ST;
			g_current_state = mode_sw_action("TERM", g_current_state, g_dest_state, ESC);
			g_dest_state = NO_WHERE;
			rs232_putchar(messg);
		}
		if(i==8 && button[i]){// keyboard 9 pressed drone switches to MANUAL_ST
			messg = 0b00000000000100000001000001010101;
			messg = append_current_mode(messg); 
			g_dest_state = MANUAL_ST;
			g_current_state = mode_sw_action("TERM", g_current_state, g_dest_state, ESC);
			g_dest_state = NO_WHERE;
			rs232_putchar(messg);
		}
		// if(i==9 && button[9]){// keyboard 10 pressed drone switches to CALIBRATION_ST
		// 	rs232_putchar(messg);
		// }
		// if(i==10 && button[10]){// keyboard 11 pressed drone switches to YAWCONTROL_ST
		// 	rs232_putchar(messg);
		// }
		// if(i==11 && button[11]){// keyboard 12 pressed drone switches to FULLCONTROL_ST
		// 	rs232_putchar(messg);
		// }
	}
}