/*------------------------------------------------------------
 * Simple pc terminal in C
 *
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *------------------------------------------------------------
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

// js
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#include <stdbool.h> 

#include "joystick.h"
// #include "../in4073.h"
// #include <in4073/in4073.h>
// #include "app_timer.h"

#define JS_DEV	"/dev/input/js0"
#define THRESHOLD_READ 20000
#define POLL_DELAY 100000 // 1000000us = 1000ms = 1s

// current axis and button readings
int	axis[6];
int	button[12];
bool time2poll;


#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
 
/*------------------------------------------------------------
 * console I/O
 *------------------------------------------------------------
 */
struct termios 	savetty;

void	term_initio()
{
	struct termios tty;

	tcgetattr(0, &savetty);
	tcgetattr(0, &tty);

	tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN] = 0;

	tcsetattr(0, TCSADRAIN, &tty);
}

void	term_exitio()
{
	tcsetattr(0, TCSADRAIN, &savetty);
}

void	term_puts(char *s)
{
	fprintf(stderr,"%s",s);
}

void	term_putchar(char c)
{
	putc(c,stderr);
}

int	term_getchar_nb()
{
        static unsigned char 	line [2];

        if (read(0,line,1)) // note: destructive read
        		return (int) line[0];

        return -1;
}

int	term_getchar()
{
        int    c;

        while ((c = term_getchar_nb()) == -1)
                ;
        return c;
}

/*------------------------------------------------------------
 * Serial I/O
 * 8 bits, 1 stopbit, no parity,
 * 115,200 baud
 *------------------------------------------------------------
 */
#include <termios.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <assert.h>
#include <time.h>

int serial_device = 0;
int fd_RS232;

void rs232_open(void)
{
  	char 		*name;
  	int 		result;
  	struct termios	tty;

       	fd_RS232 = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);  // Hardcode your serial port here, or request it as an argument at runtime

	assert(fd_RS232>=0);

  	result = isatty(fd_RS232);
  	assert(result == 1);

  	name = ttyname(fd_RS232);
  	assert(name != 0);

  	result = tcgetattr(fd_RS232, &tty);
	assert(result == 0);

	tty.c_iflag = IGNBRK; /* ignore break condition */
	tty.c_oflag = 0;
	tty.c_lflag = 0;

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; /* 8 bits-per-character */
	tty.c_cflag |= CLOCAL | CREAD; /* Ignore model status + read input */

	cfsetospeed(&tty, B115200);
	cfsetispeed(&tty, B115200);

	tty.c_cc[VMIN]  = 0;
	tty.c_cc[VTIME] = 1; // added timeout

	tty.c_iflag &= ~(IXON|IXOFF|IXANY);

	result = tcsetattr (fd_RS232, TCSANOW, &tty); /* non-canonical */

	tcflush(fd_RS232, TCIOFLUSH); /* flush I/O buffer */
}


void 	rs232_close(void)
{
  	int 	result;

  	result = close(fd_RS232);
  	assert (result==0);
}


int	rs232_getchar_nb()
{
	int 		result;
	unsigned char 	c;

	result = read(fd_RS232, &c, 1);

	if (result == 0)
		return -1;

	else
	{
		assert(result == 1);
		return (int) c;
	}
}


int 	rs232_getchar()
{
	int 	c;

	while ((c = rs232_getchar_nb()) == -1)
		;
	return c;
}


int 	rs232_putchar(int c) 			// change char to uint32_t
{
	int result;

	do {
		result = (int) write(fd_RS232, &c, 4);
	} while (result == 0);

	assert(result == 4);
	return result;
}

// the states that our QR has
enum STATE {
		SAFE_ST, 
		PANIC_ST,
		MANUAL_ST,
		CALIBRATION_ST,
		YAWCONTROL_ST,
		FULLCONTROL_ST,
		NO_WHERE
	};

enum STATE g_current_state = SAFE_ST;
enum STATE g_dest_state = NO_WHERE;
bool ESC = false;

uint32_t append_current_mode(uint32_t messg){

	switch(g_current_state){
		case 0:
			messg |= 0x00000000; // selected bits should be masked to 0 before '|='?
			break;
		case 1:
			messg |= 0x00000800;
			break;
		case 2:
			messg |= 0x00000100;
			break;
		case 3:
			messg |= 0x00000200;
			break;
		case 4:
			messg |= 0x00000300;
			break;
		case 5:
			messg |= 0x00000400;
			break;
		default:
			break;
	}
	//printf("TER: The packet to send is: "PRINTF_BINARY_PATTERN_INT32 "\n",PRINTF_BYTE_TO_BINARY_INT32(messg)); // 0000 0000 1000 0000 0001 0010 0101 0101
	return messg;
}


void mode_sw_action(){
	if (ESC) {
		g_current_state = SAFE_ST;
		return;
	}
	if (g_current_state == SAFE_ST){
		if (g_dest_state == PANIC_ST) {
			printf("TER: Can not switch to PANIC MODE while in SAFE MODE!\n");
			return;
		}
		g_current_state = g_dest_state;
		return;
	} 
	if (g_current_state == PANIC_ST){
		if (g_dest_state != SAFE_ST){
			printf("TER: an not switch to other modes else than SAFE MODE while in PANIC MODE.\n");
			return;
		}
		return;
	}
	if (g_current_state != SAFE_ST && g_current_state != PANIC_ST){
		if (g_dest_state == PANIC_ST || g_dest_state == g_current_state){
			g_current_state = g_dest_state;
			return;
		}else{
			printf("TER: Can not directly switch to other modes else than PANIC MODE in the current mode.\n");
			return;
		}
	}
}

uint32_t messg_encode(int c){
	uint32_t messg;
	switch(c){
		
		case 'a':
			// keyboard 'a' pressed, drone lift up
			messg = 0b10111111001101110000000101010101; // keyboard 'a' pressed, drone lift up, this command has a default mode -> MANUAL_ST
			if (g_current_state != SAFE_ST) messg = append_current_mode(messg); 
			break;

		case 'z':
			messg = 0b10101110001001100000000101010101; // keyboard 'z' pressed, drone lift down, this command has a default mode -> MANUAL_ST
			if (g_current_state != SAFE_ST) messg = append_current_mode(messg);
			break;

		case 'A':
			messg = 0b10111100001001000000000101010101; // keyboard '↑' pressed, drone pitch down, this command has a default mode -> MANUAL_ST
			if (g_current_state != SAFE_ST) messg = append_current_mode(messg);
			break;

		case 'B':
			messg = 0b10101100001101000000000101010101; // keyboard '↓' pressed, drone pitch up, this command has a default mode -> MANUAL_ST
			if (g_current_state != SAFE_ST) messg = append_current_mode(messg);
			break;

		case 'C':
			messg = 0b10001111000001100000000101010101; // keyboard '->' pressed, drone roll down, this command has a default mode -> MANUAL_ST
			if (g_current_state != SAFE_ST) messg = append_current_mode(messg);
			break;

		case 'D':
			messg = 0b10001110000001110000000101010101; // keyboard '<-' pressed, drone roll up, this command has a default mode -> MANUAL_ST
			if (g_current_state != SAFE_ST) messg = append_current_mode(messg);
			break;

		case 'q':
			messg = 0b10101111001001110000000101010101; // keyboard 'q' pressed, drone yaw down(left), this command has a default mode -> MANUAL_ST
			if (g_current_state != SAFE_ST) messg = append_current_mode(messg);
			break;
		case 'w':
			messg = 0b10111110001101100000000101010101; // keyboard 'w' pressed, drone yaw up(right), this command has a default mode -> MANUAL_ST
			if (g_current_state != SAFE_ST) messg = append_current_mode(messg);
			break;

		case 27: // keyboard 'ESC' pressed, drone switches to PANIC_ST
			messg = 0b00000000000000001111000001010101;
			messg = append_current_mode(messg); 
			ESC = true;
			mode_sw_action();
			break;

		case 48: // keyboard '0' pressed, dorne switches to SAFE_ST
			messg = 0b00000000000000000001000001010101;
			messg = append_current_mode(messg); 
			g_dest_state = SAFE_ST;
			mode_sw_action();
			g_dest_state = NO_WHERE;
			break;

		case 49: // keyboard '1' pressed, dorne switches to PANIC_ST
			messg = 0b00000000100000000001000001010101;
			messg = append_current_mode(messg); 
			g_dest_state = PANIC_ST;
			mode_sw_action();
			g_dest_state = NO_WHERE;
			break;

		case 50: // keyboard '2' pressed dorne switches to MANUAL_ST
			messg = 0b00000000000100000001000001010101;
			messg = append_current_mode(messg); 
			g_dest_state = MANUAL_ST;
			mode_sw_action();
			g_dest_state = NO_WHERE;
			break;

		default:
			messg = -1;

	}

	return messg;

}

unsigned int mon_time_ms(void)
{
    unsigned int    ms;
    struct timeval  tv;
    struct timezone tz;

    gettimeofday(&tv, &tz);
    ms = 1000 * (tv.tv_sec % 65); // 65 sec wrap around
    ms = ms + tv.tv_usec / 1000;
    return ms;
}

void mon_delay_ms(unsigned int ms)
{
        struct timespec req, rem;

        req.tv_sec = ms / 1000;
        req.tv_nsec = 1000000 * (ms % 1000);
        assert(nanosleep(&req,&rem) == 0);
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
uint32_t messg_encode_send_js(int *axis, int *button){
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
			mode_sw_action();
			rs232_putchar(messg);
		}
		if(i==6 && button[i]){// button 7 pressed, drone switches to SAFE_ST
			messg = 0b00000000000000000001000001010101;
			messg = append_current_mode(messg); 
			g_dest_state = SAFE_ST;
			mode_sw_action();
			g_dest_state = NO_WHERE;
			rs232_putchar(messg);
		}
		if(i==7 && button[i]){// keyboard 8 pressed, drone switches to PANIC_ST
			messg = 0b00000000100000000001000001010101;
			messg = append_current_mode(messg); 
			g_dest_state = PANIC_ST;
			mode_sw_action();
			g_dest_state = NO_WHERE;
			rs232_putchar(messg);
		}
		if(i==8 && button[i]){// keyboard 9 pressed drone switches to MANUAL_ST
			messg = 0b00000000000100000001000001010101;
			messg = append_current_mode(messg); 
			g_dest_state = MANUAL_ST;
			mode_sw_action();
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
	return 0;
}

uint32_t GetTimeStamp() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*(uint32_t)1000000+tv.tv_usec;
}

/*----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */
int main(int argc, char **argv)
{
	int	c;
	term_puts("\nTerminal program - Embedded Real-Time Systems\n");
	term_initio();
	rs232_open();
	term_puts("TER: Type ^C to exit\n");

	/* discard any incoming text
	 */
	while ((c = rs232_getchar_nb()) != -1)
		fputc(c,stderr);

	// js: initializaiton
	int 			fd;
	struct js_event js;
	unsigned int	t, i;
	if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
		perror("jstest");
		exit(1);
	}
	fcntl(fd, F_SETFL, O_NONBLOCK);// non-blocking mode

	uint32_t last_poll_time = GetTimeStamp();
	uint32_t current_time;
	time2poll = true;

	/* send & receive
	 */
	while(true){
	 
		/*---------------------------------------------------
		 *			communication: pc -> drone
		 *---------------------------------------------------
		 */
		if ((c = term_getchar_nb()) != -1){

			// distinguish the characters and arrows
			if (c == '\033') { // if the first value is esc
			    term_getchar_nb(); // skip the [
			    c = term_getchar_nb();
			    if (c!='A' && c!='B' && c!='C' && c!='D') {
			    	rs232_putchar(messg_encode(27));
				}		 
			}
			// distinguish the arrows with ESC
			rs232_putchar(messg_encode(c));
			
			if (g_current_state == PANIC_ST){
				//delay_ms(300);
				g_current_state = SAFE_ST;
			}
			
			//printf("Message sent!\n");
		}
		if ((c = rs232_getchar_nb()) != -1)
			term_putchar(c);

		/*---------------------------------------------------
		 *			communication: js -> pc
		 *---------------------------------------------------
		 */
		// mon_delay_ms(30);
		t = mon_time_ms();

		// js: read input values
		while (read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)){
			// register data
			// fprintf(stderr,".");
			switch(js.type & ~JS_EVENT_INIT) {
				case JS_EVENT_BUTTON:
					button[js.number] = js.value;
					break;
				case JS_EVENT_AXIS:
					axis[js.number] = js.value;
					break;
			}
		}
		if (errno != EAGAIN) {
			perror("\njs: error reading (EAGAIN)");
			exit (1);
		}
		// js: poll to encode and send js cmds
		current_time = GetTimeStamp();
		if((current_time-last_poll_time) >= POLL_DELAY){
			time2poll = true;
			last_poll_time = current_time;
		} 
		if(time2poll){
			messg_encode_send_js(axis, button);
			// printf("Poll \n");
			time2poll = false;
		}
		
	}


	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}

