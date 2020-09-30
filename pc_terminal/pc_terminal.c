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
#include "../states.h"

#define JS_DEV	"/dev/input/js0"
#define THRESHOLD_READ 20000
#define POLL_DELAY 100000 // 1000000us = 1000ms = 1s

//#define ENABLE_JOYSTICK

// current axis and button readings
int	axis[6];
int	button[12];
bool time2poll;

STATE_t g_current_state = SAFE_ST;
STATE_t g_dest_state = NO_WHERE;


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

// STATE_t g_current_state = SAFE_ST;
// STATE_t g_dest_state = NO_WHERE;
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
			g_current_state = mode_sw_action("TERM", g_current_state, g_dest_state, ESC);
			break;

		case 48: // keyboard '0' pressed, dorne switches to SAFE_ST
			messg = 0b00000000000000000001000001010101;
			messg = append_current_mode(messg); 
			g_dest_state = SAFE_ST;
			g_current_state = mode_sw_action("TERM", g_current_state, g_dest_state, ESC);
			g_dest_state = NO_WHERE;
			break;

		case 49: // keyboard '1' pressed, dorne switches to PANIC_ST
			messg = 0b00000000100000000001000001010101;
			messg = append_current_mode(messg); 
			g_dest_state = PANIC_ST;
			g_current_state = mode_sw_action("TERM", g_current_state, g_dest_state, ESC);
			g_dest_state = NO_WHERE;
			break;

		case 50: // keyboard '2' pressed dorne switches to MANUAL_ST
			messg = 0b00000000000100000001000001010101;
			messg = append_current_mode(messg); 
			g_dest_state = MANUAL_ST;
			g_current_state = mode_sw_action("TERM", g_current_state, g_dest_state, ESC);
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

#ifdef ENABLE_JOYSTICK
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
#endif
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
				// c = rs232_getchar(); //delay until character received again
				// term_putchar(c);
				// term_puts("Character received from FCB, leaving panic mode on PC-side");
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
#ifdef ENABLE_JOYSTICK		
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
#endif	
	}

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}

