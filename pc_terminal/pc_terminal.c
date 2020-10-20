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
#include "../comm.h"

#define JS_DEV	"/dev/input/js0"
// #define THRESHOLD_READ 2767
#define POLL_DELAY 100000 // 1000000us = 1000ms = 1s

#define USB_SEND_CHECK_INTERVAL 1000000 // Control how often USB check messages are send
#define USB_CHECK_MESSAGE 0 // Message ID for check USB type message (no need to change)

#define ENABLE_JOYSTICK

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

void term_initio()
{
	struct termios tty;

	tcgetattr(0, &savetty);
	tcgetattr(0, &tty);

	tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN] = 0;

	tcsetattr(0, TCSADRAIN, &tty);
}

void term_exitio()
{
	tcsetattr(0, TCSADRAIN, &savetty);
}

void term_puts(char *s)
{
	fprintf(stderr,"%s",s);
}

void term_putchar(char c)
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
        while ((c = term_getchar_nb()) == -1);
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

void rs232_open(void){
  	char 		*name;
  	int 		result;
  	struct termios	tty;

       	fd_RS232 = open("/dev/ttyUSB2", O_RDWR | O_NOCTTY);  // Hardcode your serial port here, or request it as an argument at runtime

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

void print_packet(uint32_t message, char *info) { 
	printf("%s\n", info);
	printf("message: "PRINTF_BINARY_PATTERN_INT8"-",PRINTF_BYTE_TO_BINARY_INT8(message >> 24));
	printf(PRINTF_BINARY_PATTERN_INT8"-",PRINTF_BYTE_TO_BINARY_INT8(message >> 16));
	printf(PRINTF_BINARY_PATTERN_INT8"-",PRINTF_BYTE_TO_BINARY_INT8(message >> 8));
	printf(PRINTF_BINARY_PATTERN_INT8"\n",PRINTF_BYTE_TO_BINARY_INT8(message));
}

void rs232_close(void){
  	int 	result;

  	result = close(fd_RS232);
  	assert (result==0);
}


int	rs232_getchar_nb(){
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


int rs232_getchar(){
	int 	c;

	while ((c = rs232_getchar_nb()) == -1)
		;
	return c;
}


int rs232_putchar(int c){ // change char to uint32_t
	int result;

	do {
		result = (int) write(fd_RS232, &c, 4);
	} while (result == 0);

	assert(result == 4);
	return result;
}

bool ESC = false;

uint32_t message_encode(int c){
	uint32_t message = BASE_MESSAGE_PACKET_BITS; // 0b00000000000000000000000001010101
	switch(c){

		case USB_CHECK_MESSAGE:// USB_COMM_CHECK_MESSAGE //todo: take other number than 99? (i chose it randomly)
			// printf("entered case for usb check message\n");
			//message = 0b00000000000000001001000001010101; // 000000000-00000000-11110000-01010101 (empty - empty - USB_check_comm - startbit)
			message = append_comm_type(message, USB_CHECK_COMM);
			message = append_mode(message, g_current_state); // append current state to check at FCB side
			break;
		case 'a':
			// printf("a pressed\n");
			message = 0b10111111001101110000000101010101; // keyboard 'a' pressed, drone lift up, this command has a default mode -> MANUAL_ST
			if (g_current_state != SAFE_ST) message = append_mode(message, g_current_state); 
			break;

		case 'z':
			message = 0b10101110001001100000000101010101; // keyboard 'z' pressed, drone lift down, this command has a default mode -> MANUAL_ST
			if (g_current_state != SAFE_ST) message = append_mode(message, g_current_state);
			break;

		case 'A':
			message = 0b10111100001001000000000101010101; // keyboard '↑' pressed, drone pitch down, this command has a default mode -> MANUAL_ST
			if (g_current_state != SAFE_ST) message = append_mode(message, g_current_state);
			break;

		case 'B':
			message = 0b10101100001101000000000101010101; // keyboard '↓' pressed, drone pitch up, this command has a default mode -> MANUAL_ST
			if (g_current_state != SAFE_ST) message = append_mode(message, g_current_state);
			break;

		case 'C':
			message = 0b10001111000001100000000101010101; // keyboard '->' pressed, drone roll down, this command has a default mode -> MANUAL_ST
			if (g_current_state != SAFE_ST) message = append_mode(message, g_current_state);
			break;

		case 'D':
			message = 0b10001110000001110000000101010101; // keyboard '<-' pressed, drone roll up, this command has a default mode -> MANUAL_ST
			if (g_current_state != SAFE_ST) message = append_mode(message, g_current_state);
			break;

		case 'q':
			message = 0b10101111001001110000000101010101; // keyboard 'q' pressed, drone yaw down(left), this command has a default mode -> MANUAL_ST
			if (g_current_state != SAFE_ST) message = append_mode(message, g_current_state);
			break;
		case 'w':
			message = 0b10111110001101100000000101010101; // keyboard 'w' pressed, drone yaw up(right), this command has a default mode -> MANUAL_ST
			if (g_current_state != SAFE_ST) message = append_mode(message, g_current_state);
			break;
		case 'u':
			message = 0b00000000000000010111000001010101; // keyboard 'u' pressed, increase P yaw control
			if (g_current_state != SAFE_ST) message = append_mode(message, g_current_state);
			break;
		case 'j':
			message = 0b00000000000000000111000001010101; // keyboard 'j' pressed, decrease P yaw control
			if (g_current_state != SAFE_ST) message = append_mode(message, g_current_state);
			break;

		case 'i':
			// TODO: fill in the encoded message
			// message = 0b00000000000000010111000001010101; // keyboard 'i' pressed, increase P1 roll/pitch control
			if (g_current_state != SAFE_ST) message = append_mode(message, g_current_state);
			break;
		case 'k':
			// TODO: fill in the encoded message
			// message = 0b00000000000000000111000001010101; // keyboard 'j' pressed, decrease P1 roll/pitch control
			if (g_current_state != SAFE_ST) message = append_mode(message, g_current_state);
			break;
		case 'o':
			// TODO: fill in the encoded message
			// message = 0b00000000000000010111000001010101; // keyboard 'o' pressed, increase P2 roll/pitch control
			if (g_current_state != SAFE_ST) message = append_mode(message, g_current_state);
			break;
		case 'l':
			// TODO: fill in the encoded message
			// message = 0b00000000000000000111000001010101; // keyboard 'l' pressed, decrease P2 roll/pitch control
			if (g_current_state != SAFE_ST) message = append_mode(message, g_current_state);
			break;


		// ESC
		case 27: // keyboard 'ESC' pressed, drone switches to PANIC_ST
			//message = 0b 00000000 00000000 11110000 01010101;
			message = append_comm_type(message, ESC_COMM);
			message = append_mode(message, g_current_state); 
			ESC = true;
			g_current_state = mode_sw_action("TERM", g_current_state, g_dest_state, ESC);
			break;

		// KEYBOARD 0 (SAFE_ST)
		case 48: // keyboard '0' pressed, dorne switches to SAFE_ST
			//message = 0b00000000000000000001000001010101; //000000000-00000000-00010000-01010101 (empty - CTRL_COMM - MODE_SW_COMM - startbit)
			message = append_comm_type(message, MODE_SW_COMM);
			message = append_mode(message, SAFE_ST); 
			g_current_state = mode_sw_action("TERM", g_current_state, SAFE_ST, ESC);
			g_dest_state = NO_WHERE;
			break;

		// KEYBOARD 1 (PANIC_ST)
		case 49: // keyboard '1' pressed, dorne switches to PANIC_ST
			//message = 0b00000000100000000001000001010101;
			message = append_comm_type(message, MODE_SW_COMM);
			message = append_mode(message, PANIC_ST); 
			g_current_state = mode_sw_action("TERM", g_current_state, PANIC_ST, ESC);
			g_dest_state = NO_WHERE;
			break;

		// KEYBOARD 2 (MANUAL_ST)
		case 50: // keyboard '2' pressed dorne switches to MANUAL_ST
			//message = 0b00000000000100000001000001010101; 
			message = append_comm_type(message, MODE_SW_COMM);
			message = append_mode(message, MANUAL_ST); 
			print_packet(message, "PC: Example change to manual mode: ");
			g_current_state = mode_sw_action("TERM", g_current_state, MANUAL_ST, ESC);
			g_dest_state = NO_WHERE;
			break;

		// KEYBOARD 3 (CALIBRATION_ST)
		case 51:
			//message = 0b00000000001000000001000001010101; 
			message = append_comm_type(message, MODE_SW_COMM);
			message = append_mode(message, CALIBRATION_ST); 
			g_current_state = mode_sw_action("TERM", g_current_state, CALIBRATION_ST, ESC);
			g_dest_state = NO_WHERE;
			break;

		// KEYBOARD 4 (YAWCONTROL_ST)
		case 52: // keyboard '4' switches drone to yaw control mode
			//message = 0b00000000001100000001000001010101;
			message = append_comm_type(message, MODE_SW_COMM);
			message = append_mode(message, YAWCONTROL_ST); 
			g_current_state = mode_sw_action("TERM", g_current_state, YAWCONTROL_ST, ESC);
			g_dest_state = NO_WHERE;
			break;
		default:
			printf("ERROR: KEYBOARD PRESS NOT RECOGNISED: %c, (message_encode) ", c);
			//exit(-1);
	}
	return message;
}

void send_js_message(uint8_t js_type, uint8_t js_number, uint32_t js_value) {
	uint32_t message = 0b00000000000000000000000001010101; // base message
	if (js_type == 1) { //buttons
		if (js_number == 0) message = append_comm_type(message, ESC_COMM);
		else message = append_comm_type(message, MODE_SW_COMM);
		
		STATE_t state_from_js_button = js_number; // The button number indicates which state (see states.h)
		message = append_mode(message, state_from_js_button);
	}
	else if ( (js_type == 2) || (js_type == 130)) { //axis (130 occurs at startup)
		message = append_comm_type(message, JS_AXIS_COMM);
		JOYSTICK_AXIS_t axis_number_from_js = js_number;
		message = append_js_axis(message, axis_number_from_js);
		message |= (js_value << 16);
	} else {
		printf("ERROR in send_js_message: UKNOWN IF BUTTON OR AXIS (js_type)\n");
		return;
	}
	//printf("PC: Sending JS: type %d, number %d, value %d\n", js_type, js_number, js_value);
	rs232_putchar(message);
}


void send_USB_check_message() {
	rs232_putchar(message_encode(USB_CHECK_MESSAGE));
}


unsigned int mon_time_ms(void){
    unsigned int    ms;
    struct timeval  tv;
    struct timezone tz;
    gettimeofday(&tv, &tz);
    ms = 1000 * (tv.tv_sec % 65); // 65 sec wrap around
    ms = ms + tv.tv_usec / 1000;
    return ms;
}

void mon_delay_ms(unsigned int ms){
        struct timespec req, rem;
        req.tv_sec = ms / 1000;
        req.tv_nsec = 1000000 * (ms % 1000);
        assert(nanosleep(&req,&rem) == 0);
}

uint32_t GetTimeStamp() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*(uint32_t)1000000+tv.tv_usec; //todo: check for overflow? <-- (32 bit allows for about 70 minutes before overflow, fix this later?)
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

	uint32_t current_time;
	uint32_t last_USB_check_time = GetTimeStamp();

#ifdef ENABLE_JOYSTICK
	// js: initializaiton
	int 			fd;
	struct js_event js;
	//unsigned int	t, i;
	if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
		perror("jstest");
		exit(1);
	}
	fcntl(fd, F_SETFL, O_NONBLOCK);// non-blocking mode
	//uint32_t last_poll_time = GetTimeStamp();
	time2poll = true;
#endif

	/* send & receive
	 */
	int counter = 0;
	while(true){
	 
		/*---------------------------------------------------
		 *			communication: pc -> drone
		 *---------------------------------------------------
		 */

		/* Send USB connection message */
		current_time = GetTimeStamp();
		if((current_time - last_USB_check_time) >= USB_SEND_CHECK_INTERVAL) {
			//printf("PC: Time to send USB check message\n");
			send_USB_check_message();
			last_USB_check_time = current_time;
		}
		if ((c = term_getchar_nb()) != -1){

			// distinguish the characters and arrows
			if (c == '\033') { // if the first value is esc
			    term_getchar_nb(); // skip the [
			    c = term_getchar_nb();
			    if (c!='A' && c!='B' && c!='C' && c!='D') {
			    	rs232_putchar(message_encode(27));
				}		 
			}
			// distinguish the arrows with ESC
 			rs232_putchar(message_encode(c));

			if (g_current_state == PANIC_ST){
				// c = rs232_getchar(); //delay until character received again
				// term_putchar(c);
				// term_puts("Character received from FCB, leaving panic mode on PC-side");
				g_current_state = SAFE_ST;
			}
			
			//printf("Message sent!\n");
		}
		if ((c = rs232_getchar_nb()) != -1) term_putchar(c);

		/*---------------------------------------------------
		 *			communication: js -> pc
		 *---------------------------------------------------
		 */
		// mon_delay_ms(30);

#ifdef ENABLE_JOYSTICK

		// //t = mon_time_ms();
		// if (read(fd, &js, sizeof(struct js_event)) != sizeof(struct js_event)) {
		// 	perror("\njstest: error reading");
		// 	exit (1);
		// } else {
		// 	printf("Event: type %d, time %d, number %d, value %d\n",
		// 	js.type, js.time, js.number, js.value);
		// }

		while (read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event))  {
			//printf("PC: JS event: type %d, time %d, number %d, value %d\n", js.type, js.time, js.number, js.value);
				send_js_message(js.type, js.number, js.value);
		}


		if (errno != EAGAIN) {
			perror("\nPC: jstest: error reading\n");
			exit (1);
		}

		// // js: poll to encode and send js cmds
		// current_time = GetTimeStamp();
		// if((current_time-last_poll_time) >= POLL_DELAY){
		// 	time2poll = true;
		// 	last_poll_time = current_time;
		// } 
		// if(time2poll){
		// 	message_encode_send_js(axis, button);
		// 	// printf("Poll \n");
		// 	time2poll = false;
		// }
#endif
	counter++;	
	}

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}

