// TODO: remove this macros after debug
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

#include "../common/configuration.h"    
#include "../common/states.h"
#include "../common/comm.h"

/* Used for keyboard control */
#define LIFT_UP 	0b01010101
#define LIFT_DOWN 	0b10101010  
#define PITCH_DOWN  0b10000100
#define PITCH_UP  	0b01001000
#define ROLL_RIGHT 	0b00100001
#define ROLL_LEFT  	0b00010010
#define YAW_LEFT 	0b10011001
#define YAW_RIGHT 	0b01100110
 	
// current axis and button readings
int	axis[6];
int	button[12];
bool time2poll;

STATE_t pc_state = SAFE_ST;

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

uint32_t current_time;
uint32_t last_js_send_time;
uint32_t last_js_axis_send_time;
 
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

/**
 * @brief      Prints the contents of a packet in binary format to the terminal
 *
 * @param[in]  message  The packet to be printed
 * @param      info     Additional information to be printed
 * 
 * @author     T. van Rietbergen
 */
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
		result = (int) write(fd_RS232, &c, PACKET_LENGTH);
	} while (result == 0);
	assert(result == PACKET_LENGTH);
	return result;
}

/**
 * @brief      Handles mode switch message
 *
 * @param[in]  message   The message
 * @param[in]  to_state  The desired state to switch to
 *
 * @return     to_state if switch is allowed, otherwise current pc_state
 * 
 * @author     T. van Rietbergen
 */
uint32_t handle_mode_switch(uint32_t message, STATE_t to_state) {
	message = append_comm_type(message, MODE_SW_COMM);
	message = append_mode(message, to_state); 

	pc_state = mode_sw_action("TERM", pc_state, to_state);
	
	if (pc_state == CALIBRATION_ST) pc_state = SAFE_ST;
	if (pc_state == PANIC_ST) pc_state = SAFE_ST;
	return message;
}

/* 
* Encode keybord commands.
* J. Cui 
*/
uint32_t message_encode(int c){
	uint32_t message = BASE_MESSAGE_PACKET_BITS; 
	switch(c){

		case STATE_SYNC_MESSAGE:// STATE_SYNC_COMM
			message = append_comm_type(message, STATE_SYNC_COMM);
			message = append_mode(message, pc_state); 
			break;
		case 'a':
			// keyboard 'a' pressed, drone lift up
			message = append_comm_type(message, CTRL_COMM);
			message = append_keyboard_motor_control(message, LIFT_UP);
			break;
		case 'z':
			// keyboard 'z' pressed, drone lift down
			message = append_comm_type(message, CTRL_COMM);
			message = append_keyboard_motor_control(message, LIFT_DOWN);
			break;
		case 'A':
			// keyboard '↑' pressed, drone pitch down
			message = append_comm_type(message, CTRL_COMM);
			message = append_keyboard_motor_control(message, PITCH_DOWN);
			break;
		case 'B':
			// keyboard '↓' pressed, drone pitch up
			message = append_comm_type(message, CTRL_COMM);
			message = append_keyboard_motor_control(message, PITCH_UP);
			break;
		case 'C':
			// keyboard '->' pressed, drone roll down
			message = append_comm_type(message, CTRL_COMM);
			message = append_keyboard_motor_control(message, ROLL_RIGHT);		
			break;
		case 'D':
			// keyboard '<-' pressed, drone roll up
			message = append_comm_type(message, CTRL_COMM);
			message = append_keyboard_motor_control(message, ROLL_LEFT);		
			break;
		case 'q':
			// keyboard 'q' pressed, drone yaw down(left)
			message = append_comm_type(message, CTRL_COMM);
			message = append_keyboard_motor_control(message, YAW_LEFT);		
			break;
		case 'w':
			// keyboard 'w' pressed, drone yaw up(right)
			message = append_comm_type(message, CTRL_COMM);
			message = append_keyboard_motor_control(message, YAW_RIGHT);		
			break;
		case 'u':
			// keyboard 'u' pressed, increase P yaw control
			message = append_comm_type(message, CHANGE_P_COMM); 
			message = append_parameter_change(message, P_RATE_YAW_INC);
			break;
		case 'j':
			// keyboard 'j' pressed, decrease P yaw control
			message = append_comm_type(message, CHANGE_P_COMM); 
			message = append_parameter_change(message, P_RATE_YAW_DEC);
			break;
		case 'i':
			// keyboard 'i' pressed, increase P1 roll/pitch control
			message = append_comm_type(message, CHANGE_P_COMM); 
			message = append_parameter_change(message, P_RATE_PITCHROLL_INC);
			break;
		case 'k':
			// keyboard 'k' pressed, decrease P1 roll/pitch control
			message = append_comm_type(message, CHANGE_P_COMM); 
			message = append_parameter_change(message, P_RATE_PITCHROLL_DEC);
			break;
		case 'o':
			// keyboard 'o' pressed, increase P2 roll/pitch control
			message = append_comm_type(message, CHANGE_P_COMM); 
			message = append_parameter_change(message, P_ANGLE_PITCHROLL_INC);
			break;
		case 'l':
			// keyboard 'l' pressed, decrease P2 roll/pitch control
			message = append_comm_type(message, CHANGE_P_COMM); 
			message = append_parameter_change(message, P_ANGLE_PITCHROLL_DEC);
			break;
		case 'y':
			// keyboard 'y' pressed, decrease control sensitivity
			message = append_comm_type(message, CHANGE_P_COMM); 
			message = append_parameter_change(message, P_SHIFT_RIGHT_VALUE_INC);
			break;
		case 'h':
			// keyboard 'h' pressed, increase control sensitivity
			message = append_comm_type(message, CHANGE_P_COMM); 
			message = append_parameter_change(message, P_SHIFT_RIGHT_VALUE_DEC);
			break;
		case 27: 
			// keyboard 'ESC' pressed, SWITCH TO PANIC MODE
			message = append_comm_type(message, ESC_COMM);
			break;
		case 48: 
			// keyboard '0' pressed, SWITCH TO PANIC MODE
			message = handle_mode_switch(message, SAFE_ST);
			break;
		case 49: 
			// KEYBOARD 1 (switch to PANIC_ST)
			message = handle_mode_switch(message, PANIC_ST);
			break;
		case 50:  
			// KEYBOARD 2 (switch to MANUAL_ST)
			message = handle_mode_switch(message, MANUAL_ST);
			break;
		case 51: 
			// KEYBOARD 3 (switch to CALIBRATION_ST)
			message = handle_mode_switch(message, CALIBRATION_ST);
			break;
		case 52: 
			// KEYBOARD 4 (switch to YAWCONTROL_ST)
			message = handle_mode_switch(message, YAWCONTROL_ST);
			break;
		case 53: 
			// KEYBOARD 5 (switch to FULLCONTROL_ST)
			message = handle_mode_switch(message, FULLCONTROL_ST);
			break;		
		default:
			;
			printf("ERROR: KEYBOARD PRESS NOT RECOGNISED: %c, (message_encode) ", c);
	}
	return message;
}

/**
 * @brief      Generate JS control message on JS event
 *
 * @param[in]  js_type    The js type (button or axis, on initalization or event)
 * @param[in]  js_number  The js number (button or axis number)
 * @param[in]  js_value   The js value (value of button or axis)
 * 
 * @author     T. van Rietbergen
 */
void send_js_message(uint8_t js_type, uint8_t js_number, uint32_t js_value) {
	uint32_t message = BASE_MESSAGE_PACKET_BITS;
	if (js_type == 129) { // button startup state (not used)
		return;
	}
	if (js_type == JS_EVENT_BUTTON && js_value == 1) { // js button pressed
		if (js_number == 0) {
			message = append_comm_type(message, ESC_COMM);
		} else {
			STATE_t to_state = js_number; // The button number indicates which state (see states.h)
			message = handle_mode_switch(message, to_state);			
		}
		print_packet(message, "PC: Send message: ");
		rs232_putchar(message);
	}
	else if ( (js_type == JS_EVENT_AXIS) || (js_type == 130)) { // js axis (130 occurs at startup)
		if( (current_time - last_js_send_time) >= PACKET_SEND_INTERVAL) { // only send js axis packet at a certain rate
			message = append_comm_type(message, JS_AXIS_COMM);
			JOYSTICK_AXIS_t axis_number_from_js = js_number;
			message = append_js_axis_type(message, axis_number_from_js);
			uint8_t js_value_smaller = (js_value >> 8);
			message |= (js_value_smaller << 16);
			last_js_axis_send_time = current_time;
			rs232_putchar(message);
			// store js value to check neutral position on pc side
			if (axis_number_from_js == LIFT_THROTTLE) {
				joystick_axis_stored_values[axis_number_from_js] = translate_throttle(js_value_smaller);
			} else {
				joystick_axis_stored_values[axis_number_from_js] = translate_axis(js_value_smaller);
			}
		}
	} 
	else {
		printf("ERROR in send_js_message: UKNOWN IF BUTTON OR AXIS (js_type)\n");
		return;
	}
}

/* 
 * Get time stamp.
 * Zehang Wu
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

	
	uint32_t last_state_sync_time = GetTimeStamp();

#ifdef ENABLE_JOYSTICK
	// js: initializaiton
	int 			fd;
	struct js_event js;
	if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
		perror("jstest");
		exit(1);
	}
	fcntl(fd, F_SETFL, O_NONBLOCK);// non-blocking mode

	last_js_send_time = GetTimeStamp();
	last_js_axis_send_time = GetTimeStamp();
#endif

	/* send & receive
	 */
	int counter = 0;
	while(true){
		/*
		 * Communication: pc -> drone
		 */
		current_time = GetTimeStamp();
		if((current_time - last_state_sync_time) >= STATE_SYNC_CHECK_INTERVAL) {
			rs232_putchar(message_encode(STATE_SYNC_MESSAGE)); // send state sync check message
			last_state_sync_time = current_time;
		}

		if ((c = term_getchar_nb()) != -1){
			// distinguish the keyboard characters and arrows
			if (c == '\033') { 
			    term_getchar_nb(); 
			    c = term_getchar_nb();
			    if (c!='A' && c!='B' && c!='C' && c!='D') {
			    	rs232_putchar(message_encode(27));
				}		 
			}
			// sned keyboard massage
 			rs232_putchar(message_encode(c));
			if (pc_state == PANIC_ST) pc_state = SAFE_ST;
		}
		if ((c = rs232_getchar_nb()) != -1) term_putchar(c);

		/*
		 * Communication: js -> pc
		 */
#ifdef ENABLE_JOYSTICK
		current_time = GetTimeStamp();
		while (read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)) {
				send_js_message(js.type, js.number, js.value);
		}
		if (errno != EAGAIN) {
			perror("\nPC: jstest: error reading\n");
			exit (1);
		}			
		last_js_send_time = current_time;
#endif

	counter++;	
	}
	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}