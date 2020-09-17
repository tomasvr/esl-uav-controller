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


#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>

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

uint32_t append_current_mode(uint32_t messg){

	switch(g_current_state){
		case 0:
			messg |= 0x00000000;
			break;
		case 1:
			messg |= 0x00000800;
			break;
		case 2:
			messg |= 0x00000200;
			break;
		case 3:
			messg |= 0x00000300;
			break;
		case 4:
			messg |= 0x00000400;
			break;
		case 5:
			messg |= 0x00000500;
			break;
	}
	printf("The packet to send is: "PRINTF_BINARY_PATTERN_INT32 "\n",PRINTF_BYTE_TO_BINARY_INT32(messg)); // 0000 0000 1000 0000 0001 0010 0101 0101
	return messg;
}

uint32_t messg_encode(int c){
	uint32_t messg;
	switch(c){
		
		case 'a':
			//messg = 'u'; // keyboard 'a' pressed, drone lift up
			messg = 0b10111111001101110000000001010101; // test messg to see how many bit we can send
			messg = append_current_mode(messg); 
			//printf("The packet to send is: "PRINTF_BINARY_PATTERN_INT32 "\n",PRINTF_BYTE_TO_BINARY_INT32(messg));
			break;

		case 'z':
			messg = 'd'; // keyboard 'z' pressed, drone lift down
			break;

		case 'A':
			messg = 'A'; // keyboard '↑' pressed, drone pitch down
			break;

		case 'B':
			messg = 'B'; // keyboard '↓' pressed, drone pitch up
			break;

		case 'C':
			messg = 'C'; // keyboard '->' pressed, drone roll down
			break;

		case 'D':
			messg = 'D'; // keyboard '<-' pressed, drone roll up
			break;

		case 'q':
			messg = 'q'; // keyboard 'q' pressed, drone yaw down(left)
			break;
		case 'w':
			messg = 'w'; // keyboard 'w' pressed, drone yaw up(right)
			break;

		case 27: // keyboard 'ESC' pressed, dorne switches to PANIC_ST
			messg = 0b00000000100000000001000001010101;
			messg = append_current_mode(messg); 
			g_current_state = SAFE_ST;
			break;

		case 48: // keyboard '0' pressed, dorne switches to SAFE_ST
			messg = 0b00000000000000000001000001010101;
			messg = append_current_mode(messg); 
			g_current_state = SAFE_ST;
			break;

		case 49: // keyboard '1' pressed, dorne switches to PANIC_ST
			messg = 0b00000000100000000001000001010101;
			messg = append_current_mode(messg); 
			g_current_state = SAFE_ST;
			break;

		case 50: // keyboard '2' pressed dorne switches to MANUAL_ST
			messg = 0b00000000000100000001000001010101;
			messg = append_current_mode(messg); 
			g_current_state = MANUAL_ST;
			break;

		default:
			messg = -1;

	}

	return messg;

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

	term_puts("Type ^C to exit\n");
	
	/* discard any incoming text
	 */
	while ((c = rs232_getchar_nb()) != -1)
		fputc(c,stderr);

	/* send & receive
	 */
	for (;;)
	{
		if ((c = term_getchar_nb()) != -1){

			// distinguish the characters and arrows
			if (c == '\033') { // if the first value is esc
			    term_getchar_nb(); // skip the [
			    c = term_getchar_nb();
			    if (c!='A' && c!='B' && c!='C' && c!='D') rs232_putchar(messg_encode(27));
			}

			// distinguish the arrows with ESC
			rs232_putchar(messg_encode(c));
			//printf("Message sent!\n");
		}

		if ((c = rs232_getchar_nb()) != -1)
			term_putchar(c);

	}

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}

