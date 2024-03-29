/*
 *	configuration.h - header file for all configuration settings for both PC and FCB
 */

#ifndef CONFIGURATION_H__
#define CONFIGURATION_H__

/* --- PC TERMINAL SETTINGS --- */

#define STATE_SYNC_CHECK_INTERVAL 1000000 // Control how often state sync messages are send (in us)
#define STATE_SYNC_MESSAGE 0 // Message ID for state sync check type message (no need to change)

#define PACKET_LENGTH 3 //in bytes
#define PACKET_SEND_INTERVAL 20000 // (in us)

// defines whether joystick is plugged in
//#define ENABLE_JOYSTICK

// defines joystick USB port (can be found using 'findusb' script)
#define JS_DEV	"/dev/input/js0"

/* --- FCB SETTINGS (in4073) --- */

// #define ENABLE_BATT_CHECK //enables the battery check

#define USB_COMM_INTERVAL_THRESHOLD 2000000 // in us (1000000 = 1 second) 
#define BATTERY_CHECK_INTERVAL_THRESHOLD 5000000

#define CALIBRATION_NUM 100

#define PANIC_MODE_LIFT 250 // base lift is added this
#define PANIC_DURATION 4000000//(in us)

#define MOTOR_MAX_CHANGE 50
#define TRIM_STEP_SIZE 1

#define JS_AXIS_MID_VALUE 		127		// used to be 32767
#define JS_AXIS_MAX_VALUE 		255		// used to be 65536
#define JS_AXIS_DIVIDE_VALUE 	254 	// used to be 65535

/* CONTROL.H SETTINGS */

// values to test with
//#define CONTROL_OUTPUT_SHIFT_VALUE 10
#define OUTPUT_SHIFT_START_VALUE 10
#define OUTPUT_SHIFT_UPPER_LIMIT 15
#define OUTPUT_SHIFT_LOWER_LIMIT 0 // never less than 0

#define MAX_DIFF_VALUE 200

#define DEBUG_LED
#define BASE_LIFT 	100

#define CONTROLLER_P_STEP_SIZE 1
#define CONTROLLER_P_UPPER_LIMIT 15
#define CONTROLLER_P_LOWER_LIMIT 1 

#define MAX_ALLOWED_SPEED 1000
#define MIN_ALLOWED_SPEED 0
#define MAX_ALLOWED_DIFF_MOTOR 50

/* --- COMMON SETTINGS --- */

/* COMM.H SETTINGS */

#define BASE_MESSAGE_PACKET_BITS 0b00000000000000000000000001010101

/* Used for keyboard control */
#define LIFT_UP 	0b01010101
#define LIFT_DOWN 	0b10101010  
#define PITCH_DOWN  0b10000100
#define PITCH_UP  	0b01001000
#define ROLL_RIGHT 	0b00100001
#define ROLL_LEFT  	0b00010010
#define YAW_LEFT 	0b10011001
#define YAW_RIGHT 	0b01100110

/* STATES.H SETTINGS */

// there are 4 axis (pitch, roll, yaw, throttle)
#define NUMBER_OF_JS_AXIS 4 

#endif
