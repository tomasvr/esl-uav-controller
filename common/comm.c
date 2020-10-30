/* 

	comm.c - contains implementation for communication between PC and FCB

*/


#include "comm.h"
#include <unistd.h>

#define BIT_LOCATION_COMM_TYPE 		8 
#define BIT_LOCATION_MODE			16 
#define BIT_LOCATION_JS_AXIS		14
#define BIT_LOCATION_MOTOR_STATES	16
#define BIT_LOCATION_PARAM			16

/* 
* Check whether pc and drone is in the same state.
* J. Cui 
*/
int check_mode_sync (uint8_t pc_state, STATE_t fcb_state){
	if (pc_state == fcb_state) {
		return 0;
	}
	return 1;
}

/**
 * @brief      Translate js axis to range: [-127, 127] (unsigned to signed) *
 * @param[in]  value  The axis value
 *
 * @return     The translated axis value
 * 
 * @author     T. van Rietbergem
 */
int8_t translate_axis(uint8_t value) {
	int8_t signed_valued;
	if (value <= 127) {
		signed_valued = value;
		return signed_valued;
	}
	signed_valued = value - 255;
	return signed_valued;
} 

/**
 * @brief      Translate throttle axis to range: [0, 255]
 *
 * @param[in]  throttle_value  The throttle value
 *
 * @return     The translated throttle value
 * 
 * @author     T. van Rietbergen
 */
uint8_t translate_throttle(int8_t throttle_value) {
	if(throttle_value <= JS_AXIS_MID_VALUE){
		throttle_value = JS_AXIS_MID_VALUE - throttle_value;
	}
	else {
		throttle_value = JS_AXIS_MAX_VALUE - throttle_value + JS_AXIS_MID_VALUE;
	}
	return throttle_value;
}

/**
 * @brief      Appends a keyboard motor control command to packet.
 *
 * @param[in]  packet       The packet
 * @param[in]  motor_states The desired motor state changes
 *
 * @return     The modified packet
 * 
 * @author     T. van Rietbergen
 */
uint32_t append_keyboard_motor_control (uint32_t packet, uint8_t motor_state_changes){
	packet |= motor_state_changes << BIT_LOCATION_MOTOR_STATES;
	return packet;
}

/**
 * @brief      Appends a js axis type to packet.
 *
 * @param[in]  packet         The packet
 * @param[in]  joystick_type  The joystick type
 *
 * @return     The modified packet
 * 
 * @author     T. van Rietbergen
 */
uint32_t append_js_axis_type (uint32_t packet, JOYSTICK_AXIS_t joystick_type){ 
	uint32_t joystick_type_uint32 = joystick_type;
	packet |= joystick_type_uint32 << BIT_LOCATION_JS_AXIS;
	return packet;
}

/**
 * @brief      Appends the communication type to packet.
 *
 * @param[in]  packet     The packet
 * @param[in]  comm_type  The communication type (packet type)
 *
 * @return     The modified packet
 * 
 * @author     T. van Rietbergen
 */
uint32_t append_comm_type(uint32_t packet, COMM_TYPE comm_type){
	uint32_t comm_type_uint32 = comm_type;
	packet |= comm_type_uint32 << BIT_LOCATION_COMM_TYPE;
	return packet;
}

/**
 * @brief      Appends current pc state to either change or check fcb state.
 *
 * @param[in]  packet  The packet
 * @param[in]  mode    The mode
 *
 * @return     The modified packet
 * 
 * @author     T. van Rietbergen
 */
uint32_t append_mode(uint32_t packet, STATE_t mode){ 
	uint32_t mode_uint32 = mode; 	
	packet |= mode_uint32 << BIT_LOCATION_MODE;			
	return packet;
}

/**
 * @brief      Appends a parameter change command to packet.
 *
 * @param[in]  packet     The packet
 * @param[in]  parameter  The parameter change
 *
 * @return     The modified packet
 * 
 * @author     T. van Rietbergen
 */
uint32_t append_parameter_change(uint32_t packet, PARAM_CHANGE_t parameter){ 
	uint32_t param_uint32 = parameter;
	packet |= param_uint32 << BIT_LOCATION_PARAM;			
	return packet;
}

/**
 * @brief      Retrieves a keyboard motor control command from packet.
 *
 * @param[in]  packet_byte  The packet byte
 *
 * @return     The keyboard motor control command.
 * 
 * @author     T. van Rietbergen
 */
uint8_t retrieve_keyboard_motor_control (uint8_t packet_byte){
	return packet_byte;

}

/**
 * @brief      Retrieves a js axis type from packet.
 *
 * @param[in]  packet_byte  The packet byte
 *
 * @return     The js axis type.
 * 
 * @author     T. van Rietbergen
 */
JOYSTICK_AXIS_t retrieve_js_axis_type(uint8_t packet_byte){
 	JOYSTICK_AXIS_t joystick_type = (packet_byte >> 6);
	return joystick_type;
}

/**
 * @brief      Retrieves a communications type from packet.
 *
 * @param[in]  packet_byte  The packet byte
 *
 * @return     The communication type.
 * 
 * @author     T. van Rietbergen
 */
COMM_TYPE retrieve_comm_type(uint8_t packet_byte){ 
	COMM_TYPE comm_type = (packet_byte & 0b00111111); // the two most left bits are reserved for axis_type
	return comm_type;
}

/**
 * @brief      Retrieves a mode from packet.
 *
 * @param[in]  packet_byte  The packet byte
 *
 * @return     The mode to change to or check.
 * 
 * @author     T. van Rietbergen
 */
STATE_t retrieve_mode(uint8_t packet_byte){
	STATE_t state = packet_byte;
	return state;
}

/**
 * @brief      Retrieves a parameter change from packet.
 *
 * @param[in]  packet_byte  The packet byte
 *
 * @return     The parameter change.
 * 
 * @author     T. van Rietbergen
 */
PARAM_CHANGE_t retrieve_parameter_change(uint8_t packet_byte){ 
	return packet_byte;
}
