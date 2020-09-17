/*

	comm.h - header file for communication between PC and FCB

*/

#ifndef comm_H_
#define comm_H__

/**
 * @brief      Send packet from PC to FCB.
 */
bool send_packet_to_fcb(uint8_t data);

#endif // comm_H__