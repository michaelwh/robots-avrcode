/*
 * config.hpp
 *
 *  Created on: Apr 6, 2012
 *      Author: mh23g08
 */

#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#define MAX_BLOCKS_CONNECTED 6
#define MODULE_ID 0
#define DEFAULT_DATA 0xFF
#define DEFAULT_FLAGS 0x00
#define MESSAGE_ERROR 0xFF
#define MESSAGE_OK 0x00
#define SERIAL_0 1
#define SERIAL_1 2
#define SERIAL_MASK 0x01
#define BLOCK_NOT_CONNECTED 0x00
#define USART_SEND_DELAY_MS	10
#define MAX_PACKET_LEN	25

#define PWM_INIT_VALUE 500
#define TOP_FREQUENCY 20000 //This should give the proper desired frequency
#define PWM_MAX 2000
#define PWM_MIN 500
#define MESSAGE_ERROR 0xFF
#define MESSAGE_OK 0x00
//Commands

#define REQUEST_ID 			0x01
#define RETURN_ID			0x02
#define REQUEST_ROUTING		0x03
#define RETURN_ROUTING		0x04
#define NETWORK_ACKNOWLEDGE 0x05
#define COMMAND_ACK 		0x06


#endif /* CONFIG_HPP_ */
