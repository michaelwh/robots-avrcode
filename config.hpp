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
//Commands
#define REQUEST_ID 			0x01
#define RETURN_ID			0x02
#define REQUEST_ROUTING		0x03
#define RETURN_ROUTING		0x04
#define NETWORK_ACKNOWLEDGE 0x05



#endif /* CONFIG_HPP_ */
