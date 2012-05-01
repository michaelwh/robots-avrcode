/*
 * commands.hpp
 *
 *  Created on: 7 Apr 2012
 *      Author: rt5g11@soton.ac.uk
 */

#ifndef COMMANDS_HPP_
#define COMMANDS_HPP_

#include "config.hpp"
#include "util.hpp"
#include "serialcomms.hpp"

enum ERRORS{
	FAIL_REQUEST,
	FAIL,
	SUCCESS,
};

class COMMAND {
	private:

	volatile uint8_t _current_cmd;
	ReliableComms* _realiable_comms;
	public:
	volatile uint8_t _ID;
	//volatile uint8_t _last_packet_ID_received[MAX_BLOCKS_CONNECTED];
	//volatile uint8_t _last_source_ID_received;
	//volatile uint8_t _last_destination_ID_received;
	//volatile uint8_t _last_packet_ID_sent;
	//volatile uint8_t _current_port;
	volatile int _block_connected[MAX_BLOCKS_CONNECTED];


	PacketRingBuffer* packet_queue;

	/*flag[0] = Slave or Master mode
	*flag[1] = Serial 0 interrupted
	*flag[2] = Serial 1 interrupted */
	//uint8_t _flags;
	/*_buffer is a shared buffer for reading the commands
	 * */
	//uint8_t *_buffer;
	//uint8_t _buffer_length;

	COMMAND(ReliableComms* rel_comms, PacketRingBuffer* queue_in);
	ERRORS update_connected();
	ERRORS request_id(uint8_t port);
	ERRORS return_id(uint8_t port);
	uint8_t return_routing(uint8_t packet_id, uint8_t destination);
	uint8_t analize_packet();
	uint8_t retransmit_packet(uint8_t port);
	int get_block_connected(uint8_t port);

	void command_update();
};



#endif /* COMMANDS_HPP_ */
