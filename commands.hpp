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
	NETWORK_COMM_SUCCESS,
	NETWORK_COMM_FAIL,
	NETWORK_COMM_TIMEOUT
};

class COMMAND {
	private:

	volatile uint8_t _current_cmd;
	ReliableComms* _realiable_comms;

	public:
	volatile uint8_t _ID;
	//volatile uint8_t _last_packet_ID_received[MAX_BLOCKS_CONNECTED];
	volatile uint8_t _last_command_received[MAX_BLOCKS_CONNECTED];
	//volatile uint8_t _last_destination_ID_received;
	//volatile uint8_t _last_packet_ID_sent;
	//volatile uint8_t _current_port;
	volatile uint8_t _block_connected[MAX_BLOCKS_CONNECTED];
	volatile bool _waiting_for_global_ack;
	volatile bool _got_ack_global_flag;
	volatile uint8_t _packet_number;

	ByteRingBuffer *_packets_destination_received;
	ByteRingBuffer *_packets_source_received;
	ByteRingBuffer *_packets_id_received;
	PacketRingBuffer* packet_queue;



	COMMAND(ReliableComms* rel_comms, PacketRingBuffer* queue_in, ByteRingBuffer *packets_id_received, ByteRingBuffer *packets_source_received, ByteRingBuffer *packets_destination_received);
	ERRORS update_connected();
	ERRORS request_id(uint8_t port);
	ERRORS return_id(uint8_t port);
	ERRORS move_top_servo_neighbour(uint8_t port, uint16_t value);
	ERRORS move_bottom_servo_neighbour(uint8_t port, uint16_t value);
	ERRORS move_top_servo_network(uint8_t destination, uint16_t value);
	ERRORS move_bottom_servo_network(uint8_t destination, uint16_t value);

	ERRORS FLOOD(Packet *packet, uint8_t skip_port);
	ERRORS send_packet_network(Packet *packet, uint8_t max_network_retry);

	uint8_t return_routing(uint8_t packet_id, uint8_t destination);
	uint8_t analize_packet();
	uint8_t retransmit_packet(uint8_t port);
	bool check_repeated(uint8_t packet_id, uint8_t source, uint8_t destination);

	void command_update();
};



#endif /* COMMANDS_HPP_ */
