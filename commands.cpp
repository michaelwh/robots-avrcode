/*
 * commands.cpp
 *
 *  Created on: 7 Apr 2012
 *      Author: rt5g11
 */
#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdlib.h>
#include "serialcomms.hpp"
#include "commands.hpp"
#include "config.hpp"
#include "debug.hpp"



/*COMMAND member function*/
COMMAND::COMMAND(ReliableComms *rel_comms, PacketRingBuffer* queue_in) {
	_realiable_comms = rel_comms;
	//The current command issued
	_current_cmd = DEFAULT_DATA;
	//The current port used to communicate
	_current_port = DEFAULT_DATA;
	//The module of this ID;
	_ID = MODULE_ID;
	//This flags contains information about interruptions.
	_flags = DEFAULT_FLAGS; //Everything set to Zero.
	//Buffer that is filled with the packet received
	_buffer = NULL;
	//the length of the buffer
	_buffer_length = 0;
	//The last source ID from the last packet (Could do without it).
	_last_source_ID_received = DEFAULT_FLAGS;
	//The last destination ID from the last packet (Could do without it).
	_last_destination_ID_received = DEFAULT_FLAGS;
	//Last packet ID that has been sent (Could do without it).
	_last_packet_ID_sent = DEFAULT_FLAGS;
	//This array holds the last packets ID received from the blocks attached to the module
	//_last_packet_ID_received = (volatile uint8_t*)malloc(MAX_BLOCKS_CONNECTED * sizeof(uint8_t));
	//This array holds the IDS of the blocks attached to the module
	//_block_connected = (volatile uint8_t*)malloc(MAX_BLOCKS_CONNECTED * sizeof(uint8_t));
	//update_connected();
	packet_queue = queue_in;
}

ERRORS COMMAND::update_connected() {
	//Buffer where the request id command stores its data.
	//uint8_t *data = (uint8_t*)malloc(sizeof(*data));
	uint8_t port = 0;
	for(; port < MAX_BLOCKS_CONNECTED; port++) {
		if(_realiable_comms->is_port_connected(port)) {
			if(request_id(port) == FAIL_REQUEST) {
				return FAIL;
			}
		}
		else
			_block_connected[port] = BLOCK_NOT_CONNECTED;//There is nothing connected there
	}
	return SUCCESS;
}

/*
 * Neighbour to Neighbour communication
 * */
ERRORS COMMAND::request_id(uint8_t port) {
	//Buffer to send the commands
	_current_cmd = REQUEST_ID;
	_current_port = port;
	uint8_t buffer[] = {Packet::make_packet_flags(false,true,false,false,false), _current_cmd};
	Packet rx_packet(buffer,2);
	if(_realiable_comms->send_packet(port,&rx_packet) != COMMS_SUCCESS) {
		return FAIL_REQUEST;
	}
		return SUCCESS;
}
/*
 * Neighbour to Neighbour communication
 * */
ERRORS COMMAND::return_id(uint8_t port) {
	_current_cmd = RETURN_ID;
//	if(!_realiable_comms->is_port_connected(port))
//	{
//		update_connected();
//		return FAIL;
//	}
	uint8_t buffer[] = {Packet::make_packet_flags(false,true,false,false,false), RETURN_ID, _ID};
	Packet rx_packet(buffer,3);
	if(_realiable_comms->send_packet(port, &rx_packet) != COMMS_SUCCESS) {
		return FAIL;
	}
	else
		return SUCCESS;
}

void COMMAND::command_update() {
	//dbgprintf("Command update, queue length: %d\n", packet_queue->length());
	if(packet_queue->length() > 0) {
//		dbgprintf("Command update, queue length: %d\n", packet_queue->length());
		Packet* packet = packet_queue->peek_first();
		uint8_t port = packet_queue->peek_first_port();
		if(!packet->is_network() && packet->data_length >= 2) {
			switch (packet->get_command()) {
				case REQUEST_ID:
					return_id(port);
					break;
				case RETURN_ID:
					if(packet->data_length >= 3) {
						_block_connected[port] = packet->data[2];
						dbgprintf("ID returned %u\n", packet->data[2]);
					}
					break;
				default:
					break;
			}
		}

		packet_queue->dequeue();
	}
}
/*
 * Network communication, it floods the network until it finds the proper module, it requires a global acknowledge.

uint8_t COMMAND::return_routing(uint8_t packet_id, uint8_t destination) {
	uint8_t buffer[13];
	bool pass = true;
	_current_cmd = RETURN_ROUTING;

	buffer[0] = 0x00; //Begin 00
	buffer[1] = 0x08; //Length of the packet NOT INCLUDING 0x00 and len
	buffer[2] = 0x05; //web packet routing with acknowledge
	buffer[3] = packet_id;
	buffer[4] = _ID;
	buffer[5] = destination;
	buffer[6] = _current_cmd;	//RETURN_ROUTING
	int i = 0;
	for(; i < MAX_BLOCKS_CONNECTED; i++)
		buffer[i + 7] = _block_connected[i];
	for(i = 0; i < MAX_BLOCKS_CONNECTED; i++) {
		if((_block_connected[i] != BLOCK_NOT_CONNECTED)) {
			if(send_packet(i,buffer,13) == MESSAGE_ERROR)
				return MESSAGE_ERROR;
		}
	}
//HERE WE MUST ADD A TIMER IF IT DOES NOT RECEIVE THE RETURN PACKET.
	do {
		pass = true;
		while(((_flags >> SERIAL_0) & SERIAL_MASK) == 0);
		_flags &= 0xFD; //Clean the flag because we have consumed the interruption.
		if(((_buffer[0] & SERIAL_MASK)) && (_buffer[4]== NETWORK_ACKNOWLEDGE) && (_buffer_length >= 4)) {
				//If it's a networking packet and it is an ACKNOWLEDGE packet and the length of the packet is the proper one
				return MESSAGE_OK;
			}
			else if (analize_packet() == MESSAGE_ERROR)
				//Analize the packet
				return MESSAGE_ERROR;
			else
				pass = false;
	}while(~pass);
	return MESSAGE_OK;

}
uint8_t COMMAND::analize_packet() {
	_flags &= 0xFD; //Clean the flag because we have consumed the interruption.
	if(~(_buffer[0] & SERIAL_MASK)) {
		//It is a neighbour to neighbour packet
		switch(_buffer[1]) {
		case REQUEST_ID:
			if(return_id()== MESSAGE_ERROR)
				return MESSAGE_ERROR;
			break;
		}
	}else {
		//It is a networking packet
		if(_buffer[4] == _ID) {
			switch(_buffer[5]) {
			case RETURN_ROUTING:
				//The packet is for this module. Sector the IDS available per module
				if(_buffer[1] == 0xFE)
					_last_packet_ID_sent = 0x01;
				else
					_last_packet_ID_sent++;
				if(return_routing(_last_packet_ID_sent,_last_destination_ID_received) == MESSAGE_ERROR)
					return MESSAGE_ERROR;
			break;
			}
		}
		else {
			//The packet is not for this ID RETRANSMIT IT
			for(int i=0; i < MAX_BLOCKS_CONNECTED; i++) {
				if(_block_connected[i] != BLOCK_NOT_CONNECTED
							&& _last_packet_ID_received[i] != _buffer[1]) {
						_current_port = i;
						//Retransmit packet because it is not for the id of this module.
						if(send_packet(i,_buffer,_buffer_length) == MESSAGE_ERROR)
							return MESSAGE_ERROR;
					}
				}
			}
		}
	return MESSAGE_OK;
}
*/




