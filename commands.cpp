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
#include "pwm.hpp"
#include "movement.hpp"
#include "flags.hpp"


/*COMMAND member function*/
COMMAND::COMMAND(ReliableComms *rel_comms, PacketRingBuffer* queue_in, ByteRingBuffer *packets_id_received, ByteRingBuffer *packets_source_received, ByteRingBuffer *packets_destination_received, PWM *pwm_move, Movement *movement_in) {
	_realiable_comms = rel_comms;
	//The current command issued
	//_current_cmd = DEFAULT_DATA;
	//The current port used to communicate
	//_current_port = DEFAULT_DATA;
	//The module of this ID;
	_ID = MODULE_ID;
	 _packets_id_received = packets_id_received;
	_packets_source_received = packets_source_received;
	_packets_destination_received = packets_destination_received;
	//The que that has the commands to do
	packet_queue = queue_in;
	_waiting_for_global_ack = false;
	_got_ack_global_flag = false;
	_packet_number = 0;
	_pwm = pwm_move;
	_movement = movement_in;
}

ERRORS COMMAND::update_connected() {
	//Buffer where the request id command stores its data.
	//uint8_t *data = (uint8_t*)malloc(sizeof(*data));
	//dbgprintf("Connected:");
	uint8_t port = 0;
	for(; port < MAX_BLOCKS_CONNECTED; port++) {
		//dbgprintf(" [%d]: %d,", port, _realiable_comms->is_port_connected(port));
		if(_realiable_comms->is_port_connected(port)) {
			//dbgprintf("Querying port %d\n", port);
			if(request_id(port) == SUCCESS) {
				//dbgprintf("Query of port %d successful\n", port);
				// stay connected long enough for reply to arrive
				_delay_ms(1);
			} else {
				_block_connected[port] = BLOCK_CONNECTED_NO_RESPONSE;
			}
		}
		else {
			_block_connected[port] = BLOCK_NOT_CONNECTED;//There is nothing connected there
		}
	}
	//dbgprintf("\n");
	return SUCCESS;
}

int COMMAND::get_block_connected(uint8_t port) {
	return _block_connected[port];
}

/*
 * Neighbour to Neighbour communication
 * */
ERRORS COMMAND::request_id(uint8_t port) {
	//Buffer to send the commands
	uint8_t buffer[] = {Packet::make_packet_flags(false,true,false,false,false), REQUEST_ID};
	Packet rx_packet(buffer,2);
	if(_realiable_comms->send_packet(port,&rx_packet, MAX_NEIGHBOR_RETRY) != COMMS_SUCCESS) {
		return FAIL_REQUEST;
	}
		return SUCCESS;
}
/*
 * Neighbour to Neighbour communication
 * */
ERRORS COMMAND::return_id(uint8_t port) {
	uint8_t buffer[] = {Packet::make_packet_flags(false,true,false,false,false), RETURN_ID, _ID};
	Packet rx_packet(buffer,3);
	if(_realiable_comms->send_packet(port, &rx_packet, MAX_NEIGHBOR_RETRY) != COMMS_SUCCESS) {
		return FAIL;
	}
	else
		return SUCCESS;
}


ERRORS COMMAND::send_pulse(uint8_t port, uint16_t pulse_value) {
	uint8_t buffer[] = {Packet::make_packet_flags(false,true,false,false,false), COMMAND_PULSE, (uint8_t) pulse_value >> 8, (uint8_t) pulse_value};
	Packet rx_packet(buffer, 4);
	if(_realiable_comms->send_packet(port, &rx_packet) != COMMS_SUCCESS)
		return FAIL;
	else
		return SUCCESS;
}


ERRORS COMMAND::move_top_servo_neighbour(uint8_t port, uint16_t value) {
	uint8_t buffer[] = {Packet::make_packet_flags(false,true,false,false,false), MOVE_TOP_SERVO, uint8_t (value >> 8), (uint8_t) value};
	Packet rx_packet(buffer,4);
	if(_realiable_comms->send_packet(port, &rx_packet, MAX_NEIGHBOR_RETRY) != COMMS_SUCCESS)
		return FAIL;
	else
		return SUCCESS;
}


ERRORS COMMAND::move_bottom_servo_neighbour(uint8_t port, uint16_t value) {
	uint8_t buffer[] = {Packet::make_packet_flags(false,true,false,false,false), MOVE_BOTTOM_SERVO, uint8_t (value >> 8), (uint8_t) value};
	Packet rx_packet(buffer,4);
	if(_realiable_comms->send_packet(port, &rx_packet, MAX_NEIGHBOR_RETRY) != COMMS_SUCCESS)
		return FAIL;
	else
		return SUCCESS;
}

ERRORS COMMAND::move_bottom_servo_network(uint8_t destination, uint16_t value) {

	//Increase the packet number
	_packet_number++;
	//Build the packet
	uint8_t buffer[] = {Packet::make_packet_flags(true,true,true,false,false),_packet_number,_ID, destination, MOVE_BOTTOM_SERVO, uint8_t (value >> 8), (uint8_t) value};
	Packet rx_packet(buffer,7);
	if (send_packet_network(&rx_packet,MAX_NETWORK_RETRY)!= NETWORK_COMM_SUCCESS)
		return FAIL;
	else
		return SUCCESS;
}

ERRORS COMMAND::move_top_servo_network(uint8_t destination, uint16_t value) {
	_packet_number++;
	uint8_t buffer[] = {Packet::make_packet_flags(true,true,true,false,false),_packet_number,_ID, destination, MOVE_TOP_SERVO, uint8_t (value >> 8), (uint8_t) value};
	Packet rx_packet(buffer,7);
	/*Romel*/
	if (send_packet_network(&rx_packet,MAX_NETWORK_RETRY)!= NETWORK_COMM_SUCCESS)
		return FAIL;
	else
		return SUCCESS;
}

ERRORS COMMAND::FLOOD(Packet *packet, uint8_t skip_port) {
	//So it floods but does not require a global ack!
	//packet->set_requires_global_ack(false);
	for(uint8_t port = 0; port < MAX_BLOCKS_CONNECTED; port++) {
		if(_realiable_comms->is_port_connected(port) && port != skip_port) {
			if(_realiable_comms->send_packet(port, packet, MIN_NETWORK_RETRY) != COMMS_SUCCESS) {
				return FAIL;
			}
		}
	}
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

				case COMMAND_PULSE:
					if(packet->data_length >= 4) {
						uint16_t pulse_value = packet->data[2] << 8 | packet->data[3];
						dbgprintf("Got pulse of value %d\n", pulse_value);
						dbgprintf("Wiggling!\n");
						_movement->move_wiggle();
						send_wiggle_after_delay = true;
					}
					break;

				case MOVE_TOP_SERVO:
					if(packet->data_length >= 4) {
						uint16_t value = ((packet->data[2] << 8) | packet->data[3]);
						_pwm->TopServoMove(value);
					}
					break;
				case MOVE_BOTTOM_SERVO:
					if(packet->data_length >= 4) {
						uint16_t value = ((packet->data[2] << 8) | packet->data[3]);
						_pwm->BottomServoMove(value);
					}
					break;
				default:
					break;
			}

		}else if(packet->is_network() && packet->data_length >= 2) {
			if(!check_repeated(packet->get_packet_id(), packet->get_source(), packet->get_destination())) {
				/*If this packet has not been received so far*/
				dbgprintf("COMMAND %u\n",packet->get_command());
				if(packet->get_destination() == _ID) {
					dbgprintf("Destination %u\n",packet->get_destination());
					/*and it is destined to this id*/
					_packet_number++;
					//It is the moment to send the Ack Back
					if(packet->get_command() != NETWORK_ACKNOWLEDGE && packet->requires_global_ack()) {
						//SEND AN ACKNOWLEDGEMENT BACK, BUT DO NOT ACKNOLEDGE TEH ACKNOWLEDGMENT!!!
						uint8_t ack_global_back[] = {Packet::make_packet_flags(true,false,false,false,true), _packet_number, _ID, packet->get_source(), NETWORK_ACKNOWLEDGE};
						Packet packet_back(ack_global_back, 5);
						//Send it, it will only try it once
						send_packet_network(&packet_back, MIN_NETWORK_RETRY);
					}
					//ALL THE NETWORK PACKA
					switch(packet->get_command()) {
					case NETWORK_ACKNOWLEDGE:
						dbgprintf("Network ACK RECEIVED\n");
						if(_waiting_for_global_ack)
							_got_ack_global_flag = true;
						break;
					case MOVE_TOP_SERVO:
						//Usual
						if(packet->data_length >= 7) {
							dbgprintf("Top Servo \n");
							uint16_t value = ((packet->data[5] << 8) | packet->data[6]);
							_pwm->TopServoMove(value);
						}
						break;
					case MOVE_BOTTOM_SERVO:
						if(packet->data_length >= 7) {
							dbgprintf("Bottom Servo \n");
							uint16_t value = ((packet->data[5] << 8) | packet->data[6]);
							_pwm->BottomServoMove(value);
						}
					break;
						default:
							dbgprintf("WEIRD AND WRONG PACKET");
						break;
					}
				}
				else {
					/*THIS PACKET IS NOT FOR ME, BUT I HAVEN'T SEEN IT SO FAR, LET'S FLOOD THE NETWORK with the packet*/
					FLOOD(packet, port);
				}

			if(_packets_destination_received->isFull() || _packets_source_received->isFull() || _packets_id_received->isFull()) {
				_packets_destination_received->dequeue();
				_packets_source_received->dequeue();
				_packets_id_received->dequeue();
			}
			_packets_destination_received->append(packet->get_destination());
			_packets_source_received->append(packet->get_source());
			_packets_id_received->append(packet->get_packet_id());
		}
		}
		packet_queue->dequeue();
	}
}

ERRORS COMMAND::send_packet_network(Packet *packet, uint8_t max_network_retry) {
	if (packet->requires_global_ack()) {
		_waiting_for_global_ack = true;
	for (uint8_t retry = 0; retry < max_network_retry; retry++) {
		//Send the packet to all the connected ports
		for(uint8_t port_to_send = 0; port_to_send < MAX_BLOCKS_CONNECTED; port_to_send++) {
			if(_realiable_comms->is_port_connected(port_to_send)) {
				dbgprintf("SENDING TO PORT %u",port_to_send);
				if(_realiable_comms->send_packet(port_to_send, packet) != COMMS_SUCCESS) {
					return NETWORK_COMM_FAIL;
				}
			}
		}
		// keep checking timer values and wait until we get an ACK or timeout
		for (uint8_t delay_i = 0; delay_i < DELAY_NETWORK_TIMEOUT_MS; delay_i++) {
			if(_got_ack_global_flag) {
				break;
			}
			_delay_us(DELAY_NETWORK_TIMEOUT_US);
		}
		if(_got_ack_global_flag)
			break;
		packet->data[1]++;
	}
	_waiting_for_global_ack = false;
	//_mux_comms->unlock_from_port();


	if(_got_ack_global_flag) {
		_got_ack_global_flag = false;
		return NETWORK_COMM_SUCCESS;
	}
	else
		//dbgprintf(">Timeout!");
		return NETWORK_COMM_TIMEOUT;
	}
	else {
		dbgprintf("Sending global ack\n");
		for(uint8_t port = 0; port < MAX_BLOCKS_CONNECTED; port++) {
			if(_realiable_comms->is_port_connected(port)) {
				if(_realiable_comms->send_packet(port, packet) != COMMS_SUCCESS) {
					return NETWORK_COMM_FAIL;
				}
			}
		}
		return NETWORK_COMM_SUCCESS;
	}
}
/*
 * It returns true if the command has been received already
 * */
bool COMMAND::check_repeated(uint8_t packet_id, uint8_t source, uint8_t destination) {
	return _packets_id_received->checkRepeated(packet_id) & _packets_source_received->checkRepeated(source) & _packets_source_received->checkRepeated(destination);
}





