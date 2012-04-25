#include <inttypes.h>
#include <stdlib.h>

#include "util.hpp"
#include "config.hpp"

ByteRingBuffer::ByteRingBuffer(uint16_t length) {
	buffer = (uint8_t*)malloc(length * sizeof(uint8_t));
	bufferlen = length;
	end = 0;
}

void ByteRingBuffer::append(uint8_t item) {
	if(numitems < bufferlen)
		numitems++;
	buffer[end] = item;
	end = (end + 1) % bufferlen;
}

uint8_t ByteRingBuffer::peek(uint8_t index) {
	return buffer[(end + index) % bufferlen];
}

uint16_t ByteRingBuffer::length(void) {
	return numitems;
}

void ByteRingBuffer::clear(void) {
	end = 0;
	numitems = 0;
}


// -------

PacketRingBuffer::PacketRingBuffer(uint8_t length, Packet* packet_buffer, uint8_t* port_buffer_in) {
	buffer = packet_buffer;
	port_buffer = port_buffer_in;
	bufferlen = length;
	numitems = 0;
	begin = 0;
	for(uint8_t i = 0; i < bufferlen; i++) {
		buffer[i].data = (uint8_t*)malloc(MAX_PACKET_LEN * sizeof(uint8_t));
		buffer[i].data_length = MAX_PACKET_LEN;
	}
}


bool PacketRingBuffer::append(uint8_t* packet_buffer, uint8_t packet_length, uint8_t port) {
	if(numitems >= bufferlen)
		return false;

	port_buffer[(begin + numitems) % bufferlen] = port;
	buffer[(begin + numitems) % bufferlen].copy_from_buffer(packet_buffer, packet_length);
	numitems++;
	return true;
}

Packet* PacketRingBuffer::peek(uint8_t index) {
	return &buffer[(begin + index) % bufferlen];
}

Packet* PacketRingBuffer::peek_first() {
	return &buffer[begin];
}

uint8_t PacketRingBuffer::peek_first_port() {
	return port_buffer[begin];
}

bool PacketRingBuffer::dequeue() {
	if(numitems <= 0)
		return false;

	begin = (begin + 1) % bufferlen;
	numitems--;
	return true;
}

uint8_t PacketRingBuffer::length(void) {
	return numitems;
}

void PacketRingBuffer::clear(void) {
	begin = 0;
	numitems = 0;
}
