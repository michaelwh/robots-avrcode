#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <inttypes.h>

#include "serialcomms.hpp"
#include "config.hpp"


class ByteRingBuffer {
public:
	ByteRingBuffer(uint16_t length);
	void append(uint8_t item);
	uint8_t peek(uint8_t index);
	uint16_t length(void);
	void clear(void);
private:
	uint8_t* buffer;
	uint16_t bufferlen;
	uint16_t end;
	uint16_t numitems;
};

class PacketRingBuffer {
public:
	PacketRingBuffer(uint8_t length, Packet* packet_buffer, uint8_t* port_buffer_in);
	bool append(uint8_t* packet_buffer, uint8_t packet_length, uint8_t port);
	Packet* peek(uint8_t index);
	uint8_t peek_first_port();
	uint8_t length(void);
	void clear(void);
	bool dequeue();
	Packet* peek_first();
private:
	Packet* buffer;
	uint8_t bufferlen;
	uint8_t begin;
	uint8_t numitems;
	uint8_t* port_buffer;
};

int randWithinRange(int min, int max);
#endif /* UTIL_HPP_ */
