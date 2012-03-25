#include <inttypes.h>
#include <stdlib.h>

#include "util.hpp"

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
