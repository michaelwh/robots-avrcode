#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <inttypes.h>


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

#endif /* UTIL_HPP_ */
