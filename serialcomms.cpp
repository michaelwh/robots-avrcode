
#include <avr/io.h>
#include <inttypes.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

#include "serialcomms.hpp"


/* USART member functions */

USART::USART(volatile uint8_t * UBRRnH, volatile uint8_t * UBRRnL, volatile uint8_t * UCSRnA, volatile uint8_t * UCSRnB, volatile uint8_t * UCSRnC, volatile uint8_t * UDRn, uint8_t UDREn) {
	_UBRRnH = UBRRnH;
	_UBRRnL = UBRRnL;
	_UCSRnA = UCSRnA;
	_UCSRnB = UCSRnB;
	_UCSRnC = UCSRnC;
	_UDRn = UDRn;
	_UDREn = UDREn;
}

void USART::init(unsigned int baud) {
	/* Set baud rate */
	unsigned int ubrr = F_CPU/16/baud-1;
	*_UBRRnH = (unsigned char)(ubrr>>8);
	*_UBRRnL = (unsigned char)ubrr;
	*_UCSRnB |= (1<<TXEN0)|(1<<RXCIE0);
	//enable_rx();
	/* Set frame format: 8data, 1stop bit */
	*_UCSRnC = (3<<UCSZ00);
}

void USART::send_blocking(uint8_t data)
{
	/* Wait for empty transmit buffer */
	while ( !( *_UCSRnA & (1<<_UDREn)) );
	/* Put data into buffer, sends the data */
	*_UDRn = data;
	while ( !( *_UCSRnA & (1<<_UDREn)) );
}

void USART::enable_rx(void) {
	/* Enable receiver and transmitter */
	*_UCSRnB |= (1<<RXEN0);
}

void USART::disable_rx(void) {
	/* Disable receiver and transmitter */
	*_UCSRnB &= ~(1<<RXEN0);
}


/* MultiplexedComms member functions */

MultiplexedComms::MultiplexedComms(USART* usart, uint8_t num_ports, volatile uint8_t ** port_snoop_ports, volatile uint8_t* port_snoop_pins) {
	_usart = usart;
	_num_ports = num_ports;
	_port_snoop_ports = port_snoop_ports;
	_port_snoop_pins = port_snoop_pins;
}

void MultiplexedComms::set_current_port(uint8_t port) {
	// not implemented yet
	// change the MUX port here
	_current_port = port;
}

void MultiplexedComms::incoming_data(uint8_t port) {
	/* Some incoming data has been detected on one of the
	 * ports */

	/* Check to see if we are transmitting or receiving, if
	 * we are then ignore the new data, if not then wait until
	 * the pin goes back to normal and start receiving it */
	if (!_receiving || !_transmitting) {
		set_current_port(port);

		while(!snoop_port(port));


		start_rx();
	}
}

void MultiplexedComms::start_rx(void) {
	_disable_incoming_data_interrupts_func();
	_current_rx_packet.have_packet_length = false;
	_current_rx_packet.packet_length = 0;
	_current_rx_packet.current_rx_byte_index = 0;
	free(_current_rx_packet.received_packet);
	_rx_done = false;
	_receiving = true;
	_rx_timeout_timer = 0;
	_usart->enable_rx();
}

void MultiplexedComms::rx_byte(uint8_t byte_in) {
	/* Called when a byte is received over the serial line */
	if(_receiving) {
		if(!_current_rx_packet.have_packet_length) {
			_current_rx_packet.packet_length = byte_in;
			if (byte_in != 0)
				_current_rx_packet.received_packet = (uint8_t*)malloc(_current_rx_packet.packet_length * sizeof(uint8_t));
			else
				finish_rx();
		} else {
			_current_rx_packet.received_packet[_current_rx_packet.current_rx_byte_index] = byte_in;
			_current_rx_packet.current_rx_byte_index++;

			if(_current_rx_packet.current_rx_byte_index >= _current_rx_packet.packet_length) {
				finish_rx();
			}

		}
	}
}

void MultiplexedComms::finish_rx(void) {
	_usart->disable_rx();
	_receiving = false;
	_rx_timeout_timer = 0;

	if(_current_rx_packet.have_packet_length && _current_rx_packet.current_rx_byte_index >= _current_rx_packet.packet_length) {
		_rx_done = true;
		if (_rx_packet_callback != NULL)
			_rx_packet_callback(_current_rx_packet.received_packet, _current_rx_packet.packet_length);
	}

	_enable_incoming_data_interrupts_func();
}

bool MultiplexedComms::send_data(uint8_t port, uint8_t* data, uint8_t data_length) {
	if(!_receiving || _current_port == port) {
		_transmitting = true;
		if(_current_port != port)
			set_current_port(port);

		for (int i = 0; i < data_length; i++) {
			_delay_ms(USART_SEND_DELAY_MS);
			_usart->send_blocking(data[i]);
		}
		_transmitting = false;
		return true;
	} else {
		return false;
	}
}


void MultiplexedComms::timer_tick(void) {
	if (_receiving) {
		_rx_timeout_timer++;
		if (_rx_timeout_timer >= 10) {
			finish_rx();
		}
	}
}

void MultiplexedComms::init(void (*rx_packet_callback)(uint8_t* rx_packet, uint8_t rx_packet_length), void (*enable_incoming_data_interrupts_func)(void), void (*disable_incoming_data_interrupts_func)(void)) {
	_rx_packet_callback = rx_packet_callback;
	_enable_incoming_data_interrupts_func = enable_incoming_data_interrupts_func;
	_disable_incoming_data_interrupts_func = disable_incoming_data_interrupts_func;
	_enable_incoming_data_interrupts_func();
}

bool MultiplexedComms::snoop_port(uint8_t port) {
	return (*_port_snoop_ports[port] & (1<<_port_snoop_pins[port]));
}
