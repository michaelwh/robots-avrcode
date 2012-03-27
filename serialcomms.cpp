#include <avr/io.h>
#include <inttypes.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/delay.h>


#include "serialcomms.hpp"
#include "pinutil.hpp"



/* USART member functions */

USART::USART(volatile uint8_t * UBRRnH, volatile uint8_t * UBRRnL, volatile uint8_t * UCSRnA, volatile uint8_t * UCSRnB, volatile uint8_t * UCSRnC, volatile uint8_t * UDRn, uint8_t UDREn, uint8_t U2Xn) {
	_UBRRnH = UBRRnH;
	_UBRRnL = UBRRnL;
	_UCSRnA = UCSRnA;
	_UCSRnB = UCSRnB;
	_UCSRnC = UCSRnC;
	_UDRn = UDRn;
	_UDREn = UDREn;
	_U2Xn = U2Xn;
}

void USART::init() {

	/* Set baud rate */
	//unsigned int ubrr = F_CPU/16/baud-1;
	#define BAUD 9600
	#include <util/setbaud.h>

	//*_UBRRnH = (unsigned char)(ubrr>>8);
	//*_UBRRnL = (unsigned char)ubrr;
	*_UBRRnH = UBRRH_VALUE;
	*_UBRRnL = UBRRL_VALUE;
	*_UCSRnB |= (1<<TXEN0)|(1<<RXCIE0);
	#if USE_2X
		*_UCSRnA |= (1 << _U2Xn);
   	#else
   		*_UCSRnA &= ~(1 << _U2Xn);
   	#endif
	//enable_rx();
	/* Set frame format: 8 data, 1 stop bit */
	*_UCSRnC = (1<<UCSZ00)|(1<<UCSZ01);
}

void USART::send_blocking(uint8_t data)
{
	/* Wait for empty transmit buffer */
	while ( !( *_UCSRnA & (1<<_UDREn)) );
	/* Put data into buffer, sends the data */
	*_UDRn = data;
}

void USART::enable_rx(void) {
	/* Enable receiver */
	*_UCSRnB |= (1<<RXEN0);
	//*_UCSRnB |= (1<<TXEN0);
}

void USART::disable_rx(void) {
	/* Disable receiver */
	*_UCSRnB &= ~(1<<RXEN0);
}


/* MultiplexedComms member functions */

MultiplexedComms::MultiplexedComms(USART* usart, uint8_t num_ports, volatile uint8_t * const * port_snoop_pins_in, const uint8_t* port_snoop_pinnos_in) {
	_usart = usart;
	_num_ports = num_ports;
	_port_snoop_pins = port_snoop_pins_in;
	_port_snoop_pinnos = port_snoop_pinnos_in;
	_receiving = false;
	_wish_to_transmit = false;
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
	if (!_receiving && (!_wish_to_transmit || port == _wish_to_transmit_port)) {
		// we want falling edge
		if(!CHECK_BIT(*_port_snoop_pins[port], _port_snoop_pinnos[port])){
			// it should be all zeros, meaning it should last about 930us
			// so sample once every 10us and check to see if is zero for
			// all that time
			bool errorflag = false;
			for(int i = 0; i < 80; i++){
				//SET_BIT(PORTC, 0);
				//CLR_BIT(PORTC, 0);
				//SET_BIT(PORTC, 0);
				if(CHECK_BIT(*_port_snoop_pins[port], _port_snoop_pinnos[port])) {
					errorflag = true;
					break;
				}
				_delay_us(10);
			}
			if (!errorflag) {
				// no errors, so continue
				// wait until end of start byte
				while(!CHECK_BIT(*_port_snoop_pins[port], _port_snoop_pinnos[port]));

				// inform the multiplexed comms module we have
				// incoming data on this port
				if(_current_port != port)
					set_current_port(port);
				start_rx();
			}

		}

	}
}

void MultiplexedComms::start_rx(void) {
	_disable_incoming_data_interrupts_func();
	_current_rx_packet.have_packet_length = false;
	_current_rx_packet.packet_length = 0;
	_current_rx_packet.current_rx_byte_index = 0;
	_rx_done = false;
	_receiving = true;
	_rx_timeout_timer = 0;
	_usart->enable_rx();
}

void MultiplexedComms::rx_byte(uint8_t byte_in) {
	/* Called when a byte is received over the serial line */
	if(_receiving) {
		_rx_timeout_timer = 0;
		if(!_current_rx_packet.have_packet_length) {
			_current_rx_packet.packet_length = byte_in;
			_current_rx_packet.have_packet_length = true;
			_current_rx_packet.current_rx_byte_index = 0;
			if (byte_in == 0)
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

void MultiplexedComms::send_data_blocking(uint8_t port, uint8_t* data, uint8_t data_length) {

	// flag that we wish to transmit data
	_wish_to_transmit_port = port;
	_wish_to_transmit = true;

	// spin until we have stopped receiving or we are receiving but
	// on the correct port
	while(_receiving && (_current_port != port));

	if(_current_port != port)
		set_current_port(port);

	for (int i = 0; i < data_length; i++) {
		//_delay_ms(USART_SEND_DELAY_MS);
		_usart->send_blocking(data[i]);
	}
	_wish_to_transmit = false;
}



void MultiplexedComms::timer_tick(void) {
	if (_receiving) {
		_rx_timeout_timer++;
		if (_rx_timeout_timer >= 10) {
			finish_rx();
		}
	}
}

void MultiplexedComms::init(void (*rx_packet_callback)(volatile uint8_t* rx_packet, uint8_t rx_packet_length), void (*enable_incoming_data_interrupts_func)(void), void (*disable_incoming_data_interrupts_func)(void)) {
	_rx_packet_callback = rx_packet_callback;
	_enable_incoming_data_interrupts_func = enable_incoming_data_interrupts_func;
	_disable_incoming_data_interrupts_func = disable_incoming_data_interrupts_func;
	_enable_incoming_data_interrupts_func();
}

bool MultiplexedComms::snoop_port(uint8_t port) {
	return (*_port_snoop_pins[port] & (1<<_port_snoop_pinnos[port]));
}
