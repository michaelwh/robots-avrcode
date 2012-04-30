#include <avr/io.h>
#include <inttypes.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/delay.h>


#include "serialcomms.hpp"
#include "pinutil.hpp"
#include "timer.hpp"
#include "debug.hpp"
#include "util.hpp"

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

void USART::init_9600() {

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

void USART::init_38400() {

	/* Set baud rate */
	//unsigned int ubrr = F_CPU/16/baud-1;
	#define BAUD 38400
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

void USART::init_76800() {

	/* Set baud rate */
	//unsigned int ubrr = F_CPU/16/baud-1;
	#define BAUD 76800
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
	while ( !( *_UCSRnA & (1<<_UDREn)) );
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


/* :param num_ports: the number of 'ports' this usart will be connected to
 * :param port_snoop_pins_in: a list of pointers to the PIN registers for each of the snoop pins,
 *                            the format of this list is: [snoop_PIN_0][orientation_PIN_0][snoop_PIN_1][orientation_PIN_1]... */
MultiplexedComms::MultiplexedComms(USART* usart, uint8_t num_ports, volatile uint8_t * const * port_snoop_pins_in, const uint8_t* port_snoop_pinnos_in, volatile uint8_t * const * port_snoop_orientation_pins_in, const uint8_t* port_snoop_orientation_pinnos_in) {
	_usart = usart;
	_num_ports = num_ports;
	_port_snoop_pins = port_snoop_pins_in;
	_port_snoop_pinnos = port_snoop_pinnos_in;
	_port_snoop_orientation_pins = port_snoop_orientation_pins_in;
	_port_snoop_orientation_pinnos = port_snoop_orientation_pinnos_in;
	_rx_state = RX_IDLE;
	_wish_to_transmit = false;
	_locked_to_port = false;
}

void MultiplexedComms::set_current_port(uint8_t port) {
	// not implemented yet
	// change the MUX port here
	_current_port = port;
	_set_mux_port_func(port);
}

void MultiplexedComms::incoming_data_blocking(uint8_t port) {

	/* Some incoming data has been detected on one of the
	 * ports */

	/* Check to see if we are transmitting or receiving, if
	 * we are then ignore the new data, if not then wait until
	 * the pin goes back to normal and start receiving it */
	if (_rx_state == RX_IDLE && (!_wish_to_transmit || port == _wish_to_transmit_port) && (!_locked_to_port || port == _current_port)) {
		// we want falling edge
		if(!CHECK_BIT(*_port_snoop_pins[port], _port_snoop_pinnos[port])){
			// it should be all zeros, meaning it should last about 930us
			// so sample once every 10us and check to see if is zero for
			// all that time
			bool errorflag = false;
			/*for(int i = 0; i < 70; i++){
				//SET_BIT(PORTC, 1);
				//CLR_BIT(PORTC, 1);
				//SET_BIT(PORTC, 1);
				if(CHECK_BIT(*_port_snoop_pins[port], _port_snoop_pinnos[port])) {
					errorflag = true;
					break;
				}
				_delay_us(10);
			}*/
			if (!errorflag) {
				// no errors, so continue
				// wait until end of start byte
				//while(!CHECK_BIT(*_port_snoop_pins[port], _port_snoop_pinnos[port]));

				// inform the multiplexed comms module we have
				// incoming data on this port
				if(_current_port != port)
					set_current_port(port);
				start_rx();
			}


		}

	}
}

//void MultiplexedComms::imcoming_data_async(uint8_t port) {
//	if (_rx_state == RX_IDLE && (!_wish_to_transmit || port == _wish_to_transmit_port)) {
//			// we want falling edge
//			if(!CHECK_BIT(*_port_snoop_pins[port], _port_snoop_pinnos[port])){
//				_rx_state = POSSIBLE_RX_DETECTED;
//				_possible_rx_ms_counter = 0;
//				_possible_rx_timer_last_val = get_current_ms_timer_value();
//			}
//	}
//}


void MultiplexedComms::start_rx(void) {
	_disable_incoming_data_interrupts_func();
	_current_rx_packet.have_packet_length = false;
	_current_rx_packet.packet_length = 0;
	_current_rx_packet.current_rx_byte_index = 0;
	_rx_done = false;
	_rx_state = RX_ACTIVE;
	_rx_timeout_timer = 0;
	_usart->enable_rx();
}

void MultiplexedComms::rx_byte(uint8_t byte_in) {
	/* Called when a byte is received over the serial line */

	if(_rx_state == RX_ACTIVE) {
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
	_rx_state = RX_IDLE;
	_rx_timeout_timer = 0;

	if(_current_rx_packet.have_packet_length && _current_rx_packet.current_rx_byte_index >= _current_rx_packet.packet_length) {
		_rx_done = true;
		if (_rx_packet_callback != NULL)
			_rx_packet_callback(_current_port, _current_rx_packet.received_packet, _current_rx_packet.packet_length);
	}


	_enable_incoming_data_interrupts_func();
}

void MultiplexedComms::send_data_blocking(uint8_t port, uint8_t* data, uint8_t data_length) {
	/* Will send data over the USART to the appropriate port. Will also take care of sending
	 * the start byte and length byte. */

	// flag that we wish to transmit data
	_wish_to_transmit_port = port;
	_wish_to_transmit = true;

	// spin until we have stopped receiving or we are receiving but
	// on the correct port
	while(_rx_state != RX_IDLE && (_current_port != port));

	if(_current_port != port)
		set_current_port(port);


	// send the start byte
	_usart->send_blocking(0x00);

	_delay_us(350);

	// send the data length
	_usart->send_blocking(data_length);


	// send the data
	for (int i = 0; i < data_length; i++) {
		//_delay_ms(USART_SEND_DELAY_MS);
		_usart->send_blocking(data[i]);
	}
	_wish_to_transmit = false;
}



void MultiplexedComms::timer_ms_tick(void) {
	if (_rx_state == RX_ACTIVE) {
		_rx_timeout_timer++;
		if (_rx_timeout_timer >= 10) {
			finish_rx();
		}
	}
}

void MultiplexedComms::init(void (*rx_packet_callback)(uint8_t rx_port, volatile uint8_t* rx_packet, uint8_t rx_packet_length), void (*enable_incoming_data_interrupts_func)(void), void (*disable_incoming_data_interrupts_func)(void), void (*set_mux_port_in)(uint8_t)) {
	_rx_packet_callback = rx_packet_callback;
	_enable_incoming_data_interrupts_func = enable_incoming_data_interrupts_func;
	_disable_incoming_data_interrupts_func = disable_incoming_data_interrupts_func;
	_set_mux_port_func = set_mux_port_in;
	_enable_incoming_data_interrupts_func();
}

uint8_t MultiplexedComms::get_num_ports(void) {
	return _num_ports;
}

bool MultiplexedComms::snoop_port(uint8_t port, bool orientation) {
	if(orientation) {
		return (*_port_snoop_orientation_pins[port] & (1<<_port_snoop_orientation_pinnos[port]));
	} else {
		return (*_port_snoop_pins[port] & (1<<_port_snoop_pinnos[port]));
	}
}

void MultiplexedComms::lock_to_port(uint8_t port) {

	while(_rx_state != RX_IDLE || _locked_to_port);

	_locked_to_port = true;

	if(_current_port != port)
		set_current_port(port);
}

void MultiplexedComms::unlock_from_port(void) {
	_locked_to_port = false;
}

// ----------- PACKET METHODS

Packet::Packet(uint8_t* data_in, uint8_t data_length_in) {
	data = data_in;
	data_length = data_length_in;
}

Packet::Packet() {
 data_length = 0;
}


uint8_t Packet::get_command(void) {
	if(is_ack())
		return data[1]; // TODO: Change this to appropriate data
	else
		return data[1];
}


// ------------ END PACKET METHODS

// ------------ FLAGS METHODS


bool Packet::is_network(void) {
	return CHECK_BIT(data[0], 0);
}

bool Packet::requires_ack(void) {
	if(data_length >= 1)
		return CHECK_BIT(data[0], 1);
	else
		return false;
}

bool Packet::requires_global_ack(void) {
	if(data_length >= 1)
		return CHECK_BIT(data[0], 2);
	else
		return false;
}

bool Packet::is_ack(void) {
	if(data_length >= 1)
		return CHECK_BIT(data[0], 3);
	else
		return false;
}

bool Packet::is_global_ack(void) {
	if(data_length >= 1)
		return CHECK_BIT(data[0], 4);
	else
		return false;
}

void Packet::set_is_network(bool is_network) {
	if(is_network)
		SET_BIT(data[0], 0);
	else
		CLR_BIT(data[0], 0);
}

void Packet::set_requires_ack(bool requires_ack) {
	if(requires_ack)
		SET_BIT(data[0], 1);
	else
		CLR_BIT(data[0], 1);
}

void Packet::set_requires_global_ack(bool requires_global_ack) {
	if(requires_global_ack)
		SET_BIT(data[0], 2);
	else
		CLR_BIT(data[0], 2);
}

void Packet::set_is_ack(bool is_ack) {
	if(is_ack)
		SET_BIT(data[0], 3);
	else
		CLR_BIT(data[0], 3);
}

void Packet::set_is_global_ack(bool is_global_ack) {
	if(is_global_ack)
		SET_BIT(data[0], 4);
	else
		CLR_BIT(data[0], 4);
}

uint8_t Packet::make_packet_flags(bool is_network, bool requires_ack, bool requires_global_ack, bool is_ack, bool is_global_ack) {

	uint8_t packet_flags = 0x00;
	if(is_network)
		SET_BIT(packet_flags, 0);

	if(requires_ack)
			SET_BIT(packet_flags, 1);

	if(requires_global_ack)
			SET_BIT(packet_flags, 2);

	if(is_ack)
			SET_BIT(packet_flags, 3);

	if(is_global_ack)
			SET_BIT(packet_flags, 4);

	return packet_flags;
}

void Packet::copy_from_buffer(uint8_t* buffer, uint8_t buffer_length) {
	data_length = buffer_length;
	for(uint8_t i = 0; i < buffer_length; i++){
		data[i] = buffer[i];
	}
}

// ----------- END FLAGS METHODS

ReliableComms::ReliableComms(MultiplexedComms* mux_comms_in) {
	_mux_comms = mux_comms_in;
}

comms_status_t ReliableComms::send_packet(uint8_t port, Packet* packet, const uint8_t max_retries) {
	// check to see if something is connected to this port (is pulled high)
	if(is_port_connected(port)) {
		// if something is connected then attempt to transmit on this port
		//dbgprintf(">Something is connected to port\n");
		// check to see if we need an ACK
		if(!packet->is_ack())
			_delay_us(500);

		if (packet->requires_ack()) {

			//dbgprintf(">ACK required\n");
			waiting_for_ack = true;
			got_ack_flag = false;
			//dbgprintf(">Locking to port\n");
			//_mux_comms->lock_to_port(port);
			//dbgprintf(">Locked to port\n");
			for (uint8_t retry = 0; retry < max_retries; retry++) {
				//dbgprintf(">Sending (retry %d of %d)\n", retry, max_retries);
				_delay_us(randWithinRange(PRESEND_RANDOM_DELAY_US_MIN, PRESEND_RANDOM_DELAY_US_MAX));
				_mux_comms->send_data_blocking(port, packet->data, packet->data_length);
				//dbgprintf(">Sent (retry %d of %d)\n", retry, max_retries);
				// keep checking timer values and wait until we get an ACK or timeout
				for (uint8_t delay_i = 0; delay_i < 100; delay_i++) {
					if(got_ack_flag) {
						//dbgprintf(">Got ack flag!\n");
						break;
					}
					_delay_us(80);
				}
				if(got_ack_flag)
					break;
			}
			waiting_for_ack = false;
			//_mux_comms->unlock_from_port();


			if(got_ack_flag) {
				got_ack_flag = false;
				return COMMS_SUCCESS;
			}
			else
				//dbgprintf(">Timeout!");
				return COMMS_ERROR_TIMEOUT;
		} else {
			_mux_comms->send_data_blocking(port, packet->data, packet->data_length);
			return COMMS_SUCCESS;
		}
	} else {
		//dbgprintf(">Nothing connected to port %u\n", port);
		return COMMS_ERROR_NOTCONNECTED;
	}
}

bool ReliableComms::is_port_connected(uint8_t port) {
/* Returns true if the port is connected, false otherwise. */
	return _mux_comms->snoop_port(port, false);
}

bool ReliableComms::get_port_orientation(uint8_t port) {
	return _mux_comms->snoop_port(port, true);
}

void ReliableComms::rx_ack(uint8_t port, Packet* packet) {
	if(waiting_for_ack) {
		got_ack_flag = true;
	}
}

