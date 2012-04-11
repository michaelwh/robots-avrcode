//#ifndef F_CPU
//	#define F_CPU 16000000UL
//#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdlib.h>

#include  "pinutil.hpp"
#include "serialcomms.hpp"
#include "timer.hpp"

#define DEBUG_SENDER

USART USART0(&UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UCSR0C, &UDR0, UDRE0, U2X0);

const uint8_t num_ports = 2;
volatile uint8_t * const port_snoop_pins[] = { &PINB, &PINB };
const uint8_t port_snoop_pinsnos[] = { 0, 1 };
volatile bool prev_pinchange_values[num_ports];

MultiplexedComms multiplexedComms(&USART0, num_ports, port_snoop_pins, port_snoop_pinsnos);





/* Temp testing variables */
volatile bool send_test_bytes = false;
volatile uint8_t* test_bytes;
volatile uint8_t test_bytes_len = 0;
volatile uint8_t test_bytes_port = 0;
volatile uint16_t timer_test_counter = 0;
//uint16_t test_start_timer_val = 0;
//volatile uint16_t test_num_ms_elapsed = 0;
/* End temp testing variables */

ISR(PCINT0_vect) {
	/* Pin change interrupt issued when a change is detected on any of masked PCINT0 pins */
	// check each port to determine which triggered the pinchange interrupt
	// might need to store previous state of each pin and compare to determine which one caused
	// interrupt

//	SET_BIT(PORTC, 1);
//	CLR_BIT(PORTC, 1);
//	_delay_us(100);
//	SET_BIT(PORTC, 1);

	int port = -1;
	for (uint8_t port_i = 0; port_i < num_ports; port_i++) {
		if (CHECK_BIT(*port_snoop_pins[port_i], port_snoop_pinsnos[port_i])) {
			prev_pinchange_values[port_i] = true;
		} else {
			if(prev_pinchange_values[port_i] == true) {
				port = port_i;
			}
			prev_pinchange_values[port_i] = false;
		}
	}
	if (port != -1) {
		multiplexedComms.incoming_data_blocking(port);
	}
}


ISR(USART0_RX_vect) {
	/* interrupt that fires whenever a data byte is received over serial */
	uint8_t rx_byte = UDR0;
	multiplexedComms.rx_byte(rx_byte);
}

ISR(TIMER1_COMPA_vect) {
	//SET_BIT(PORTC, 1);
	//CLR_BIT(PORTC, 1);
	multiplexedComms.timer_ms_tick();

	//test_num_ms_elapsed++;
	//SET_BIT(PORTC, 1);
}

void setup_pinchange_interrupts(void) {
	// configure as inputs
	CLR_BIT(DDRB, 0);
	CLR_BIT(DDRB, 1);

	// activate internal pull-ups
	//SET_BIT(DDRB, 0);
	//SET_BIT(DDRB, 1);

	// enable interrupts on these pins
	PCMSK0 = (1<<0)|(1<<1);
}

void pinchange_interrupts_enable(void) {
	PCIFR |= (1<<PCIF0);
	PCICR |= (1<<PCIE0);
	for (uint8_t port_i = 0; port_i < num_ports; port_i++) {
		if(CHECK_BIT(*port_snoop_pins[port_i], port_snoop_pinsnos[port_i])) {
			prev_pinchange_values[port_i] = true;
		} else {
			prev_pinchange_values[port_i] = false;
		}
	}

}

void pinchange_interrupts_disable(void) {
	PCICR &= ~(1<<PCIE0);
}

void setup_mux(void) {
	SET_BIT(DDRC, 4);
	CLR_BIT(PORTC, 4);
}

void set_mux_port(uint8_t port) {
//	SET_BIT(PORTC, 1);
//	CLR_BIT(PORTC, 1);
//	_delay_us(10);
//	SET_BIT(PORTC, 1);
	switch (port) {
		case 0:
			CLR_BIT(PORTC, 4);
			break;
		case 1:
			SET_BIT(PORTC, 4);
			break;
		default:
			break;
	}
}

void rx_packet_callback_func(uint8_t rx_port, volatile uint8_t* rx_packet, uint8_t rx_packet_length) {
	//I will have it from length onwards (Excluding length)
	send_test_bytes = true;
	test_bytes_port = rx_port;
	test_bytes_len = rx_packet_length + 2;
	test_bytes = (uint8_t*)malloc(test_bytes_len * sizeof(uint8_t));
	test_bytes[0] = 0x00;
	test_bytes[1] = rx_packet_length;
	for (int i = 2; i < test_bytes_len; i++)
		test_bytes[i] = rx_packet[i-2];
}

int main(void) {
	cli();
	SET_BIT(DDRC, 0);
	SET_BIT(PORTC, 0);
	SET_BIT(DDRC, 1);
	SET_BIT(PORTC, 1);
	SET_BIT(DDRC, 2);
	SET_BIT(PORTC, 2);
	SET_BIT(DDRC, 3);
	SET_BIT(PORTC, 3);
	for(int i = 0; i < 5; i++) {
		SET_BIT(PORTC, 0);
		CLR_BIT(PORTC, 0);
		SET_BIT(PORTC, 0);
	}

	setup_pinchange_interrupts();
	setup_mux();
	USART0.init();
	multiplexedComms.init(rx_packet_callback_func, pinchange_interrupts_enable, pinchange_interrupts_disable, set_mux_port);
	millisecond_timer_enable();
	sei();

//	while(false) {
//		uint16_t current_timer_val;
//		uint16_t current_ticks;
//		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
//			current_timer_val = get_current_ms_timer_value();
//			current_ticks = test_num_ms_elapsed;
//		}
//
//		if(get_us_elapsed(test_start_timer_val, current_timer_val, test_num_ms_elapsed) >= 900) {
//			test_num_ms_elapsed = 0;
//			test_start_timer_val = current_timer_val;
//			SET_BIT(PORTC, 3);
//			CLR_BIT(PORTC, 3);
//			SET_BIT(PORTC, 3);
//		}
//	}

	while(true) {
#ifdef DEBUG_SENDER
		uint8_t t_bytes[] = { 0x00, 4, 12, 13, 14, 15 };
		multiplexedComms.send_data_blocking(0, t_bytes, 6);
		_delay_ms(100);
#endif
		if (send_test_bytes) {
			send_test_bytes = false;
			//uint8_t data_to_send[] = { 0x00, 0x01, 0x03 };

			multiplexedComms.send_data_blocking(test_bytes_port, (uint8_t*)test_bytes, test_bytes_len);
			free((void*)test_bytes);
		}
	}

}
