//#ifndef F_CPU
//	#define F_CPU 8000000UL
//#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdlib.h>

#include  "pinutil.hpp"
#include "serialcomms.hpp"
#include "timer.hpp"

USART USART0(&UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UCSR0C, &UDR0, UDRE0, U2X0);

volatile uint8_t * const port_snoop_pins[] = { &PINB };
const uint8_t port_snoop_pinsnos[] = { 0 };

MultiplexedComms multiplexedComms(&USART0, 1, port_snoop_pins, port_snoop_pinsnos);

volatile uint16_t timer_test_counter = 0;

/* Temp testing variables */
volatile bool send_test_bytes = false;
volatile uint8_t* test_bytes;
volatile uint8_t test_bytes_len = 0;
//uint16_t test_start_timer_val = 0;
//volatile uint16_t test_num_ms_elapsed = 0;
/* End temp testing variables */

ISR(PCINT0_vect) {
	/* Pin change interrupt issued when a change is detected on any of masked PCINT0 pins */
	multiplexedComms.incoming_data_blocking(0);
}


ISR(USART_RX_vect) {
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
	CLR_BIT(DDRB, 0);
	PCMSK0 = (1<<0);
}

void pinchange_interrupts_enable(void) {
	PCICR |= (1<<PCIE0);
}

void pinchange_interrupts_disable(void) {
	PCICR &= ~(1<<PCIE0);
}

void setup_mux(void) {

}

void set_mux_port(uint8_t port) {
	uint8_t port_sel = port;
}

void rx_packet_callback_func(volatile uint8_t* rx_packet, uint8_t rx_packet_length) {
	send_test_bytes = true;

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
	while(false) {
		USART0.send_blocking(0x41);
		_delay_ms(USART_SEND_DELAY_MS);
		for (uint8_t i = 1; i < 8; i++) {
			USART0.send_blocking(0x42 + i);
			_delay_ms(USART_SEND_DELAY_MS);
		}
		_delay_ms(100);
	}
	uint8_t data_to_send[] = { 0x00, 0x01, 0x03 };
	while(false) {
		multiplexedComms.send_data_blocking(0, data_to_send, 3);
		_delay_ms(100);
	}
	while(false) {
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

	}

	while(true) {
		if (send_test_bytes) {
			send_test_bytes = false;
			//uint8_t data_to_send[] = { 0x00, 0x01, 0x03 };

			multiplexedComms.send_data_blocking(0, (uint8_t*)test_bytes, test_bytes_len);
			free((void*)test_bytes);
		}
	}

}
