#ifndef F_CPU
	#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "serialcomms.hpp"


USART USART0(&UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UCSR0C, &UDR0, UDRE0);

volatile uint8_t* port_snoop_ports[] = { &PORTB };
volatile uint8_t port_snoop_pins[] = { 0 };


MultiplexedComms multiplexedComms(&USART0, 1, port_snoop_ports, port_snoop_pins);

volatile uint16_t timer_test_counter = 0;

ISR(PCINT0_vect) {
	/* Pin change interrupt issued when a change is detected on any of masked PCINT0 pins */
	multiplexedComms.incoming_data(0);
}

ISR(TIMER0_OVF_vect) {
	/* currently called ~50 times a second */
	multiplexedComms.timer_tick();
}

ISR(USART_RX_vect) {
	/* interrupt that fires whenever a data byte is received over serial */
	uint8_t rx_byte = UDR0;
	multiplexedComms.rx_byte(rx_byte);
}

void setup_pinchange_interrupts(void) {
	PCMSK0 = (1<<0);
}

void pinchange_interrupts_enable(void) {
	PCICR |= (1<<PCIE0);

}

void pinchange_interrupts_disable(void) {
	PCICR &= ~(1<<PCIE0);
}

void timer_setup() {
	/* set the 8 bit timer to overflow mode with a a prescaler of 1024
	 * meaning that it will trigger the overflow interrupt ~50 times
	 * a second */

	// set internal clock, prescaler of 1024
	TCCR0B = (1<<CS02)|(1<<CS00);

	// enable timer overflow interrupt
	TIMSK0 = (1<<TOIE0);

}

void rx_packet_callback_func(uint8_t* rx_packet, uint8_t rx_packet_length) {
	uint8_t data_to_send[] = { 0x41 };
	multiplexedComms.send_data(0, data_to_send, 1);
}

int main(void) {
	cli();
	DDRB = 0x00;
	DDRD = 0xFF;
	timer_setup();
	setup_pinchange_interrupts();
	USART0.init(9600);
	multiplexedComms.init(rx_packet_callback_func, pinchange_interrupts_enable, pinchange_interrupts_disable);
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
	uint8_t data_to_send[] = { 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48 };
	while(false) {
		multiplexedComms.send_data(0, data_to_send, 8);
		_delay_ms(100);
	}
	while(true) {
	}

}
