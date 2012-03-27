//#ifndef F_CPU
//	#define F_CPU 8000000UL
//#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

#include  "pinutil.hpp"
#include "serialcomms.hpp"


USART USART0(&UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UCSR0C, &UDR0, UDRE0, U2X0);

volatile uint8_t * const port_snoop_pins[] = { &PINB };
const uint8_t port_snoop_pinsnos[] = { 0 };



MultiplexedComms multiplexedComms(&USART0, 1, port_snoop_pins, port_snoop_pinsnos);

volatile uint16_t timer_test_counter = 0;

/* Temp testing variables */
volatile bool send_test_bytes = false;
volatile uint8_t* test_bytes;
volatile uint8_t test_bytes_len = 0;
/* End temp testing variables */

ISR(PCINT0_vect) {
	/* Pin change interrupt issued when a change is detected on any of masked PCINT0 pins */
	multiplexedComms.incoming_data(0);
}

ISR(TIMER0_OVF_vect) {
	//SET_BIT(PORTC, 1);
	///CLR_BIT(PORTC, 1);
	//SET_BIT(PORTC, 1);
	multiplexedComms.timer_tick();
}

ISR(USART_RX_vect) {
	/* interrupt that fires whenever a data byte is received over serial */
	uint8_t rx_byte = UDR0;
	multiplexedComms.rx_byte(rx_byte);
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

void timer_setup() {
	/* set the 8 bit timer to overflow mode with a a prescaler of 1024
	 * meaning that it will trigger the overflow interrupt ~50 times
	 * a second */

	// set internal clock, prescaler of 1024
	TCCR0B = (1<<CS01)|(1<<CS00);

	// enable timer overflow interrupt
	TIMSK0 = (1<<TOIE0);

}

void rx_packet_callback_func(volatile uint8_t* rx_packet, uint8_t rx_packet_length) {
	send_test_bytes = true;

	test_bytes_len = rx_packet_length;
	test_bytes = (uint8_t*)malloc(test_bytes_len * sizeof(uint8_t));
	for (int i = 0; i < test_bytes_len; i++)
		test_bytes[i] = rx_packet[i];


//	SET_BIT(PORTC, 1);
//	CLR_BIT(PORTC, 1);
//	SET_BIT(PORTC, 1);
}

int main(void) {
	cli();
	SET_BIT(DDRC, 0);
	SET_BIT(PORTC, 0);
	SET_BIT(DDRC, 1);
	SET_BIT(PORTC, 1);
	for(int i = 0; i < 5; i++) {
		SET_BIT(PORTC, 0);
		CLR_BIT(PORTC, 0);
		SET_BIT(PORTC, 0);
	}

	timer_setup();
	setup_pinchange_interrupts();
	USART0.init();
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
	uint8_t data_to_send[] = { 0x00, 0x01, 0x03 };
	while(false) {
		multiplexedComms.send_data_blocking(0, data_to_send, 3);
		_delay_ms(100);
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
