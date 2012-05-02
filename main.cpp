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
#include "config.hpp"
#include "pwm.hpp"
#include "debug.hpp"
#include "commands.hpp"
#include "util.hpp"

USART USART0(&UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UCSR0C, &UDR0, UDRE0, U2X0);



const uint8_t num_ports = 6;
volatile uint8_t * const port_snoop_pins[] = { &PINB, &PINB, &PINB, &PINB, &PINB, &PIND };
const uint8_t port_snoop_pinsnos[] = { 0, 1, 2, 3, 4, 6 };

volatile uint8_t * const port_snoop_orientation_pins[] = { &PINC, &PINC, &PINC, &PINC, &PINC, &PINC };
const uint8_t port_snoop_orientation_pinsnos[] = { 2, 3, 4, 5, 6, 7 };

volatile bool prev_pinchange_values[num_ports];
//volatile bool prev_pinchange_orientation_values[num_ports];

MultiplexedComms multiplexedComms(&USART0, num_ports, port_snoop_pins, port_snoop_pinsnos, port_snoop_orientation_pins, port_snoop_orientation_pinsnos);

ReliableComms reliable_comms(&multiplexedComms);

Packet packet_queue_buffer[5];
uint8_t port_queue_buffer[5];
PacketRingBuffer queue(MAX_PACKET_STORED, packet_queue_buffer, port_queue_buffer);

/*Byte queues for the received packets*/
ByteRingBuffer packets_id_received(MAX_NETWORK_PACKET_STORED);
ByteRingBuffer packets_source_received(MAX_NETWORK_PACKET_STORED);
ByteRingBuffer packets_destination_received(MAX_NETWORK_PACKET_STORED);

COMMAND cmd(&reliable_comms, &queue , &packets_id_received, &packets_source_received, &packets_destination_received);


/* Temp testing variables */
volatile bool send_test_bytes = false;
volatile uint8_t* test_bytes;
volatile uint8_t test_bytes_len = 0;
volatile uint8_t test_bytes_port = 0;
volatile uint16_t timer_test_counter = 0;
/* End temp testing variables */

volatile uint16_t ms_counter = 0;

void pinchange_interrupt(void) {

	/* Pin change interrupt issued when a change is detected on any of masked PCINT0 pins */
	// check each port to determine which triggered the pinchange interrupt
	// might need to store previous state of each pin and compare to determine which one caused
	// interrupt

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
		//SET_BIT(PORTC, 0);
		//CLR_BIT(PORTC, 0);
		//SET_BIT(PORTC, 0);
		multiplexedComms.incoming_data_blocking(port);
	}
}
ISR(PCINT1_vect) {
	pinchange_interrupt();
}

ISR(PCINT3_vect) {
	pinchange_interrupt();
}

ISR(USART0_RX_vect) {
	/* interrupt that fires whenever a data byte is received over serial */
	uint8_t rx_byte = UDR0;
	multiplexedComms.rx_byte(rx_byte);
}


ISR(TIMER0_COMPA_vect) {

	//CLR_BIT(PORTC, 1);
	//_delay_us(200);
	multiplexedComms.timer_ms_tick();
	ms_counter++;
}

void setup_pinchange_interrupts(void) {
	// configure as inputs
	CLR_BIT(DDRB, 0);
	CLR_BIT(DDRB, 1);
	CLR_BIT(DDRB, 2);
	CLR_BIT(DDRB, 3);
	CLR_BIT(DDRB, 4);
	CLR_BIT(DDRD, 6);

	// interrupt mask on these pins
	PCMSK1 |= 0x1F;
	PCMSK3 |= (1<<PCINT30);
}

void pinchange_interrupts_enable(void) {
	PCIFR |= (1<<PCIF1)|(1<<PCIF3);
	PCICR |= (1<<PCIE1)|(1<<PCIE3);
	for (uint8_t port_i = 0; port_i < num_ports; port_i++) {
		if(CHECK_BIT(*port_snoop_pins[port_i], port_snoop_pinsnos[port_i])) {
			prev_pinchange_values[port_i] = true;
		} else {
			prev_pinchange_values[port_i] = false;
		}
	}
	/*for (uint8_t port_i = 0; port_i < num_ports; port_i++) {
		if(CHECK_BIT(*port_snoop_pins[port_i], port_snoop_pinsnos[port_i])) {
			prev_pinchange_orientation_values[port_i] = true;
		} else {
			prev_pinchange_orientation_values[port_i] = false;
		}
	}*/

}

void pinchange_interrupts_disable(void) {
	PCICR &= ~((1<<PCIE1)|(1<<PCIE3));
}

void setup_mux(void) {
	SET_BIT(DDRA, 0);
	CLR_BIT(PORTA, 0);
	SET_BIT(DDRA, 1);
	CLR_BIT(PORTA, 1);
	SET_BIT(DDRA, 2);
	CLR_BIT(PORTA, 2);
}

void set_mux_port(uint8_t port) {
//	SET_BIT(PORTC, 1);
//	CLR_BIT(PORTC, 1);
//	_delay_us(10);
//	SET_BIT(PORTC, 1);
	// least significant three bits of port should map directly onto least significant three bits of PORTA
	//SET_BIT(PORTC, 1);
	//CLR_BIT(PORTC, 1);
	//_delay_us(200);
	//SET_BIT(PORTC, 1);
	PORTA = ((0xF8) & PORTA) | port;
}

void print_packet(Packet* packet) {
	dbgprintf("Packet length: %d, data:", packet->data_length);
	for(int i = 0; i < packet->data_length; i++)
		dbgprintf(" %d", packet->data[i]);
	dbgprintf("\n");
}

void rx_packet_callback_func(uint8_t rx_port, volatile uint8_t* rx_packet, uint8_t rx_packet_length) {
	//I will have it from length onwards (Excluding length)
//	dbgprintf("Got packet from port %d, length is %d, packet:", rx_port, rx_packet_length);
//	for(int i = 0; i < rx_packet_length; i++)
//		dbgprintf(" %d", rx_packet[i]);
//	dbgprintf("\n");
//	SET_BIT(PORTC, 1);
//	CLR_BIT(PORTC, 1);
//	SET_BIT(PORTC, 1);

	Packet packet((uint8_t*)rx_packet, rx_packet_length);
	if(packet.is_ack()) {
		//dbgprintf("Packet is ack\n");
		reliable_comms.rx_ack(rx_port, &packet);
	} else {
		bool appendret = cmd.packet_queue->append((uint8_t*)rx_packet, rx_packet_length, rx_port);

		if(appendret && packet.requires_ack()){
			//send_test_bytes = true;
			//test_bytes_port = rx_port;

			uint8_t data_to_send[] = { Packet::make_packet_flags(false, false, false, true, false), COMMAND_ACK };

			Packet packet(data_to_send, 2);
			reliable_comms.send_packet(rx_port, &packet);
		}

	}





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
	srand(MODULE_ID);
	PWM::init();
	setup_pinchange_interrupts();
	setup_mux();
	USART0.init_76800();
	init_debug();
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


//	Packet queue[5];
//	PacketRingBuffer packetBuff(5, queue);
//
//	uint8_t t_bytes[] = { Packet::make_packet_flags(false, true, false, false, false), 4, 12, 13, 14, 15 };
//	uint8_t t_bytes_2[] = { Packet::make_packet_flags(false, true, false, false, false), 9, 18 };
//
//	packetBuff.append(t_bytes, 6);
//	packetBuff.append(t_bytes_2, 3);
//	packetBuff.append(t_bytes_2, 3);
//	packetBuff.append(t_bytes_2, 3);
//	dbgprintf("Should return true %d", packetBuff.append(t_bytes_2, 3));
//	dbgprintf("Should return false %d", packetBuff.append(t_bytes_2, 3));
//
//	dbgprintf("First\n");
//	print_packet(packetBuff.peek_first());
//	packetBuff.dequeue();
//	dbgprintf("Should return true %d", packetBuff.append(t_bytes, 6));
//	dbgprintf("Second\n");
//	print_packet(packetBuff.peek_first());

//	if(cmd._ID == 0) {
//		dbgprintf("Requesting ID\n");
//		dbgprintf("Returned: %d\n", cmd.request_id(1));
//	}



#if 0
	uint16_t value = PWM_MAX;
	while(true) {


		cmd.command_update();

		if(ms_counter > 3000) {
			// 3 seconds have passed
			ms_counter = 0;

			dbgprintf("Current connected ports:");
			for(uint8_t port = 0; port < MAX_BLOCKS_CONNECTED; port++) {
				dbgprintf(" [Port %u: ", port);
				int conn = cmd.get_block_connected(port);
				if (conn == BLOCK_NOT_CONNECTED)
					dbgprintf("Not connected");
				else if(conn == BLOCK_CONNECTED_NO_RESPONSE)
					dbgprintf("No response");
				else
					dbgprintf("%d", conn);
				dbgprintf("]");
			}
			dbgprintf("\n");

			dbgprintf("Updating connected...\n");
			cmd.update_connected();


			if(value != PWM_MAX)
				value = PWM_MAX;
			else
				value = PWM_MIN;

			PWM::BottomServoMove(value);
		}

#endif
#if 0
		dbgprintf("Making packet\n");
		uint8_t t_bytes[] = { Packet::make_packet_flags(false, true, false, false, false), 4, 12, 13, 14, 15 };

		Packet rx_packet(t_bytes, 6);
		dbgprintf("Packet made\n");
		dbgprintf("Beginning send packet\n");
		uint8_t return_code = reliable_comms.send_packet(1, &rx_packet);
		dbgprintf("Packet sent with return code %d\n", return_code);


		_delay_ms(1000);
#endif
#if 0



#endif
#if 0
		if (send_test_bytes) {
			send_test_bytes = false;
			//dbgprintf("Sending ID\n");

			cmd.return_id(test_bytes_port);
//			uint8_t data_to_send[] = { Packet::make_packet_flags(false, false, false, true, false), COMMAND_ACK };
//
//			Packet packet(data_to_send, 2);
//			reliable_comms.send_packet(test_bytes_port, &packet);
//			//multiplexedComms.send_data_blocking(test_bytes_port, (uint8_t*)test_bytes, test_bytes_len);
			//free((void*)test_bytes);
		}
#endif
	}


}
