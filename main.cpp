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
#include "movement.hpp"
#include "flags.hpp"

USART USART0(&UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UCSR0C, &UDR0, UDRE0, U2X0);



const uint8_t num_ports = 6;
// PCB module which uses different pins for snoop and orientation
// so we assume it has ID 0 and assign it different pins
volatile uint8_t * const port_snoop_pins[] = { &PINB, &PIND, &PINB, &PINB, &PINB, &PINB };
const uint8_t port_snoop_pinsnos[] = { 2, 6, 3, 4, 0, 1 };

volatile uint8_t * const port_snoop_orientation_pins[] = { &PINC, &PINC, &PINC, &PINC, &PINC, &PINC };
const uint8_t port_snoop_orientation_pinsnos[] = { 5, 4, 2, 3, 7, 6 };


volatile uint16_t millisecond_counter = 0;

volatile bool send_wiggle_after_delay = false;

volatile bool prev_pinchange_values[num_ports];
//volatile bool prev_pinchange_orientation_values[num_ports];

MultiplexedComms multiplexedComms(&USART0, num_ports, port_snoop_pins, port_snoop_pinsnos, port_snoop_orientation_pins, port_snoop_orientation_pinsnos);

Controller controller_pc;

ReliableComms reliable_comms(&multiplexedComms);

PWM pwm_move;

Packet packet_queue_buffer[5];
uint8_t port_queue_buffer[5];
PacketRingBuffer queue(MAX_PACKET_STORED, packet_queue_buffer, port_queue_buffer);

/*Byte queues for the received packets*/
ByteRingBuffer packets_id_received(MAX_NETWORK_PACKET_STORED);
ByteRingBuffer packets_source_received(MAX_NETWORK_PACKET_STORED);
ByteRingBuffer packets_destination_received(MAX_NETWORK_PACKET_STORED);

Movement movement(&pwm_move);

COMMAND cmd(&reliable_comms, &queue , &packets_id_received, &packets_source_received, &packets_destination_received, &pwm_move, &movement);



/* Temp testing variables */
volatile bool send_test_bytes = false;
volatile uint8_t* test_bytes;
volatile uint8_t test_bytes_len = 0;
volatile uint8_t test_bytes_port = 0;
volatile uint16_t timer_test_counter = 0;
/* End temp testing variables */


//CounterTimer three_second_counter_timer(&millisecond_counter);
//CounterTimer send_wiggle_timer;
//CounterTimer wiggle_timer;
//CounterTimer any_connected_timer;

//volatile uint16_t three_second_ms_counter = 0;
//volatile uint16_t one_second_ms_counter = 0;

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

ISR(USART1_RX_vect) {
	/* interrupt that fires whenever a data byte is received over serial 1 */
	uint8_t rx_byte = UDR1;
	controller_pc.rx1_byte(rx_byte);

}

uint8_t ms_pwm_count = 0;

ISR(TIMER0_COMPA_vect) {

	//CLR_BIT(PORTC, 1);
	//_delay_us(200);
	multiplexedComms.timer_ms_tick();
	controller_pc.timer_ms_tick();

	//SET_BIT(PORTC, 1);
	//test_num_ms_elapsed++;
	//SET_BIT(PORTC, 1);

//	timer_test++;
//	if (timer_test >= 500) {
//		timer_test = 0;
//		if(cmd._ID == 0) {
//			dbgprintf("Requesting ID\n");
//			dbgprintf("Returned: %d\n", cmd.request_id(1));
//		}
//	}
	//three_second_ms_counter++;
	//one_second_ms_counter++;
	millisecond_counter++;

	//ms_pwm_count++;
	//if(ms_pwm_count >= 50) {
	//	pwm_move.PWMStep();
	//	ms_pwm_count = 0;
	//}


	//ms_counter++;

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
	//print_packet(&packet);
	if(packet.is_ack()) {
		//dbgprintf("Packet is ack\n");
		reliable_comms.rx_ack(rx_port, &packet);
	} else {
		//If the port is the computer then assume that the packets have been sent by this module
		if(rx_port == 255) {
			//Change ID of teh packet to this one
			rx_packet[2] = cmd._ID;
		}
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



bool any_connected = false;


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
	//PWM::init();
	init_debug();
	dbgprintf("Starting up..\n");
	setup_pinchange_interrupts();
	setup_mux();
	USART0.init_76800();
	controller_pc.init(rx_packet_callback_func);
	multiplexedComms.init(rx_packet_callback_func, pinchange_interrupts_enable, pinchange_interrupts_disable, set_mux_port);
	millisecond_timer_enable();
	//one_second_counter_timer.reset();
	//three_second_counter_timer.reset();
	dbgprintf("Finished starting up...");
	sei();




	//wiggle_timer.reset();
	//any_connected_timer.reset();

	movement.move_forward();

	while(true) {
		movement.movement_step();
		cmd.command_update();


		/*if(any_connected_timer.has_elapsed(2500)) {
			for (uint8_t port_i = 0; port_i < MAX_BLOCKS_CONNECTED; port_i++) {
				if (reliable_comms.is_port_connected(port_i))
					any_connected = true;
			}

			if (!any_connected)
				movement.move_forward();

			any_connected_timer.reset();
		}




		if(send_wiggle_after_delay && wiggle_timer.has_elapsed(800)) {
			dbgprintf("Sending pulse!\n");
			for (uint8_t port_i = 0; port_i < MAX_BLOCKS_CONNECTED; port_i++)
				cmd.send_pulse(port_i, 100);
			send_wiggle_after_delay = false;
			wiggle_timer.reset();
		}*/


#if MODULE_ID == 0
		//if (any_connected) {
		//	if(!send_wiggle_after_delay && wiggle_timer.has_elapsed(2000)) {
		//		movement.move_wiggle();
		//		send_wiggle_after_delay = true;
		//		wiggle_timer.reset();
		//	}
		//}
#endif


	}

#if 0
	for (uint8_t i = 0; i < 20; i++) {
				/*PWM::BottomServoMove(value);*/
				//one_second_ms_counter = 0;
			//cmd.move_forward();
		pwm_move.TopServoMove(value);
		_delay_ms(200);
		pwm_move.BottomServoMove(value);
		_delay_ms(200);
		pwm_move.TopServoMove(1000);
		}
#endif

#if 0
	for (uint8_t i = 0; i < 20; i++) {
				/*PWM::BottomServoMove(value);*/
				//one_second_ms_counter = 0;
				pwm_move.TopServoMove(value);
				_delay_ms(200);
				pwm_move.BottomServoMove(value);
				_delay_ms(200);
				pwm_move.TopServoMove(PWM_MAX);
				_delay_ms(200);
				pwm_move.BottomServoMove(PWM_MIN);
				_delay_ms(200);
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


