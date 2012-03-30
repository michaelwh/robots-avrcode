#ifndef COMMS_HPP_
#define COMMS_HPP_

#include <inttypes.h>

#define USART_SEND_DELAY_MS	10

#define MAX_PACKET_LEN	50

class USART
{
public:
	USART(volatile uint8_t * UBRRnH, volatile uint8_t * UBRRnL, volatile uint8_t * UCSRnA, volatile uint8_t * UCSRnB, volatile uint8_t * UCSRnC, volatile uint8_t * UDRn,  uint8_t UDREn, uint8_t U2Xn);
	void init();
	void send_blocking(uint8_t data);
	void enable_rx(void);
	void disable_rx(void);
private:
	volatile uint8_t * _UBRRnH;
	volatile uint8_t * _UBRRnL;
	volatile uint8_t * _UCSRnA;
	volatile uint8_t * _UCSRnB;
	volatile uint8_t * _UCSRnC;
	volatile uint8_t * _UDRn;
	uint8_t _UDREn;
	uint8_t _U2Xn;

};


struct rx_packet_t {
	bool have_packet_length;
	uint8_t packet_length;
	uint8_t current_rx_byte_index;
	uint8_t received_packet[MAX_PACKET_LEN];
};

typedef enum ReceiveState_t { RX_IDLE, POSSIBLE_RX_DETECTED, RX_ACTIVE } ReceiveState;

class MultiplexedComms
{
public:
	MultiplexedComms(USART* usart, uint8_t num_ports, volatile uint8_t * const * port_snoop_ports_in, const uint8_t* port_snoop_pins_in);
	void init(void (*rx_packet_callback)(volatile uint8_t*, uint8_t), void (*enable_incoming_data_interrupts_func)(void), void (*disable_incoming_data_interrupts_func)(void), void (*set_mux_port_in)(uint8_t));
	void incoming_data_blocking(uint8_t port);
	//void imcoming_data_async(uint8_t port);
	void send_data_blocking(uint8_t port, uint8_t* data, uint8_t data_length);
	void rx_byte(uint8_t byte_in);
	void timer_ms_tick(void);
	volatile uint8_t** send_queues;
private:
	void set_current_port(uint8_t port);
	void start_rx(void);
	void finish_rx(void);
	bool snoop_port(uint8_t port);


	volatile uint8_t _num_ports;
	USART* _usart;
	//volatile bool _receiving;
	ReceiveState _rx_state;
	volatile bool _wish_to_transmit;
	volatile uint8_t _wish_to_transmit_port;
	volatile uint8_t _current_port;
	volatile bool _rx_done;

	// counts number of milliseconds that have passed since
	// possible RX was detected
	//volatile uint16_t _possible_rx_ms_counter;
	//volatile uint16_t _possible_rx_timer_last_val;

	volatile uint8_t _rx_timeout_timer;

	volatile rx_packet_t _current_rx_packet;
	volatile uint8_t * const* _port_snoop_pins;
	const uint8_t * _port_snoop_pinnos;

	void (*_rx_packet_callback)(volatile uint8_t*, uint8_t);
	void (*_enable_incoming_data_interrupts_func)(void);
	void (*_disable_incoming_data_interrupts_func)(void);
	void (*_set_mux_port_func)(uint8_t);

};

#endif /* COMMSLOW_HPP_ */
