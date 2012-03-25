#ifndef COMMS_HPP_
#define COMMS_HPP_

#include <inttypes.h>

#define USART_SEND_DELAY_MS	10


class USART
{
public:
	USART(volatile uint8_t * UBRRnH, volatile uint8_t * UBRRnL, volatile uint8_t * UCSRnA, volatile uint8_t * UCSRnB, volatile uint8_t * UCSRnC, volatile uint8_t * UDRn,  uint8_t UDREn);
	void init(unsigned int baud);
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

};


struct rx_packet_t {
	bool have_packet_length;
	uint8_t packet_length;
	uint8_t current_rx_byte_index;
	uint8_t* received_packet;
};

class MultiplexedComms
{
public:
	MultiplexedComms(USART* usart, uint8_t num_ports, volatile uint8_t ** port_snoop_ports, volatile uint8_t* port_snoop_pins);
	void init(void (*rx_packet_callback)(uint8_t*, uint8_t), void (*enable_incoming_data_interrupts_func)(void), void (*disable_incoming_data_interrupts_func)(void));
	void incoming_data(uint8_t port);
	bool send_data(uint8_t port, uint8_t* data, uint8_t data_length);
	void rx_byte(uint8_t byte_in);
	void timer_tick(void);
private:
	void set_current_port(uint8_t port);
	void start_rx(void);
	void finish_rx(void);
	bool snoop_port(uint8_t port);


	volatile uint8_t _num_ports;
	USART* _usart;
	volatile bool _receiving;
	volatile bool _transmitting;
	volatile uint8_t _current_port;
	volatile bool _rx_done;

	volatile uint8_t _rx_timeout_timer;

	volatile rx_packet_t _current_rx_packet;
	volatile uint8_t ** _port_snoop_ports;
	volatile uint8_t * _port_snoop_pins;

	void (*_rx_packet_callback)(uint8_t*, uint8_t);
	void (*_enable_incoming_data_interrupts_func)(void);
	void (*_disable_incoming_data_interrupts_func)(void);

};

#endif /* COMMSLOW_HPP_ */