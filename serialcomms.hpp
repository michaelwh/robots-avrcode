#ifndef COMMS_HPP_
#define COMMS_HPP_

#include <inttypes.h>

#include "config.hpp"



enum comms_status_t {
	COMMS_SUCCESS,
	COMMS_ERROR_TIMEOUT,
	COMMS_ERROR_NOTCONNECTED,
};


// --- commands begin



// --- commands end

class USART
{
public:
	USART(volatile uint8_t * UBRRnH, volatile uint8_t * UBRRnL, volatile uint8_t * UCSRnA, volatile uint8_t * UCSRnB, volatile uint8_t * UCSRnC, volatile uint8_t * UDRn,  uint8_t UDREn, uint8_t U2Xn);
	void init_9600();
	void init_38400();
	void init_76800();
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
	MultiplexedComms(USART* usart, uint8_t num_ports, volatile uint8_t * const * port_snoop_ports_in, const uint8_t* port_snoop_pins_in, volatile uint8_t * const * port_snoop_orientation_pins_in, const uint8_t* port_snoop_orientation_pinnos_in);
	void init(void (*rx_packet_callback)(uint8_t, volatile uint8_t*, uint8_t), void (*enable_incoming_data_interrupts_func)(void), void (*disable_incoming_data_interrupts_func)(void), void (*set_mux_port_in)(uint8_t));
	void incoming_data_blocking(uint8_t port);
	//void imcoming_data_async(uint8_t port);
	void send_data_blocking(uint8_t port, uint8_t* data, uint8_t data_length);
	void rx_byte(uint8_t byte_in);
	void timer_ms_tick(void);
	volatile uint8_t** send_queues;
	bool snoop_port(uint8_t port, bool orientation);
	uint8_t get_num_ports(void);
	void lock_to_port(uint8_t port);
	void unlock_from_port(void);

private:
	void set_current_port(uint8_t port);
	void start_rx(void);
	void finish_rx(void);

	volatile bool _locked_to_port;

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
	volatile uint8_t * const* _port_snoop_orientation_pins;
	const uint8_t * _port_snoop_orientation_pinnos;



	void (*_rx_packet_callback)(uint8_t, volatile uint8_t*, uint8_t);
	void (*_enable_incoming_data_interrupts_func)(void);
	void (*_disable_incoming_data_interrupts_func)(void);
	void (*_set_mux_port_func)(uint8_t);



};


class Packet
{
public:

	uint8_t* data;
	uint8_t data_length;

	Packet();

	Packet(uint8_t* data_in, uint8_t data_length_in);

	uint8_t get_command(void);
	uint8_t get_destination(void);
	uint8_t get_source(void);
	uint8_t get_packet_id(void);

	static uint8_t make_packet_flags(bool is_network, bool requires_ack, bool requires_global_ack, bool is_ack, bool is_global_ack);

	bool is_network(void);
	bool requires_ack(void);
	bool requires_global_ack(void);
	bool is_ack(void);
	bool is_global_ack(void);

	void set_is_network(bool is_network);
	void set_requires_ack(bool requires_ack);
	void set_requires_global_ack(bool requires_global_ack);
	void set_is_ack(bool is_ack);
	void set_is_global_ack(bool is_global_ack);

	void copy_from_buffer(uint8_t* buffer, uint8_t buffer_length);
};

class ReliableComms
{
public:
	 ReliableComms(MultiplexedComms* mux_comms_in);

	 //! Function to send packets to a specified port
	 /*! Sends a packet to the specified port. If the requires_ack flag is set
	  * on the packet then this function will retry sending the packet until either
	  * a maximum number of retries is met or an ack is received
	  * Returns:
	  * 	- COMMS_SUCCESS on success
	  * 	- COMMS_ERROR_TIMEOUT on failure due to ack not being recieved before timeout
	  * 	- COMMS_ERROR_NOTCONNECTED on failure due to port not being connected*/
	 comms_status_t send_packet(uint8_t port, Packet* packet, const uint8_t max_retries = 10);

	 void rx_ack(uint8_t port, Packet* packet);
	 bool is_port_connected(uint8_t port);
	 bool get_port_orientation(uint8_t port);

	 volatile bool waiting_for_ack;
	 volatile bool got_ack_flag;

private:


	MultiplexedComms* _mux_comms;
};

#endif /* COMMSLOW_HPP_ */
