/*
 * movement.hpp
 *
 *  Created on: May 16, 2012
 *      Author: mh23g08
 */

#ifndef MOVEMENT_HPP_
#define MOVEMENT_HPP_

#include <stdint.h>
#include "pwm.hpp"
#include "timer.hpp"


enum movement_state_t {
	MOVEMENT_STOPPED,
	MOVEMENT_FORWARD
};


class Movement {

public:
	PWM* _pwm;
	CounterTimer _movement_counter_timer;
	movement_state_t _movement_state;


	uint8_t _current_sequence_index;
	bool _done_current_sequence_index;

	//uint16_t* _current_sequence_top_values;
	//uint16_t* _current_sequence_bottom_values;
	//uint16_t* _current_sequence_delays;
	//uint16_t _current_sequence_length;

	Movement(PWM* pwm);
	void movement_step();
	void move_forward();
	void _do_step(const uint16_t* sequence_top_values, const uint16_t* sequence_bottom_values, const uint16_t* sequence_delays, const uint8_t sequence_length);

};


#endif /* MOVEMENT_HPP_ */
