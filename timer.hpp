/*
 * timer.hpp
 *
 *  Created on: Mar 27, 2012
 *      Author: mh23g08
 */

#ifndef TIMER_HPP_
#define TIMER_HPP_
#include <stdint.h>
#include <avr/io.h>

const uint16_t millisecond_top_val = 999;

extern volatile uint16_t millisecond_counter;

void millisecond_timer_enable(void);
inline uint16_t get_current_ms_timer_value(void) {
	return TCNT1;
}

class CounterTimer {
public:
	uint16_t _start_counter_value;
	volatile uint16_t* _counter_p;
	CounterTimer();
	CounterTimer(volatile uint16_t* counter_p_in);
	bool has_elapsed(uint16_t time);
	void reset(void);
};
//uint32_t get_us_elapsed(uint16_t start_timer_val, uint16_t current_timer_val, uint16_t num_ms_ticks);

#endif /* TIMER_HPP_ */
