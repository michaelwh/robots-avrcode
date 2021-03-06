/*
 * timer.cpp
 *
 *  Created on: Mar 27, 2012
 *      Author: mh23g08
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "pinutil.hpp"

#include "timer.hpp"

// http://members.shaw.ca/climber/avrinterrupts.html
void millisecond_timer_enable(void) {
	/* set the 8 bit timer to clear and interrupt every 1ms */

	// set the top compare value
	OCR0A = 0xF9;

	// enable top match interrupt
	TIMSK0 = (1<<OCIE0A);

	// enable clear on top compare mode
	// also enable timer and set clock prescaler to 64
	TCCR0A = (1<<WGM01);
	TCCR0B = (1<<CS00)|(1<<CS01);
}

CounterTimer::CounterTimer() {
	_start_counter_value = 0;
	_counter_p = &millisecond_counter;
}

CounterTimer::CounterTimer(volatile uint16_t* counter_p_in) {
	_start_counter_value = 0;
	_counter_p = counter_p_in;
}

bool CounterTimer::has_elapsed(uint16_t time) {
	uint16_t elapsed = 0;
	if(*_counter_p < _start_counter_value) {
		// overflow has occurred
		elapsed = (65535 - _start_counter_value) + *_counter_p;
	} else {
		elapsed = *_counter_p - _start_counter_value;
	}

	if(elapsed >= time) {
		return true;
	} else {
		return false;
	}
}


void CounterTimer::reset(void) {
	_start_counter_value = *_counter_p;
}

//uint32_t get_us_elapsed(uint16_t start_timer_val, uint16_t current_timer_val, uint16_t num_ms_ticks) {
//	/* Give this function the value of the timer when you started timing (found by get_current_ms_timer_value) and
//	 * the number of millisecond ticks elapsed since then (you should increment this every time the timer ISR is called)
//	 * then this function will give you the microseconds elapsed since then */
//
//		if(num_ms_ticks <= 0) {
////			SET_BIT(PORTC, 0);
////			CLR_BIT(PORTC, 0);
////			SET_BIT(PORTC, 0);
//			return (uint32_t)(current_timer_val - start_timer_val);
//		} else {
////			SET_BIT(PORTC, 2);
////			CLR_BIT(PORTC, 2);
////			SET_BIT(PORTC, 2);
//			return (uint32_t)(millisecond_top_val - start_timer_val + (1000 * (num_ms_ticks - 1)) + current_timer_val);
//		}
//
//}


