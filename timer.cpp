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
	/* set the 16 bit timer count at 1MHZ and clear when it reaches 999,
	 * therefore interrupting every 1ms */

	// set the top compare value
	// write high then low - MUST be in this order
	// since this is a special 16-bit write operation
	OCR1AH = ((uint8_t)(millisecond_top_val>>8));
	OCR1AL = ((uint8_t)millisecond_top_val);

	// enable top match interrupt
	TIMSK1 = (1<<OCIE1A);

	// enable clear on top compare mode
	// also enable timer and set clock prescaler to 8
	TCCR1B = (1<<WGM12)|(1<<CS11);
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


