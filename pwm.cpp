#include <avr/io.h>
#include <inttypes.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "pwm.hpp"
#include "config.hpp"
/*
 * pwm.cpp
 *
 *  Created on: Apr 11, 2012
 *      Author: rt5g11
 */

PWM::PWM() {
	//Sets the prescaler of the timer to
	uint8_t sreg;
	sreg = SREG;
	_target_value_top = 0;
	_target_value_bottom = 0;
	_current_value_top = 0;
	_current_value_bottom = 0;
	//Set pin PD5 as an output pin
	DDRD |= (1 << PD5) | (1 << PD4);
	//Setting the mode of operation for the timer 1, Non inverting phase correct PWM prescaler 8 and ICR with the value
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1) ;
	TCCR1B |= (1 << WGM13) | (1 << CS11);
	cli(); //Disable all interrupts for the 16 bits read
	ICR1 = TOP_FREQUENCY;
	OCR1A = PWM_INIT_VALUE;
	OCR1B = PWM_INIT_VALUE;
	sei();
}


void PWM::PWMStep(void) {
	_do_step(&_current_value_top, &_target_value_top);
	_do_step(&_current_value_bottom, &_target_value_bottom);
	//cli();
	OCR1A = _current_value_top;
	OCR1B = _current_value_bottom;
	//sei();
}


inline void PWM::_do_step(uint16_t* current_value, uint16_t* target_val) {
	if(*current_value == *target_val) {
		// we are at the target value so don't need to move
	} else if((*current_value + (PWM_STEP/2)) >= *target_val && (*current_value - (PWM_STEP/2)) <= *target_val) {
		// we are in within one PWM_STEP of the target value so we should stop stepping and jump directly to that value
		*current_value = *target_val;
	} else if(*current_value > *target_val) {
		// we are above the target value so need to decrease our value
		*current_value = *current_value - PWM_STEP;
	} else {
		// we are below the target value so need to increase our value
		*current_value = *current_value + PWM_STEP;
	}
}

uint8_t PWM::TopServoMove(uint16_t pwm_value) {
	if((pwm_value > PWM_MAX) | (pwm_value < PWM_MIN))
		return MESSAGE_ERROR;
	else {
		_target_value_top = pwm_value;
		return MESSAGE_OK;
	}
#if 0
	if((pwm_value > PWM_MAX) | (pwm_value < PWM_MIN))
		return MESSAGE_ERROR;
	else {

		if(	_current_value_top > pwm_value) {
			for(uint16_t i = _current_value_top - PWM_STEP; i > pwm_value; i = i - PWM_STEP) {
				cli();
					OCR1B = i;
				sei();
			}
		}
		else {
			for(uint16_t i = _current_value_top + PWM_STEP; i < pwm_value; i = i + PWM_STEP) {
				cli();
					OCR1B = i;
				sei();
			}
		}
		cli();
			OCR1B = pwm_value;
		sei();
		//Change delay for something that does not destroy the communications
		_delay_ms(20);
	}
	_current_value_top = pwm_value;
	return MESSAGE_OK;
#endif
}

uint8_t PWM::BottomServoMove(uint16_t pwm_value) {
	if((pwm_value > PWM_MAX) | (pwm_value < PWM_MIN))
		return MESSAGE_ERROR;
	else {
		_target_value_bottom = pwm_value;
		return MESSAGE_OK;
	}
#if 0
	if((pwm_value > PWM_MAX) | (pwm_value < PWM_MIN))
		return MESSAGE_ERROR;
	else {

		if(_current_value_bottom > pwm_value) {
			for(uint16_t i = _current_value_bottom - PWM_STEP; i > pwm_value; i = i - PWM_STEP) {
				cli();
					OCR1A = i;
				sei();
			}
		}
		else {
			for(uint16_t i = _current_value_bottom + PWM_STEP; i < pwm_value; i = i + PWM_STEP) {
				cli();
					OCR1A = i;
				sei();
			}
		}
		cli();
			OCR1A = pwm_value;
		sei();
		//Change delay for something that does not destroy the communications
		_delay_ms(20);
	}
	_current_value_bottom = pwm_value;
	return MESSAGE_OK;
#endif
}
