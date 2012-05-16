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



uint8_t PWM::TopServoMove(uint16_t pwm_value) {

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
}

uint8_t PWM::BottomServoMove(uint16_t pwm_value) {
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
}
