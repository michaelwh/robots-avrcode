#include <avr/io.h>
#include <inttypes.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "pwm.hpp"
/*
 * pwm.cpp
 *
 *  Created on: Apr 11, 2012
 *      Author: rt5g11
 */

#define PWM_INIT_VALUE 500
#define TOP_FREQUENCY 20000 //This should give the proper desired frequency
#define PWM_MAX 2000
#define PWM_MIN 500
#define MESSAGE_ERROR 0xFF
#define MESSAGE_OK 0x00

void PWM::init(){
	//Sets the prescaler of the timer to
	uint8_t sreg;
	sreg = SREG;

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
		cli();
		OCR1B = pwm_value;
		sei();
	}
	return MESSAGE_OK;
}

uint8_t PWM::BottomServoMove(uint16_t pwm_value) {
	if((pwm_value > PWM_MAX) | (pwm_value < PWM_MIN))
		return MESSAGE_ERROR;
	else {
		cli();
		OCR1A = pwm_value;
		sei();
	}
	return MESSAGE_OK;
}
