/*
 * movement.cpp
 *
 *  Created on: May 16, 2012
 *      Author: mh23g08
 */

#include <stdint.h>
#include "movement.hpp"
#include "timer.hpp"
#include "config.hpp"
#include "pwm.hpp"

#define PWM_MIDDLE ((PWM_MAX + PWM_MIN) / 2)

// 0 means no movement
const uint16_t sequence_forward_top_values[] = {
		0,
		PWM_MIDDLE,
		0,
		PWM_MAX
};

const uint16_t sequence_forward_bottom_values[] = {
		PWM_MIDDLE,
		0,
		PWM_MIN,
		0
};

const uint16_t sequence_forward_delays[] = {
		200,
		200,
		200,
		200
};

const uint8_t sequence_forward_length = 4;

const uint16_t sequence_wiggle_top_values[] = {
		PWM_MIDDLE,
		PWM_MIDDLE + 200,
		PWM_MIDDLE - 200,
		PWM_MIDDLE
};

const uint16_t sequence_wiggle_bottom_values[] = {
		PWM_MIDDLE,
		PWM_MIDDLE - 200,
		PWM_MIDDLE + 200,
		PWM_MIDDLE
};

const uint16_t sequence_wiggle_delays[] = {
		200,
		200,
		200,
		200
};

const uint8_t sequence_wiggle_length = 4;



Movement::Movement(PWM* pwm) {
	_pwm = pwm;
	_movement_state = MOVEMENT_STOPPED;
	_current_sequence_index = 0;
	_done_current_sequence_index = false;
	_movement_counter_timer.reset();
}

void Movement::movement_step() {
	switch (_movement_state) {
		case MOVEMENT_STOPPED:
			_current_sequence_index = 0;
			_done_current_sequence_index = false;
			break;
		case MOVEMENT_FORWARD:
			_do_step(sequence_forward_top_values, sequence_forward_bottom_values, sequence_forward_delays, sequence_forward_length);
			break;
		case MOVEMENT_WIGGLE:
			if(_do_step(sequence_wiggle_top_values, sequence_wiggle_bottom_values, sequence_wiggle_delays, sequence_wiggle_length))
				_movement_state = MOVEMENT_STOPPED;
			break;
		default:
			break;
	}
}

// returns true when at end of sequence
bool Movement::_do_step(const uint16_t* sequence_top_values, const uint16_t* sequence_bottom_values, const uint16_t* sequence_delays, const uint8_t sequence_length) {
	if(!_done_current_sequence_index) {
		// if we are just starting this movement sequence then move the servos to the correct positions
		if(sequence_top_values[_current_sequence_index] != 0)
			_pwm->TopServoMove(sequence_top_values[_current_sequence_index]);
		if(sequence_bottom_values[_current_sequence_index] != 0)
			_pwm->BottomServoMove(sequence_bottom_values[_current_sequence_index]);
		_done_current_sequence_index = true;
		_movement_counter_timer.reset();
	}

	// now check to see if we are ready to move on
	if(sequence_delays[_current_sequence_index] != 0) {
		if(_movement_counter_timer.has_elapsed(sequence_delays[_current_sequence_index])) {
			if(_current_sequence_index + 1 >= sequence_length) {
				//_current_sequence_index = (_current_sequence_index + 1) % sequence_length; // wrap around
				_current_sequence_index = 0;
				_done_current_sequence_index = false;
				return true;
			} else {
				_current_sequence_index++;
				_done_current_sequence_index = false;
				return false;
			}
		}
	} else {
		if(_current_sequence_index + 1 >= sequence_length) {
			//_current_sequence_index = (_current_sequence_index + 1) % sequence_length; // wrap around
			_current_sequence_index = 0;
			_done_current_sequence_index = false;
			return true;
		} else {
			_current_sequence_index++;
			_done_current_sequence_index = false;
			return false;
		}
	}
}

void Movement::move_forward(){
	_current_sequence_index = 0;
	_done_current_sequence_index = false;
	_movement_state = MOVEMENT_FORWARD;
	//uint16_t value = (PWM_MAX + PWM_MIN)/2;
	//_pwm->BottomServoMove(value);
	//_delay_ms(200);
	//_pwm->TopServoMove(value);
	//_delay_ms(200);
	//_pwm->BottomServoMove(PWM_MIN);
	//_delay_ms(200);
	//_pwm->TopServoMove(PWM_MAX);
	//_delay_ms(200);
}

void Movement::move_wiggle() {
	_current_sequence_index = 0;
	_done_current_sequence_index = false;
	_movement_state = MOVEMENT_WIGGLE;
}
