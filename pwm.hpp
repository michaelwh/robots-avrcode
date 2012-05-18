/*
 * pwm.hpp
 *
 *  Created on: Apr 11, 2012
 *      Author: rt5g11
 */

#ifndef PWM_HPP_
#define PWM_HPP_
#include <stdint.h>

class PWM {

public:
	 PWM(void);
	 //uint16_t _current_value_top;
	 //uint16_t _current_value_bottom;
	 //uint16_t _target_value_top;
	 //uint16_t _target_value_bottom;



	 uint8_t TopServoMove(uint16_t pwm_value);
	 uint8_t BottomServoMove(uint16_t pwm_value);
	 //inline void _do_step(uint16_t* current_value, uint16_t* target_val);
	 //void PWMStep(void);
};



#endif /* PWM_HPP_ */
