/*
 * pwm.hpp
 *
 *  Created on: Apr 11, 2012
 *      Author: rt5g11
 */

#ifndef PWM_HPP_
#define PWM_HPP_

class PWM {
public:
	static void init();
	static uint8_t TopServoMove(uint16_t pwm_value);
	static uint8_t BottomServoMove(uint16_t pwm_value);
};



#endif /* PWM_HPP_ */
