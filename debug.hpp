/*
 * debug.hpp
 *
 *  Created on: Apr 19, 2012
 *      Author: mh23g08
 */

#ifndef DEBUG_HPP_
#define DEBUG_HPP_

#include <stdio.h>
#include <avr/io.h>
#include "serialcomms.hpp"

 #define dbgprintf(...) printf(__VA_ARGS__)

extern USART USART1;


void init_debug(void);

#endif /* DEBUG_HPP_ */
