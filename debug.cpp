/*
 * debug.cpp
 *
 *  Created on: Apr 19, 2012
 *      Author: mh23g08
 */

#include <stdio.h>
#include <avr/io.h>

#include "debug.hpp"
#include "serialcomms.hpp"



//static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

#ifdef __cplusplus
extern "C"{
 FILE * uart_str;
}
#endif


USART USART1(&UBRR1H, &UBRR1L, &UCSR1A, &UCSR1B, &UCSR1C, &UDR1, UDRE0, U2X0);


static int uart_putchar(char c, FILE *stream)
{
    if (c == '\n') uart_putchar('\r', stream);

    USART1.send_blocking((uint8_t)c);

    return 0;
}


void init_debug(void) {
	USART1.init_76800();

	uart_str = fdevopen(uart_putchar, NULL);
	stdout = uart_str; //Required for printf init
}


