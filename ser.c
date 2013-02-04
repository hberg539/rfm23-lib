/*
 * ser.c
 */

#include <avr/io.h>
#include <stdint.h>
#include <stdio.h>
#include "ser.h"

FILE ser_file = FDEV_SETUP_STREAM(ser_write, ser_read, _FDEV_SETUP_RW);

// init serial
void ser_init() {
		
	// baud rate
	UBRR0H = (uint8_t)(SER_UBRR >> 8);
	UBRR0L = (uint8_t)SER_UBRR;
	
	// 8data, 1stop bit
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
		
	// enable transmission and reception
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
	
	// disable double speed (for arduino bootloader)
	UCSR0A &= ~(1 << U2X0);
}

// enable rx interrupt
void ser_en_rx_int() {
	UCSR0B |= (1 << RXCIE0);
}

// set serial as stdout
void ser_set_stdout() {
	 stdout = &ser_file;
}

// set serial as stdin
void ser_set_stdin() {
	stdin = &ser_file;
}

// write serial
void ser_write(uint8_t data) {
	// wait for empty transmit buffer
	while(!(UCSR0A&(1<<UDRE0))){};
		
	// send data
	UDR0 = data;
}

// read serial
uint8_t ser_read() {
	// wait for byte
	while(!(UCSR0A&(1<<RXC0)) ){};

	return (uint8_t)UDR0;
}
