/*
 * temperature.c
 *
 * Created: 07.07.2012 10:51:27
 *  Author: hberg539
 */ 

#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "ser.h"
#include "rfm23.h"

int main(void)
{
	// init serial
	// 9600 Baud, 8 data, 1 stop bit
	ser_init();
	ser_set_stdout();
	
	// init rfm23 module
	rfm23_init();
	
	// rfm23 test
	// test spi single read/write
	// and burst read/write
	if (rfm23_test()) {
		printf("rfm23 test successfull.\n");
	} else {
		printf("rfm23 test failed.\n");
	}
	
    while(1)
    {
		// get temperature from internal sensor
		uint8_t temp = rfm23_get_temperature();
		
		// temp in C = return_value * 0.5 - 64
		temp = (temp * 0.5) - 64;
		
		printf("current temperature: %d\n", temp);
		
		_delay_ms(2000);
    }
}
