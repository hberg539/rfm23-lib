/*
 * ser.h
 */

#define F_CPU 16000000

#define SER_BAUDRATE 9600
#define SER_UBRR (((F_CPU / (SER_BAUDRATE * 16UL))) - 1)

void ser_init();
void ser_set_stdout();
void ser_write(uint8_t data);
uint8_t ser_read();