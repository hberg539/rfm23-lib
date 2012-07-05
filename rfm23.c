/*
	rfm23.c
*/

#include <avr/io.h>
#include <util/delay.h>
#include "rfm23.h"

/*
	spi functions
*/


/* init spi */
void rfm23_spi_init() {
	
	// set mosi, select (ss/cs) and clock as output
	RFM23_SPI_DDR = (1 << RFM23_SPI_MOSI) | (1 << RFM23_SPI_SELECT) | (1 << RFM23_SPI_CLOCK);
	
	// set miso as input
	RFM23_SPI_DDR &= ~(1 << RFM23_SPI_MISO);
	
	// select
	rfm23_spi_select();
	
	// enable spi (SPE), set as master (MSTR) and set clock rate (SPR)
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
}

/* write spi */
uint8_t rfm23_spi_write(uint8_t val) {
	
	// fill value into spi data register
	SPDR = val;
	
	// wait
	while (!(SPSR & (1 << SPIF)));
	
	return (uint8_t)SPDR;
	
}

/* select low */
void rfm23_spi_select() {
	
	// when module is selected,
	// set SELECT to LOW
	RFM23_SPI_PORT &= ~(1 << RFM23_SPI_SELECT);
}

/* select high */
void rfm23_spi_unselect() {
	
	// when module is unselected,
	// set SELECT to HIGH
	RFM23_SPI_PORT |= (1 << RFM23_SPI_SELECT);
}


/*
	general functions
*/

/* initialize rfm module */
void rfm23_init() {
	
	// configure nirq port as input
	RFM23_NIRQ_DDR &= ~(1 << RFM23_NIRQ);
	
	// init spi
	rfm23_spi_init();
	
	// wait for POR 16ms
	_delay_ms(16);
}

/* test read/write */
uint8_t rfm23_test() {
	
	uint8_t reg = 0x05;
	uint8_t value = 0xEE;
	
	// read register
	uint8_t val_orig = rfm23_read(reg);
	
	// write register
	rfm23_write(reg, value);
	
	// read register
	uint8_t val_new = rfm23_read(reg);
	
	// set orig register value
	rfm23_write(reg, val_orig);
	
	// test if the written register value
	// has been read
	if (val_new == value) {
		return 0xFF;
	} else {
		return 0x00;
	}
}

/* software reset module */
void rfm23_reset() {
	
}


/*
	read/write functions
*/

/* write to rfm registers */
void rfm23_write(uint8_t addr, uint8_t val) {
	
	// first bit 1 means write
	addr |= (1 << 7);
	
	// select module
	rfm23_spi_select();
	
	// write register address
	rfm23_spi_write(addr);
	
	// write value
	rfm23_spi_write(val);
	
	// unselect module and finish operation
	rfm23_spi_unselect();
}

/* read from rfm registers */
uint8_t rfm23_read(uint8_t addr) {
	
	// first bit 0 means read
	addr &= ~(1 << 7);
	
	// select module
	rfm23_spi_select();
	
	// write register address
	rfm23_spi_write(addr);
	
	// write dummy value
	uint8_t val = rfm23_spi_write(0x00);
	
	// unselect module and finish operation
	rfm23_spi_unselect();
	
	return val;
}

/* write to rfm registers in burst mode */
void rfm23_write_burst(uint8_t addr, uint8_t (*val)[], uint8_t len) {

	// first bit 1 means write
	addr |= (1 << 7);
	
	// select module
	rfm23_spi_select();
	
	// write register address
	rfm23_spi_write(addr);
	
	// write values
	for (uint8_t i = 0; i < len; i++) {
		rfm23_spi_write((*val)[i]);
	}
	
	// unselect module and finish operation
	rfm23_spi_unselect();
}

/* read from rfm registers in burst mode */
void rfm23_read_burst(uint8_t addr, uint8_t (*val)[], uint8_t len) {
	
	// first bit 0 means read
	addr &= ~(1 << 7);
	
	// select module
	rfm23_spi_select();
	
	// write register address
	rfm23_spi_write(addr);
	
	// read values
	for (uint8_t i = 0; i < len; i++) {
		(*val)[i] = rfm23_spi_write(0x00);
	}
	
	// unselect module and finish operation
	rfm23_spi_unselect();
}


/*
	interrupt functions
*/

/* returns 0xFF when NIRQ-Pin is low = rfm interrupt */
uint8_t rfm23_on_nirq() {
	return !(RFM23_NIRQ_PIN & (1 << RFM23_NIRQ));
}

/* returns 0xFF when interrupt has happened.
   interrupt is set once by rfm23_handle_interrupt()
   and will be reset when this function is called.
   
   on_nirq() (or hardware interrupt) -> handle_interrupt()
   -> on_interrupt()
  */
uint8_t rfm23_on_interrupt() {
	
	// save current status
	uint8_t tmp = RFM23_STATUS;
	
	// if interrupt bit is set
	// -> return 0xFF and reset
	if (tmp & (1 << RFM23_STATUS_INTERRUPT)) {
		
		// reset bit
		RFM23_STATUS &= ~(1 << RFM23_STATUS_INTERRUPT);
		
		return 0xFF;
	}
	
	return 0x00;
}

/* handle the interrupt */
void rfm23_handle_interrupt() {
	
	// read interrupt status register
	// -> nirq pin of the rfm resets
	RFM23_ISR1 = rfm23_read(RFM23_03h_ISR1);
	RFM23_ISR2 = rfm23_read(RFM23_04h_ISR2);
	
	// set interrupt bit
	RFM23_STATUS |= (1 << RFM23_STATUS_INTERRUPT);
	
	// wait some ms
	// dont know why this is needed
	// ... @TODO
	_delay_ms(10);	
}

/* enable interrupts */
void rfm23_enable_interrupt_1(uint8_t ir) {
	rfm23_write(RFM23_05h_ENIR1, ir);
}

void rfm23_enable_interrupt_2(uint8_t ir) {
	rfm23_write(RFM23_06h_ENIR2, ir);
}

/* return saved interrupt status registers */
uint8_t rfm23_get_isr_1() {
	return rfm23_read(RFM23_03h_ISR1);
}

uint8_t rfm23_get_isr_2() {
	return rfm23_read(RFM23_04h_ISR2);
}


/*
	operating mode functions
*/

/* mode READY */
void rfm23_mode_ready() {
	
	// go to READY mode
	rfm23_write(RFM23_07h_OPMODE, RFM23_07h_OPMODE_XTON);
	
	// wait for module
	_delay_us(200);	
}

/* mode RXON */
void rfm23_mode_rx() {
	
	// go to RXON mode
	rfm23_write(RFM23_07h_OPMODE, RFM23_07h_OPMODE_RXON);
	
	// wait for module
	_delay_us(200);	
}

/* mode TXON */
void rfm23_mode_tx() {
	
	// go to TXON mode
	rfm23_write(RFM23_07h_OPMODE, RFM23_07h_OPMODE_TXON);
	
	// wait for module
	_delay_us(200);
}

/* mode WAKEUP
   module goes into sleep mode and wakes up after
   a defined period of time.
   configuration of wake-up time is necessary.
   time can be set with set_wakeup_time()
   */
void rfm23_mode_wakeup() {
	
	// go to IDLE mode
	rfm23_write(RFM23_07h_OPMODE, RFM23_07h_OPMODE_ENWT);
	
	// read status register to reset
	rfm23_read(RFM23_03h_ISR1);
	rfm23_read(RFM23_04h_ISR2);
	
	// wait for module
	_delay_us(200);	
}


/*
	fifo functions
*/

/* clear rx fifo */
void rfm23_clear_rxfifo() {
	rfm23_write(0x08, 0x02);
	rfm23_write(0x08, 0x00);
}

/* clear tx fifo */
void rfm23_clear_txfifo() {
	rfm23_write(0x08, 0x01);
	rfm23_write(0x08, 0x00);
}


/*
	send & receive functions
*/

/* send data */
void rfm23_send(uint8_t (*data)[], uint8_t len) {
	
	// clear tx fifo
	rfm23_clear_txfifo();
	
	// set packet length
	rfm23_write(0x3e, len);
	
	// write data into fifo
	rfm23_write_burst(0x7f, data, len);
	
	// send data
	rfm23_write(0x07, 0x09);
}

/* receive data */
void rfm23_receive(uint8_t *data, uint8_t *len) {
	len = rfm23_get_packet_length();
	rfm23_read_burst(0x7f, &data, len);
}

/* get packet length */
uint8_t rfm23_get_packet_length() {
	return rfm23_read(0x4b);
}