/*
	rfm23.c
*/

#include <avr/io.h>
#include <stdio.h>
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
	
	// wait for POR 16.8ms
	_delay_ms(17);
	
	// configure nirq port as input
	RFM23_NIRQ_DDR &= ~(1 << RFM23_NIRQ);
	
	// init spi
	rfm23_spi_init();
	
	// sw reset
	rfm23_reset();
	
	// disable all interrupts
	rfm23_enable_interrupt_1(0x00);
	rfm23_enable_interrupt_2(0x00);
	
	// read all interrupts
	rfm23_read(RFM23_03h_ISR1);
	rfm23_read(RFM23_04h_ISR2);
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
	rfm23_write(RFM23_07h_OPMODE, RFM23_07h_OPMODE_SWRES);
	_delay_ms(1);
}


/*
	read/write functions
*/

/* write to rfm registers */
void rfm23_write(uint8_t addr, uint8_t val) {
	
	// debug
	uint8_t daddr = addr;
	
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
	
	// debug
/*#ifdef RFM23_DEBUG_WRITE
	printf("[d] write:%x:%x\n", daddr, val);
#endif*/
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
	
	// debug
/*#ifdef RFM23_DEBUG_READ
	printf("[d] read:%x:%x\n", addr, val);
#endif*/
	
	return val;
}

/* write to rfm registers in burst mode */
void rfm23_write_burst(uint8_t addr, uint8_t val[], uint8_t len) {

	// debug
	uint8_t daddr = addr;

	// first bit 1 means write
	addr |= (1 << 7);
	
	// select module
	rfm23_spi_select();
	
	// write register address
	rfm23_spi_write(addr);
	
	// write values
	for (uint8_t i = 0; i < len; i++) {
		rfm23_spi_write(val[i]);
	}
	
	// unselect module and finish operation
	rfm23_spi_unselect();
	
	// debug
#ifdef RFM23_DEBUG_WRITE
	printf("[d] writeburst %x: ", daddr);
	for (uint8_t i = 0; i < len; i++) {
		printf("%x ", val[i]);
	}
	printf("\n");
#endif
}

/* read from rfm registers in burst mode */
void rfm23_read_burst(uint8_t addr, uint8_t val[], uint8_t len) {
	
	// first bit 0 means read
	addr &= ~(1 << 7);
	
	// select module
	rfm23_spi_select();
	
	// write register address
	rfm23_spi_write(addr);
	
	// read values
	for (uint8_t i = 0; i < len; i++) {
		val[i] = rfm23_spi_write(0x00);
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
		
#ifdef RFM23_DEBUG_INTERRUPT
		printf("[d] on_interrupt\n");
#endif
		
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
	
#ifdef RFM23_DEBUG_INTERRUPT
	printf("[d] int handle: ");
	if (RFM23_ISR1 & (1 << 0)) { printf("[icrcerror] "); }
	if (RFM23_ISR1 & (1 << 1)) { printf("[ipkvalid] "); }
	if (RFM23_ISR1 & (1 << 2)) { printf("[ipksent] "); }
	if (RFM23_ISR1 & (1 << 3)) { printf("[iext] "); }
	if (RFM23_ISR1 & (1 << 4)) { printf("[irxffafull] "); }
	if (RFM23_ISR1 & (1 << 5)) { printf("[itxffaem] "); }
	if (RFM23_ISR1 & (1 << 6)) { printf("[itxffafull] "); }
	if (RFM23_ISR1 & (1 << 7)) { printf("[ifferr] "); }		
	if (RFM23_ISR2 & (1 << 0)) { printf("[ipor] "); }
	if (RFM23_ISR2 & (1 << 1)) { printf("[ichiprdy] "); }
	if (RFM23_ISR2 & (1 << 2)) { printf("[ifbd] "); }
	if (RFM23_ISR2 & (1 << 3)) { printf("[iwut] "); }
	if (RFM23_ISR2 & (1 << 4)) { printf("[irssi] "); }
	if (RFM23_ISR2 & (1 << 5)) { printf("[ipreainval] "); }
	if (RFM23_ISR2 & (1 << 6)) { printf("[ipreaval] "); }
	if (RFM23_ISR2 & (1 << 7)) { printf("[iswdet] "); }
	printf("\n");
#endif
	
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
	return RFM23_ISR1;
}

uint8_t rfm23_get_isr_2() {
	return RFM23_ISR2;
}


/*
	operating mode functions
*/

/* mode READY */
void rfm23_mode_ready() {
	
	// go to READY mode
	rfm23_write(RFM23_07h_OPMODE, RFM23_07h_OPMODE_XTON);
	
	// wait for module
	_delay_ms(1);
}

/* mode RXON */
void rfm23_mode_rx() {
	
	// go to RXON mode
	rfm23_write(RFM23_07h_OPMODE, RFM23_07h_OPMODE_RXON);
	
	// wait for module
	_delay_ms(1);
}

/* mode TXON */
void rfm23_mode_tx() {
	
	// go to TXON mode
	rfm23_write(RFM23_07h_OPMODE, RFM23_07h_OPMODE_TXON);
	
	// wait for module
	_delay_ms(1);
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
	_delay_ms(10);
}


/*
	fifo functions
*/

/* clear rx fifo */
void rfm23_clear_rxfifo() {
	uint8_t opmode08h = rfm23_read(0x08);
	
	rfm23_write(0x08, opmode08h | (1 << 1));
	rfm23_write(0x08, opmode08h | (0 << 1));
}

/* clear tx fifo */
void rfm23_clear_txfifo() {
	uint8_t opmode08h = rfm23_read(0x08);
	
	rfm23_write(0x08, opmode08h | (1 << 0));
	rfm23_write(0x08, opmode08h | (0 << 0));
}


/*
	send & receive functions
*/

/* send data */
void rfm23_send(uint8_t data[], uint8_t len) {
	
	// clear tx fifo
	rfm23_clear_txfifo();
	
	// set packet length
	rfm23_write(0x3e, len);
	
	// write data into fifo
	rfm23_write_burst(0x7f, data, len);

	// send data
	rfm23_write(0x07, 0x09);
	
	// wait
	rfm23_wait_opmode(RFM23_07h_OPMODE_TXON);
}

void rfm23_send_addressed(uint8_t addr, uint8_t data[], uint8_t len) {
	
	// set receiver address (header2)
	rfm23_write(0x3b, addr);
	
	// send data
	rfm23_send(data, len);
}

void rfm23_set_address(uint8_t addr) {
	
	// set sender address (header1)
	rfm23_write(0x3c, addr);
	rfm23_write(0x40, addr);
	
	// only receive when header2 match
	//rfm23_write(0x32, 0x84);
}

/* get address */
uint8_t rfm23_get_address() {
	return rfm23_read(0x3c);
}

/* get sender address */
uint8_t rfm23_get_packet_sender_address() {
	return rfm23_read(0x49);
}

/* get receiver address */
uint8_t rfm23_get_packet_receiver_address() {
	return rfm23_read(0x48);
}

/* packet is broadcast */
uint8_t rfm23_packet_is_broadcast() {
	if (rfm23_get_packet_receiver_address() == 0x00) {
		return 0xff;
	} else {
		return 0x00;
	}
}

/* receive data */
void rfm23_receive(uint8_t data[], uint8_t len) {
	rfm23_read_burst(0x7f, data, len);
}

/* get packet length */
uint8_t rfm23_get_packet_length() {
	return rfm23_read(0x4b);
}


/*
	wait functions
*/

/* wait for IPKSENT interrupt */
/* @TODO */
void rfm23_wait_packet_sent(uint8_t timeout) {
	printf("wait for packet sent...\n");
	
	//uint8_t current_time = 0;
	
	// handle interrupt
	rfm23_handle_interrupt();
	
	while (!(rfm23_get_isr_1() & (1 << RFM23_03h_ISR1_IPKSENT))/* && !(current_time > timeout)*/) {
		rfm23_handle_interrupt();
	}
}

/* wait for finished opmode */
void rfm23_wait_opmode(uint8_t opmode) {
	while (rfm23_read(RFM23_07h_OPMODE) & opmode);
}


/*
	other functions
*/

/* get temperature
   temp = return_value * 0.5 - 64
*/
uint8_t rfm23_get_temperature() {
	
	// set adc input and reference
	rfm23_write(0x0f, 0x00 | (1 << 6) | (1 << 5) | (1 << 4));
	
	// set temperature range
	// -64 to 64°C, ADC8 LSB: 0.5°C
	rfm23_write(0x12, 0x00 | (1 << 5));
	
	// adcstart
	rfm23_write(0x0f, 0x00 | (1 << 7));
	
	// wait for adc_done
	while (!rfm23_read(0x0f) & (1 << 7));
	
	// return adc value
	return rfm23_read(0x11);
}

/* wake-up timer */
void rfm23_set_wakeup_time(uint8_t seconds) {
	rfm23_write(0x14, 0x0A);
	
	uint16_t sec = 8 * seconds; // 1 = 125ms, 8 = 1000ms = 1s
	rfm23_write(0x15, (sec >> 8) & 0xFF);
	rfm23_write(0x16, sec & 0xFF);
}