/*
	rfm23.h
*/

#include <stdint.h>

// spi defines
#define RFM23_SPI_PORT			PORTB
#define RFM23_SPI_DDR			DDRB
#define RFM23_SPI_PIN			PINB
#define RFM23_SPI_MOSI			PINB3
#define RFM23_SPI_MISO			PINB4
#define RFM23_SPI_CLOCK			PINB5
#define RFM23_SPI_SELECT		PINB2

// nirq interrupt defines
#define RFM23_NIRQ				PIND2
#define RFM23_NIRQ_PORT			PORTD
#define RFM23_NIRQ_DDR			DDRD
#define RFM23_NIRQ_PIN			PIND


// interrupt status register 1
#define RFM23_03h_ISR1				0x03
#define RFM23_03h_ISR1_IFFERR		0x00 | (1 << 7)
#define RFM23_03h_ISR1_ITXFFAFULL	0x00 | (1 << 6)
#define RFM23_03h_ISR1_ITXFFAEM		0x00 | (1 << 5)
#define RFM23_03h_ISR1_IRXFFAFULL	0x00 | (1 << 4)
#define RFM23_03h_ISR1_IEXT			0x00 | (1 << 3)
#define RFM23_03h_ISR1_IPKSENT		0x00 | (1 << 2)
#define RFM23_03h_ISR1_IPKVALID		0x00 | (1 << 1)
#define RFM23_03h_ISR1_ICRCERROR	0x00 | (1 << 0)

// interrupt status register 2
#define RFM23_04h_ISR2				0x04
#define RFM23_04h_ISR2_ISWDET		0x00 | (1 << 7)
#define RFM23_04h_ISR2_IPREAVAL		0x00 | (1 << 6)
#define RFM23_04h_ISR2_IPREAINVAL	0x00 | (1 << 5)
#define RFM23_04h_ISR2_IRSSI		0x00 | (1 << 4)
#define RFM23_04h_ISR2_IWUT			0x00 | (1 << 3)
#define RFM23_04h_ISR2_ILBD			0x00 | (1 << 2)
#define RFM23_04h_ISR2_ICHIPRDY		0x00 | (1 << 1)
#define RFM23_04h_ISR2_IPOR			0x00 | (1 << 0)

// interrupt enable 1 and 2
// single bit are equal as in 03h
// and 04h
#define RFM23_05h_ENIR1				0x05
#define RFM23_06h_ENIR2				0x06


// operating modes
#define RFM23_07h_OPMODE			0x07
#define RFM23_07h_OPMODE_SWRES		0x00 | (1 << 7)
#define RFM23_07h_OPMODE_ENLBD		0x00 | (1 << 6)
#define RFM23_07h_OPMODE_ENWT		0x00 | (1 << 5)
#define RFM23_07h_OPMODE_X32KSEL	0x00 | (1 << 4)
#define RFM23_07h_OPMODE_TXON		0x00 | (1 << 3)
#define RFM23_07h_OPMODE_RXON		0x00 | (1 << 2)
#define RFM23_07h_OPMODE_PLLON		0x00 | (1 << 1)
#define RFM23_07h_OPMODE_XTON		0x00 | (1 << 0)

// interrupt status register
volatile static uint8_t RFM23_ISR1 = 0x00;
volatile static uint8_t RFM23_ISR2 = 0x00;

// status variable (only internal usage)
volatile static uint8_t RFM23_STATUS = 0x00;
#define RFM23_STATUS_INTERRUPT		0x00 | (1 << 0)



/*
	spi functions
*/
void rfm23_spi_init();
uint8_t rfm23_spi_write(uint8_t val);
void rfm23_spi_select();
void rfm23_spi_unselect();


/*
	general
*/
void rfm23_init();
uint8_t rfm23_test();
void rfm23_reset();


/*
	read/write functions
*/
void rfm23_write(uint8_t addr, uint8_t val);
uint8_t rfm23_read(uint8_t addr);
void rfm23_write_burst(uint8_t addr, uint8_t (*val)[], uint8_t len);
void rfm23_read_burst(uint8_t addr, uint8_t (*val)[], uint8_t len);


/*
	interrupt functions
*/
uint8_t rfm23_on_nirq();
uint8_t rfm23_on_interrupt();
void rfm23_handle_interrupt();
void rfm23_enable_interrupt_1(uint8_t ir);
void rfm23_enable_interrupt_2(uint8_t ir);
uint8_t rfm23_get_isr_1();
uint8_t rfm23_get_isr_2();


/*
	operating mode functions
*/
void rfm23_mode_ready();
void rfm23_mode_tx();
void rfm23_mode_rx();
void rfm23_mode_wakeup();


/*
	fifo functions
*/
void rfm23_clear_rxfifo();
void rfm23_clear_txfifo();


/*
	send & receive functions
*/
void rfm23_send(uint8_t (*val)[], uint8_t len);
void rfm23_receive(uint8_t *data, uint8_t *len);
uint8_t rfm23_get_packet_length();


/*
	addressed send & receive functions
*/
void rfm23_set_address(uint8_t addr);
void rfm23_send_addressed(uint8_t addr, uint8_t (*val)[], uint8_t len);


/*
	wait functions
*/
uint8_t rfm23_wait_packet_sent(uint8_t timeout);

uint8_t rfm23_wait_interrupt_1(uint8_t reg, uint8_t timeout);
uint8_t rfm23_wait_interrupt_2(uint8_t reg, uint8_t timeout);


/*
	other functions
*/
uint8_t rfm23_get_temperature();
void rfm23_set_txpower(uint8_t txpower);
void rfm23_set_wakeup_time(uint8_t seconds);
