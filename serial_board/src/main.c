/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>

# define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "twi_common.h"
#include "twim.h"
#include "sysclk.h"

#define TWI_MASTER       TWIE
#define TWI_MASTER_PORT  PORTE
//#define TWI_SLAVE        TWIF
#define TWI_SPEED        50000
#define TWI_MASTER_ADDR  0x50
#define TWI_SLAVE_ADDR1   0x60

#define DATA_LENGTH     8

//TWI_Slave_t slave;

uint8_t data[DATA_LENGTH] = {
	0x05, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

uint8_t recv_data[DATA_LENGTH] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

twi_options_t m_options = {
	.speed     = TWI_SPEED,
	.chip      = TWI_MASTER_ADDR,
	.speed_reg = TWI_BAUD(/*sysclk_get_cpu_hz()*/F_CPU, TWI_SPEED)
};

/*
static void slave_process(void) {
	int i;
	for(i = 0; i < DATA_LENGTH; i++) {
		recv_data[i] = slave.receivedData[i];
	}
}
*/

/*
ISR(TWIC_TWIS_vect) {
	TWI_SlaveInterruptHandler(&slave);
}
*/

void send_twi(uint8_t addr, uint8_t* msg, uint8_t n);
void twi_init(void);

void send_twi(uint8_t addr, uint8_t* msg, uint8_t n)
{
	twi_package_t packet = {
		.addr_length = 0,
		.chip        = addr,
		.buffer      = (void *)msg,
		.length      = n,
		.no_wait     = false
	};

	
	//uint8_t i;

	

	/*
	sysclk_enable_peripheral_clock(&TWI_SLAVE);
	TWI_SlaveInitializeDriver(&slave, &TWI_SLAVE, *slave_process);
	TWI_SlaveInitializeModule(&slave, TWI_SLAVE_ADDR1,
	TWI_SLAVE_INTLVL_MED_gc);
	*/

	/*
	for (i = 0; i < TWIS_SEND_BUFFER_SIZE; i++) {
		slave.receivedData[i] = 0;
	}
	*/

	

	twi_master_write(&TWI_MASTER, &packet);

	/*
	do {
		// Nothing
	} while(slave.result != TWIS_RESULT_OK);
	*/
}

void twi_init(void) {
	TWI_MASTER_PORT.PIN0CTRL = PORT_OPC_WIREDANDPULL_gc;
	TWI_MASTER_PORT.PIN1CTRL = PORT_OPC_WIREDANDPULL_gc;

	irq_initialize_vectors();

	sysclk_enable_peripheral_clock(&TWI_MASTER);
	twi_master_init(&TWI_MASTER, &m_options);
	//twi_master_enable(&TWI_MASTER);
	//cpu_irq_enable();
}

#define TEST_MESSAGE_SIZE 2

int main (void)
{
	uint8_t msg[TEST_MESSAGE_SIZE] = {0x05,0x4D};
	uint8_t n = TEST_MESSAGE_SIZE;
	
	twi_init();
	
	uint8_t i = 1;
	while (1)
	{
		// send a msg
		send_twi(TWI_SLAVE_ADDR1, msg, n);
		
		// modify the msg for next transmission, and add a small delay between each transmission
		msg[0] = (i++%6) ;
		_delay_ms(100);
	}
}
