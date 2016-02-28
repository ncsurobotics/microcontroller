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
#include <avr/io.h>
#include "twis.h"
#include "sysclk.h"

void twi_init(void);
void twi_clear(void);
void do_nothing(void);

/* -------------------------------------------
--- implementing official Atmel Driver -------
-------------------------------------------*/
#define TWI_SLAVE        TWIC
#define TWI_SPEED        50000
#define TWI_SLAVE_ADDR   0x60

#define DATA_LENGTH     8

TWI_Slave_t TB_I2Cs_module;

static void slave_process(void);
void recv_twi(void);

uint8_t recv_data[DATA_LENGTH] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

volatile uint8_t serial_idle_shutdown;

static void slave_process(void) {
	int i;

	for(i = 0; i < DATA_LENGTH; i++) {
		recv_data[i] = TB_I2Cs_module.receivedData[i];
	}
}

ISR(TWIC_TWIS_vect) {
	TWI_SlaveInterruptHandler(&TB_I2Cs_module);
}

/*
 * No serial messages received for one second. There are potentially hardware issues, so signal a shutdown of the board.
 */
ISR(TCD0_OVF_vect) {
	serial_idle_shutdown = 1;
}

/*
 * Blocks until either a serial message is received or one second has passed without serial input
 */
void recv_twi(void)
{
	while(TB_I2Cs_module.result != TWIS_RESULT_OK && !serial_idle_shutdown);
	TCD0.CNTH = 0x00;
	TCD0.CNTL = 0x00;
}

/*
 * Initializes TWI slave settings
 */
void twi_init(void)
{
	irq_initialize_vectors();
	sysclk_enable_peripheral_clock(&TWI_SLAVE);
	TWI_SlaveInitializeDriver(&TB_I2Cs_module, &TWI_SLAVE, *slave_process);
	TWI_SlaveInitializeModule(&TB_I2Cs_module, TWI_SLAVE_ADDR, TWI_SLAVE_INTLVL_MED_gc);
	cpu_irq_enable();
}

/*
 * Clear serial input buffer
 */
void twi_clear(void)
{
	uint8_t i;
	for (i = 0; i < TWIS_SEND_BUFFER_SIZE; i++) {
		TB_I2Cs_module.receivedData[i] = 0;
	}
}


/* -----------------------------------
Thruster Board functions 
-------------------------------------- */

typedef struct PIN_struct {
	char* name;
	char* description;
	uint8_t pos;
	PORT_t* port;
} pin_t;

pin_t dir1 = {.name="dir1",
	.description="GPIO for controlling the directionality of motorcontroller #1",
	.pos=4,
	.port=&PORTC,
};

void TB_init(void);

void TB_init(void) {
	dir1.port->DIR |= (1<<dir1.pos);
}

/* -----------------------------------
Main program
-------------------------------------- */

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	TCD0.PER = 977; // Overflow in 1000 ms (Might actually be 500 ms, need to test)
	TCD0.CTRLA = 0b0111; // 1:1024 pre-scaler, 1 MHz : 977 Hz (Might actually be 2 MHz : 1953 Hz, need to test)
	TCD0.INTCTRLA |= 0b10;
	/*dir1.port->DIR |= 1<<dir1.pos;
	while (1) {
		dir1.port->OUT ^= 1<<dir1.pos;
		_delay_ms(100);
	}*/
	
	board_init();

	sei(); // should be taken care of in the irq_initialize_vectors(...), but just in case...
	twi_init();
	TB_init();
	
	while (1)
	{
		// Attempt to receive data. Will block until data has been received or a second has passed.
		recv_twi();
		
		// Perform temporary shutdown code pin if serial communication has timed out
		if (serial_idle_shutdown)
		{
			dir1.port->OUT |= (1<<dir1.pos);
			serial_idle_shutdown = 0;
		} else if (recv_data[0] = 0x3f) {
			dir1.port->OUT &= ~(1<<dir1.pos); // Received thruster command set a pin low
		}
		
		// Wipe receive buffer. Reset slave state for next call to recv_data.
		twi_clear();
		TWI_SlaveInitializeDriver(&TB_I2Cs_module, &TWI_SLAVE, *slave_process);
	}
}
