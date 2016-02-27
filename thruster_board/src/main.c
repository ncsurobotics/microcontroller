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
void do_nothing(void);

/* -------------------------------------------
--- implementing official Atmel Driver -------
-------------------------------------------*/
//#define TWI_MASTER       TWIC
//#define TWI_MASTER_PORT  PORTC
#define TWI_SLAVE        TWIC
#define TWI_SPEED        50000
//#define TWI_MASTER_ADDR  0x50
#define TWI_SLAVE_ADDR   0x60

#define DATA_LENGTH     8

TWI_Slave_t TB_I2Cs_module;

static void slave_process(void);
void recv_twi(void);

/* ---
uint8_t data[DATA_LENGTH] = {
	0x0f, 0x1f, 0x2f, 0x3f, 0x4f, 0x5f, 0x6f, 0x7f
};
----- */

uint8_t recv_data[DATA_LENGTH] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* No master to use in this implementation -------------
twi_options_t m_options = {
	.speed     = TWI_SPEED,
	.chip      = TWI_MASTER_ADDR,
	.speed_reg = TWI_BAUD(sysclk_get_cpu_hz(), TWI_SPEED)
};
-------------------------------------------------------- */
static void slave_process(void) {
	int i;

	for(i = 0; i < DATA_LENGTH; i++) {
		recv_data[i] = TB_I2Cs_module.receivedData[i];
	}
}

ISR(TWIC_TWIS_vect) {
	TWI_SlaveInterruptHandler(&TB_I2Cs_module);
}

void /*send_and_*/recv_twi(void)
{
	/* ---------------------------
	twi_package_t packet = {
		.addr_length = 0,
		.chip        = TWI_SLAVE_ADDR,
		.buffer      = (void *)data,
		.length      = DATA_LENGTH,
		.no_wait     = false
	};
	---------------------------- */

	uint8_t i;

	//TWI_MASTER_PORT.PIN0CTRL = PORT_OPC_WIREDANDPULL_gc;
	//TWI_MASTER_PORT.PIN1CTRL = PORT_OPC_WIREDANDPULL_gc;

	// in init function -------------- irq_initialize_vectors();

	//sysclk_enable_peripheral_clock(&TWI_MASTER);
	//twi_master_init(&TWI_MASTER, &m_options);
	//twi_master_enable(&TWI_MASTER);

	// in init function -------------- sysclk_enable_peripheral_clock(&TWI_SLAVE);
	 // in init function -------------- TWI_SlaveInitializeDriver(&TB_I2Cs_module, &TWI_SLAVE, *slave_process);
	// in init function -------------- TWI_SlaveInitializeModule(&TB_I2Cs_module, TWI_SLAVE_ADDR, TWI_SLAVE_INTLVL_MED_gc);

	for (i = 0; i < TWIS_SEND_BUFFER_SIZE; i++) {
		TB_I2Cs_module.receivedData[i] = 0;
	}

	cpu_irq_enable();

	//twi_master_write(&TWI_MASTER, &packet);

	// wait for transaction to occur & complete
	do {
		// Nothing
	} while(TB_I2Cs_module.result != TWIS_RESULT_OK);
}

void twi_init(void)
{
	irq_initialize_vectors();
	sysclk_enable_peripheral_clock(&TWI_SLAVE);
	TWI_SlaveInitializeDriver(&TB_I2Cs_module, &TWI_SLAVE, *slave_process);
	TWI_SlaveInitializeModule(&TB_I2Cs_module, TWI_SLAVE_ADDR, TWI_SLAVE_INTLVL_MED_gc);
	/*
	twi_master_options_t opt = {
		.speed = 50000,
		.chip  = 0x50
	};
	twi_master_setup(&TWIM0, &opt);
	*/
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
	.pos=1,
	.port=&PORTA,
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
	dir1.port->DIR |= 1<<dir1.pos;
	while (1) {
		dir1.port->OUT ^= 1<<dir1.pos;
		_delay_ms(100);
	}
	
	board_init();

	sei(); // should be taken care of in the irq_initialize_vectors(...), but just in case...
	twi_init();
	TB_init();
	
	while (1)
	{
		// attempt to recv data. Will get stuck inside this function if
		// master device is not connected to the same bus and transmitting data.
		recv_twi();
		
		// if recv buffer is unchanged.
		if (recv_data[1] != 0x3f) {
			dir1.port->OUT |= (1<<dir1.pos); //let a pin remain high.
		}
		
		// if recv buffer has been changed.
		else {
			dir1.port->OUT &= ~(1<<dir1.pos); //set a pin low.
		}
		
		// wipe all receive buffers clean. Force recv_twi() to hang until next data transmision occurs & completes.
		TWI_SlaveInitializeDriver(&TB_I2Cs_module, &TWI_SLAVE, *slave_process);
	}
}
