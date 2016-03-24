/*
 * I2C.c
 *
 * Created: 3/10/2016 2:21:24 AM
 *  Author: Josh
 */ 

#include "config.h"

#include "I2C.h"
#include "twi.h"
#include <avr/interrupt.h>

/* -----------------------------------
Global Variables
---------------------------------- */
TWI_Slave_t TB_I2Cs_driver;

static void slave_process_internal();
static void (*slave_process)(register8_t *, register8_t *);

/* -----------------------------------
Initialization
-------------------------------------- */

void init_I2C(void *user_slave_process) {
	/* generate a handle for the slave-read data processing function. */
	slave_process = user_slave_process;
	/* ^^^ this function gets passed to the real TWI driver. */
	
	/* initialize the driver and its object */
	TWI_SlaveInitializeDriver(&TB_I2Cs_driver, &TWI_SLAVE, slave_process_internal);
	/* ^^^ slave_process_internal(...) run only when the slaveReadHandler executes.
	i.e., when the master has called for a write operation (i.e. master has written data to the slave) */

	/* driver initialized. Now take its object and init the TWI system */
	TWI_SlaveInitializeModule(&TB_I2Cs_driver, TWI_SLAVE_ADDR, TWI_SLAVE_INTLVL_MED_gc);
}

void I2C_reload(void *slave_process_param) {
	/* initialize the driver and its object */
	slave_process = slave_process_param;
	TWI_SlaveInitializeDriver(&TB_I2Cs_driver, &TWI_SLAVE, slave_process_internal);
}


ISR(TWIC_TWIS_vect) {
	TWI_SlaveInterruptHandler(&TB_I2Cs_driver);
}

/* The definition of slave_process is not in this file. Rather, it was supplied
by the user as an argument in init_I2C(...). This supplied function shall take
two pointers to 8 bit blocks of data. one block will be for received data, while
the other block will be for sent data. */
static void slave_process_internal() {
	slave_process(TB_I2Cs_driver.receivedData, TB_I2Cs_driver.sendData);
}