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
static void *slave_process;

/* -----------------------------------
Initialization
-------------------------------------- */

void init_I2C(void *slave_process_param) {
	/* initialize the driver and its object */
	slave_process = slave_process_param;
	TWI_SlaveInitializeDriver(&TB_I2Cs_driver, &TWI_SLAVE, slave_process_internal);

	/* driver initialized. Now take its object and init TWI system */
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

static void slave_process_internal() {
	slave_process(TB_I2Cs_driver.receivedData, TB_I2Cs_driver.sendData);
}