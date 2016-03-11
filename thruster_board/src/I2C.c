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

#define RECV_BUF_DATA_LENGTH     8

/* -----------------------------------
Global Variables
---------------------------------- */
TWI_Slave_t TB_I2Cs_driver;

uint8_t recv_buf[RECV_BUF_DATA_LENGTH] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static void slave_process(void);

/* -----------------------------------
Initialization
-------------------------------------- */

void init_I2C(void) {
	/* initialize the driver and its object */
	TWI_SlaveInitializeDriver(&TB_I2Cs_driver,&TWI_SLAVE, *slave_process);

	/* driver initialized. Now take its object and init TWI system */
	TWI_SlaveInitializeModule(&TB_I2Cs_driver,TWI_SLAVE_ADDR,TWI_SLAVE_INTLVL_MED_gc);
}

void I2C_reload(void) {
	/* initialize the driver and its object */
	TWI_SlaveInitializeDriver(&TB_I2Cs_driver,&TWI_SLAVE, *slave_process);
}

ISR(TWIC_TWIS_vect) {
	TWI_SlaveInterruptHandler(&TB_I2Cs_driver);
}

/* -----------------------------------
Receive operations
-------------------------------------- */

void I2C_recv(data_t msg[], uint8_t n) {
	recv_twi();
	
	/* collect the captured data */
	for(int i=0; i < n; i++) {
		msg[i] = TB_I2Cs_driver.receivedData[i];
	}
	
	/* Note... user may execute I2C_reload immediately outside of this function */
}

void recv_twi(void) {
	while(TB_I2Cs_driver.result != TWIS_RESULT_OK);
}


static void slave_process(void) {
	int i;

	for(i = 0; i < RECV_BUF_DATA_LENGTH; i++) {
		recv_buf[i] = TB_I2Cs_driver.receivedData[i];
	}
}