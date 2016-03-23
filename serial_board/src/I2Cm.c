/*
 * I2C.c
 *
 * Created: 3/22/2016 6:10:56 PM
 *  Author: Josh
 */ 

#include "I2Cm.h"
#include "drivers/twim.h"
#include "sw.h"
#include "config.h"

/**********************************
**** Settings *********************
***********************************/

/* options are fairly simple. They actually are configured in config.h */
static twi_options_t m_options = {
	.speed     = TWI_SPEED,
	.chip      = TWI_MASTER_ADDR,
	.speed_reg = TWI_BAUD(F_CPU, TWI_SPEED)
};

/**********************************
**** FUNCTIONS ********************
***********************************/

/* Initialize I2C for the entire chip */
void I2Cm_initTWI(void) {
	twi_master_init(&TWI_MASTER, &m_options);
}

/* Read from a slave device while specifying what data (register) the master wants to read */
void I2Cm_readSlaveRegister(uint8_t addr, uint8_t reg, uint8_t n, uint8_t *buf) {
	
}

/* Blind read from a specified slave device */
status_code_t I2Cm_read(uint8_t slave_addr, uint8_t n, uint8_t *msg) {
	/* create variable for transmission */
	twi_package_t packet = {
		.chip			= slave_addr,
		.addr_length	= 0,
		.buffer			= (void *)msg,
		.length			= n,
		.no_wait		= false
	};
	
	/* issue (read) transmission to depth sensor. This is a blocking operation. */
	return twi_master_transfer(&TWI_MASTER, &packet, true);
}