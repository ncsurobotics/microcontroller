/*
 * depth.c
 *
 * Created: 3/9/2016 3:22:09 AM
 *  Author: Josh
 */ 

#include "config.h"
#include "twi.h"

enum DepthState {
	NOT_RUNNING = 0,
	BYTE1 = 1,
	BYTE2 = 2
};
	
#define MAX 128

twi_sock_t depth_sock = {
	.bufr = {0},
	.r_count = 0,
	.bufw = {0},
	.w_count = 0,
};

void depth_init(void) {
	/* Enable TWI interface for communicating with the depth sensor ADC */
	//TWI_MASTER.MASTER.BAUD = 155;
	//TWI_MASTER.MASTER.CTRLA = TWI_MASTER_INTLVL_LO_gc | TWI_MASTER_RIEN_bm | TWI_MASTER_WIEN_bm | TWI_MASTER_ENABLE_bm;
	//twi_attach(&depth_sock);
	
	// do nothing
}

char* depth_get_reading2(char message[3]) {
	status_code_t status;
	uint8_t* msg = {0};

	/* Force bus to idle */
	//TWIE.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;

	/* create variable for transmission */
	twi_package_t packet = {
		.chip			= DEPTH_TWI_ADDR_SLAVE,
		.addr_length	= 0,
		.buffer			= (void *)msg,
		.length			= 2,
		.no_wait		= false
	};
	
	/* issue (read) tranmssion to depth sensor. This is a blocking operation. */
	status = twi_master_read(&TWI_MASTER, &packet);
	
	/* If I2C transmission was a success*/
	if (status == STATUS_OK) {
		message[0] = SW_DEPTH;
		message[1] = *((uint8_t*) packet.buffer + 0);
		message[2] = *((uint8_t*)packet.buffer + 1);
		
		return message;
	} else {
		message[0] = SW_ERROR;
		message[1] = TWI_ERROR;
		message[2] = TWI_MASTER.MASTER.STATUS;
		
		return message;
	}
}