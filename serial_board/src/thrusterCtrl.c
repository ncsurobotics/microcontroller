#include "sw.h"
#include "twi.h"
#include "config.h"

#include <stdlib.h>

void init_motors(void) {
    // nothing to init
}

/* Set speed of motor. Value is -128 to 128 */
int thruster_setThrusterSpeed(Motor motor, int speed) {
	status_code_t status;
	
	//uint8_t buffer[2] = {motor, speed};

	/* create variable for transmission */
	twi_package_t packet = {
		.chip			= THRUSTER_TWI_ADDR,
		.addr[0]		= motor,
		.addr_length	= 1,
		.buffer			= &speed,
		.length			= 1,
		.no_wait		= false
	};
	
	/* issue (write) tranmssion to thrusterboard. This is a blocking operation. */
	status = twi_master_write(&TWI_MASTER, &packet);
	
	/* If I2C transmission was a success*/
	if (status == STATUS_OK) {
		return 0;
		
	} else {
		return status;
	}
}