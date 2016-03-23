/*
 * depth.c
 *
 * Created: 3/9/2016 3:22:09 AM
 *  Author: Josh
 */ 

#include "config.h"
#include "I2Cm.h"


/* Enable TWI interface for communicating with the depth sensor ADC */
void depth_init(void) {
	//do nothing... because initialization is done through I2Cm.c
}

char* depth_get_reading2(char message[3]) {
	status_code_t status;
	uint8_t msg[2] = {0};
	
	/* read data from depth sensor */
	status = I2Cm_read(DEPTH_SENSOR_SLAVE_ADDR, 2, msg);
	
	/* If I2C transmission was a success*/
	if (status == STATUS_OK) {
		message[0] = SW_DEPTH;
		message[1] = msg[0];
		message[2] = msg[1];
		return message;
		
	} else {
		message[0] = SW_ERROR;
		message[1] = TWI_ERROR;
		message[2] = TWI_MASTER.MASTER.STATUS;
		return message;	
		
	}
}