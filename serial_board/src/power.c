/*
 * power.c
 *
 * Created: 3/13/2016 3:02:31 AM
 *  Author: Josh
 */ 

#include "power.h"
#include "config.h"

void check_batteries(void) {
	/* initialize variables */
	char message[3] = {SW_BATTERY, 0, 0};
	
	/* obtain a measurement from the power bus (sensor on power board) */
	uint8_t power_bus_volts = power_getPowerBusVoltage();
	
	/* if voltage is good */
	if (power_bus_volts > LOW_POWER_VOLTAGE) {
		// do nothing
		
	/* else report to seawolf that battery voltage is low! */
	} else {
		message[1] = LIPO;
		serial_send_bytes(message, 3);
	}
	
}

void check_kill(void) {
	static int8_t prev_kill_status = -1;
	
	// initialize variables
	char message[3] = {SW_KILL, 0 , 0};
	
	/* read data from power board via I2C */
	int8_t kill_status = power_getKillStatus();
	
	/* if kill status has changed, notify seawolf */
	if(kill_status != prev_kill_status) {
		/* notify seawolf */
		message[2] = kill_status;
		serial_send_bytes(message, 3);
		
		//update known killswitch state
		prev_kill_status = kill_status;
	}
}

uint8_t power_getPowerBusVoltage(void) {
	static uint8_t i = 0;
	
	/* simulate working power board code */
	return (i++ % 3) + 0xfe;
}

int8_t power_getKillStatus(void) {
	static int8_t i = 0;
	
	if ((i++ % 10) == 1) {
		return 1;
	} else {
		return 0;
	}
}