/*
 * power.c
 *
 * Created: 3/13/2016 3:02:31 AM
 *  Author: Josh
 */ 

#include "power.h"
#include "config.h"
#include "I2Cm.h"
#include "central_data_dictionary.h"

/* query power board for battery bus voltage and notify seawolf if voltage is low */
void check_batteries(void) {
	
	/* initialize variables */
	char message[3] = {SW_BATTERY, 0, 0};
	uint8_t voltage[2] = {0};
	
	/* obtain a measurement from the power bus (sensor on power board) */
	power_getPowerBusVoltage_raw(voltage);
	uint16_t power_bus_volts = ((uint16_t)voltage[1])<<8 | ((uint16_t) voltage[0]);
	
	/* if voltage is good */
	if (power_bus_volts > LOW_POWER_VOLTAGE) {
		// do nothing
		
	/* else report to seawolf that battery voltage is low! */
	} else {
		message[1] = LIPO;
		serial_send_bytes(message, 3);
	}
	
}

/* query power board for kill switch status and report to seawolf if there has been a change */
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

void power_getPowerBusVoltage_raw(uint8_t buf[2]) {

	/* send kill_switch status request to power board and recv data*/
	I2Cm_readSlaveRegister(POWER_BOARD_SLAVE_ADDR, POWERBOARD_I2C_VPOWERBUS, 2, buf);
	
}

/* Reports status of the kill switch */
int8_t power_getKillStatus(void) {
	uint8_t robot_killed = -1;
	
	/* send kill_switch status request to power board */ 
	I2Cm_readSlaveRegister(POWER_BOARD_SLAVE_ADDR, POWERBOARD_I2C_KILLSWITCH, 1, &robot_killed);
	
	/* return status of kill switch */
	return robot_killed;
}