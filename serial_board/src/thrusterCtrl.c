#include "sw.h"
#include "drivers/twi.h"
#include "config.h"

#include <stdlib.h>

#define THRUSTER_N_THRUSTERS 6

typedef struct thruster_struct {
	bool updated;
	int value;
} thruster_t;

// initialize thruster array data type
static thruster_t thruster[THRUSTER_N_THRUSTERS];

void init_motors(void) {

	// initialize thruster data buffer
    for(int i=0; i<THRUSTER_N_THRUSTERS; i++) {
		
		thruster[i].updated = true;
		thruster[i].value = 0;
	}
}

void thruster_thrusterRequest(Motor motor, int speed) {
	/* quietly requests a thruster speed change. The speed change will
	be applied the next time (...)_applyThrusterUpdates() is ran.
	ARGS:
	  * motor: motor specifying which thruster the speed change should be applied to.
	  * speed: the new speed of the motor. */
	
	// update the thruster data-buffer
	thruster[motor].value = speed;
	thruster[motor].updated = true;
}

void thruster_applyThrusterUpdates(void) {
	/* applies all requested update to the thrusters. */
	
	// apply as many updates as necessary
	for(int i=0; i<THRUSTER_N_THRUSTERS; i++) {
		
		// if a new value for this thruster has just come in from seawolf, update the thruster.
		if (thruster[i].updated==true) {
			// send command to thruster board
			thruster_setThrusterSpeed(i, thruster[i].value);
			
			// reset update status
			thruster[i].updated = false;
		}
	}
}

/* Set speed of motor. Value is -128 to 128 */
int thruster_setThrusterSpeed(Motor motor, int speed) {
	/* issues a command to change the speed of a specific thruster.
	ARGS:
	  * motor: specified thruster.
	  * speed: desired speed of the thruster. */
	
	status_code_t status;

	/* create variable for transmission */
	twi_package_t packet = {
		.chip			= THRUSTER_BOARD_SLAVE_ADDR,
		.addr[0]		= motor,
		.addr_length	= 1,
		.buffer			= &speed,
		.length			= 1,
		.no_wait		= false
	};
	
	/* issue (write) transmission to thrusterboard. This is a blocking operation. */
	status = twi_master_write(&TWI_MASTER, &packet);
	
	/* If I2C transmission was a success*/
	if (status == STATUS_OK) {
		return 0;
		
	} else {
		return status;
	}
}