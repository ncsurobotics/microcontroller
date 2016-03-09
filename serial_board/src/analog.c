/*
 * analog.c
 *
 * Created: 3/8/2016 8:14:50 PM
 *  Author: Josh
 */ 


#include "analog.h"
#include "sw.h"

#include "tempSense.h"
#include "depth.h"

#include "twi.h"

enum DepthState {
	NOT_RUNNING = 0,
	BYTE1 = 1,
	BYTE2 = 2
};

/*
static bool twi_is_error(void) {
	uint8_t status;
	bool rxack;
	bool arblost;
	bool rif;
	bool wif;
	bool buserr;

	status = TWIE.MASTER.STATUS;

	rxack =  (status & TWI_MASTER_RXACK_bm) != 0;
	arblost = (status & TWI_MASTER_ARBLOST_bm) != 0;
	buserr = (status & TWI_MASTER_BUSERR_bm) != 0;
	rif = (status & TWI_MASTER_RIF_bm) != 0;
	wif = (status & TWI_MASTER_WIF_bm) != 0;

	if((wif && arblost) || (wif && rxack) || buserr) {
		return true;
		} else if(rif && !rxack) {
		return false;
	}

	return true;
}

static void twi_error(void) {
	char message[3];

	message[0] = SW_ERROR;
	message[1] = TWI_ERROR;
	message[2] = TWIE.MASTER.STATUS;

	serial_send_bytes(message, 3);
}
*/

/*	
ISR(TWIE_TWIM_vect) {
	if(twi_is_error()) {
		twi_error();
		depth_state = NOT_RUNNING;
		return;
	}

	switch(depth_state) {
		case NOT_RUNNING:
		return;

		case BYTE1:
		depth_message[1] = TWIE.MASTER.DATA;
		TWIE.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;
		depth_state = BYTE2;
		break;

		case BYTE2:
		depth_message[2] = TWIE.MASTER.DATA;
		TWIE.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc | TWI_MASTER_ACKACT_bm;
		serial_send_bytes(depth_message, 3);
		depth_state = NOT_RUNNING;
		break;
	}
} */



void init_analog(void) {
	tempSense_init();
	depth_init();
}