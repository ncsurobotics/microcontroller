/**
 * \file
 *
 * \brief Serial Board Source Code
 *
 *
 
 */

#include "sw.h"

#include "uart.h"
#include "analog.h"
#include "I2Cm.h"
#include "config.h"
	
/* ******************************************
**** Copying old microcontroller code *******
********************************************* */


void enable_interrupts(void);
void software_reset(void);
void synchronize_comm(void);

void enable_interrupts (void) {
	/* Enable all interrupt levels */
	PMIC.CTRL |= (PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm);
	
	/* Enable interrupts globally */
	sei();
}

void software_reset(void) {
	CCP = CCP_IOREG_gc;
	RST.CTRL = RST_SWRST_bm;
}

pin_t top_left = {
	.port = &PORTC,
	.pos = 2,
};


void synchronize_comm(void) {
	/* Send 0xFF until 0x00 is received */
	while(true) {
		serial_send_byte(0xff);
		
	
		/* if data in the buffer is available, read the buffer. If the
		byte read is 0x00, then break out of this while loop.*/
		if(serial_available() && (serial_read_byte() == 0x00)) {
			break;
		}
	}
	
	/* if there is any more data available after sending 0x00... that is
	a problem. Send an error/notification to seawolf. There is no 
	code on seawolf to catch this... so, it'll probably screw up
	some code on seawolf's side of things if this happens. */
	if(serial_available()) {
		char command[3] = {
			SW_ERROR,
			SYNC_ERROR,
			0
		};
		
		serial_send_bytes(command, 3);
	}
	
	
	serial_send_byte(0xf0);
}

static void invalid_request(char command) {
	char message[3] = {SW_ERROR, INVALID_REQUEST, command};
	serial_send_bytes(message, 3);
}

char msg[3] = {0,0,0};

int main (void) {
	char command[3] = {0};
	top_left.port->DIR |= 1<<top_left.pos;
	
	/* Enable 32MHz clock and wait for it to be ready */
	OSC.CTRL |= OSC_RC32MEN_bm;
	while((OSC.STATUS & OSC_RC32MRDY_bm) == 0x00);
	
	/* Set clock to 32Mhz and then lock it */
    CCP = CCP_IOREG_gc;
    CLK.CTRL = CLK_SCLKSEL_RC32M_gc;
    CLK.LOCK = CLK_LOCK_bm;
	
	/* initialize peripherals */
	//init_status(); // for debugging purposes only
	init_serial();	
	I2Cm_initTWI();
	init_analog();
	init_motors();
	
	/* enable interrupts */
	enable_interrupts();
	
	#ifndef DISABLE_SYNC
	/* stream 0xff as a signal to seawolf that avr is ready for synchronization */
	synchronize_comm();
	#endif
	
	#ifdef DISABLE_SYNC
		#pragma message ("Comm synchronization disabled.")
	#endif
	
	/* after initializing the serial link, start sending depth and
    temperature information */
    start_scheduler(); // periodically sends depth/temp data to seawolf
	
	while(true) {
		/* WARNING: do not put I2C commands inside of this loop. Doing so will create many opportunities for
		I2C stuff embedded in the interrupt driven scheduler (see schedule.c) to interrupt whatever I2C code you write here,
		which will create some really hard to debug I2C "latch-ups" that will be seemingly random and hard to debug. Really, it
		all is because I2C commands on the serial board commands aren't really supposed to be interrupted by other I2C commands on the serial board).
		Rather, the best practice for asynchronous I2C request from seawolf is to handle them just like requests: Quietly note that a
		change/command on the I2C bus is being requested, and let the scheduler handle when that request is actually performed.
		This is done perfectly by the thruster_thrusterRequest(...) command, and future I2C stuff should be done in a similar fashion. */
		
		serial_read_bytes(command, 3);
		
		switch(command[0]) {
		case SW_RESET:
			software_reset();
			break;
		
		case SW_NOP:
			break;
		
		case SW_MOTOR:
			thruster_thrusterRequest(command[1], command[2]);
			
			break;
			
		case SW_STATUS:
			set_status((uint8_t)command[2]);
			break;
			
		case SW_TEMP:
			ADCA.CH1.CTRL |= ADC_CH_START_bm;
			break;
			
		default:
			invalid_request(command[0]);
			realign_buffer();
			break;
		}
	}
	
	return 0;
}