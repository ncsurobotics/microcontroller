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
#include "twi.h"
#include "config.h"
	
/* ******************************************
**** Copying old microcontroller code *******
********************************************* */
#define test1
//#define test2
//#define test3
//#define test4

static int kill_status = -1;

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
	init_serial();	
	init_twi();
	init_analog();
	init_motors();
	
	/* enable interrupts */
	enable_interrupts();
	
	#ifndef DISABLE_SYNC
	/* stream 0xff as a signal to seawolf that avr is ready for synchronization */
	synchronize_comm();
	#endif
	
	/* after initializing the serial link, start sending depth and
    temperature information */
    start_scheduler(); // periodically sends depth/temp data to seawolf
	
	while(true) {
		serial_read_bytes(command, 3);
		
		switch(command[0]) {
		case SW_RESET:
			software_reset();
			break;
			
		//case SW_NOP:
		//	break;
		
		case SW_MOTOR:
			msg[0] = SW_MOTOR;
			msg[1] = thruster_setThrusterSpeed(command[1], command[2]);
			serial_send_bytes(msg, 3);
			break;

		}
	}

	return 0;
}