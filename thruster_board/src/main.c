/*
 * thruster_board.c
 *
 * Created: 3/10/2016 12:13:23 AM
 * Author : Josh
 */ 

#include "sw.h"
#include "thruster.h"
#include "I2C.h"

#include <avr/interrupt.h>

void enable_interrupts(void) {
	/* Enable all interrupt levels */
	PMIC.CTRL |= (PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm);

	/* Enable interrupts globally */
	sei();
}

void software_reset(void) {
	CCP = CCP_IOREG_gc;
	RST.CTRL = RST_SWRST_bm;
}

int main(void)
{	
		
    /* Enable 32Mhz clock and wait for it to be ready */
    OSC.CTRL |= OSC_RC32MEN_bm;
    while((OSC.STATUS & OSC_RC32MRDY_bm) == 0x00);

    /* Set clock to 32Mhz and then lock it */
    CCP = CCP_IOREG_gc;
    CLK.CTRL = CLK_SCLKSEL_RC32M_gc;
    CLK.LOCK = CLK_LOCK_bm;
	
	/* configure thruster control */
	setup_thrusterIO();
	
	
	
	/* setup I2C */
	enable_interrupts();
	init_I2C();
	
	/* continually get thruster values */
	data_t command[2] = {0, 0};
	PORTC.DIRSET = 1 << 2;
	while (1) {
		/* receive I2C Data */
		I2C_recv(command, 2);
		I2C_reload();
		
		/* New thruster speed*/
		thruster_set_speed(command[0],command[1]);
		
		/* debugging: toggle port to trigger oscilloscope capture */
		PORTC.OUT ^= 1 << 2;
	}

}

