/*
 * scheduler.c
 *
 * Created: 3/8/2016 6:29:47 PM
 *  Author: Josh
 */ 

#include "sw.h"

#include <avr/io.h>
#include "scheduler.h"
#include "analog.h"
#include "depth.h"


static volatile int counter = 0;
static char depth_message[3] = {SW_DEPTH, 0, 0};

/* 100 Hz timer */
/* BACKGROUND: TCC0_OVF is automatically cleared upon the
execution of this interrupt. */
ISR(TCC0_OVF_vect) {
	/* Increment counter, roll over at 1000 (once every 10 seconds) */
	counter = (counter + 1) % 1000;
	
	/* Send depth at 10 Hz */
	if (counter % 10 == 0) {
		depth_get_reading2(depth_message);
		serial_send_bytes(depth_message, 3);
	}
	
	/* Send termperature once a second */
	if(counter % 100 == 0) {
		ADCA.CH1.CTRL |= ADC_CH_START_bm;
	}
	
	/* Check batteries every 5 seconds */
	if(counter % 500 == 0) {
		//check_batteries();
	}
	
	/* Check kill status every 100ms */
	if(counter % 10) {
		//check_kill();
	}
	
	update_status(counter);
}

void start_scheduler(void) {
	/* Enable timer 0 on port C. Run timer/counter at 1/64 system clock (500kHz) */
	TCC0.CTRLA = TC_CLKSEL_DIV64_gc;
	TCC0.CTRLB = TC_WGMODE_SS_gc;
	
	/* "Mark" every 10ms (100Hz) with an overflow that will trigger an interrupt. */
	TCC0.PER = 5000;
	
	/* Enable overflow interrupt at low level */
	TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc;
}