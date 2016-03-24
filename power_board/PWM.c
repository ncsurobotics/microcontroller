/*
 * PWM.c
 *
 * Created: 2/14/2016 11:41:12 PM
 *  Author: admin
 */ 

#ifndef F_CPU
#define F_CPU 1000000UL // 1 MHz clock speed
#endif

#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include "PWM.h"

typedef struct Pin {
    char* name;
    char* function;
    PORT_t* port;
    uint8_t pos;
    char* PWM_designation;
} pin;

pin test_pin = {.name = "PD3",
                .function = "PWM Sample Pin",
                .port = &PORTD,
                .pos = 3,
                .PWM_designation = "OC0D"
};

#define TOP 1000 // period will be 1000us... or 1KHz
#define DUTY 500 // 50% duty cycle
#define TEST_PIN  2

void PWM_init(void) {
	// Configure pin for output
	PORTD.DIR |= 1<<test_pin.pos;    // Set pin to be an output
	
	// Configure the TC
	TCD0.PER = TOP; // Set the period to 1000us.
	TCD0.CTRLA = 0x01; // use a 1:1 prescaler
    TCD0.CTRLB = (1<<7); // enable PWM on  D (should change this to port E later, as port C is supposed to be for uart communication)
	TCD0.CTRLB |= 0x03; // and Set the Waveform Generator to PWM mode
    //TCD0.CCD = DUTY;	//Write the new compare value to CCD
    //Wait for TCOVERFLOW Flag to set (OVFIF) in INTFLAGS
    //Clear the TC Overflow flag	
	
}

void PWM_set1000(uint16_t val) {
    TCD0.CCD = val;
	//PORTD.OUT = 1<<test_pin.pos;
}