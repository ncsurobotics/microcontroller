/*
 * pwm.c
 *
 * Created: 3/10/2016 12:18:35 AM
 *  Author: Josh
 */ 
#include "pwm.h"


//#define DUTY TOP/10 // 10% duty cycle
#define DUTY 0
#define TEST_PIN  2

void pwm_init_pin( pin_pwm_t* pin ) {
	
	/* set pin as output */
	pin->port->DIR |= (1 << pin->pos);
	
	
	
	/* configure the frequency for this pin */
	pin->pwm_ch->PER = TOP;
	pin->pwm_ch->CTRLA = 0x01; // set the prescaler to 1:1
	
	/* enable the appropiate capture compare */
	if ( (pin->CC) == CCA ) {
		pin->pwm_ch->CTRLB |= 1<<4;
		pin->pwm_ch->CCA = DUTY;
		
	} else if  ( (pin->CC) == CCB ) {
		pin->pwm_ch->CTRLB |= 1<<5;
		pin->pwm_ch->CCB = DUTY;
		
	} else if  ( (pin->CC) == CCC ) {
		pin->pwm_ch->CTRLB |= 1<<6;
		pin->pwm_ch->CCC = DUTY;
		
	} else if  ( (pin->CC) == CCD ) {
		pin->pwm_ch->CTRLB |= 1<<7;
		pin->pwm_ch->CCD = DUTY;
		
	}
	
	
	
	/* set waveform generator to PWM mode */
	pin->pwm_ch->CTRLB |= 0x03;
	
}

void set_duty60000(pin_pwm_t pin, uint16_t duty) {
	
	/* enable the appropiate capture compare */
	if ( (pin.CC) == CCA ) {
		pin.pwm_ch->CCA = duty;
		
	} else if  ( (pin.CC) == CCB ) {
		pin.pwm_ch->CCB = duty;
		
	} else if  ( (pin.CC) == CCC ) {
		pin.pwm_ch->CCC = duty;
		
	} else if  ( (pin.CC) == CCD ) {
		pin.pwm_ch->CCD = duty;
		
	}
}