/*
 * ADC.c
 *
 * Created: 2/14/2016 10:44:13 PM
 *  Author: admin
 */ 

#include "sw.h"
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include "ADC.h"

uint8_t ReadSignatureByte(uint16_t Address);

void ADC_init(void) {
	// Enable the ADC
	ADCA.CTRLA = ADC_ENABLE_bm;
	
	// Configure the ADC
	ADCA.CTRLB = (0<<4); //0 = Unsigned Mode
	ADCA.REFCTRL = (0b001<<4); //Reference set to VCC/1.6
	ADCA.EVCTRL = 0 ; // no events
	ADCA.PRESCALER = ADC_PRESCALER_DIV128_gc ;
	ADCA.CALL = ReadSignatureByte(0x20) ; //ADC Calibration Byte 0
	ADCA.CALH = ReadSignatureByte(0x21) ; //ADC Calibration Byte 1
	//ADCA.SAMPCTRL = This register does not exist
	_delay_us(400); // Wait at least 25 clocks
}

uint16_t ADC_read_sample(uint8_t Channel) {
	if ((ADCA.CTRLA & ADC_ENABLE_bm) == 0) {
		ADC_init();
	}
	
	ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | 1; //Gain=1, Single Ended
	ADCA.CH0.MUXCTRL = (Channel<<3);
	ADCA.CH0.INTCTRL = 0 ; // No interrupt
	//ADCA.CH0.SCAN Another bogus register
	for(uint8_t Waste = 0; Waste<2; Waste++)
	{
		ADCA.CH0.CTRL |= ADC_CH_START_bm; // Start conversion
		while (ADCA.INTFLAGS==0) ; // Wait for complete
		ADCA.INTFLAGS = ADCA.INTFLAGS ;
	}
	
	return ADCA.CH0RES ; 
}

uint8_t ReadSignatureByte(uint16_t Address)
{
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	uint8_t Result;
	__asm__ ("lpm %0, Z\n" : "=r" (Result) : "z" (Address));
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	return Result;
}