/*
 * tempSense.c
 *
 * Created: 3/9/2016 3:26:40 AM
 *  Author: Josh
 */ 

#include "tempSense.h"
#include "sw.h"

ISR(ADCA_CH1_vect) {
	char message[3];

	message[0] = SW_TEMP;
	message[1] = ADCA.CH1.RESH;
	message[2] = ADCA.CH1.RESL;

	serial_send_bytes(message, 3);
}

void tempSense_init(void) {
	/* Set signed mode */
    ADCA.CTRLB = ADC_CONMODE_bm;
    ADCA.CTRLB = 0x00;

    /* Select 1V internal vref as voltage reference for ADC and enable
       temperature reference */
    ADCA.REFCTRL = ADC_REFSEL_INT1V_gc | ADC_TEMPREF_bm;

    /* Set ADC frequency of 2MHz / 256 */
    ADCA.PRESCALER = ADC_PRESCALER_DIV256_gc;

    /* Set temperature ADC channel settings */
    ADCA.CH1.CTRL = ADC_CH_INPUTMODE_INTERNAL_gc;
    ADCA.CH1.MUXCTRL = ADC_CH_MUXINT_TEMP_gc;
    ADCA.CH1.INTCTRL = ADC_CH_INTLVL_LO_gc;
	
	/* Enable ADC */
	ADCA.CTRLA |= ADC_ENABLE_bm;

}

