/*
 * pwm.h
 *
 * Created: 3/10/2016 12:50:15 AM
 *  Author: Josh
 */ 


#ifndef PWM_H_
#define PWM_H_

#include "sw.h"

typedef enum {
	CCA,
	CCB,
	CCC,
	CCD
} CC_reg_t;
 
typedef struct Pwm_Pin {
	char* description;
	uint8_t pos;
	PORT_t* port;
	TC0_t* pwm_ch; //TCD1.CCA
	CC_reg_t CC;
} pin_pwm_t;

void set_duty60000(pin_pwm_t pin, uint16_t duty);
void pwm_init_pin( pin_pwm_t* pin );


#endif /* PWM_H_ */