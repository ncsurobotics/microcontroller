/*
 * PWM.h
 *
 * Created: 2/14/2016 11:41:24 PM
 *  Author: admin
 */ 


#ifndef PWM_H_
#define PWM_H_

#define TOP 1000 // period will be 1000us... or 1KHz
#define DUTY 500 // 50% duty cycle

void PWM_init(void);
void PWM_set1000(uint16_t val);

#endif /* PWM_H_ */