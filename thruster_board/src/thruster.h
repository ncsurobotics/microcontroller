/*
 * thruster.h
 *
 * Created: 3/10/2016 12:36:57 AM
 *  Author: Josh
 */ 


#ifndef THRUSTER_H_
#define THRUSTER_H_

#include "sw.h"
#include "pwm.h"

/* thruster object */
typedef struct Thruster_Struct {
	pin_t dir_pin;
	pin_pwm_t pwm_pin;
	
	int value;
	
} thruster_t;

void setup_thrusterIO(void);
void thruster_set_speed(uint8_t thruster_id, int8_t speed);
void thruster_command_thruster(thruster_t thruster, uint8_t duty, uint8_t dir);






#endif /* THRUSTER_H_ */