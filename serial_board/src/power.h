/*
 * power.h
 *
 * Created: 3/13/2016 3:03:06 AM
 *  Author: Josh
 */ 


#ifndef POWER_H_
#define POWER_H_

#include "sw.h"

void check_batteries(void);
void check_kill(void);
uint8_t power_getPowerBusVoltage(void);
int8_t	power_getKillStatus(void);

#endif /* POWER_H_ */