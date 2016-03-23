/*
 * I2Cm.h
 *
 * Created: 3/22/2016 6:12:26 PM
 *  Author: Josh
 */ 


#ifndef I2CM_H_
#define I2CM_H_

#include "utils/status_codes.h"
#include <avr/io.h>
#include <stdbool.h>

void I2Cm_initTWI(void);
status_code_t I2Cm_read(uint8_t slave_addr, uint8_t n, uint8_t *msg);
status_code_t I2Cm_readSlaveRegister(uint8_t slave_addr, uint8_t reg, uint8_t n, uint8_t *buf);


#endif /* I2CM_H_ */