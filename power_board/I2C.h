/*
 * I2C.h
 *
 * Created: 3/10/2016 2:22:35 AM
 *  Author: Josh
 */ 


#ifndef I2C_H_
#define I2C_H_

#include "sw.h"

void init_I2C(void);
void recv_twi(void);
void I2C_recv(uint8_t msg[], uint8_t n);
void I2C_reload(void);

#endif /* I2C_H_ */