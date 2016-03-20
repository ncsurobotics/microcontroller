/*
 * I2C.h
 *
 * Created: 3/10/2016 2:22:35 AM
 *  Author: Josh
 */ 


#ifndef I2C_H_
#define I2C_H_

#include "sw.h"

#define RECV_BUF_DATA_LENGTH 8

void init_I2C(void *slave_process);
void I2C_reload(void *slave_process);

#endif /* I2C_H_ */