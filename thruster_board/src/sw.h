/*
 * sw.h
 *
 * Created: 3/10/2016 12:15:09 AM
 *  Author: Josh
 */ 


#ifndef SW_H_
#define SW_H_

/* CPU speed defined before inclusion of util/delay.h so the delay routines
   are with respect to the the actual clock rate */
#define F_CPU 32000000UL

#include <avr/io.h>
#include <util/delay.h>

/* human readable pin struct data type */
typedef struct PIN_struct {
	char* description;
	uint8_t pos;
	PORT_t* port;
} pin_t;

typedef uint8_t data_t;

#endif /* SW_H_ */