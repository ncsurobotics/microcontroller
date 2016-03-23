/*
 * analog.c
 *
 * Created: 3/8/2016 8:14:50 PM
 *  Author: Josh
 */ 


#include "analog.h"
#include "sw.h"
#include "tempSense.h"
#include "depth.h"
#include "drivers/twi.h"


void init_analog(void) {
	tempSense_init();
	depth_init();
}