/*
 * config.h
 *
 * Created: 3/9/2016 1:34:33 AM
 *  Author: Josh
 */ 

#include "sw.h"

#ifndef CONFIG_H_
#define CONFIG_H_

/* debug */
//#define DISABLE_SYNC //useful for jumping to a section of code

/* TWI */
#define TWI_MASTER				TWIE
#define TWI_MASTER_PORT			PORTE
#define TWI_SPEED				50000
#define TWI_MASTER_ADDR			0x50

/* Depth Sensor */
#define DEPTH_SENSOR_SLAVE_ADDR	0x9A>>1 // 0x9A is what it says on the data sheet... but
										// it didnt explain how the 8th bit is actually r/w
										// bit. so actually, the address 0x4b, but we use
										// 0x9A>>1 instead.

/* Thruster board TWI */
#define THRUSTER_BOARD_SLAVE_ADDR	0x60

/* ******* POWER ****
******************* */ 
#define POWER_BOARD_SLAVE_ADDR	0x62
#define LOW_POWER_VOLTAGE		0xfe

#endif /* CONFIG_H_ */