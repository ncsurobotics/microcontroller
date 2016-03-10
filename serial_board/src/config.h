/*
 * config.h
 *
 * Created: 3/9/2016 1:34:33 AM
 *  Author: Josh
 */ 

#include "sw.h"

#ifndef CONFIG_H_
#define CONFIG_H_

/* TWI */
#define TWI_MASTER				TWIE
#define TWI_MASTER_PORT			PORTE
#define TWI_SPEED				50000
#define TWI_SLAVE_ADDR1			0x60
#define TWI_MASTER_ADDR			0x50
#define CONF_TWIM_INTLVL        TWI_MASTER_INTLVL_MED_gc
#define CONF_PMIC_INTLVL        PMIC_MEDLVLEN_bm

/* Depth Sensor */
#define DEPTH_TWI_ADDR_SLAVE	0x9A

/* Thruster board TWI */
#define THRUSTER_TWI_ADDR		0x60


#endif /* CONFIG_H_ */