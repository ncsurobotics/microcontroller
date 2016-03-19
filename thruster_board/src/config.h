/*
 * config.h
 *
 * Created: 3/10/2016 12:28:59 AM
 *  Author: Josh
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_

/* TWI */
#define SLAVE_DEVICE
//#define MASTER_DEVICE

#ifdef SLAVE_DEVICE
#ifndef MASTER_DEVICE
	#define TWI_SLAVE				TWIC
	#define TWI_SPEED				50000
	#define TWI_SLAVE_ADDR			0x60
	#define CONF_TWIM_INTLVL        TWI_MASTER_INTLVL_MED_gc
	#define CONF_PMIC_INTLVL        PMIC_MEDLVLEN_bm
#endif
#endif

#ifdef MASTER_DEVICE
#ifndef SLAVE_DEVICE
	//#define TWI_SLAVE				TWIC
	//#define TWI_SPEED				50000
	//#define TWI_SLAVE_ADDR			0x60
	//#define CONF_TWIM_INTLVL        TWI_MASTER_INTLVL_MED_gc
	//#define CONF_PMIC_INTLVL        PMIC_MEDLVLEN_bm
#endif
#endif

/* PWM config */
#define F_PWM 20e3 // Frequency of the PWM signal controlling motors



#endif /* CONFIG_H_ */