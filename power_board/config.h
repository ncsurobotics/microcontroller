/*
 * power_board_config.h
 *
 * Created: 2/20/2016 5:39:09 AM
 *  Author: Josh
 */ 


#ifndef POWER_BOARD_CONFIG_H_
#define POWER_BOARD_CONFIG_H_

/* Subsystem on/off */
// #define electronics_disabled

/* Remote switches */
#define remote_pwr_switch_enabled


/* TWI */
#define SLAVE_DEVICE
//#define MASTER_DEVICE

#ifdef SLAVE_DEVICE
#ifndef MASTER_DEVICE
	#define TWI_SLAVE				TWIC
	#define TWI_SPEED				50000
	#define TWI_SLAVE_ADDR			0x62
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

#endif /* POWER_BOARD_CONFIG_H_ */