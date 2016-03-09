/*
 * twi.c
 *
 * Created: 3/9/2016 1:29:24 AM
 *  Author: Josh
 */ 

#include "sw.h"
#include "config.h"
#include "twi.h"

#include <stdlib.h>


twi_options_t m_options = {
	.speed     = TWI_SPEED,
	.chip      = TWI_MASTER_ADDR,
	.speed_reg = TWI_BAUD(/*sysclk_get_cpu_hz()*/F_CPU, TWI_SPEED)
};

int stream_count = 0;

void init_twi(void) {
	/* initialize master with settings from config.h */
	twi_master_init(&TWI_MASTER, &m_options);
}

/* Issue read command to slave */
void twi_query(twi_sock_t* sock, uint8_t addr, uint8_t n_out,  uint8_t n_in) {
	
}

/* write to a slave device without expecting any response */
void twi_send(twi_sock_t* sock, uint8_t addr, uint8_t n_out) {
	
}