/*
 * CFile1.c
 *
 * Created: 3/10/2016 12:16:03 AM
 *  Author: Josh
 */ 

#include "thruster.h"
#include "sw.h"
#include "pwm.h"

#define SCALE60000	TOP/127

typedef enum {
	PORT	= 0,
	STAR	= 1,
	STERN	= 2,
	BOW		= 3,
	STRAFET	= 4,
	STRAFEB = 5,
	n_thrusters = 6
} thruster;

/* thruster array */
/*thruster_t*/ //thrusters[6];

thruster_t port_thruster = {
	.dir_pin = {.port=&PORTA, .pos=1},
	.pwm_pin = {.port=&PORTC, .pos=4, .pwm_ch=&TCC1, .CC=CCA},
	.value = 0,
};

thruster_t star_thruster = {
	.dir_pin = {.port=&PORTA, .pos=2},
	.pwm_pin = {.port=&PORTC, .pos=5, .pwm_ch=&TCC1, .CC=CCB},
	.value = 0,
};

thruster_t stern_thruster = {
	.dir_pin = {.port=&PORTA, .pos=3},
	//.pwm_pin = {.port=&PORTC, .pos=6, .pwm_ch=&TCC0, .CC=CCD}, //
	.pwm_pin = {.port=&PORTE, .pos=0, .pwm_ch=&TCE0, .CC=CCA},
	.value = 0,
};

thruster_t bow_thruster = {
	.dir_pin = {.port=&PORTA, .pos=4},
	//.pwm_pin = {.port=&PORTC, .pos=7, .pwm_ch=&TCC0, .CC=CCD},
	.pwm_pin = {.port=&PORTE, .pos=1, .pwm_ch=&TCE0, .CC=CCB},
	.value = 0,
};

thruster_t strafet_thruster = {
	.dir_pin = {.port=&PORTA, .pos=5},
	.pwm_pin = {.port=&PORTD, .pos=0, .pwm_ch=&TCD0, .CC=CCA},
	.value = 0,
};

thruster_t strafeb_thruster = {
	.dir_pin = {.port=&PORTA, .pos=6},
	.pwm_pin = {.port=&PORTD, .pos=1, .pwm_ch=&TCD0, .CC=CCB},
	.value = 0,
};

static thruster_t thruster_array[6];

void setup_thrusterIO (void) {
	/* generate IO */
	thruster_array[PORT] = port_thruster;
	thruster_array[STAR] = star_thruster;
	thruster_array[STERN] = stern_thruster;
	thruster_array[BOW] = bow_thruster;
	thruster_array[STRAFET] = strafet_thruster;
	thruster_array[STRAFEB] = strafeb_thruster;
	
	//PORTA.DIR |= 1<<3;
	//PORTA.OUT |= 1<<2;
	//thruster_array[2].dir_pin.port->OUT |= (1 << thruster_array[2].dir_pin.pos);
	//while(1) {}
	
	for (int i=0; i < 6; i++) {
		/* init direction pins */
		thruster_array[i].dir_pin.port->OUT |= (1 << thruster_array[i].dir_pin.pos);
		thruster_array[i].dir_pin.port->DIR |= (1 << thruster_array[i].dir_pin.pos);
		
		/* init pwm pins */
		pwm_init_pin( &(thruster_array[i].pwm_pin) );
	}
	
	
}
void config_thruster_array(thruster_t** thruster_array) {
}

void thruster_init_thruster(thruster_t* thruster) {

}

void thruster_set_speed(uint8_t thruster_id, int8_t speed) {
	uint8_t dir;
	uint16_t duty_cycle;
	
	if (speed >= 0) {
		dir = 1;
		duty_cycle = (uint8_t) speed;
	} else {
		dir = 0;
		duty_cycle = (uint8_t) (-speed);
	}
	
	if (duty_cycle > 127) {duty_cycle = 127;}
	
	/* update thruster value over I2C */
	thruster_command_thruster(thruster_array[thruster_id], duty_cycle, dir);
}

void thruster_command_thruster(thruster_t thruster_obj, uint8_t duty, uint8_t dir) {
	set_duty60000(thruster_obj.pwm_pin, SCALE60000*(uint16_t)duty );
	if (dir == 0) {
			thruster_obj.dir_pin.port->OUT &= ~(1 << thruster_obj.dir_pin.pos);
		} else {
			thruster_obj.dir_pin.port->OUT |= (1 << thruster_obj.dir_pin.pos);
		}
}



