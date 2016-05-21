#include "sw.h"
#include <avr/io.h>
#include <util/delay.h>
#include "ADC.h"
#include "PWM.h"
#include "config.h"
#include "I2C.h"
#include "avr/interrupt.h"

#define IO_PORT PORTB
#define MASTER_PWR_PIN  1
#define MASTER_OFF_PIN  0
#define MESSAGE_IS_ROBOT_KILLED 0
#define LPF_BUFFER_SIZE 255 //do not make greater than 255.
//^^ yields about 5 second low pass filter for voltage detection

enum states {
    MASTER_OFF,
    STARTUP,
    MASTER_ON,
	SHUTDOWN,
};

typedef struct lpf_struct
{
	uint16_t*	buffer;
	uint8_t		n_samples;
	uint8_t		head;
	uint16_t	value;
} lpf_t;

enum states state = MASTER_OFF;

static void slave_process(uint8_t *receivedData, uint8_t *sendData);

void init_io(void);
void close_master_relay(int* fail);
void master_power_off(int* fail);
int kill_btn_depressed(void);
void hard_ack(void);
void delay_ms( int ms );
int voltage_sensed_on_master_bus(void);
int check_remote_power_switch(void);
void default_startup(int* fail);
int toggl_sw_state(void);
int remote_pwr_toggled(void);
int voltage_sensed_on_master_bus_long(lpf_t *data);

// telemetry function
void tm_sample_all(void);

int power_bus_voltage_channel           = 3;
uint8_t thruster_bus_voltage_channel		= 4;
uint16_t POWER_BUS_THRESHOLD_VOLTAGE    = 0x0A80; //0x0A80 = ~18.3V
uint8_t robot_killed = 0;


/* ****************************
Typical GPIO Pin on the board
***************************** */

pin_t SHDN_Elec_pin = {.name="!SHDN Electronics",
	.description="GPIO for controlling group A electronics",
	.pos=2,
	.port=&PORTB,
};

pin_t remote_pwr_pin = {.name="Remote_Pwr_Toggle",
	.description="Input for toggling Main Robot Power",
	.pos=2,
	.port=&PORTC,
};


int main(void)
{
	// Debug pin
	PORTD.DIR |= 1<<2;
		
	/* enable interrupts */
    PMIC.CTRL |= (PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm);
    sei(); // enable global interrupts.
	
	/* initialize modules */
	init_I2C(&slave_process);
	PWM_init();
	PWM_set1000( 500 );
	
	/* other variables */
	uint16_t V1_12b;
	int pwm_lsb = 4;
    int fail = 0; // initial our fail flag to 0.
	
    /* initialize IO pins */
	init_io();
	int pause_toggle_power_switch = 0;
	
	/* low_pass filter */
	uint16_t v1_buffer[LPF_BUFFER_SIZE] = {1<<12}; //1<<12 is full scale ADC
		
	//short hand array initialization didn't work, resorting to for-loop
	for(int i=0; i < LPF_BUFFER_SIZE; i++) { 
		v1_buffer[i] = 1<<12;
	}
		
	lpf_t v1_voltage_lpf = {.buffer = v1_buffer, 
		.n_samples = LPF_BUFFER_SIZE,
		.head = 0,
		.value = v1_buffer[0]};
		
	
	/* main loop */
    while (1) {
        switch(state) {
        
        case MASTER_OFF:
			//hard_ack();
            if( voltage_sensed_on_master_bus() ) {state = STARTUP;}
			if( remote_pwr_toggled()) {state = STARTUP;}
			break;
            
        case STARTUP:
            default_startup(&fail);
            if ( !fail ) {
                state = MASTER_ON;
            } else {
                state = MASTER_OFF;
            }
			
			//Reset the V1 LPF buffer
			for(int i=0; i < LPF_BUFFER_SIZE; i++) {
				v1_buffer[i] = 1<<12; //full scale for 12 bit adc
			}
			
            break;
        
        case MASTER_ON: //this is essentially the idle phase
     
            // Enter shutdown mode if user presses kill button.
            if (kill_btn_depressed()==1) {
                state = SHUTDOWN;
            }
			// If low voltage detected, force turn off the robot.
			if( !voltage_sensed_on_master_bus_long(&v1_voltage_lpf) ) {state = SHUTDOWN;}
			//if( !voltage_sensed_on_master_bus() ) {state = SHUTDOWN;}
			
			// Check kill switch (low voltage means kill robot)
			robot_killed = ADC_read_sample( thruster_bus_voltage_channel ) < 0x09E5;
			if (robot_killed) {
				PORTD.OUT &= ~(1<<2);
			} else {
				PORTD.OUT |= 1<<2;
			}
			
			if( remote_pwr_toggled()) {state = SHUTDOWN;}
            break;
            
        case SHUTDOWN:
            if ( !fail ) {
				master_power_off(&fail);
                state = MASTER_OFF;
            } else {
                state = MASTER_ON;
            }
			
			_delay_ms( 50 );
            break;
            
        }
		
		
		/* ubiquitous ADC sampling code (should be in a function called ADC_updateVPwrBus() */
		//V1_12b = ADC_read_sample( power_bus_voltage_channel );
		//PWM_set1000( V1_12b/pwm_lsb -23 ); //raw
		PWM_set1000( v1_voltage_lpf.value/pwm_lsb -23 ); //low pass filter
		
		/* ubiquitous Remote toggle switch code */
		if (!pause_toggle_power_switch) {
			check_remote_power_switch();
		}
    }
        
}

/* ------------------------------
 Main loop, DiscretE IO & Relays
-------------------------------- */

void init_io(void) {
	// initialize the master relay
	IO_PORT.DIR |= 1<<MASTER_PWR_PIN;
	
	// initialize the power board shut down button
	IO_PORT.PIN0CTRL = 0b011<<3; //Set kill button to use a pull up resistor.
	
	// initialize downstream electronics power supply
	#ifndef electronics_disabled
	SHDN_Elec_pin.port->DIR |= 1<<SHDN_Elec_pin.pos;
	SHDN_Elec_pin.port->OUT &= ~(1<<SHDN_Elec_pin.pos);
	#endif
	
	// initialize remote power control
	#ifdef remote_pwr_switch_enabled
	remote_pwr_pin.port->DIR &= ~(1<<remote_pwr_pin.pos);
	remote_pwr_pin.port->OUT &= ~(1<<remote_pwr_pin.pos);
	remote_pwr_pin.port->PIN2CTRL = 0b011<<3; // set toggle switch to use a pull up resistor
	#endif
}

int remote_pwr_toggled(void) {
	// initializition
	static int prev_toggl_switch_state = 0;
	int ret_val = 0;
	
	
	if (prev_toggl_switch_state == 0) {
		// check toggle switch
		if (toggl_sw_state()  == 1) {
			prev_toggl_switch_state = 1;
		}
		
	} else if (prev_toggl_switch_state == 1) {
		// check toggle switch
		if (toggl_sw_state() == 0 ) {
			prev_toggl_switch_state = 0;
			ret_val = 1;
		}
	}
	

	return ret_val;
	
}

// Checks the state of the toggle switch, and return something that indicates whether it has been activated or not
int toggl_sw_state(void) { 
	int val = (remote_pwr_pin.port->IN & (1<<remote_pwr_pin.pos)); //
	return (val == 0); //toggle switch is active low
}

/* energize the master relay and power the computer as long as the "off"
button isn't being pressed at the same time. */
void default_startup(int* fail) {
	close_master_relay(fail);
	if ( *fail == 1 ) {
		return;
	}
	
	#ifndef electronics_disabled
	SHDN_Elec_pin.port->OUT |= 1<<SHDN_Elec_pin.pos;
	#endif
}

/* will energize mater relay as long as "off" button is not being depressed at
same time. */
void close_master_relay(int* fail)
{
    if ( kill_btn_depressed()==0 ) {
        IO_PORT.OUT = 1<<MASTER_PWR_PIN;    //Turn on the robot.
        *fail = 0;
    } else {
        *fail = 1;
    }
}

/* Checks the GPIO wired to the push button for turning off the robot. This 
button is labeled "kill", though this isn't to be confused with the 
"kill switch" located outside the robot for turning on/off the thruster
circuit.
  parameters:
    - fail: equals 0 if success.*/
void master_power_off(int* fail)
{
    IO_PORT.OUT &= ~(1<<MASTER_PWR_PIN);
    *fail = 0;
}

int kill_btn_depressed(void) {
    if ( IO_PORT.IN & (1<<MASTER_OFF_PIN) ) {
        return 0; //kill robot is not depressed
        
    } else {
        return 1; //kill robot btn is depressed (B0 sees a GND)
    }
}

int voltage_sensed_on_master_bus(void) {
	int val = ADC_read_sample( power_bus_voltage_channel );
	if (val >= POWER_BUS_THRESHOLD_VOLTAGE ) {
		return 1;
	} else {
		return 0;
	}
}

/* not the most efficient low pass filter, but it gets the job done. */
int voltage_sensed_on_master_bus_long(lpf_t *data) {
	//int val = ADC_read_sample( power_bus_voltage_channel );
	//write new value
	 data->buffer[data->head] = ADC_read_sample( power_bus_voltage_channel );
	 
	//increment pointer
	data->head = (data->head + 1) % data->n_samples;
	
	//sum all values
	float sum = 0;
	for(uint8_t i=0; i < data->n_samples; i++) {
		sum += (float) data->buffer[i];
	}
	
	//shift down to divide
	data->value = (uint16_t) (sum / data->n_samples);
	
	if (data->value >= POWER_BUS_THRESHOLD_VOLTAGE ) {
		return 1;
	} else {
		return 0;
	}
}

void hard_ack(void) {
    int dt = 400;
    int pause = 4;
    int wait = pause*dt;
    int taps[] = {dt,dt,pause*dt};
    int n = 3;
    int i;
    
    //first click
    IO_PORT.OUT ^= 1<<MASTER_PWR_PIN;
    
    for (i=0; i<n; i++) {
        delay_ms( taps[i] );
        IO_PORT.OUT ^= 1<<MASTER_PWR_PIN;
    }
    
    delay_ms( wait );
}

void delay_ms( int ms )
{
    int i;
    for (i=0; i < ms; i++)
    {
      _delay_ms(1);
    }
}

static int prev_remote1_state = 0;
int check_remote_power_switch(void) {
	// init
	int ret_val = 0;
	
	// determine whether looking for a rising or falling edge.
	int remote1_current_state = (remote_pwr_pin.port->IN) & (1<<remote_pwr_pin.pos);
	
	if (prev_remote1_state == 0) {
		if ( remote1_current_state == 1 ) {				// see if signal has gone high
			prev_remote1_state = remote1_current_state; // rising edge detected.
		} else {
			// do nothing.
		}
		
	// previous state was 1. Looking for a falling edge.
	} else {
		if (remote1_current_state == 0) {	// see if signal has gone low
			ret_val = 1;					// falling edge detected.
			prev_remote1_state = remote1_current_state; // save state.
		} else {
			// do nothing
		}
	}
	
	return ret_val;
}


/* ------------------------------
// Telemetry Section
-------------------------------- */

// new pins
pin_t V1_pin = {.name="V1",
	.description="Measures Main Power Bus Voltage",
	.pos=3,
	.port=&PORTA,
};

pin_t V2_pin = {.name="V2",
	.description="Measures Thruster Power Bus Voltage",
	.pos=4,
	.port=&PORTA,
};

pin_t I1_pin = {.name="I1",
	.description="Measures Main Power Bus Current",
	.pos=0,
	.port=&PORTA,
};

pin_t I2_pin = {.name="I2",
	.description="Measures Thruster Power Bus Current",
	.pos=1,
	.port=&PORTA,
};

// grouping structures
typedef struct tm_channel_struct {
	pin_t* pin;
	uint16_t result;
} tm_channel_t;

tm_channel_t V1_chan = {.pin=&V1_pin};
tm_channel_t V2_chan = {.pin=&V2_pin};
tm_channel_t I1_chan = {.pin=&I1_pin};
tm_channel_t I2_chan = {.pin=&I2_pin};

typedef struct telemetry_struct {
	tm_channel_t* V1;
	tm_channel_t* I1;
	
	tm_channel_t* V2;
	tm_channel_t* I2;
} tm_t;

tm_t POWERBOARD_tm = {.V1 = &V1_chan,
	.V2 = &V2_chan,
	.I1 = &I1_chan,
	.I2 = &I2_chan,
};

void tm_sample_all(void) {
	// code to sample V1,V2,I1,I2 in one pass.
	return;
}

/*
 * Handle incoming message
 */
static void slave_process(uint8_t *receivedData, uint8_t *sendData) {
	PORTE.DIR |= 1<<3;
	PORTE.OUT ^= 1<<3;
	PORTE.OUT ^= 1<<3;
	switch (receivedData[0]) {
	case MESSAGE_IS_ROBOT_KILLED:
		// Request for kill switch state
		sendData[0] = robot_killed;
		break;
	default:
		// Unknown message type
		sendData[0] = -1; //error msg
		break;
	}
}