#ifndef F_CPU
#define F_CPU 1000000UL // 1 MHz clock speed
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "ADC.h"
#include "PWM.h"

#define IO_PORT PORTB
#define MASTER_PWR_PIN  1
#define MASTER_OFF_PIN  0

enum states {
    MASTER_OFF,
    STARTUP,
    MASTER_ON,
	SHUTDOWN,
};

enum states state = MASTER_OFF;

void master_power_on(int* fail);
void master_power_off(int* fail);
int kill_btn_depressed(void);
void hard_ack(void);
void delay_ms( int ms );
int voltage_sensed_on_master_bus(void);

int power_bus_voltage_channel           = 3;
uint16_t POWER_BUS_THRESHOLD_VOLTAGE    = 0x0CF1; //0x0CF1 = 18V


int main(void)
{
	PWM_init();
	PWM_set1000( 500 );
	
	uint16_t V1_12b;
	int pwm_lsb = 4;
	
    int fail = 0;
    // initialize IO
    IO_PORT.DIR = 1<<MASTER_PWR_PIN; 
    IO_PORT.PIN0CTRL = 0b011<<3;
    
    while (1) {
        switch(state) {
        
        case MASTER_OFF:
            if( voltage_sensed_on_master_bus() ) {state = STARTUP;}
            
        case STARTUP:
            master_power_on(&fail);
            if ( !fail ) {
                state = MASTER_ON;
            } else {
                state = MASTER_OFF;
            }
            break;
        
        case MASTER_ON: //this is essentially the idle phase
     
            // Enter shutdown mode if user presses kill button.
            if (kill_btn_depressed()==1) {
                state = SHUTDOWN;
            }
            break;
            
        case SHUTDOWN:
            master_power_off(&fail);
            if ( !fail ) {
                state = MASTER_OFF;
            } else {
                state = MASTER_ON;
            }
            break;
            
        }
        
        //Remove this when ready to write UART Code.
        V1_12b = ADC_read_sample( power_bus_voltage_channel );
        PWM_set1000( V1_12b/pwm_lsb -23 );
        
        _delay_ms( 0 );
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



void master_power_on(int* fail)
{
    if ( kill_btn_depressed()==0 ) {
        IO_PORT.OUT = 1<<MASTER_PWR_PIN;    //Turn on the robot.
        *fail = 0;
    } else {
        *fail = 1;
    }
}

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

void hard_ack(void) {
    int dt = 100;
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