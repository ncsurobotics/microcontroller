#ifndef F_CPU
#define F_CPU 1000000UL // 16 MHz clock speed
#endif

#include <avr/io.h>
#include <util/delay.h>

#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m))
#define BIT(x)	(0x01 << (x))

#define IO_PORT PORTB
#define MASTER_PWR_PIN  1
#define MASTER_OFF_PIN  0

enum states {
    MASTER_OFF,
    STARTUP,
    MASTER_ON,
    LED_OFF,
};

enum states state = MASTER_OFF;

void master_power_on(int* fail);
void master_power_off(int* fail);
int kill_btn_depressed(void);
void hard_ack(void);
void delay_ms( int ms );


int main(void)
{
    int fail = 0;
    // initialize IO
    IO_PORT.DIR = 1<<MASTER_PWR_PIN; 
    IO_PORT.PIN0CTRL = 0b011<<3;
    
    while (1) {
        switch(state) {
        
        case MASTER_OFF:
            state = STARTUP;
            // if( voltage_sensed_on_master_bus() ) {state = STARTUP;}
            
        case STARTUP:
            master_power_on(&fail);
            if ( !fail ) {state = MASTER_ON;}
            break;
        
        case MASTER_ON:
            if (kill_btn_depressed()==1) {
                master_power_off(&fail);
                if ( !fail ) {state = MASTER_OFF;}
            }
            break;
            
        }
        _delay_ms( 2000 );
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