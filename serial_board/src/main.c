/**
 * \file
 *
 * \brief Serial Board Source Code
 *
 *
 
 */
#include <asf.h>

#include "sw.h"

#include "twi_common.h"
#include "twim.h"
#include "sysclk.h"
#include "uart.h"




#define TWI_MASTER       TWIE
#define TWI_MASTER_PORT  PORTE
//#define TWI_SLAVE        TWIF
#define TWI_SPEED        50000
#define TWI_MASTER_ADDR  0x50
#define TWI_SLAVE_ADDR1   0x60

#define DATA_LENGTH     8

//TWI_Slave_t slave;

uint8_t data[DATA_LENGTH] = {
	0x05, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

uint8_t recv_data[DATA_LENGTH] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

twi_options_t m_options = {
	.speed     = TWI_SPEED,
	.chip      = TWI_MASTER_ADDR,
	.speed_reg = TWI_BAUD(/*sysclk_get_cpu_hz()*/F_CPU, TWI_SPEED)
};

/*
static void slave_process(void) {
	int i;
	for(i = 0; i < DATA_LENGTH; i++) {
		recv_data[i] = slave.receivedData[i];
	}
}
*/

/*
ISR(TWIC_TWIS_vect) {
	TWI_SlaveInterruptHandler(&slave);
}
*/

void send_twi(uint8_t addr, uint8_t* msg, uint8_t n);
void twi_init(void);

void send_twi(uint8_t addr, uint8_t* msg, uint8_t n)
{
	twi_package_t packet = {
		.addr_length = 0,
		.chip        = addr,
		.buffer      = (void *)msg,
		.length      = n,
		.no_wait     = false
	};

	
	//uint8_t i;

	

	/*
	sysclk_enable_peripheral_clock(&TWI_SLAVE);
	TWI_SlaveInitializeDriver(&slave, &TWI_SLAVE, *slave_process);
	TWI_SlaveInitializeModule(&slave, TWI_SLAVE_ADDR1,
	TWI_SLAVE_INTLVL_MED_gc);
	*/

	/*
	for (i = 0; i < TWIS_SEND_BUFFER_SIZE; i++) {
		slave.receivedData[i] = 0;
	}
	*/

	

	twi_master_write(&TWI_MASTER, &packet);

	/*
	do {
		// Nothing
	} while(slave.result != TWIS_RESULT_OK);
	*/
}

void twi_init(void) {
	TWI_MASTER_PORT.PIN0CTRL = PORT_OPC_WIREDANDPULL_gc;
	TWI_MASTER_PORT.PIN1CTRL = PORT_OPC_WIREDANDPULL_gc;

	irq_initialize_vectors();

	sysclk_enable_peripheral_clock(&TWI_MASTER);
	twi_master_init(&TWI_MASTER, &m_options);
	//twi_master_enable(&TWI_MASTER);
	//cpu_irq_enable();
}

#define TEST_MESSAGE_SIZE 2

void test_program1(void);
void test_program1(void) {
	uint8_t msg[TEST_MESSAGE_SIZE] = {0x05,0x4D};
	uint8_t n = TEST_MESSAGE_SIZE;
		
	twi_init();
		
	uint8_t i = 1;
	while (1)
	{
		// send a msg
		send_twi(TWI_SLAVE_ADDR1, msg, n);
			
		// modify the msg for next transmission, and add a small delay between each transmission
		msg[0] = (i++%6) ;
		_delay_ms(100);
	}
}

/* ******************************************
**** Copying old microcontroller code *******
********************************************* */
#define test1
//#define test2
//#define test3
//#define test4

static int kill_status = -1;

int enable_interrupts (void) {
	/* Enable all interrupt levels */
	PMIC.CTRL |= (PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm);
	
	/* Enable interrupts globally */
	sei();
}

void software_reset(void) {
	CCP = CCP_IOREG_gc;
	RST.CTRL = RST_SWRST_bm;
}

pin_t top_left = {
	.port = &PORTC,
	.pos = 2,
};


void synchronize_comm(void) {
	/* Send 0xFF until 0x00 is received */
	while(true) {
		serial_send_byte(0xff);
		
	
		/* if data in the buffer is available, read the buffer. If the
		byte read is 0x00, then break out of this while loop.*/
		if(serial_available() && (serial_read_byte() == 0x00)) {
			break;
		}
	}
	
	/* if there is any more data after 0x00 available... that is
	a problem. Send an error/notification to seawolf. There is no 
	code on seawolf to catch this... so, it'll probably screw up
	some code on seawolf's side of things if this happens. */
	if(serial_available()) {
		char command[3] = {
			SW_ERROR,
			SYNC_ERROR,
			0
		};
		
		serial_send_bytes(command, 3);
	}
	
	
	serial_send_byte(0xf0);
}

int main (void) {
	char command[3] = {0};
	top_left.port->DIR |= 1<<top_left.pos;
	
	/* Enable 32MHz clock and wait for it to be ready */
	OSC.CTRL |= OSC_RC32MEN_bm;
	while((OSC.STATUS & OSC_RC32MRDY_bm) == 0x00);
	
	/* Set clock to 32Mhz and then lock it */
    CCP = CCP_IOREG_gc;
    CLK.CTRL = CLK_SCLKSEL_RC32M_gc;
    CLK.LOCK = CLK_LOCK_bm;
	
	init_serial();	
	enable_interrupts();
	
	synchronize_comm();
	
	while(true) {
		serial_read_bytes(command, 3);
		
		switch(command[0]) {
		case SW_RESET:
			software_reset();
			break;
		}
	}
		
	return 0;
}