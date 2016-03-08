#include <string.h>

#include "sw.h"
#include "uart.h"


#define UART_RX_BUFF_SIZE 128

enum BufferState {
	NORMAL = 0,
	REALIGN = 1
};

/* Rx buffer */
static volatile unsigned char buffer[UART_RX_BUFF_SIZE];
static volatile int windex = 0;
static volatile int rindex = 0;

static enum BufferState state = NORMAL;
static int marker_count = 0;

pin_t pin_tx = {
	.name="pin_tx",
	.description="USART TX pin",
	.port=&PORTC,
	.pos=7,
};

/* 
BACKGROUND: 
  * BUFOVF (Buffer overflow) is a flag that indicates data loss due to a 
  receiver buffer full condition. The receiver buffer is only two bytes long 
  (+ one more byte/char waiting in the receive shift register). This flag will
  be set high the moment a USART start bit is detected while these two (three)
  chars/bytes are in the micro's possession.
  
BRIEF:
   RXC_vect runs the moment there is unread data in the receive register.
   */
ISR(USARTC1_RXC_vect) {
	int next_windex = (windex + 1) % UART_RX_BUFF_SIZE;
	
	switch(state) {
	case NORMAL:
		/* If buffer has overflowed, or if rindex is a value it's
		not supposed to be (if rindex surpasses windex by any amount,
		next_windex will eventually == rindex). 
		*/
		if ((USARTC1.STATUS & USART_BUFOVF_bm) || next_windex == rindex) {
			char message[3]={0};
			
			message[0] = SW_ERROR;
			message[1] = SERIAL_ERROR;
			
			if(next_windex == rindex) {
				message[2] = 0;
			} else {
				message[2] = 1;
			}
			
			serial_send_bytes(message, 3);
			realign_buffer();
		}
		
		/* add the unread byte to the driver-level buffer */
		buffer[windex] = USARTC1.DATA; //shift register will refill this reg if 
									   //there is still more data to read.
		
		/* update the writing (incoming data) index pointer */
		windex = next_windex;
		break;
		
	/* REALIGN:
	Seawolf and micro will handshake with each other as a 
	way to resynchronize themselves */
	case REALIGN:
		// read incoming data
		buffer[windex] = USARTC1.DATA;
	
		/* if read byte is a MARKER byte */
		if(buffer[windex] == SW_MARKER) {
			marker_count++;
				
			if(marker_count == 3) {
				state = NORMAL;
					
				/* Clear buffer */
				rindex = windex;
			}
		} 
		
		/* there is probably pre-existing data in the rx buffer that has
		to clear out if it reaches this point... or a packet got dropped
		and it didn't see the full 3 MARKERS. Either way, it seems like
		the code will get stuck indefinately if it ever gets here. This is a
		bug that may have to be fixed someday. */
		else {
			marker_count = 0;
			windex = next_windex;
		}
		break;
	}
}

void init_serial(void) {
	/* Init tx pin as output */
	pin_tx.port->OUTSET = 1<<pin_tx.pos;
	pin_tx.port->DIRSET = 1<<pin_tx.pos;
	
	/* Set baud to 57600 approximately (57606 exactly) */
	USARTC1.BAUDCTRLA = 110;
	USARTC1.BAUDCTRLB = 0xa8;
	
	/* UART, 1 stop bit, no parity bits, 8 data bit (8N1) */
	USARTC1.CTRLC = 0x03;
	
	/* Enable Rx interrupt */
	USARTC1.CTRLA = USART_RXCINTLVL_MED_gc;
	
	/* Enable transmit and receive */
	USARTC1.CTRLB = 0x18;
	
}

/* Send a message to seawolf to indicate that it's setting marker_count 
   to zero.  */
void realign_buffer(void) {
	char message[3] = {0};
	
	/* if call is redundant... . */
	if(state == REALIGN) {
		return;
	}
	
	message[0] = SW_REALIGN;
	message[1] = 0;
	message[2] = 0;
	
	state = REALIGN;
	marker_count = 0;
	
	serial_send_bytes(message, 3);
}

/* Send a single byte of data */
void serial_send_byte(char c) {
	/* Wait for tx buffer to report that it's empty. */
	while((USARTC1.STATUS & USART_DREIF_bm) == 0x00) {
		;;
	}
	
	USARTC1.DATA = c;
}

void serial_send_bytes(char* s, int n) {
	/* disable interrupts */
	cli();
	
	while(n) {
		serial_send_byte(*s);
		s++;
		n--;
	}
	
	/* re-enable interrupts */
	sei();
}

/* Returns the number of bytes that are available for reading.

Returns: Returns 0 if no bytes are available. */
int serial_available(void) {
	volatile int r, w;
	
	/* disable interrupts */
	cli();
	
	r = rindex;
	w = windex;
	
	/* re-enable interrupts */
	sei();
	
	/* determine number of bytes between the w and r pointer...
	and return that to the user as the number of byte that can
	be read. */
	if(r <= w) {
		return (w - r);
		
	/* the w pointer has wrapped to the beginning of the ring
	buffer. Do some math to figure out how many bytes are between
	the w and r pointers. */
	} else {
		return (UART_RX_BUFF_SIZE - r) + w;
	}
}

/* Read a single byte */
int serial_read_byte(void) {
	int c;
	
	/* Wait for data to be available. data will be
	in "buffer" when it's available. */
	while(serial_available() <= 0) {
		;;
	}
	
	c = buffer[rindex];						   //...
	rindex = (rindex + 1) % UART_RX_BUFF_SIZE; //update rindex;
	
	return c;
}

/* Read multiple bytes 
---- s: point to caller-level buffer. 
---- n: number of bytes to read. 
*/
void serial_read_bytes(char* s, int n) {
	while(serial_available() < n) {
		;;
	}
		
	cli();
	/* if wrap is unnecessary */
	if(rindex + n <= UART_RX_BUFF_SIZE) {
		memcpy(s, (void*) buffer + rindex, n);
		
	/* wrap */
	} else {
		/* Size of first chunk */
		int chunk_size = UART_RX_BUFF_SIZE - rindex;
			
		memcpy(s, (void*) buffer + rindex, chunk_size);
		memcpy(s + chunk_size, (void*) buffer, n - chunk_size);
	}
		
	sei();
		
	/* Update rindex to represent the current location of the 
	read ptr. */
	rindex = (rindex + n) % UART_RX_BUFF_SIZE;
}