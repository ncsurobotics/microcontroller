/*
 * uart.h
 *
 * Created: 3/8/2016 11:00:26 AM
 *  Author: Josh
 */ 


#ifndef UART_H_
#define UART_H_

void init_serial(void);
void realign_buffer(void);
void serial_send_byte(char c);
void serial_send_bytes(char* s, int n);
void serial_print(char* s);
int serial_available(void);
int serial_read_byte(void);
void serial_read_bytes(char* s, int n);

#endif /* UART_H_ */