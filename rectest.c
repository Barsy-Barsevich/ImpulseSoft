#define FOSC 16000000UL// Clock Speed
#define BAUD 38400
#define MYUBRR FOSC/16/BAUD-1

#define F_CPU 16000000UL
// <>
#include <avr/io.h> 
#include <util/delay.h> 
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
// <>

void UART_INI(void);
void UART_INI_BAUD(uint32_t);
void UART_TRANSMIT(char);
void UART_PRINT(char[], uint8_t);
uint8_t UART0_RECEIVE(void);
void main(void);

void UART_INI(){
    /* Set baud rate */
    UBRR0H = 0;
    UBRR0L = 52;//19200
    /* Enable receiver and transmitter */
    UCSR0B = (1<<TXEN0)|(1<<RXEN0);
    /* Set frame format: 8data, 2stop bit */
    //UCSR0C = (1<<USBS)|(3<<UCSZ0);
}
void UART_INI_BAUD(uint32_t baud_rate){
    /* Set baud rate */
    uint16_t ubrr = FOSC/(16*baud_rate)-1;
    UBRR0H = (uint8_t)(ubrr>>8);
    UBRR0L = (uint8_t)ubrr;
    /* Enable receiver and transmitter */
    UCSR0B = (1<<TXEN0);
    /* Set frame format: 8data, 2stop bit */
    //UCSR0C = (1<<USBS)|(3<<UCSZ0);
}
void UART_TRANSMIT(char data) {
/* Wait for empty transmit buffer */
    while ( !( UCSR0A & (1<<UDRE0)) );
    /* Put data into buffer, sends the data */
    UDR0 = data;
}
void UART_PRINT(char *data, uint8_t count) {
    for (uint8_t i=0; i<count; i++){
        UART_TRANSMIT(data[i]);
    }
}
uint8_t UART0_RECEIVE() {
    // Wait for data to be received
    while (!(UCSR0A & (1<<RXC)));
    // Get and return received data from buffer
    return UDR0;
}

void main() {
    UART_INI();
    DDRC = 0xFF;
    uint8_t data[3];
    data[0] = UART0_RECEIVE();
    data[1] = UART0_RECEIVE();
    data[2] = UART0_RECEIVE();
    PORTC = data[3];
}



