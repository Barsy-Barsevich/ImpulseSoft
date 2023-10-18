#define FOSC 16000000UL// Clock Speed
#define BAUD 38400
#define MYUBRR FOSC/16/BAUD-1

void UART_INI(void);
void UART_INI_BAUD(uint32_t);
void UART_TRANSMIT(char);
void UART_PRINT(char[], uint8_t);
void UART_PRINT_STR(char[]);
void UART_PRINTLN(char[], uint8_t);
void UART_PRINTLN_STR(char[]);

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
    UCSR0B = (1<<TXEN0)|(1<<RXEN0);
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
void UART_PRINT_STR(char *data) {
    for (uint8_t i=0; i<(sizeof(data)); i++){
        UART_TRANSMIT(data[i]);
    }
}   
void UART_PRINTLN(char *data, uint8_t count) {
    UART_PRINT(data, count);
    UART_TRANSMIT('\n');
}
void UART_PRINTLN_STR(char *data) {
    UART_PRINT_STR(data);
    UART_TRANSMIT('\n');
}
uint8_t UART0_RECEIVE() {
    // Wait for data to be received
    while (!(UCSR0A & (1<<RXC)));
    // Get and return received data from buffer
    return UDR0;
}
