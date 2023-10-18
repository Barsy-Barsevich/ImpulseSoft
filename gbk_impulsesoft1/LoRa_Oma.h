//******************************************************************************
//---<Лорный драйвер -- почувствуй себя сумасшедшим>----------------------------
//******************************************************************************
#define RXD1_PIN    PD2 // Порт Лорки -- D
#define TXD1_PIN    PD3
#define AUX_PIN     PD4
#define M0_PIN      PD6
#define M1_PIN      PD5
//******************************************************************************
//---<Fun>----------------------------------------------------------------------
void LORA_INI(void);
void LORA_Power_Down_Mode(void);
void LORA_Normal_Mode(void);
void LORA_Send_Packet(char[]);
void UART1_TRANSMIT(char);
//******************************************************************************
//---<Main>---------------------------------------------------------------------
void LORA_INI() {
    //инициализируем все порты
    DDRD |= (1<<TXD1_PIN)|(1<<M0_PIN)|(1<<M1_PIN);
    DDRD &= ~((1<<RXD1_PIN)|(1<<AUX_PIN));
    //затем инитим порт сериал 1
    UBRR1H = 0;
    UBRR1L = 51;//103;//9600
    UCSR1B = (1<<TXEN1); //разрешаем трансмиттер и ресивер
    //переключаемся в режим энергосбережения
    _delay_us(1000);
    PORTD |= (1<<M0_PIN)|(1<<M1_PIN);
    _delay_us(1000);
    /*запихиваем настроечные данные:
    - Address - 0x8591 !
    - Air rate - 19.2bps
    - Baud rate - 9600
    - Packet size - 128 bytes
    - RSSI disable
    - Transmitting power - 30 dBm */
    UART1_TRANSMIT(0xC0);
    UART1_TRANSMIT(0x00);
    UART1_TRANSMIT(0x05);
    UART1_TRANSMIT(0x85);
    UART1_TRANSMIT(0x91);
    UART1_TRANSMIT(0x65);
    UART1_TRANSMIT(0x40);
    UART1_TRANSMIT(0x1D);//*/
    /*UART1_TRANSMIT(0xC0);
    UART1_TRANSMIT(0x00);
    UART1_TRANSMIT(0x05);
    UART1_TRANSMIT(0x00);
    UART1_TRANSMIT(0x03);
    UART1_TRANSMIT(0x62);
    UART1_TRANSMIT(0x00);
    UART1_TRANSMIT(0x23);*/
}
void LORA_Power_Down_Mode(){
    PORTD |= (1<<M0_PIN)|(1<<M1_PIN);
    _delay_us(1000);
}
void LORA_Normal_Mode(){
    PORTD &= ~((1<<M0_PIN)|(1<<M1_PIN));
    _delay_us(1000);
}
void LORA_Send_Packet(char* arr){
    for (uint8_t i=0; i<128; i++){
        UART1_TRANSMIT(arr[i]);
    }
}
void UART1_TRANSMIT(char data) {
    /* Wait for empty transmit buffer */
    while ( !( UCSR1A & (1<<UDRE1)) );
    /* Put data into buffer, sends the data */
    UDR1 = data;
}