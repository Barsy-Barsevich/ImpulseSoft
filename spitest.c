#define MOSI_PORT   PORTB
#define MISO_PORT   PORTB
#define SCK_PORT    PORTB
#define MOSI_PIN    PB3
#define MISO_PIN    PB2
#define SCK_PIN     PB4
#define SS_PIN      PB1

#define F_CPU 8000000UL
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

void main(void);


void SPI_INI() { // Инициализация программного SPI
    //Инициализация всех портов
    PORTB |= (1<<SS_PIN);
    PORTB &= ~(1<<MISO_PIN);
    DDRB |= (1<<SS_PIN)|(1<<MOSI_PIN)|(1<<SCK_PIN);
    DDRB &= ~(1<<MISO_PIN);
    //Special
    //FlashADDR = 0;
}

uint8_t SPI_ex(uint8_t data) {
    uint8_t counter = 8;
    uint8_t output = 0;
    asm volatile(
    "spi_cycle_%=:\n\t"
    //Определение MOSI .. 5 тактов
    "SBIS %7,7\n\t"     // 1(2) такт
    "CBI %1,%2\n\t"     // 2 такта
    "SBIC %7,7\n\t"     // 1(2) такт
    "SBI %1,%2\n\t"     // 2 такта
    //поднятие фронта sck
    "SBI %5,%6\n\t"     // 2 такта
    //чтение бита miso .. 3 такта
    "LSL %7\n\t"        // 1 такт
    "LSL %0\n\t"
    "SBIC %3-2,%4\n\t"  // 1(2) такт
    "INC %0"            // 1 такт
    //опускание фронта sck
    "CBI %5,%6\n\t"     // 2 такта
    //зацикливание // 3 такта
    "DEC %8\n\t"        // 1 такт
    "BRNE spi_cycle_%=\n\t" // 2 такта
    :"=r" (output)
    :"I" (_SFR_IO_ADDR(MOSI_PORT)), "I" (MOSI_PIN), //MOSIPORT - %1; MOSIDDR - %1-1; MOSIPIN - %1-2
     "I" (_SFR_IO_ADDR(MISO_PORT)), "I" (MISO_PIN), //MISOPORT - %3; MISODDR - %3-1; MISOPIN - %3-2
     "I" (_SFR_IO_ADDR(SCK_PORT)),  "I" (SCK_PIN),  //SCKPORT - %5; SCKDDR - %5-1; SCKPIN - %5-2
     "r" (data), "r" (counter) //data - %7; counter - %8
    );
    return output;
}

void main() {
    SPI_INI();
    _delay_us(4);
    SPI_ex(0x55);
    SPI_ex(0xCC);
    SPI_ex(0xE4);
}



