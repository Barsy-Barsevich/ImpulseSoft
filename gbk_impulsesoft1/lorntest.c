#define F_CPU 16000000L
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
#include "UARTOma_m128.h"
//#include "TIME_Oma.h"
#include "LoRa_Oma.h"

char str[128];

void main();


void main(){
    LORA_INI();
    for (uint8_t i=0; i<128; i++){
        str[i] = 'A';
    }
    LORA_Normal_Mode();
    
    
    while (1) {
        LORA_Send_Packet(str);
        _delay_ms(1000);
    }
}
