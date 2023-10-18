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
//******************************************************************************
// ------<Вычисление углов отклонения активных рулей>---------------------------
float InertionMoment, PowShoulder;
float Force_Roll, Force_Pitch, Force_Yaw;
float Stab_A_Force, Stab_B_Force, Stab_C_Force, Stab_D_Force;
float StabA, StabB, StabC, StabD;
uint16_t STABA_BIAS, STABB_BIAS, STABC_BIAS, STABD_BIAS;
//******************************************************************************
#include "UARTOma_m128.c"

void main(void);

void main() {
    UART_INI_BAUD(9600);
    while (1) {
        UART_TRANSMIT('A');
    }
}
