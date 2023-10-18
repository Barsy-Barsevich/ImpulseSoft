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
#include "Servo_driver.c"

void main(void);

void main() {
    Servo_Driver_INI();
    Stab_Servo_Enable();
    Servo_Print_Zero();
    while (1) {
        StabA = 15;
        StabB = 15;
        StabC = 15;
        StabD = 15;
        Servo_Print_Stab();
        SERVO_OBT_OCR = SERVO_HALF_PRESCALER;
        _delay_ms(500);
        StabA = -15;
        StabB = -15;
        StabC = -15;
        StabD = -15;
        Servo_Print_Stab();
        SERVO_OBT_OCR = 1;
        _delay_ms(500);
    }
}
