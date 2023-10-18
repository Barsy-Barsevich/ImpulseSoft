/*
MPU9250 calibration
Программа для калибровки MPU9250
*/

#define F_CPU 16000000UL

#include <avr/io.h> 
#include <util/delay.h> 
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "WireOma_m128.h"
#include "UARTOma_m128.h"
#include "MPU9250_Oma/MPU9250_toolkits.h"
//#include "EEPROM_oma/EEPROM_read.c"
#include "EEPROM_oma/EEPROM_write.c"

//******************************************************************************
#define MPU_ADDR 0x68 //AD0=0
//#define MPU_ADDR 0x69 //AD0=1
#define AK8963_ADDR 0x0C
//******************************************************************************
// vars
uint8_t gyro_offset[6];
uint8_t accel_offset[6];
//float mag_bias_factory[3];
//float mag_bias[3];
//float mag_scale[3];

//******************************************************************************
// fun
void main();
void uint8_to_hex(char[], uint8_t);

//******************************************************************************

void main() {

    float mag_bias_factory[3];
    float mag_bias[3];
    float mag_scale[3];
    DDRD =  0b11111111;
    PORTD = 0b00011011;
    
    UART_INI();// UART INI
    TWBR = 17; // TWI FREQ = 400000 kHz (f_cpu = 16MHz)
    _delay_ms(100);
    
    //UART_PRINTLN_STR("Start...");
    UART_TRANSMIT('S');
    UART_TRANSMIT('t');
    UART_TRANSMIT('a');
    UART_TRANSMIT('r');
    UART_TRANSMIT('t');
    UART_TRANSMIT(';');
    
    if (MPU_INIT(MPU_ADDR) == true){
        //char str[] = "MPU intialization done";
        //UART_PRINTLN_STR(str);
        UART_TRANSMIT('M');
        UART_TRANSMIT(';');
    }
    else {
        //char str[] = "MPU initialization failed";
        //UART_PRINTLN_STR(str);
        UART_TRANSMIT('F');
        while (1) {}
    }
    /*if (AK8963_INI(AK8963_ADDR, mag_bias_factory) == true){
        //UART_PRINTLN_STR("AK8963 initialization done");
        UART_TRANSMIT('A');
        UART_TRANSMIT(';');
    }
    else {
        //UART_PRINTLN_STR("AK8963 initialization failed");
        UART_TRANSMIT('F');
        while (1) {}
    }*/
    
    char data[2];
    uint16_t eeaddr = 0;
    
    UART_TRANSMIT('\n');
    //UART_PRINTLN_STR("Calibrating accelerometer & gyroscope!");
    CALIBRATION_ACCEL_GYRO(accel_offset, gyro_offset);
    //UART_PRINT_STR("Accel: ");
    UART_TRANSMIT('A');
    UART_TRANSMIT(':');
    for (uint8_t i=0; i<6; i++){
        UART_TRANSMIT(' ');
        EEPROM_write(eeaddr, accel_offset[i]);
        eeaddr++;
        uint8_to_hex(data, accel_offset[i]);
        //UART_PRINT_STR(data);
        UART_TRANSMIT(data[1]);
        UART_TRANSMIT(data[0]);
    }
    
    //UART_PRINT_STR("\nGyro:  ");
    UART_TRANSMIT('\n');
    UART_TRANSMIT('G');
    UART_TRANSMIT(':');
    for (uint8_t i=0; i<6; i++){
        UART_TRANSMIT(' ');
        EEPROM_write(eeaddr, gyro_offset[i]);
        eeaddr++;
        uint8_to_hex(data, gyro_offset[i]);
        //UART_PRINT_STR(data);
        UART_TRANSMIT(data[1]);
        UART_TRANSMIT(data[0]);
    }
    UART_TRANSMIT('\n');
    // Стоять!! это очень важно проинициализировать магнетометр перед
    // калибровкой! Иначе все пойдет по п**де!!!
    // Я убил, бл*ть, 2 часа на поиск подставы
    MPU_INIT(MPU_ADDR);
    if (AK8963_INI(AK8963_ADDR, mag_bias_factory) == true){
        //UART_PRINTLN_STR("AK8963 initialization done");
        UART_TRANSMIT('A');
        UART_TRANSMIT(';');
    }
    else {
        //UART_PRINTLN_STR("AK8963 initialization failed");
        UART_TRANSMIT('F');
        while (1) {}
    }
    UART_TRANSMIT('\n');
    //UART_PRINTLN_STR("Calibrating magnetometer!");
    CALIBRATE_MAG(mag_bias, mag_scale, mag_bias_factory);
    //mag_bias[0] = 3.14;
    UART_TRANSMIT('-');
    UART_TRANSMIT('\n');
    UART_TRANSMIT('M');
    UART_TRANSMIT('B');
    UART_TRANSMIT(':');
    char *u;
    u = (char*)&mag_bias[0];
    for (uint8_t i=0; i<4; i++){
        UART_TRANSMIT(' ');
        EEPROM_write(eeaddr, u[i]);
        eeaddr++;
        uint8_to_hex(data, u[i]);
        UART_TRANSMIT(data[1]);
        UART_TRANSMIT(data[0]);
    }
    UART_TRANSMIT(' ');
    u = (char*)&mag_bias[1];
    for (uint8_t i=0; i<4; i++){
        UART_TRANSMIT(' ');
        EEPROM_write(eeaddr, u[i]);
        eeaddr++;
        uint8_to_hex(data, u[i]);
        UART_TRANSMIT(data[1]);
        UART_TRANSMIT(data[0]);
    }
    UART_TRANSMIT(' ');
    u = (char*)&mag_bias[2];
    for (uint8_t i=0; i<4; i++){
        UART_TRANSMIT(' ');
        EEPROM_write(eeaddr, u[i]);
        eeaddr++;
        uint8_to_hex(data, u[i]);
        UART_TRANSMIT(data[1]);
        UART_TRANSMIT(data[0]);
    }
    UART_TRANSMIT('\n');
    UART_TRANSMIT('M');
    UART_TRANSMIT('S');
    UART_TRANSMIT(':');
    u = (char*)&mag_scale[0];
    for (uint8_t i=0; i<4; i++){
        UART_TRANSMIT(' ');
        EEPROM_write(eeaddr, u[i]);
        eeaddr++;
        uint8_to_hex(data, u[i]);
        UART_TRANSMIT(data[1]);
        UART_TRANSMIT(data[0]);
    }
    UART_TRANSMIT(' ');
    u = (char*)&mag_scale[1];
    for (uint8_t i=0; i<4; i++){
        UART_TRANSMIT(' ');
        EEPROM_write(eeaddr, u[i]);
        eeaddr++;
        uint8_to_hex(data, u[i]);
        UART_TRANSMIT(data[1]);
        UART_TRANSMIT(data[0]);
    }
    UART_TRANSMIT(' ');
    u = (char*)&mag_scale[2];
    for (uint8_t i=0; i<4; i++){
        UART_TRANSMIT(' ');
        EEPROM_write(eeaddr, u[i]);
        eeaddr++;
        uint8_to_hex(data, u[i]);
        UART_TRANSMIT(data[1]);
        UART_TRANSMIT(data[0]);
    }
    UART_TRANSMIT('\n');
    UART_TRANSMIT('M');
    UART_TRANSMIT('F');
    UART_TRANSMIT(':');
    u = (char*)&mag_bias_factory[0];
    for (uint8_t i=0; i<4; i++){
        UART_TRANSMIT(' ');
        EEPROM_write(eeaddr, u[i]);
        eeaddr++;
        uint8_to_hex(data, u[i]);
        UART_TRANSMIT(data[1]);
        UART_TRANSMIT(data[0]);
    }
    UART_TRANSMIT(' ');
    u = (char*)&mag_bias_factory[1];
    for (uint8_t i=0; i<4; i++){
        UART_TRANSMIT(' ');
        EEPROM_write(eeaddr, u[i]);
        eeaddr++;
        uint8_to_hex(data, u[i]);
        UART_TRANSMIT(data[1]);
        UART_TRANSMIT(data[0]);
    }
    UART_TRANSMIT(' ');
    u = (char*)&mag_bias_factory[2];
    for (uint8_t i=0; i<4; i++){
        UART_TRANSMIT(' ');
        EEPROM_write(eeaddr, u[i]);
        eeaddr++;
        uint8_to_hex(data, u[i]);
        UART_TRANSMIT(data[1]);
        UART_TRANSMIT(data[0]);
    }
    UART_TRANSMIT('\n');
    UART_TRANSMIT('E');
    UART_TRANSMIT('n');
    UART_TRANSMIT('d');
    UART_TRANSMIT('.');
    UART_TRANSMIT('\n');
}


void uint8_to_hex(char *str, uint8_t data) {
    str[1] = ((data>>4)&0x0F) | 0x30;
    str[0] = (data & 0x0F) | 0x30;
    if (str[0] > 0x39) str[0] += 7;
    if (str[1] > 0x39) str[1] += 7;
}
    



