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
/*// <>
#include "WireOma_m128.h"
#include "UARTOma_m128.h"
#include "TIME_Oma.h"
#include "MPU9250_toolkits.h"
#include "MPU9250_register_map.h"
#include "BME280_toolkits.h"
#include "BME280_register_map.h"
#include "Madgwick.h"
#include "Barsotion_AngleCalc.h"
#include "Servo_driver.c"
#include "EEPROM_read.c"
*/
//******************************************************************************
// ------<Флаги состояния системы>----------------------------------------------
bool Start_flag = false;
bool Stage_2_flag = false;
bool Apogee_flag = false;
bool Ignition_flag = false;
bool Earthing_flag = false;
uint8_t system_mode = 0;
//******************************************************************************
// ------<Флаги ошибок>---------------------------------------------------------
bool mag_error_flag = false;
bool mpu_error_flag = false;
bool twi_error_flag = false;
bool temp_error_flag = false;
bool press_error_flag = false;
bool hum_error_flag = false;
uint8_t flags_error = 0;
//******************************************************************************
// ------<Значения датчиков. Сырые данные>--------------------------------------
uint8_t raw1_accel[6];
uint8_t raw1_gyro[6];
uint8_t raw1_mag[6];
uint8_t raw1_temp[2];
uint8_t raw1_pressure[3];
uint8_t raw1_temperature[3];
uint8_t raw1_humidity[2];
uint8_t raw2_accel[6];
uint8_t raw2_gyro[6];
uint8_t raw2_mag[6];
uint8_t raw2_temp[2];
uint8_t raw2_pressure[3];
uint8_t raw2_temperature[3];
uint8_t raw2_humidity[2];
//******************************************************************************
// ------<Значения датчиков. Обработанные данные>-------------------------------
float Accel[3], Gyro[3], Mag[3];
float Pressure, Temperature, Humidity;
float VertSpeed, Altitude, AccelModule;
float pre_Altitude;
//******************************************************************************
// ------<Значения калибровки магнетометра>-------------------------------------
float mag_bias[3];
float mag_scale[3];
float mag_bias_factory[3];
//******************************************************************************
// ------<Значения времени>-----------------------------------------------------
//Системное время
volatile uint32_t systime_micros = 0;
//Время цикла
float deltat = 0.01;
//Время работы двигателя
float eng_time = 0;
float eng_time2, eng_time3;
uint32_t last_upd_time = 0;
//******************************************************************************
// ------<TWI>------------------------------------------------------------------
//Переменная, определяющая, какая TWI-команда будет выполняться следующей
volatile uint8_t twi_mode;
//Массив указателей на функции
void (*twi_fun[45])(void);
//Если установлен - работает главная последовательность чтения датчиков
//по прерываниям. Если сброшен - соответственно, не работает
volatile bool twi_main_cycle_flag = false;
//Флаг готовности*pressure_out модуля TWI к дальнейшим действиям
volatile bool twi_ready_flag = false;
//Время последней отправки команды модулю TWI. Если разница между systime_micros
//и last_twi_time превышает определенный предел, значит, шина TWI сломана
uint32_t last_twi_time = 0;
//******************************************************************************
// ------<Фильтр Маджвика и IMU>------------------------------------------------
//Векторы кватерниона
float q0 = 1.0;
float q1 = 0.0;
float q2 = 0.0;
float q3 = 0.0;
//Крен, тангаж, рыскание
float rpy[3];
float Roll, Pitch, Yaw;
float dRoll, dPitch, dYaw;
float pre_Roll, pre_Pitch, pre_Yaw;
//******************************************************************************
// ------<Вычисление углов отклонения активных рулей>---------------------------
float InertionMoment, PowShoulder;
float Force_Roll, Force_Pitch, Force_Yaw;
float Stab_A_Force, Stab_B_Force, Stab_C_Force, Stab_D_Force;
float StabA, StabB, StabC, StabD;
//******************************************************************************
//Массив для вывода значений в последовательный порт. Используется для отладки
char str[30];
//Formats
// |Time|Flags|Pressure|Temp|Vspeed|Humidity|AX|AY|AZ|GX|GY|GZ|MX|MY|MZ|StabA|StabB|StabC|StabD
char data_to_send[128];



float staba_fil[20];
float stabb_fil[20];
float stabc_fil[20];
float stabd_fil[20];


#include "TIME_Oma.h"
#include "WireOma_m128.h"
#include "UARTOma_m128.h"
#include "EEPROM_read.c"
#include "MPU9250_toolkits.h"
#include "MPU9250_register_map.h"
#include "BME280_toolkits.h"
#include "BME280_register_map.h"
#include "Madgwick.h"
#include "Barsotion_AngleCalc.h"
#include "Servo_driver.c"


void main(void);
void loop(void);
void MAIN_Sensors_Data_test(void);
void MAIN_Sensors_Buffer_Copying(void);
void MAIN_time_update(void);
void MAIN_data_to_send_prepare(void);
void MAIN_data_to_send_form(void);


void main() {
    TIME_INI();
    UART_INI_BAUD(1000000);
    Servo_Driver_INI();
    Stab_Servo_Enable();
    TWI_INI();
    _delay_ms(100);
    
    twi_fun[1] = TW_1;
    twi_fun[2] = TW_2;
    twi_fun[3] = TW_3;
    twi_fun[4] = TW_4;
    twi_fun[5] = TW_5;
    twi_fun[6] = TW_6;
    twi_fun[7] = TW_7;
    twi_fun[8] = TW_8;
    twi_fun[9] = TW_9;
    twi_fun[10] = TW_10;
    twi_fun[11] = TW_11;
    twi_fun[12] = TW_12;
    twi_fun[13] = TW_13;
    twi_fun[14] = TW_14;
    twi_fun[15] = TW_15;
    twi_fun[16] = TW_16;
    twi_fun[17] = TW_17;
    twi_fun[18] = TW_18;
    twi_fun[19] = TW_19;
    twi_fun[20] = TW_20;
    twi_fun[21] = TW_21;
    twi_fun[22] = TW_22;
    twi_fun[23] = TW_23;
    twi_fun[24] = TW_24;
    twi_fun[25] = TW_25;
    twi_fun[26] = TW_26;
    twi_fun[27] = TW_27;
    twi_fun[28] = TW_28;
    twi_fun[29] = TW_29;
    twi_fun[30] = TW_30;
    twi_fun[31] = TW_31;
    twi_fun[32] = TW_32;
    twi_fun[33] = TW_33;
    twi_fun[34] = TW_34;
    twi_fun[35] = TW_35;
    twi_fun[36] = TW_36;
    twi_fun[37] = TW_37;
    twi_fun[38] = TW_38;
    twi_fun[39] = TW_39;
    twi_fun[40] = TW_40;
    twi_fun[41] = TW_41;
    twi_fun[42] = TW_42;
    twi_fun[43] = TW_43;
    twi_fun[44] = TW_44;
    
    UART_TRANSMIT('S');
    UART_TRANSMIT('t');
    UART_TRANSMIT('a');
    UART_TRANSMIT('r');
    UART_TRANSMIT('t');
    UART_TRANSMIT(';');
    if (MPU_INIT() == true){
        UART_TRANSMIT('M');
        UART_TRANSMIT(';');
    }
    else {
        UART_TRANSMIT('F');
        while (1) {}
    }
    if (AK8963_INI(AK8963_ADDR, mag_bias_factory) == true){
        UART_TRANSMIT('A');
        UART_TRANSMIT(';');
    }
    else {
        UART_TRANSMIT('F');
        while (1) {}
    }
    if (BME_INI() == true){
        UART_TRANSMIT('B');
        UART_TRANSMIT(';');
    }
    else {
        UART_TRANSMIT('F');
        while (1) {}
    }
    read_eeprom_calibration(mag_bias, mag_scale);
    
    last_twi_time = micros();
    twi_main_cycle_flag = true;
    twi_mode = 0;
    TW_0();
    _delay_ms(1000);
    
    q0 = 1;
    q1 = 0;
    q2 = 0;
    q3 = 0;
    
    
    while (1) loop();
}

void loop() {
    //(1) Тест значений, полученных с датчиков
    MAIN_Sensors_Data_test();
    //(2) Копирование данных из буфера raw1 в буфер raw2
    MAIN_Sensors_Buffer_Copying();
    //(3) Старт TWI-чтения
    twi_mode = 0;
    TW_0();
    //(4) Обновление данных
    if (!mpu_error_flag){   GET_ACCEL(Accel);
                            GET_GYRO(Gyro); }
    if (!mag_error_flag)    GET_MAG(Mag, mag_bias, mag_scale, mag_bias_factory);
    if (!temp_error_flag)   GET_TEMPERATURE(&Temperature);
    if (!press_error_flag){ GET_PRESSURE(&Pressure);
                            GET_ALTITUDE_VERTSPEED(); }
    if (!hum_error_flag)    GET_HUMIDITY(&Humidity);
        
    //(5) Установка текущего времени; вычисление времени цикла deltat
    MAIN_time_update();
    
    if (!mpu_error_flag) {
    //(6) Алгоритм Маджвика. Вычисление Roll, Pitch, Yaw, dRoll, dPitch, dYaw
    MADGWICK_UPDATE();
    Madgwick_computeAngles();
    Roll = rpy[2];
    Pitch = rpy[1];
    Yaw = rpy[0];
    CALC_DRPY_AND_DELTA_FILTER();
    
    //(7) Вычисление сил стабилизаторов
    CALC_RPY_Forces();
    CALC_Stab_Forces();
    //(9) Вычисление углов отклонения и печать на серво
    CALC_STAB_ANGLE();  
    } 
    if (1)  Servo_Print_Stab();
    else    Servo_Print_Zero();
    


    
    if (twi_error_flag) UART_TRANSMIT('W');
    if (mpu_error_flag) UART_TRANSMIT('M');
    if (mag_error_flag) UART_TRANSMIT('A');
    if (temp_error_flag) UART_TRANSMIT('T');
    if (press_error_flag) UART_TRANSMIT('P');
    if (hum_error_flag) UART_TRANSMIT('H');
    
    /*ultoa(micros(), str, 10);
        for (uint8_t i = 0; i<10; i++) {
            UART_TRANSMIT(str[i]);
        }*/
    UART_TRANSMIT(' ');
    dtostrf(StabA, 7, 5, str);
        for (uint8_t i = 0; i<6; i++) {
            UART_TRANSMIT(str[i]);
        }
    UART_TRANSMIT(' ');
    dtostrf(StabB, 7, 5, str);
        for (uint8_t i = 0; i<6; i++) {
            UART_TRANSMIT(str[i]);
        }
    UART_TRANSMIT(' ');
    dtostrf(StabC, 7, 5, str);
        for (uint8_t i = 0; i<6; i++) {
            UART_TRANSMIT(str[i]);
        }
    UART_TRANSMIT(' ');
    dtostrf(StabD, 7, 5, str);
        for (uint8_t i = 0; i<6; i++) {
            UART_TRANSMIT(str[i]);
        }//*/
    UART_TRANSMIT(' ');
    dtostrf(Roll*180/PI, 7, 5, str);
        for (uint8_t i = 0; i<6; i++) {
            UART_TRANSMIT(str[i]);
        }
    UART_TRANSMIT(' ');
    dtostrf(Pitch*180/PI, 7, 5, str);
        for (uint8_t i = 0; i<6; i++) {
            UART_TRANSMIT(str[i]);
        }
    UART_TRANSMIT(' ');
    dtostrf(Yaw*180/PI, 7, 5, str);
        for (uint8_t i = 0; i<6; i++) {
            UART_TRANSMIT(str[i]);
        }//*/
    
    UART_TRANSMIT('\n');
}





void MAIN_Sensors_Data_test() {
    
    
    
    //(1) Тест значений магнетометра
    if (raw1_mag[0] & raw1_mag[1] & raw1_mag[2] &
        raw1_mag[3] & raw1_mag[4] & raw1_mag[5] == 0xFF) mag_error_flag = true;
    else mag_error_flag = false;
    //(2) Тест значений акселерометра и гироскопа
    if (raw1_accel[0] & raw1_accel[1] & raw1_accel[2] &
        raw1_accel[3] & raw1_accel[4] & raw1_accel[5] &
        raw1_gyro[0] & raw1_gyro[1] & raw1_gyro[2] &
        raw1_gyro[3] & raw1_gyro[4] & raw1_gyro[5] == 0xFF) mpu_error_flag=true;
    else mpu_error_flag = false;
    //(3) Проверка значений термометра
    if ((raw1_temperature[0] == 0/*0x80*/) && (raw1_temperature[1] &
        raw1_temperature[2] == 0)) temp_error_flag = true;
    else temp_error_flag = false;
    //(4) Проверка значений барометра
    if ((raw1_pressure[0] == 0/*0x80*/) && (raw1_pressure[1] &
        raw1_pressure[2] == 0)) press_error_flag = true;
    else press_error_flag = false;
    //(5) Проверка значений гигрометра
    if (raw1_humidity[0] & raw1_humidity[1] == 0xFF) hum_error_flag = true;
    else hum_error_flag = false;
    //(6) Проверка шины TWI
    if (micros() - last_twi_time > 10000) {
        //Если ошибка TWI, значит мы не можем верить прочитанным данным датчиков
        twi_error_flag = true;
        mpu_error_flag = true;
        mag_error_flag = true;
        hum_error_flag = true;
        temp_error_flag = true;
        press_error_flag = true;
        _delay_ms(5);
    }
    else {
        //Если TWI восстановилось, инициализируем датчики
        if (twi_error_flag == true) {
            //MPU_INIT();
            //AK8963_INI(AK8963_ADDR, mag_bias_factory);
            //BME_INI();
        }
        twi_error_flag = false;
    }
}
void MAIN_Sensors_Buffer_Copying() {
    // Копирование данных из буфера raw1 в буфер raw2
    //(1) Копирование данных акселерометра, гироскопа, магнетометра
    if (!(mpu_error_flag | twi_error_flag)) {
        for (uint8_t i=0; i<6; i++) {
            raw2_accel[i] = raw1_accel[i];
            raw2_gyro[i] = raw1_gyro[i];
            raw2_mag[i] = raw1_mag[i];
        }
    }
    //(2) Копирование данных термометра
    if (!(temp_error_flag | twi_error_flag)) {
        raw2_temperature[0] = raw1_temperature[0];
        raw2_temperature[1] = raw1_temperature[1];
        raw2_temperature[2] = raw1_temperature[2];
    }
    //(3) Копирование данных барометра
    if (!(press_error_flag | twi_error_flag)) {
        raw2_pressure[0] = raw1_pressure[0];
        raw2_pressure[1] = raw1_pressure[1];
        raw2_pressure[2] = raw1_pressure[2];
    }
    //(4) Копирование данных гигрометра
    if (!(hum_error_flag | twi_error_flag)) {
        raw2_humidity[0] = raw1_humidity[0];
        raw2_humidity[1] = raw1_humidity[1];
    }
    // В случае обнаружения ошибки барометра или термометра, при расчетах
    // углов поворота должны использоваться предыдущие значения датчика.
    // В случае ошибки магнетометра вместо фильтра Маджвика при расчетах
    // углов поворота должен использоваться упрощенный IMU-фильтр.
    // В случае ошибки акселерометра и гироскопа, углы поворота стабилизаторов
    // должны быть немедленно выведены в 0
    // В случае ошибки гигрометра ничего страшного не происходит
}
void MAIN_time_update() {
    //Установка текущего времени; вычисление времени цикла deltat
    systime_micros = micros();
    deltat = (float)(systime_micros - last_upd_time)*0.000001;
    last_upd_time = systime_micros;
}

void MAIN_data_to_send_prepare() {
    data_to_send[0] = 'I';
    data_to_send[1] = 'm';
    data_to_send[2] = ';';
    data_to_send[7] = ';';
    data_to_send[10] = ';';
    data_to_send[15] = ';';
    data_to_send[20] = ';';
    data_to_send[25] = ';';
    data_to_send[30] = ';';
    data_to_send[35] = ';';
    data_to_send[40] = ';';
    data_to_send[45] = ';';
    data_to_send[50] = ';';
    data_to_send[55] = ';';
    data_to_send[60] = ';';
    data_to_send[65] = ';';
    data_to_send[70] = ';';
    data_to_send[75] = ';';
    data_to_send[80] = ';';
    data_to_send[85] = ';';
    data_to_send[90] = ';';
    data_to_send[95] = ';';
    data_to_send[100] = ';';
    data_to_send[105] = ';';
    data_to_send[110] = ';';
    data_to_send[115] = ';';
    data_to_send[120] = ';';
}

void MAIN_data_to_send_form() {
    // data_to_send[0] = 'I';
    // data_to_send[1] = 'm';
    // data_to_send[2] = ';';
    uint8_t idx = 3;
    char *u;
    // (1) Время ***************************************************************
    uint32_t time = micros();
    u = (char*)&time;
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    // (2) Состояние системы ***************************************************
    data_to_send[idx] = system_mode;
    idx++;
    // (3) Флаги ошибок ********************************************************
    data_to_send[idx] = flags_error;
    idx++;
    idx++;
    // (4,5,6) Акселерометр ****************************************************
    u = (char*)&Accel[0];
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    u = (char*)&Accel[1];
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    u = (char*)&Accel[2];
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    // (7,8,9) Гироскоп ********************************************************
    u = (char*)&Gyro[0];
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    u = (char*)&Gyro[1];
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    u = (char*)&Gyro[2];
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    // (10,11,12) Магнитометр **************************************************
    u = (char*)&Mag[0];
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    u = (char*)&Mag[1];
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    u = (char*)&Mag[2];
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    // (13) Барометр *****************************************************
    u = (char*)&Pressure;
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    // (14) Термометр **********************************************************
    u = (char*)&Temperature;
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    // (15) Гигрометр **********************************************************
    u = (char*)&Humidity;
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    // (16) Угол крена *********************************************************
    u = (char*)&Roll;
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    // (17) Угол тангажа *******************************************************
    u = (char*)&Pitch;
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    // (18) Угол рыскания ******************************************************
    u = (char*)&Yaw;
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    // (19) Высота *************************************************************
    u = (char*)&Altitude;
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    // (20) Вертикальная скорость **********************************************
    u = (char*)&VertSpeed;
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    // (21) Модуль ускорения ***************************************************
    u = (char*)&AccelModule;
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    // (23) Стабилизатор A *****************************************************
    u = (char*)&StabA;
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    // (24) Стабилизатор B *****************************************************
    u = (char*)&StabB;
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    // (25) Стабилизатор C *****************************************************
    u = (char*)&StabC;
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
    idx++;
    idx++;
    // (26) Стабилизатор D *****************************************************
    u = (char*)&StabD;
    data_to_send[idx] = u[0];
    idx++;
    data_to_send[idx] = u[1];
    idx++;
    data_to_send[idx] = u[2];
    idx++;
    data_to_send[idx] = u[3];
}
