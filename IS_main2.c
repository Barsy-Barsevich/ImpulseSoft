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
#define MAG_ERROR_FLAG      0x80
#define MPU_ERROR_FLAG      0x40
#define TWI_ERROR_FLAG      0x20
#define TEMP_ERROR_FLAG     0x10
#define PRESS_ERROR_FLAG    0x08
#define HUM_ERROR_FLAG      0x04
// <>
//******************************************************************************
// ------<Cостояниe системы>----------------------------------------------
void (*condition_mode[12])(void);
uint8_t system_mode = 0;
uint32_t mode3_time, mode4_time, mode5_time, mode7_time, mode8_time;
//******************************************************************************
// ------<Флаги ошибок>---------------------------------------------------------
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
float pre_Altitude, pre_Accel[3];
//******************************************************************************
// ------<Значения калибровки магнетометра>-------------------------------------
float mag_bias[3];
float mag_scale[3];
float mag_bias_factory[3];
//******************************************************************************
// ------<Значения времени>-----------------------------------------------------
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
int16_t STABA_BIAS, STABB_BIAS, STABC_BIAS, STABD_BIAS;
//******************************************************************************
//Массив для вывода значений в последовательный порт. Используется для отладки
char str[30];
//Formats
// |Time|Mode|Flags|AX|AY|AZ|GX|GY|GZ|MX|MY|MZ|Pressure|Temp|Humidity|R|P|Y|
// Altitude|VertSpeed|AccelModule|StabA|StabB|StabC|StabD
char data_to_send[128];
uint32_t FlashADDR = 0;
//******************************************************************************
// ------<Инклуды всех файлов>--------------------------------------------------
#include "TIME_Oma.c"
#include "WireOma_m128.c"
#include "UARTOma_m128.c"
#include "EEPROM_read.c"
#include "MPU9250_toolkits.c"
#include "MPU9250_register_map.h"
#include "BME280_toolkits.c"
#include "BME280_register_map.h"
#include "Madgwick.c"
#include "Barsotion_AngleCalc.c"
#include "Servo_driver.c"
#include "LoRa_Oma.c"
#include "SPIFlash_Oma.c"
#include "Stage.c"
#include "MAIN_functions.c"
#include "Condition.c"
//******************************************************************************
// ------<Заголовки функций>----------------------------------------------------
void main(void);
void loop(void);
//******************************************************************************

//******************************************************************************
void main() {
    system_mode = 0;
    // (1) Инициализация всех интерфейсов **************************************
    //Инициализация служебных портов 2 ступени
    STAGE_INI(); 
    //запуск часов машинного времени
    TIME_INI();
    //Инициализация интерфейса UART0
    UART_INI_BAUD(1000000);
    //Инициализация драйвера серво
    Servo_Driver_INI();
    //Stab_Servo_Enable();
    //Инициализация интерфейса TWI
    TWI_INI();
    //Инициализация интерфейса SPI
    SPI_INI();
    //Инициализация LoRa
    LORA_INI();
    //Инициализация системы условий
    CONDITION_MODE_INI();
    
    // (2) Программная задержка -- для устаканивания интерфейсов ***************
    _delay_ms(100);
    
    // (3) Предустановка значений в переменные *********************************
    //Подготовка массива для вывода данных телеметрии в накопитель и радиоканал
    MAIN_data_to_send_prepare();
    //Предустановка кватерниона (без нее Маджвик не работает. Не удалять!)
    q0 = 1;
    q1 = 0;
    q2 = 0;
    q3 = 0;
    // (4) Инициализация датчиков, сенсоров ************************************
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
        //while (1) {}
    }
    if (AK8963_INI(AK8963_ADDR, mag_bias_factory) == true){
        UART_TRANSMIT('A');
        UART_TRANSMIT(';');
    }
    else {
        UART_TRANSMIT('F');
        //while (1) {}
    }
    if (BME_INI() == true){
        UART_TRANSMIT('B');
        UART_TRANSMIT(';');
    }
    else {
        UART_TRANSMIT('F');
        //while (1) {}
    }
    
    // (5) Чтение калибровочных констант IMU из EEPROM микроконтроллера
    read_eeprom_calibration(mag_bias, mag_scale);
    
    // (6) Включение системы условий
    CONDITION_MODE_0();//*/
    // (6*)
    //system_mode = 1;//*/
    /* Данные экзекуции проводятся непосредственно после старта! */
    //Предустановка last_twi_time
    last_twi_time = micros();
    //Инициализация системы TWI
    twi_main_cycle_flag = true;
    twi_mode = 0;
    TW_0();
    _delay_ms(10);
    while (1) loop();
}

void loop() {
    //(1) Тест значений, полученных с датчиков
    MAIN_Sensors_Data_test();
    
    flags_error &= ~(MPU_ERROR_FLAG | MAG_ERROR_FLAG);
    //(2) Копирование данных из буфера raw1 в буфер raw2
    MAIN_Sensors_Buffer_Copying();
    
    //(3) Старт TWI-чтения
    twi_mode = 0;
    TW_0();
    
    //(4) Обновление данных
    if (!(flags_error & MPU_ERROR_FLAG)) {
        GET_ACCEL(Accel);
        GET_ACCEL_MODULE();
        GET_GYRO(Gyro);
    }
    if (!(flags_error & MAG_ERROR_FLAG)) {
        GET_MAG(Mag, mag_bias, mag_scale, mag_bias_factory);
    }
    if (!(flags_error & TEMP_ERROR_FLAG)) {
        GET_TEMPERATURE(&Temperature);
    }
    if (!(flags_error & PRESS_ERROR_FLAG)) {
        GET_PRESSURE(&Pressure);
        GET_ALTITUDE_VERTSPEED();
    }
    if (!(flags_error & HUM_ERROR_FLAG)) {
        GET_HUMIDITY(&Humidity);
    }
    
    //(5) Проверка условий
    condition_mode[system_mode];
    
    VertSpeed = 25;
    Stab_Servo_Enable();
    eng_time = 2;
    system_mode = 2;//*/
    
    //(6) Установка текущего времени; вычисление времени цикла deltat
    MAIN_time_update();
    
    //(7) Алгоритм Маджвика. Вычисление Roll, Pitch, Yaw, dRoll, dPitch, dYaw
    if (!(flags_error & MPU_ERROR_FLAG)) {
        MADGWICK_UPDATE();
        Madgwick_computeAngles();
        /*Roll = rpy[2];
        Pitch = rpy[1];
        Yaw = rpy[0];*/
        Roll = rpy[0];
        Pitch = rpy[1];
        Yaw = rpy[2];
        CALC_DRPY_AND_DELTA_FILTER();
    //(8) Вычисление сил стабилизаторов
        CALC_IM_PS_PARAMETERS();
        CALC_RPY_Forces();
        CALC_Stab_Forces();
    //(9) Вычисление углов отклонения и печать на серво
        CALC_STAB_ANGLE();  
        Servo_Print_Stab();
    }
    else    Servo_Print_Zero();
    
    //(10) Формирование строки телеметрии; отправка по радио; запись в ЭСППЗУ  
    //MAIN_data_to_send_form();
    SS_DOWN();
    bool flag = SPI_ex(W25_Read_SR1) & 0x01;
    SS_UP();
    if (flag != 1) {
        MAIN_data_to_send_form();
        WRITE_DATA_TO_ROM(data_to_send);
    }
    //LORA_Send_Packet(data_to_send);

    //(11) Отправка отладочных данных
    MAIN_DEBUG_UART0();
}
