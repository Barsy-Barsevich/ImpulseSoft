//#define TWI_vect         _VECTOR(17)
#define MPU_ADDR_W      (0x68<<1)
#define MPU_ADDR_R      (0x68<<1)|1
#define AK8963_ADDR_W   (0x0C<<1)
#define AK8963_ADDR_R   (0x0C<<1)|1
#define BME_ADDR_W      (0x76<<1)
#define BME_ADDR_R      (0x76<<1)|1

void TWI_INI();
bool TWI_START_Condition();
bool TWI_STOP_Condition();
bool TWI_Write_Byte(uint8_t);
uint8_t TWI_Read_Byte(bool);

void TW_0(void);
void TW_1(void);
void TW_2(void);
void TW_3(void);
void TW_4(void);
void TW_5(void);
void TW_6(void);
void TW_7(void);
void TW_8(void);
void TW_9(void);
void TW_10(void);
void TW_11(void);
void TW_12(void);
void TW_13(void);
void TW_14(void);
void TW_15(void);
void TW_16(void);
void TW_17(void);
void TW_18(void);
void TW_19(void);
void TW_20(void);
void TW_21(void);
void TW_22(void);
void TW_23(void);
void TW_24(void);
void TW_25(void);
void TW_26(void);
void TW_27(void);
void TW_28(void);
void TW_29(void);
void TW_30(void);
void TW_31(void);
void TW_32(void);
void TW_33(void);
void TW_34(void);
void TW_35(void);
void TW_36(void);
void TW_37(void);
void TW_38(void);
void TW_39(void);
void TW_40(void);
void TW_41(void);
void TW_42(void);
void TW_43(void);
void TW_44(void);

//******************************************************************************
// Прерывание по вектору TWI
//******************************************************************************
ISR(TWI_vect) {
    twi_mode += 1;
    if (twi_main_cycle_flag == true) {
        twi_fun[twi_mode]();
    }
    twi_ready_flag = true;
}

//******************************************************************************
// Инициализация модуля TWI
//******************************************************************************
void TWI_INI() {
    TWBR = 12; // TWI FREQ = 400000 kHz (f_cpu = 16MHz)
    //Предустановка массива указателей на функции twi_fun
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
}

//******************************************************************************
// Универсальные функции работы с шиной TWI
//******************************************************************************
bool TWI_START_Condition() {
    // <Send START condition>
    // Установка TWINT -> временный запрет работы TWI
    // Установка TWSTA -> генерация стартовой последовательности
    // Установка TWEN -> разрешение работы модуля TWI
    // Установка TWIE -> разрешение прерываний по вектору TWI
    // Установка TWEA -> разрешение выдачи сигнала подтверждения, когда идет
    // прием данных
    twi_ready_flag = false;
    sei();
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWIE);
    _delay_us(15);
    if (twi_ready_flag == false) return (false);
    return (true);
}
bool TWI_Write_Byte(uint8_t data) {
    twi_ready_flag = false;
    TWDR = data;
    sei();
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
    _delay_us(30);
    if (twi_ready_flag == false) return (false);
    return (true);
}
uint8_t TWI_Read_Byte(bool ACKM) {
    twi_ready_flag = false;
    TWDR = 0xFF;
    sei();
    if (ACKM == true)   TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    else                TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
    _delay_us(30);
    if (twi_ready_flag == true) return (TWDR);
    else return(0);
}
bool TWI_STOP_Condition() {
    twi_ready_flag = false;
    sei();
    TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE);
    _delay_us(15);
    if (twi_ready_flag == false) return (false);
    return (true);
}

//******************************************************************************
// MPU9250
//******************************************************************************
void TW_0() {
    //Старт MPU
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_1() {
    //Посылка адреса MPU
    TWDR = MPU_ADDR_W;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_2() {
    //Посылка адреса регистра MPU_ACCEL_XOUT_H
    TWDR = /*MPU_ACCEL_XOUT_H*/59;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_3() {
    //Рестарт MPU
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_4() {
    //Посылка адреса MPU
    TWDR = MPU_ADDR_R;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_5() {
    //Чтение MPU_ACCEL_XOUT_H
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_6() {
    //Чтение MPU_ACCEL_XOUT_L
    raw1_accel[0] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_7() {
    //Чтение MPU_ACCEL_YOUT_H
    raw1_accel[1] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_8() {
    //Чтение MPU_ACCEL_YOUT_L
    raw1_accel[2] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_9() {
    //Чтение MPU_ACCEL_ZOUT_H
    raw1_accel[3] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_10() {
    //Чтение MPU_ACCEL_ZOUT_L
    raw1_accel[4] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_11() {
    //Чтение MPU_TEMP_OUT_H
    raw1_accel[5] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_12() {
    //Чтение MPU_TEMP_OUT_L
    raw1_temp[0] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_13() {
    //Чтение MPU_GYRO_XOUT_H
    raw1_temp[1] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_14() {
    //Чтение MPU_GYRO_XOUT_L
    raw1_gyro[0] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_15() {
    //Чтение MPU_GYRO_YOUT_H
    raw1_gyro[1] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_16() {
    //Чтение MPU_GYRO_YOUT_L
    raw1_gyro[2] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_17() {
    //Чтение MPU_GYRO_ZOUT_H
    raw1_gyro[3] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_18() {
    //Чтение MPU_GYRO_ZOUT_L
    raw1_gyro[4] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_19() {
    raw1_gyro[5] = TWDR;
    // Важно отметить, что операция STOP не генерирует прерывания
    TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE);
    _delay_us(4);
//******************************************************************************
// AK8963
//******************************************************************************
    //Старт AK8963
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_20() {
    //Посылка адреса AK8963
    TWDR = AK8963_ADDR_W;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_21() {
    //Посылка адреса регистра AK8963_HXL
    TWDR = 0x03;//AK8963_HXL
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_22() {
    //Рестарт AK8963
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_23() {
    //Посылка адреса AK8963
    TWDR = AK8963_ADDR_R;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_24() {
    //Чтение AK8963_HXL
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_25() {
    //Чтение AK8963_HXH
    raw1_mag[1] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_26() {
    //Чтение AK8963_HYL
    raw1_mag[0] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_27() {
    //Чтение AK8963_HYH
    raw1_mag[3] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_28() {
    //Чтение AK8963_HZL
    raw1_mag[2] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_29() {
    //Чтение AK8963_HZH
    raw1_mag[5] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_30() {
    //Чтение AK8963_ST2
    //В моде Continious очень важно после операции чтения данных прочитать
    //значение регистра ST2, в противном случае датчик не позволит сенсору
    //перезаписывать данные в выходные регистры
    raw1_mag[4] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_31() {
    // Важно отметить, что операция STOP не генерирует прерывания
    TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE);
    _delay_ms(4);
//******************************************************************************
// BME280
//******************************************************************************
    //Старт BME
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_32() {
    //Посылка адреса BME
    TWDR = BME_ADDR_W;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_33() {
    //Посылка адреса регистра BME_PRESS_MSB
    TWDR = 0xF7/*BME_PRESS_MSB*/;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_34() {
    //Рестарт BME
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_35() {
    //Посылка адреса BME
    TWDR = BME_ADDR_R;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_36() {
    //Чтение BME_PRESS_MSB
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_37() {
    //Чтение BME_PRESS_LSB
    raw1_pressure[0] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_38() {
    //Чтение BME_PRESS_XLSB
    raw1_pressure[1] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_39() {
    //Чтение BME_TEMP_MSB
    raw1_pressure[2] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_40() {
    //Чтение BME_TEMP_LSB
    raw1_temperature[0] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_41() {
    //Чтение BME_TEMP_XLSB
    raw1_temperature[1] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_42() {
    //Чтение BME_HUM_MSB
    raw1_temperature[2] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
    last_twi_time = micros();
}
void TW_43() {
    //Чтение BME_HUM_LSB
    raw1_humidity[0] = TWDR;
    TWDR = 0xFF;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
void TW_44() {
    raw1_humidity[1] = TWDR;
    // Важно отметить, что операция STOP не генерирует прерывания
    TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE);
    last_twi_time = micros();
}
