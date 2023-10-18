/*
MPU_toolkits
Мини-библиотека для работы с MPU9250
*/
//*************************************************************************************************************************
#include "MPU9250_register_map.h"
//#include "/home/victor/AVR/EEPROM_oma/EEPROM_read.c"
//*************************************************************************************************************************
#define MPU_ADDR 0x68 //AD0=0
//#define MPU_ADDR 0x69 //AD0=1
#define AK8963_ADDR 0x0C
#define MAG_Resolution 10.*4912./32768  // Потому что 16 бит
#define BIAS_TO_CURRENT_BITS 1          // Потому что 16 бит
//*************************************************************************************************************************
// Функции:
// Калибровка
void CALIBRATION_ACCEL_GYRO(uint8_t[], uint8_t[]);
void CALIBRATE_MAG(float[], float[], float[]);
// Инициализация
bool MPU_INIT(void/*uint8_t*/);
bool AK8963_INI(uint8_t, float[]);
// Инструменты
bool writeRegister(uint8_t, uint8_t, uint8_t);
uint8_t readRegister(uint8_t, uint8_t, uint8_t*);
// Чтение данных
bool GET_ACCEL_RAW(int16_t[]);
bool GET_ACCEL(float[]);
bool GET_ACCEL_MODULE(void);
bool GET_GYRO_RAW(int16_t[]);
bool GET_GYRO(float[]);
bool GET_MAG_RAW(int16_t[]);
bool GET_MAG(float[], float[], float[], float[]);
// Оффсеты
void set_gyro_offset(uint8_t[]);
void set_accel_offset(uint8_t[]);
void read_eeprom_calibration(float[], float[]);
//void uint8_to_hex(char[], uint8_t);
//*************************************************************************************************************************
// Глобальные переменные
//uint8_t MPU_ADDR;
//uint8_t AK8963_ADDR;
//*************************************************************************************************************************
// Калибровка гироскопа и акселерометра
//*************************************************************************************************************************
void CALIBRATION_ACCEL_GYRO(uint8_t *accel_offset, uint8_t *gyro_offset){
    //vars
    int32_t accel_bias[3];
    int32_t gyro_bias[3];
    accel_bias[0] = 0;
    accel_bias[1] = 0;
    accel_bias[2] = 0;
    gyro_bias[0] =  0;
    gyro_bias[1] =  0;
    gyro_bias[2] =  0;

    writeRegister(MPU_ADDR, MPU_PWR_MGMT_1, (1<<MPU_H_RESET)); // Программный сброс девайса
    _delay_ms(100);
    writeRegister(MPU_ADDR, MPU_PWR_MGMT_1, 0x00); //использовать встроенный тактовый генератор
    writeRegister(MPU_ADDR, MPU_PWR_MGMT_2, 0x00); //разрешить работу всех гироскопов и акселерометров
    _delay_ms(200);                                    
    // Configure device for bias calculation
    writeRegister(MPU_ADDR, MPU_INT_ENABLE, 0x00);   // Запретить прерывания
    writeRegister(MPU_ADDR, MPU_FIFO_EN, 0x00);      // Запретить FIFO
    writeRegister(MPU_ADDR, MPU_I2C_MST_CTRL, 0x00); // Запретить I2C master
    writeRegister(MPU_ADDR, MPU_USER_CTRL, 0x00);    // Запретить FIFO и I2C master моды
    writeRegister(MPU_ADDR, MPU_USER_CTRL, 0x0C);    // Reset FIFO and DMP
    _delay_ms(15);
    // Configure MPU6050 gyro and accelerometer for bias calculation
    writeRegister(MPU_ADDR, MPU_CONFIG, 0x01);       // Set low-pass filter to 188 Hz
    writeRegister(MPU_ADDR, MPU_SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
    writeRegister(MPU_ADDR, MPU_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeRegister(MPU_ADDR, MPU_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity    
    
    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g
    int16_t _accel_raw[3];
    int16_t _gyro_raw[3];
    // Находим средние значения угловой скорости и ускорения за 200 итераций
    for (uint8_t i=200; i>0; i--){
        //Чтение данных гироскопа и акселерометра
        GET_ACCEL_RAW(_accel_raw);
        GET_GYRO_RAW(_gyro_raw);
        //Суммирование
        accel_bias[0] += (int32_t)_accel_raw[0];
        accel_bias[1] += (int32_t)_accel_raw[1];
        accel_bias[2] += (int32_t)_accel_raw[2];
        gyro_bias[0] +=  (int32_t)_gyro_raw[0];
        gyro_bias[1] +=  (int32_t)_gyro_raw[1];
        gyro_bias[2] +=  (int32_t)_gyro_raw[2];
    }
    //Нахождение среднего арифметического
    accel_bias[0] /= 200;
    accel_bias[1] /= 200;
    accel_bias[2] /= 200;
    gyro_bias[0]  /= 200;
    gyro_bias[1]  /= 200;
    gyro_bias[2]  /= 200;
    //Удаление данных о гравитации
    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}
    //Формирование gyro_bias для загрузки в офсетные регистры гироскопа, которые сбрасываются каждый раз при включении датчика.
    gyro_offset[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    gyro_offset[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    gyro_offset[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    gyro_offset[3] = (-gyro_bias[1]/4)       & 0xFF;
    gyro_offset[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    gyro_offset[5] = (-gyro_bias[2]/4)       & 0xFF;
    /*char str[12];
    dtostrf((float)gyro_bias[0]/4, 7, 5, str);
    for (uint8_t i = 0; i<12; i++) {
        UART_TRANSMIT(str[i]);
    }
    UART_TRANSMIT(0x20);
    dtostrf((float)gyro_bias[1]/4, 7, 5, str);
    for (uint8_t i = 0; i<12; i++) {
        UART_TRANSMIT(str[i]);
    }
    UART_TRANSMIT(0x20);
    dtostrf((float)gyro_bias[2]/4, 7, 5, str);
    for (uint8_t i = 0; i<12; i++) {
        UART_TRANSMIT(str[i]);
    }
    UART_TRANSMIT('\n');*/
    set_gyro_offset(gyro_offset);
    //writeRegister(MPU_ADDR, MPU_XG_OFFSET_H, gyro_offset[0]);
    //writeRegister(MPU_ADDR, MPU_XG_OFFSET_L, gyro_offset[1]);
    //writeRegister(MPU_ADDR, MPU_YG_OFFSET_H, gyro_offset[2]);
    //writeRegister(MPU_ADDR, MPU_YG_OFFSET_L, gyro_offset[3]);
    //writeRegister(MPU_ADDR, MPU_ZG_OFFSET_H, gyro_offset[4]);
    //writeRegister(MPU_ADDR, MPU_ZG_OFFSET_L, gyro_offset[5]);
    //Формирование accel_bias для загрузки в офсетные регистры акселерометра. Важно обратить внимание на то, что эти регистры
    //содержат значения factory_trim, которые должны быть добавлены к высчитанным accel_bias. При загрузке эти регистры имеют
    //ненулевые значения. Также, бит 0 МЛБ должен быть сохранен, так как используется при измерении температуры. Регистры
    //смещения акселерометра имеют смещение 2048 МЛБ / g, таким образом, полученные данные следует делить на 8
    int32_t accel_bias_reg[3]; // A place to hold the factory accelerometer trim biases
    uint8_t data1, data2;
    readRegister(MPU_ADDR, MPU_XA_OFFSET_L, &data1);
    readRegister(MPU_ADDR, MPU_XA_OFFSET_H, &data2);
    accel_bias_reg[0] = (uint16_t)data1 | ((uint16_t)data2 << 8);
    readRegister(MPU_ADDR, MPU_YA_OFFSET_L, &data1);
    readRegister(MPU_ADDR, MPU_YA_OFFSET_H, &data2);
    accel_bias_reg[1] = (uint16_t)data1 | ((uint16_t)data2 << 8);
    readRegister(MPU_ADDR, MPU_ZA_OFFSET_L, &data1);
    readRegister(MPU_ADDR, MPU_ZA_OFFSET_H, &data2);
    accel_bias_reg[2] = (uint16_t)data1 | ((uint16_t)data2 << 8);
    /*accel_bias_reg[0] = (readRegister(MPU_ADDR, MPU_XA_OFFSET_L)) | (readRegister(MPU_ADDR, MPU_XA_OFFSET_H)<<8);
    accel_bias_reg[1] = (readRegister(MPU_ADDR, MPU_YA_OFFSET_L)) | (readRegister(MPU_ADDR, MPU_YA_OFFSET_H)<<8);
    accel_bias_reg[2] = (readRegister(MPU_ADDR, MPU_ZA_OFFSET_L)) | (readRegister(MPU_ADDR, MPU_ZA_OFFSET_H)<<8);*/
    // Define array to hold mask bit for each accelerometer bias axis
    uint8_t mask_bit[3];
    for(uint8_t i=0; i<3; i++) {
        // If temperature compensation bit is set, record that fact in mask_bit
        if (accel_bias_reg[i] % 2) {
            mask_bit[i] = 0x01;
        } 
        else {
            mask_bit[i] = 0x00;
        }
        // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
        accel_bias_reg[i] -= ((int16_t)accel_bias[i]>>3);
        // Учёт бита компенсации температуры
        if (mask_bit[i]) {
            accel_bias_reg[i] |= 0x0001;
        }
        else {
            accel_bias_reg[i] &= 0xFFFE;
        }
    }  
    accel_offset[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    accel_offset[1] = (accel_bias_reg[0])      & 0xFF;
    accel_offset[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    accel_offset[3] = (accel_bias_reg[1])      & 0xFF;
    accel_offset[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    accel_offset[5] = (accel_bias_reg[2])      & 0xFF;
    set_accel_offset(accel_offset);
    //writeRegister(MPU_ADDR, MPU_XA_OFFSET_H, accel_offset[0]);
    //writeRegister(MPU_ADDR, MPU_XA_OFFSET_L, accel_offset[1]);
    //writeRegister(MPU_ADDR, MPU_YA_OFFSET_H, accel_offset[2]);
    //writeRegister(MPU_ADDR, MPU_YA_OFFSET_L, accel_offset[3]);
    //writeRegister(MPU_ADDR, MPU_ZA_OFFSET_H, accel_offset[4]);
    //writeRegister(MPU_ADDR, MPU_ZA_OFFSET_L, accel_offset[5]);
}
//*************************************************************************************************************************
// Калибровка магнитометра
//*************************************************************************************************************************
void CALIBRATE_MAG(float *mag_bias, float *mag_scale, float *mag_bias_factory){
    // shoot for ~fifteen seconds of mag data
    writeRegister(AK8963_ADDR, AK8963_CNTL1,
        (1<<AK8963_BIT)|
        (0<<AK8963_MODE3)|
        (1<<AK8963_MODE2)|
        (1<<AK8963_MODE1)|
        (0<<AK8963_MODE0));
    _delay_ms(10);
    
    int32_t bias[3];
    float scale[3]; //изменено с int32_t на float 12.05.23
    int16_t mag_max[3];
    mag_max[0] = -32767;
    mag_max[1] = -32767;
    mag_max[2] = -32767;
    int16_t mag_min[3];
    mag_min[0] = 32767;
    mag_min[1] = 32767;
    mag_min[2] = 32767;
    int16_t mag_raw[3];
    
    for (uint16_t i=0; i<1500; i++) { //must be 1500
        GET_MAG_RAW(mag_raw);
        for (uint8_t j=0; j<3; j++) {
            if (mag_raw[j] < mag_min[j]) mag_min[j] = mag_raw[j];
            if (mag_raw[j] > mag_max[j]) mag_max[j] = mag_raw[j];
        }
        _delay_ms(12);
    }
    // Get hard iron correction
    bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
    bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
    bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts
    float bias_resolution = 10. * 4912. / 32768;
    mag_bias[0] = (float)bias[0] * bias_resolution * mag_bias_factory[0];  // save mag biases in G for main program
    mag_bias[1] = (float)bias[1] * bias_resolution * mag_bias_factory[1];
    mag_bias[2] = (float)bias[2] * bias_resolution * mag_bias_factory[2];
    // Get soft iron correction estimate
    //*** multiplication by mag_bias_factory added in accordance with the following comment
    //*** https://github.com/kriswiner/MPU9250/issues/456#issue-836657973
    scale[0] = (float)(mag_max[0] - mag_min[0]) * mag_bias_factory[0] / 2;  // get average x axis max chord length in counts
    scale[1] = (float)(mag_max[1] - mag_min[1]) * mag_bias_factory[1] / 2;  // get average y axis max chord length in counts
    scale[2] = (float)(mag_max[2] - mag_min[2]) * mag_bias_factory[2] / 2;  // get average z axis max chord length in counts
    float avg_rad = (scale[0] + scale[1] + scale[2]) / 3.0;
    char str[12];
    dtostrf(scale[0], 5, 5, str);
    for (uint8_t i = 0; i<12; i++) {
        UART_TRANSMIT(str[i]);
    }
    mag_scale[0] = avg_rad / ((float)scale[0]);
    mag_scale[1] = avg_rad / ((float)scale[1]);
    mag_scale[2] = avg_rad / ((float)scale[2]);
}
//*************************************************************************************************************************
// Инициализация акселерометра и гироскопа
//*************************************************************************************************************************
bool MPU_INIT(/*uint8_t addr*/) {
    //MPU_ADDR = addr;
    // Настройка режима энергопотребления
    if (writeRegister(MPU_ADDR, MPU_PWR_MGMT_1,
        (1<<MPU_H_RESET)|
        (0<<MPU_SLEEP)|
        (0<<MPU_CYCLE)|
        (0<<MPU_GYRO_STANDBY)) != true) return false;
    // Тест на MPU9250
    uint8_t data;
    readRegister(MPU_ADDR, MPU_WHO_I_AM, &data);
    //if (!readRegister(addr, MPU_WHO_I_AM, &data)) return false;
    if (data != 0x73) return false;
    // Установка BANDWITH
    if (writeRegister(MPU_ADDR, MPU_CONFIG,
        (1<<MPU_DLPF_CFG2)|
        (0<<MPU_DLPF_CFG1)|
        (0<<MPU_DLPF_CFG0)|
        (0<<MPU_Fchoice_b1)|
        (0<<MPU_Fchoice_b0)) != true) return false;
    // предел гироскопа 2000dps, предел акселерометра 16g
    if (writeRegister(MPU_ADDR, MPU_GYRO_CONFIG, 0b00011000) != true) return false;
    if (writeRegister(MPU_ADDR, MPU_ACCEL_CONFIG, 0b00011000) != true) return false;
    if (writeRegister(MPU_ADDR, MPU_ACCEL_CONFIG_2,
        (0<<MPU_accel_fchoice_b)|
        (1<<MPU_A_DLPFCFG2)|
        (0<<MPU_A_DLPFCFG1)|
        (0<<MPU_A_DLPFCFG0)) != true) return false;
    // Разрешить работу гироскопа и акселерометра
    if (writeRegister(MPU_ADDR, MPU_PWR_MGMT_2,
        (0<<MPU_DISABLE_XA)|
        (0<<MPU_DISABLE_YA)|
        (0<<MPU_DISABLE_ZA)|
        (0<<MPU_DISABLE_XG)|
        (0<<MPU_DISABLE_YG)|
        (0<<MPU_DISABLE_ZG)) != true) return false;
    // Очень важно! Включаем bypass mode шины twi, без этого магнетометр не будет работать
    if (writeRegister(MPU_ADDR, MPU_INT_PIN_CFG, 0b00000010) != true) return false;
    return true;
}
//*************************************************************************************************************************
// Инициализация магнитометра
//*************************************************************************************************************************
bool AK8963_INI(uint8_t addr, float *mag_bias_factory) {
    //AK8963_ADDR = addr;
    // test
    uint8_t data;
    if (!readRegister(AK8963_ADDR, AK8963_WIA, &data)) return false;
    if (data != 0x48) return false;
    // Power down mode
    if (writeRegister(AK8963_ADDR, AK8963_CNTL1, 0x00) != true) return false;
    _delay_ms(10);
    // Fuse ROM access mode
    if (writeRegister(AK8963_ADDR, AK8963_CNTL1, 0x0F) != true) return false;
    _delay_ms(10);
    // Чтение значений чувствительности, полученных на заводе
    uint8_t raw_data1, raw_data2, raw_data3;
    readRegister(AK8963_ADDR, AK8963_ASAX, &raw_data1);
    readRegister(AK8963_ADDR, AK8963_ASAY, &raw_data2);
    readRegister(AK8963_ADDR, AK8963_ASAZ, &raw_data3);
    // Получение значений чувствительности по осям
    mag_bias_factory[0] = (float)(raw_data1 - 128) / 256. + 1.;
    mag_bias_factory[1] = (float)(raw_data2 - 128) / 256. + 1.;
    mag_bias_factory[2] = (float)(raw_data3 - 128) / 256. + 1.;
    // Power down mode
    if (writeRegister(AK8963_ADDR, AK8963_CNTL1, 0x00) != true) return false;
    _delay_ms(10);
    // Continious 2 Measurement mode
    if (writeRegister(AK8963_ADDR, AK8963_CNTL1,
        (1<<AK8963_BIT)|    // Шкала 16 бит
        (0<<AK8963_MODE3)|  // Single Measurement mode
        (1<<AK8963_MODE2)|
        (1<<AK8963_MODE1)|
        (0<<AK8963_MODE0)) != true) return false;
}
//*************************************************************************************************************************
// Инструменты
//*************************************************************************************************************************
bool writeRegister(uint8_t addr, uint8_t reg, uint8_t data) {
    uint8_t _i2c_address = addr;
    if (TWI_START_Condition() == false)    return (false);
    if (TWI_Write_Byte(_i2c_address << 1) == false) return (false);
    if (TWI_Write_Byte(reg) == false) return (false);
    if (TWI_Write_Byte(data) == false)     return (false);
    if (TWI_STOP_Condition() == false)     return (false);
    return (true);
}
uint8_t readRegister(uint8_t addr, uint8_t reg, uint8_t *out) {
    uint8_t _i2c_address = addr;
    if (TWI_START_Condition() == false)                   return (false);
    if (TWI_Write_Byte(_i2c_address << 1) == false)       return (false);
    if (TWI_Write_Byte(reg) == false)                     return (false);
    if (TWI_START_Condition() == false)                   return (false);
    if (TWI_Write_Byte((_i2c_address << 1) | 1) == false) return (false);
    *out = TWI_Read_Byte(false);
    if (TWI_STOP_Condition() == false)                    return (false);
    return true;
}
//*************************************************************************************************************************
// Чтение данных с акселерометра, гироскопа, магнетометра
//*************************************************************************************************************************
bool GET_ACCEL_RAW(int16_t *accel_raw) {
    uint8_t data1, data2;
    if (!readRegister(MPU_ADDR, MPU_ACCEL_XOUT_L, &data1)) return false;
    if (!readRegister(MPU_ADDR, MPU_ACCEL_XOUT_H, &data2)) return false;
    accel_raw[0] = (uint16_t)data1 | ((uint16_t)data2 << 8);
    if (!readRegister(MPU_ADDR, MPU_ACCEL_YOUT_L, &data1)) return false;
    if (!readRegister(MPU_ADDR, MPU_ACCEL_YOUT_H, &data2)) return false;
    accel_raw[1] = (uint16_t)data1 | ((uint16_t)data2 << 8);
    if (!readRegister(MPU_ADDR, MPU_ACCEL_ZOUT_L, &data1)) return false;
    if (!readRegister(MPU_ADDR, MPU_ACCEL_ZOUT_H, &data2)) return false;
    accel_raw[2] = (uint16_t)data1 | ((uint16_t)data2 << 8);
    if ((accel_raw[0]==0)&&(accel_raw[1]==0)&&(accel_raw[2]==0)) return false;
    return true;
    /*accel_raw[0] = (readRegister(MPU_ADDR, MPU_ACCEL_XOUT_L)) |
        (readRegister(MPU_ADDR, MPU_ACCEL_XOUT_H)<<8);
    accel_raw[1] = (readRegister(MPU_ADDR, MPU_ACCEL_YOUT_L)) |
        (readRegister(MPU_ADDR, MPU_ACCEL_YOUT_H)<<8);
    accel_raw[2] = (readRegister(MPU_ADDR, MPU_ACCEL_ZOUT_L)) |
        (readRegister(MPU_ADDR, MPU_ACCEL_ZOUT_H)<<8);
    if ((accel_raw[0]==0)&&(accel_raw[1]==0)&&(accel_raw[2]==0)) return false;
    return true;*/
}
bool GET_ACCEL(float *accel) {
    int16_t a_raw[3];
    //ВНИМАНИЕ! МЕНЯЕМ МЕСТАМИ X U Y!!!!!!!!!!!
    /*a_raw[0] = (((uint16_t)raw2_accel[0]<<8)|(uint16_t)raw2_accel[1]); //X
    a_raw[1] = (((uint16_t)raw2_accel[2]<<8)|(uint16_t)raw2_accel[3]); //Y
    a_raw[2] = (((uint16_t)raw2_accel[4]<<8)|(uint16_t)raw2_accel[5]); //Z*/
    a_raw[2] = (((uint16_t)raw2_accel[0]<<8)|(uint16_t)raw2_accel[1]); //X
    a_raw[1] = (((uint16_t)raw2_accel[2]<<8)|(uint16_t)raw2_accel[3]); //Y
    a_raw[0] = (((uint16_t)raw2_accel[4]<<8)|(uint16_t)raw2_accel[5]); //Z
    //if (GET_ACCEL_RAW(a_raw) == false) return false;
    accel[0] = a_raw[0]*0.000488281; //*16/32768
    accel[1] = a_raw[1]*0.000488281;
    accel[2] = a_raw[2]*0.000488281;
    return true;
}
bool GET_ACCEL_MODULE() {
    AccelModule = sqrt(Accel[0]*Accel[0] + Accel[1]*Accel[1] + Accel[2]*Accel[2]);
    return true;
}
bool GET_GYRO_RAW(int16_t *gyro_raw) {
    uint8_t data1, data2;
    if (!readRegister(MPU_ADDR, MPU_GYRO_XOUT_L, &data1)) return false;
    if (!readRegister(MPU_ADDR, MPU_GYRO_XOUT_H, &data2)) return false;
    gyro_raw[0] = (uint16_t)data1 | ((uint16_t)data2 << 8);
    if (!readRegister(MPU_ADDR, MPU_GYRO_YOUT_L, &data1)) return false;
    if (!readRegister(MPU_ADDR, MPU_GYRO_YOUT_H, &data2)) return false;
    gyro_raw[1] = (uint16_t)data1 | ((uint16_t)data2 << 8);
    if (!readRegister(MPU_ADDR, MPU_GYRO_ZOUT_L, &data1)) return false;
    if (!readRegister(MPU_ADDR, MPU_GYRO_ZOUT_H, &data2)) return false;
    gyro_raw[2] = (uint16_t)data1 | ((uint16_t)data2 << 8);
    if ((gyro_raw[0]==0)&&(gyro_raw[1]==0)&&(gyro_raw[2]==0)) return false;
    return true;
    /*gyro_raw[0] = (readRegister(MPU_ADDR, MPU_GYRO_XOUT_L)) |
        (readRegister(MPU_ADDR, MPU_GYRO_XOUT_H)<<8);
    gyro_raw[1] = (readRegister(MPU_ADDR, MPU_GYRO_YOUT_L)) |
        (readRegister(MPU_ADDR, MPU_GYRO_YOUT_H)<<8);
    gyro_raw[2] = (readRegister(MPU_ADDR, MPU_GYRO_ZOUT_L)) |
        (readRegister(MPU_ADDR, MPU_GYRO_ZOUT_H)<<8);
    if ((gyro_raw[0]==0)&&(gyro_raw[1]==0)&&(gyro_raw[2]==0)) return false;
    return true;*/
}
bool GET_GYRO(float *gyro) {
    //int16_t g_raw[3];
    //if (GET_GYRO_RAW(g_raw) == false) return false;
    /*gyro[0] = ((int16_t)raw2_gyro[0]<<8 | (int16_t)raw2_gyro[1])*0.061035156; //*2000/32768
    gyro[1] = ((int16_t)raw2_gyro[2]<<8 | (int16_t)raw2_gyro[3])*0.061035156;
    gyro[2] = ((int16_t)raw2_gyro[4]<<8 | (int16_t)raw2_gyro[5])*0.061035156;*/
    gyro[2] = ((int16_t)raw2_gyro[0]<<8 | (int16_t)raw2_gyro[1])*0.061035156; //*2000/32768
    gyro[1] = ((int16_t)raw2_gyro[2]<<8 | (int16_t)raw2_gyro[3])*0.061035156;
    gyro[0] = ((int16_t)raw2_gyro[4]<<8 | (int16_t)raw2_gyro[5])*0.061035156;
    return true;
}
bool GET_MAG_RAW(int16_t *mraw) {
    // Чтение сырых данных
    uint8_t data1, data2;
    if (!readRegister(AK8963_ADDR, AK8963_HXL, &data1)) return false;
    if (!readRegister(AK8963_ADDR, AK8963_HXH, &data2)) return false;
    mraw[0] = (uint16_t)data1 | ((uint16_t)data2 <<8);
    if (!readRegister(AK8963_ADDR, AK8963_HYL, &data1)) return false;
    if (!readRegister(AK8963_ADDR, AK8963_HYH, &data2)) return false;
    mraw[1] = (uint16_t)data1 | ((uint16_t)data2 <<8);
    if (!readRegister(AK8963_ADDR, AK8963_HZL, &data1)) return false;
    if (!readRegister(AK8963_ADDR, AK8963_HZH, &data2)) return false;
    mraw[2] = (uint16_t)data1 | ((uint16_t)data2 <<8);
    /*mraw[0] = (readRegister(AK8963_ADDR, AK8963_HXL)) |
        (readRegister(AK8963_ADDR, AK8963_HXH)<<8);
    mraw[1] = (readRegister(AK8963_ADDR, AK8963_HYL)) |
        (readRegister(AK8963_ADDR, AK8963_HYH)<<8);
    mraw[2] = (readRegister(AK8963_ADDR, AK8963_HZL)) |
        (readRegister(AK8963_ADDR, AK8963_HZH)<<8);*/
    writeRegister(AK8963_ADDR, AK8963_CNTL1,
        (1<<AK8963_BIT)|    // Шкала 16 бит
        (0<<AK8963_MODE3)|  // Single Measurement mode
        (0<<AK8963_MODE2)|
        (0<<AK8963_MODE1)|
        (1<<AK8963_MODE0));
    if ((mraw[0]==0)&&(mraw[1]==0)&&(mraw[2]==0)) return false;
    return true;
}
bool GET_MAG(float *m, float *mag_bias, float *mag_scale, float *mag_bias_factory) {
    int16_t mag_raw[3];
    //МЕНЯЕМ МЕСТАМИ X НА -Z, A Z -- НА -X
    mag_raw[0] = ((uint16_t)raw2_mag[0]<<8 | (uint16_t)raw2_mag[1]);
    mag_raw[1] = ((uint16_t)raw2_mag[2]<<8 | (uint16_t)raw2_mag[3]);
    mag_raw[2] = ((uint16_t)raw2_mag[4]<<8 | (uint16_t)raw2_mag[5]);
    // Чтение сырых данных
    //if (GET_MAG_RAW(mag_raw) == false) return false;
    // Обработка
    /*m[0] = (float)(mag_raw[0] * MAG_Resolution * mag_bias_factory[0] - mag_bias[0] * BIAS_TO_CURRENT_BITS) / mag_scale[0];
    m[1] = (float)(mag_raw[1] * MAG_Resolution * mag_bias_factory[1] - mag_bias[1] * BIAS_TO_CURRENT_BITS) / mag_scale[1];
    m[2] = (float)(mag_raw[2] * MAG_Resolution * mag_bias_factory[2] - mag_bias[2] * BIAS_TO_CURRENT_BITS) / mag_scale[2];*/
    m[2] = -(float)(mag_raw[0] * MAG_Resolution * mag_bias_factory[0] - mag_bias[0] * BIAS_TO_CURRENT_BITS) / mag_scale[0];
    m[1] = (float)(mag_raw[1] * MAG_Resolution * mag_bias_factory[1] - mag_bias[1] * BIAS_TO_CURRENT_BITS) / mag_scale[1];
    m[0] = -(float)(mag_raw[2] * MAG_Resolution * mag_bias_factory[2] - mag_bias[2] * BIAS_TO_CURRENT_BITS) / mag_scale[2];
    return true;
}
//*************************************************************************************************************************
// Установка смещений, полученных в результате калибровки
//*************************************************************************************************************************
void set_gyro_offset(uint8_t *g_off) {
    writeRegister(MPU_ADDR, MPU_XG_OFFSET_H, g_off[0]);
    writeRegister(MPU_ADDR, MPU_XG_OFFSET_L, g_off[1]);
    writeRegister(MPU_ADDR, MPU_YG_OFFSET_H, g_off[2]);
    writeRegister(MPU_ADDR, MPU_YG_OFFSET_L, g_off[3]);
    writeRegister(MPU_ADDR, MPU_ZG_OFFSET_H, g_off[4]);
    writeRegister(MPU_ADDR, MPU_ZG_OFFSET_L, g_off[5]);
}
void set_accel_offset(uint8_t *a_off) {
    writeRegister(MPU_ADDR, MPU_XA_OFFSET_H, a_off[0]);
    writeRegister(MPU_ADDR, MPU_XA_OFFSET_L, a_off[1]);
    writeRegister(MPU_ADDR, MPU_YA_OFFSET_H, a_off[2]);
    writeRegister(MPU_ADDR, MPU_YA_OFFSET_L, a_off[3]);
    writeRegister(MPU_ADDR, MPU_ZA_OFFSET_H, a_off[4]);
    writeRegister(MPU_ADDR, MPU_ZA_OFFSET_L, a_off[5]);
}

//*************************************************************************************************************************
// Чтение калибровочных констант из памяти EEPROM
// Загрузка констант -- calibrate_full.c из этой же директории. Формат -- 
// accel_bias_X(L) + accel_bias_X(H) + accel_bias_Y(L) + accel_bias_Y(H) +
// accel_bias_Z(L) + accel_bias_Z(H) + gyro_bias_X(L) + gyro_bias_X(H) +
// gyro_bias_Y(L) + gyro_bias_Y(H) + gyro_bias_Z(L) + gyro_bias_Z(H) +
// mag_bias_X (4B) + mag_bias_Y (4B) + mag_bias_Z (4B) +
// mag_scale_X (4B) + mag_scale_Y (4B) + mag_scale_Z (4B)
 //*************************************************************************************************************************
void read_eeprom_calibration(float *m_bias, float *m_scale) {
    uint8_t offset[6];
    uint16_t eeaddr = 0;
    for (uint8_t i=0; i<6; i++) {
        offset[i] = EEPROM_read(eeaddr);
        eeaddr++;
    }
    set_accel_offset(offset);
    for (uint8_t i=0; i<6; i++) {
        offset[i] = EEPROM_read(eeaddr);
        eeaddr++;
    }
    set_gyro_offset(offset);
    char *u;
    for (uint8_t j=0; j<3; j++) {
        u = (char*)&m_bias[j];
        for (uint8_t i=0; i<4; i++) {
            u[i] = EEPROM_read(eeaddr);
            eeaddr++;
        }
    }
    for (uint8_t j=0; j<3; j++) {
        u = (char*)&m_scale[j];
        for (uint8_t i=0; i<4; i++) {
            u[i] = EEPROM_read(eeaddr);
            eeaddr++;
        }
    }
}
//*************************************************************************************************************************


/*void uint8_to_hex(char *str, uint8_t data) {
    str[1] = ((data>>4)&0x0F) | 0x30;
    str[0] = (data & 0x0F) | 0x30;
    if (str[0] > 0x39) str[0] += 7;
    if (str[1] > 0x39) str[1] += 7;
}*/
