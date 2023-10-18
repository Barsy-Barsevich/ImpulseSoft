//******************************************************************************
// ------<Заголовки функций>----------------------------------------------------
void MAIN_DEBUG_UART0(void);
void MAIN_Sensors_Data_test(void);
void MAIN_Sensors_Buffer_Copying(void);
void MAIN_time_update(void);
void MAIN_data_to_send_prepare(void);
void MAIN_data_to_send_form(void);
//******************************************************************************
//---<Подпрограмма для отладки>-------------------------------------------------
void MAIN_DEBUG_UART0() {
    if (flags_error & TWI_ERROR_FLAG) UART_TRANSMIT('W');
    if (flags_error & MPU_ERROR_FLAG) UART_TRANSMIT('M');
    if (flags_error & MAG_ERROR_FLAG) UART_TRANSMIT('A');
    if (flags_error & TEMP_ERROR_FLAG) UART_TRANSMIT('T');
    if (flags_error & PRESS_ERROR_FLAG) UART_TRANSMIT('P');
    if (flags_error & HUM_ERROR_FLAG) UART_TRANSMIT('H');
    dtostrf(deltat, 7, 5, str);
        for (uint8_t i = 0; i<6; i++) {
            UART_TRANSMIT(str[i]);
        }
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
        }
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
//---<Тест данных датчиков>-----------------------------------------------------
void MAIN_Sensors_Data_test() {
    //(0) Обнуление флагов ошибок
    flags_error = 0;
    //(1) Тест значений магнетометра
    if (raw1_mag[0] | raw1_mag[1] | raw1_mag[2] |
        raw1_mag[3] | raw1_mag[4] | raw1_mag[5] == 0x0) flags_error |= MAG_ERROR_FLAG;
    //char sr[3];
    //utoa(((uint16_t)raw1_accel[0]<<8)|(uint16_t)raw1_accel[1],sr,3);
    //UART_PRINTLN(sr,3);
    //else flags_error &= ~MAG_ERROR_FLAG;
    //(2) Тест значений акселерометра и гироскопа
    if (raw1_accel[0] | raw1_accel[1] | raw1_accel[2] |
        raw1_accel[3] | raw1_accel[4] | raw1_accel[5] |
        raw1_gyro[0] | raw1_gyro[1] | raw1_gyro[2] |
        raw1_gyro[3] | raw1_gyro[4] | raw1_gyro[5] == 0x0) flags_error |= MPU_ERROR_FLAG;
    //else flags_error &= ~MPU_ERROR_FLAG;
    //(3) Проверка значений термометра
    if ((raw1_temperature[0] == 0/*0x80*/) && (raw1_temperature[1] &
        raw1_temperature[2] == 0)) flags_error |= TEMP_ERROR_FLAG;
    //else flags_error &= ~TEMP_ERROR_FLAG;
    //(4) Проверка значений барометра
    if ((raw1_pressure[0] == 0/*0x80*/) && (raw1_pressure[1] &
        raw1_pressure[2] == 0)) flags_error |= PRESS_ERROR_FLAG;
    //else flags_error &= ~PRESS_ERROR_FLAG;
    //(5) Проверка значений гигрометра
    if (raw1_humidity[0] & raw1_humidity[1] == 0xFF) flags_error |= HUM_ERROR_FLAG;
    //else flags_error &= ~HUM_ERROR_FLAG;
    //(6) Проверка шины TWI
    /*if (micros() - last_twi_time > 100000) {
        //Если ошибка TWI, значит мы не можем верить прочитанным данным датчиков
        flags_error |= TWI_ERROR_FLAG;
        flags_error |= MPU_ERROR_FLAG;
        flags_error |= MAG_ERROR_FLAG;
        flags_error |= HUM_ERROR_FLAG;
        flags_error |= TEMP_ERROR_FLAG;
        flags_error |= PRESS_ERROR_FLAG;
        _delay_ms(5);
    }
    else {
        //Если TWI восстановилось, инициализируем датчики
        if (flags_error | TWI_ERROR_FLAG) {
            //MPU_INIT();
            //AK8963_INI(AK8963_ADDR, mag_bias_factory);
            //BME_INI();
        }
        flags_error &= ~TWI_ERROR_FLAG;
    }*/
}
//---<Копирование из raw1 в raw2>-----------------------------------------------
void MAIN_Sensors_Buffer_Copying() {
    // Копирование данных из буфера raw1 в буфер raw2
    //(1) Копирование данных акселерометра, гироскопа, магнетометра
    if (!(flags_error & MPU_ERROR_FLAG)) {
        for (uint8_t i=0; i<6; i++) {
            raw2_accel[i] = raw1_accel[i];
            raw2_gyro[i] = raw1_gyro[i];
            raw2_mag[i] = raw1_mag[i];
        }
    }
    //(2) Копирование данных термометра
    if (!(flags_error & TEMP_ERROR_FLAG)) {
        raw2_temperature[0] = raw1_temperature[0];
        raw2_temperature[1] = raw1_temperature[1];
        raw2_temperature[2] = raw1_temperature[2];
    }
    //(3) Копирование данных барометра
    if (!(flags_error & PRESS_ERROR_FLAG)) {
        raw2_pressure[0] = raw1_pressure[0];
        raw2_pressure[1] = raw1_pressure[1];
        raw2_pressure[2] = raw1_pressure[2];
    }
    //(4) Копирование данных гигрометра
    if (!(flags_error & HUM_ERROR_FLAG)) {
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

//---<Обновление данных времени>------------------------------------------------
void MAIN_time_update() {
    //Установка текущего времени; вычисление времени цикла deltat
    uint32_t systime_micros = micros();
    deltat = (float)(systime_micros - last_upd_time)*0.000001;
    last_upd_time = systime_micros;
    eng_time += deltat;
    eng_time2 = eng_time * eng_time;
    eng_time3 = eng_time2 * eng_time;
}
//---<Подпрограммы формирования массива данных телеметрии>----------------------
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
    data_to_send[120] = '\n';
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
