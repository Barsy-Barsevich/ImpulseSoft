#define STAGE1_EXTRACTING_TIME  4000000UL
#define STAGE2_IGNITION_TIME    10000000UL
#define STAGE2_OBT_TIME         1500000UL
#define STAGE2_SSMS_TIME        5000000UL

void CONDITION_MODE_INI(void);
void CONDITION_MODE_0(void);
void CONDITION_MODE_1(void);
void CONDITION_MODE_2(void);
void CONDITION_MODE_3(void);
void CONDITION_MODE_4(void);
void CONDITION_MODE_5(void);
void CONDITION_MODE_6(void);
void CONDITION_MODE_7(void);
void CONDITION_MODE_8(void);
void CONDITION_MODE_9(void);
void CONDITION_MODE_10(void);
void CONDITION_MODE_11(void);

void CONDITION_MODE_INI() {
    condition_mode[0] = CONDITION_MODE_0;
    condition_mode[1] = CONDITION_MODE_1;
    condition_mode[2] = CONDITION_MODE_2;
    condition_mode[3] = CONDITION_MODE_3;
    condition_mode[4] = CONDITION_MODE_4;
    condition_mode[5] = CONDITION_MODE_5;
    condition_mode[6] = CONDITION_MODE_6;
    condition_mode[7] = CONDITION_MODE_7;
    condition_mode[8] = CONDITION_MODE_8;
    condition_mode[9] = CONDITION_MODE_9;
    condition_mode[10] = CONDITION_MODE_10;
    condition_mode[11] = CONDITION_MODE_11;
}
void CONDITION_MODE_0() { //ожидание, режим энергосбережения
    //ожидание команды готовности
    //*ожидание команды присоединения 1й ступени
    //*ожидание команды присоединения 1й ступени
    //*ожидание команды закрытия обтекателя
    //*ожидание команды открытия обтекателя
    while (system_mode == 0) {
        uint8_t data = UART0_RECEIVE();
        switch (data) {
            case '0':
                STAGE1_SENSORS();
                break;
            case '1':
                STAGE1_EXTRACT();
                break;
            case '2':
                STAGE1_ADD();
                break;
            case '3':
                STAGE1_STOP();
                break;
            case '4':
                STAGE2_OBT_OPEN();
                break;
            case '5':
                STAGE2_OBT_CLOSE();
                break;
            case '6':
                STAGE2_SSMS_OPEN();
                break;
            case '7':
                STAGE2_SSMS_CLOSE();
                break;
            case '8':
                STAGE2_SSMS_STOP();
                break;
            case '9':
                STAGE2_SAS_0();
                break;
            case 'A':
                STAGE2_SAS_90();
                break;
            case 'B':
                STAGE2_SAS_180();
                break;
            case 'C':
                STAGE2_SAS_A_05p();
                break;
            case 'D':
                STAGE2_SAS_A_05m();
                break;
            case 'E':
                STAGE2_SAS_B_05p();
                break;
            case 'F':
                STAGE2_SAS_B_05m();
                break;
            case 'G':
                STAGE2_SAS_C_05p();
                break;
            case 'H':
                STAGE2_SAS_C_05m();
                break;
            case 'I':
                STAGE2_SAS_D_05p();
                break;
            case 'J':
                STAGE2_SAS_D_05m();
                break;
            case 'K':
                STAGE_READY_MODE();
                break;
        }
        UART_TRANSMIT('O');
        UART_TRANSMIT('K');
        UART_TRANSMIT('\n');
    }
}
void CONDITION_MODE_1() { //готовность к старту
    //ожидание условия запуска ракеты
    if ((Accel[1] > 2)&&(Altitude > 4)) {
        system_mode += 1;
        //micro_seconds = 0;
        eng_time = 0;
    } 
}
void CONDITION_MODE_2() { //работа двигателя 1й ступени
    //Запуск САС
    if (Altitude > 10) Stab_Servo_Enable();
    //ожидание уменьшения ускорения и времени 3.5с после старта
    if ((micros() > 3500000UL)&&(Accel[1] < 0)) {
        system_mode += 1;
        //подача команды на моторы отделения
        STAGE1_EXTRACT();
        mode3_time = micros();
    }
}
void CONDITION_MODE_3() { //разделение ступеней
    //подача команды на моторы отделения и ожидание 5с
    if (micros() - mode3_time > STAGE1_EXTRACTING_TIME) {
        if (STAGE1_IS_EXTRACTED()) {
            system_mode += 1;
            STAGE1_STOP();
            //подача команды на зажигание двигателя
            STAGE2_IGNITION_COMMAND();
            mode4_time = micros();
        }
        //Ступени не разъеденились
        else {
            system_mode = 10;
            mode7_time = micros();
        }
    }
}
void CONDITION_MODE_4() { //зажигание двигателя 2й ступени
    //подача команды на зажигание двигателя и ожидание увеличения ускорения
    if (Accel[1] > 2) {
        system_mode += 1;
        STAGE2_STOP_IGNITION();
        mode5_time = micros();
        eng_time = 0;
    }    
    if (micros() - mode4_time > STAGE2_IGNITION_TIME) {
        //Двигатель не зажегся
        system_mode = 11;
        STAGE2_STOP_IGNITION();
    }
}
void CONDITION_MODE_5() { //работа двигателя 2й ступени
    //ожидание уменьшения ускорения
    if (((micros() - mode5_time) > 5500000UL)&&(Accel[1] < 0)) system_mode += 1;
}

void CONDITION_MODE_6() { //выгорание двигателя 2й ступени, полёт до апогея
    //стабилизаторы в нулевое положение
    Servo_Print_Zero();
    //ожидание апогея
    if (VertSpeed < 2) {
        system_mode += 1;
        //открытие обтекателя + 1.5с
        STAGE2_OBT_OPEN();
        mode7_time = micros();
    }
}
void CONDITION_MODE_7() { //выпуск системы спасения
    //запрет работы САС
    Stab_Servo_Disable();
    //запуск двигателя ССМС + 5с
    if (micros() - mode7_time > STAGE2_OBT_TIME) {
        system_mode += 1;
        STAGE2_SSMS_OPEN();
        mode8_time = micros();
    }
}
void CONDITION_MODE_8() { //полет к земле
    //ожидание 5с
    if (micros() - mode8_time > STAGE2_SSMS_TIME) {
        system_mode += 1;
        STAGE2_SSMS_STOP();
    }
}
void CONDITION_MODE_9() { //приземление
    //своя игра
}
//фатальные ситуации
void CONDITION_MODE_10() {
    STAGE1_ADD();
    _delay_ms(3000);
    STAGE1_STOP();
    system_mode = 6;
}
void CONDITION_MODE_11() {
    system_mode = 6;
}
