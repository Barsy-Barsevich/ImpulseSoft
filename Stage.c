#define EN1_PIN         PC2
#define EN2_PIN         PC7
#define EN3_PIN         PA2
#define EN4_PIN         PA7
#define IN11_PIN        PC3
#define IN12_PIN        PC6
#define IN21_PIN        PC5
#define IN22_PIN        PC4
#define IN31_PIN        PA4
#define IN32_PIN        PA5
#define IN41_PIN        PA6
#define IN42_PIN        PA3
#define PH0_PIN         PF7
#define PH1_PIN         PF6
#define PH2_PIN         PF5
#define AV_PIN          PF4
#define SIM_DTR_PIN     PF1
#define FIRE_PIN        PG0

#define SAS_STAB_BIAS_STEP      200

void STAGE_INI(void);
bool STAGE1_IS_EXTRACTED(void);
void STAGE1_SENSORS(void);
void STAGE1_EXTRACT(void);
void STAGE1_ADD(void);
void STAGE1_STOP(void);
void STAGE2_IGNITION_COMMAND(void);
void STAGE2_STOP_IGNITION(void);
void STAGE2_OBT_OPEN(void);
void STAGE2_OBT_CLOSE(void);
void STAGE2_SSMS_OPEN(void);
void STAGE2_SSMS_CLOSE(void);
void STAGE2_SSMS_STOP(void);
void STAGE2_SAS_0(void);
void STAGE2_SAS_90(void);
void STAGE2_SAS_180(void);
void STAGE2_SAS_A_05p(void);
void STAGE2_SAS_A_05m(void);
void STAGE2_SAS_B_05p(void);
void STAGE2_SAS_B_05m(void);
void STAGE2_SAS_C_05p(void);
void STAGE2_SAS_C_05m(void);
void STAGE2_SAS_D_05p(void);
void STAGE2_SAS_D_05m(void);
void STAGE_READY_MODE(void);

void STAGE_INI() {
    PORTG = (1<<FIRE_PIN);
    DDRG = (1<<FIRE_PIN);
    DDRA |= (1<<EN3_PIN)|(1<<IN42_PIN)|(1<<IN31_PIN)|(1<<IN32_PIN)|
        (1<<IN41_PIN)|(1<<EN4_PIN);
    DDRC = (1<<EN1_PIN)|(1<<IN11_PIN)|(1<<IN22_PIN)|(1<<IN21_PIN)|(1<<IN12_PIN)|
        (1<<EN2_PIN);
    DDRF &= ~((1<<PH0_PIN)|(1<<PH1_PIN)|(1<<PH2_PIN)|(1<<AV_PIN)|(1<<SIM_DTR_PIN));
    SERVO_OBT_OCR = SERVO_PRESCALER - 100;
}
bool STAGE1_IS_EXTRACTED() {
    if ((PINF & (1<<PH0_PIN) != 0) && (PINF & (1<<PH1_PIN) != 0)) return true;
    else return false;
}
void STAGE1_SENSORS() {
    UART_TRANSMIT('O');
    UART_TRANSMIT('K');
    UART_TRANSMIT('<');
    uint8_t datasw1 = PINF & (1<<PH0_PIN);
    uint8_t datasw2 = PINF & (1<<PH1_PIN);
    if ((datasw1 == 0) && (datasw2 == 0)) {
        UART_TRANSMIT('1'); // 1 -- ступень присоединена
    }
    else if ((datasw1 != 0) && (datasw2 != 0)) {
        UART_TRANSMIT('2'); // 2 -- ступень отсоединена
    }
    else UART_TRANSMIT('3');// 3 -- непонятное состояние
    UART_TRANSMIT('>');
    //UART_TRANSMIT('\n');
}
void STAGE1_EXTRACT() {
    PORTA |= (1<<IN31_PIN)|(1<<IN41_PIN); // Stage 1 -- Motors 3&4
    PORTA &= ~((1<<IN32_PIN)|(1<<IN42_PIN));
    PORTA |= (1<<EN3_PIN)|(1<<EN4_PIN);
}
void STAGE1_ADD() {
    PORTA |= (1<<IN32_PIN)|(1<<IN42_PIN);
    PORTA &= ~((1<<IN31_PIN)|(1<<IN41_PIN));
    PORTA |= (1<<EN3_PIN)|(1<<EN4_PIN);
}
void STAGE1_STOP() {
    PORTA &= ~((1<<EN3_PIN)|(1<<EN4_PIN));
    PORTA &= ~((1<<IN31_PIN)|(1<<IN32_PIN)|(1<<IN41_PIN)|(1<<IN42_PIN));
}
void STAGE2_IGNITION_COMMAND() {
    PORTG &= ~(1<<FIRE_PIN);
}
void STAGE2_STOP_IGNITION() {
    PORTG |= (1<<FIRE_PIN);
}
void STAGE2_OBT_OPEN() {
    SERVO_OBT_OCR = SERVO_HALF_PRESCALER;
}
void STAGE2_OBT_CLOSE() {
    SERVO_OBT_OCR = 10000;
}
void STAGE2_SSMS_OPEN() { // SSMS -- Motor 1
    PORTC |= (1<<IN11_PIN);
    PORTC &= ~(1<<IN12_PIN);
    PORTC |= (1<<EN1_PIN);
}
void STAGE2_SSMS_CLOSE() {
    PORTC |= (1<<IN12_PIN);
    PORTC &= ~(1<<IN11_PIN);
    PORTC |= (1<<EN1_PIN);
}
void STAGE2_SSMS_STOP() {
    PORTC &= ~((1<<EN1_PIN)|(1<<IN11_PIN)|(1<<IN12_PIN));
}
void STAGE2_SAS_0() {    
    Stab_Servo_Enable();
    SERVO_A_OCR = 1;
    SERVO_B_OCR = 1;
    SERVO_C_OCR = 1;
    SERVO_D_OCR = 1;
    _delay_ms(500);
    Stab_Servo_Disable();    
}
void STAGE2_SAS_90() {
    Stab_Servo_Enable();
    SERVO_A_OCR = SERVO_HALF_PRESCALER;
    SERVO_B_OCR = SERVO_HALF_PRESCALER;
    SERVO_C_OCR = SERVO_HALF_PRESCALER;
    SERVO_D_OCR = SERVO_HALF_PRESCALER;
    _delay_ms(500);
    Stab_Servo_Disable();
}
void STAGE2_SAS_180() {
    Stab_Servo_Enable();
    SERVO_A_OCR = SERVO_PRESCALER - 1;
    SERVO_B_OCR = SERVO_PRESCALER - 1;
    SERVO_C_OCR = SERVO_PRESCALER - 1;
    SERVO_D_OCR = SERVO_PRESCALER - 1;
    _delay_ms(500);
    Stab_Servo_Disable();
}
void STAGE2_SAS_A_05p() {
    STABA_BIAS += SAS_STAB_BIAS_STEP;
    Stab_Servo_Enable();
    Servo_Print_Zero();
    _delay_ms(500);
    Stab_Servo_Disable();
}
void STAGE2_SAS_A_05m() {
    STABA_BIAS -= SAS_STAB_BIAS_STEP;
    Stab_Servo_Enable();
    Servo_Print_Zero();
    _delay_ms(500);
    Stab_Servo_Disable();
}
void STAGE2_SAS_B_05p() {
    STABB_BIAS += SAS_STAB_BIAS_STEP;
    Stab_Servo_Enable();
    Servo_Print_Zero();
    _delay_ms(500);
    Stab_Servo_Disable();
}
void STAGE2_SAS_B_05m() {
    STABB_BIAS -= SAS_STAB_BIAS_STEP;
    Stab_Servo_Enable();
    Servo_Print_Zero();
    _delay_ms(500);
    Stab_Servo_Disable();
}
void STAGE2_SAS_C_05p() {
    STABC_BIAS += SAS_STAB_BIAS_STEP;
    Stab_Servo_Enable();
    Servo_Print_Zero();
    _delay_ms(500);
    Stab_Servo_Disable();
}
void STAGE2_SAS_C_05m() {
    STABC_BIAS -= SAS_STAB_BIAS_STEP;
    Stab_Servo_Enable();
    Servo_Print_Zero();
    _delay_ms(500);
    Stab_Servo_Disable();
}
void STAGE2_SAS_D_05p() {
    STABD_BIAS += SAS_STAB_BIAS_STEP;
    Stab_Servo_Enable();
    Servo_Print_Zero();
    _delay_ms(500);
    Stab_Servo_Disable();
}
void STAGE2_SAS_D_05m() {
    STABD_BIAS -= SAS_STAB_BIAS_STEP;
    Stab_Servo_Enable();
    Servo_Print_Zero();
    _delay_ms(500);
    Stab_Servo_Disable();
}
void STAGE_READY_MODE() {
    //LORA_Normal_Mode();
    system_mode = 1;
}
