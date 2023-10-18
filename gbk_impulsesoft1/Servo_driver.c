//Тестовая плата
//#define SERVO_A_OCR             OCR3A
//#define SERVO_B_OCR             OCR3B
//#define SERVO_C_OCR             OCR3C
//#define SERVO_D_OCR             OCR1A
//#define SERVO_OBT_OCR           OCR1B
//Отлаженная плата
#define SERVO_A_OCR             OCR3B
#define SERVO_B_OCR             OCR3C
#define SERVO_C_OCR             OCR1A
#define SERVO_D_OCR             OCR1B
#define SERVO_OBT_OCR           OCR1C
//Тестовая плата
//#define SERVOON_PIN             PB7
//#define SERVOON_DDR             DDRB
//#define SERVOON_PORT            PORTB
//Отлаженная плата
#define SERVOON_PIN             PG3
#define SERVOON_DDR             DDRG
#define SERVOON_PORT            PORTG

// В описании SPT5410 сказано, что серво нужна частота ШИМа 330 Гц.
// 16000000/330 как раз будет 48486. 
#define SERVO_PRESCALER         48486
#define SERVO_HALF_PRESCALER    24243
#define ANGLE_TO_OCR            48486/180
//Когда ICR используется как ТОР, функция IC отключена


void Servo_Driver_INI(void);
void Servo_Print_Stab(void);
void Servo_Print_Zero(void);
void Stab_Servo_Enable(void);
void Stab_Servo_Disable(void);

void Servo_Driver_INI() {
    // OCR1A & OCR1B output, ICP1 input (0)
    DDRB |= (1<<PB5)|(1<<PB6);
    DDRE |= (1<<PE3)|(1<<PE4)|(1<<PE5);
    PORTB &= ~((1<<PB5)|(1<<PB6));
    PORTE &= ~((1<<PE3)|(1<<PE4)|(1<<PE5));
    SERVOON_DDR |= (1<<SERVOON_PIN);
    SERVOON_PORT |= (1<<SERVOON_PIN);
    //Set ICR1 & ICR3 to 48486 (частота 330 Гц как в аптеке)
    ICR1 = SERVO_PRESCALER;
    ICR3 = SERVO_PRESCALER;
    //Set OCRs to 24243
    SERVO_A_OCR = SERVO_HALF_PRESCALER;
    SERVO_B_OCR = SERVO_HALF_PRESCALER;
    SERVO_C_OCR = SERVO_HALF_PRESCALER;
    SERVO_D_OCR = SERVO_HALF_PRESCALER;
    //Prescaler 1/1, Fast PWM ICR-TOP
    TCCR3A = (1<<COM3A1)|
             (0<<COM3A0)|
             (1<<COM3B1)|
             (0<<COM3B0)|
             (1<<COM3C1)|
             (0<<COM3C0)|
             (1<<WGM31)|
             (0<<WGM30);
    TCCR3B = (1<<WGM33)|
             (1<<WGM32)|
             (0<<CS32)|
             (0<<CS31)|
             (1<<CS30);
    TCCR1A = (1<<COM1A1)|
             (0<<COM1A0)|
             (0<<COM1B1)|
             (0<<COM1B0)|
             (0<<COM1C1)|
             (0<<COM1C0)|
             (1<<WGM11)|
             (0<<WGM10);
    TCCR1B = (1<<WGM13)|
             (1<<WGM12)|
             (0<<CS12)|
             (0<<CS11)|
             (1<<CS10);
}

void Servo_Print_Stab() {
    // Нельзя допусить печать слишком больших и слишком малых углов
    //Угол не может быть равен 0, в таком случае импульсов ШИМа не будет.
    //То же справедливо и для угла 180 градусов.
    StabA += 90.0;
    StabB += 90.0;
    StabC += 90.0;
    StabD += 90.0;
    if (StabA < 0) StabA = 1;
    if (StabA > 179) StabA = 179;
    if (StabB < 0) StabB = 1;
    if (StabB > 179) StabB = 179;
    if (StabC < 0) StabC = 1;
    if (StabC > 179) StabC = 179;
    if (StabD < 0) StabD = 1;
    if (StabD > 179) StabD = 179;
    // 90 градусов соответствуют значению 24243
    SERVO_A_OCR = (uint16_t)(StabA * ANGLE_TO_OCR);
    SERVO_B_OCR = (uint16_t)(StabB * ANGLE_TO_OCR);
    SERVO_C_OCR = (uint16_t)(StabC * ANGLE_TO_OCR);
    SERVO_D_OCR = (uint16_t)(StabD * ANGLE_TO_OCR);
}

void Servo_Print_Zero() {
    // 90 градусов соответствуют значению 24243
    SERVO_A_OCR = SERVO_HALF_PRESCALER;
    SERVO_B_OCR = SERVO_HALF_PRESCALER;
    SERVO_C_OCR = SERVO_HALF_PRESCALER;
    SERVO_D_OCR = SERVO_HALF_PRESCALER;
}

void Stab_Servo_Enable() {
    SERVOON_PORT &= ~(1<<SERVOON_PIN);
}

void Stab_Servo_Disable() {
    SERVOON_PORT |= (1<<SERVOON_PIN);
}
