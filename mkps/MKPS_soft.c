#define F_CPU 1000000L

#include <avr/io.h>

#define PM1_PIN         5 //датчик ступени 1
#define PM2_PIN         4 //датчик ступени 2
#define PM_CLOSE_PIN    3 //датчик хода мотора на закрытие
#define PM_OPEN_PIN     2 //датчик хода мотора на открытие
#define IN1_PIN         1
#define IN2_PIN         0

void main(void);
void MOTOR_RIGHT(void);
void MOTOR_LEFT(void);
void MOTOR_STOP(void);

void MOTOR_RIGHT() {
    PORTB |= (1<<IN1_PIN);
    PORTB &= ~(1<<IN2_PIN);
}
void MOTOR_LEFT() {
    PORTB |= (1<<IN2_PIN);
    PORTB &= ~(1<<IN1_PIN);
}
void MOTOR_STOP() {
    PORTB &= ~((1<<IN1_PIN)|(1<<IN2_PIN));
}
void main() {
    //инициализация порта B
    DDRB |= (1<<IN1_PIN)|(1<<IN2_PIN);
    DDRB &= ~((1<<PM1_PIN)|(1<<PM2_PIN)|(1<<PM_CLOSE_PIN)|(1<<PM_OPEN_PIN));
    PORTB |= (1<<PM1_PIN)|(1<<PM2_PIN)|(1<<PM_CLOSE_PIN)|(1<<PM_OPEN_PIN);
    PORTB &= ~((1<<IN1_PIN)|(1<<IN2_PIN));
    MOTOR_STOP();
    //условия работы
    while (1) {
        //Если оба батчика ступени разжаты, то мотор влево до датчика 2
        if ((PINB & (1<<PM1_PIN)) & (PINB & (1<<PM2_PIN))) { //оба разжаты
            if (PINB & (1<<PM_OPEN_PIN)) MOTOR_LEFT();
            else MOTOR_STOP();
        }
        //Если оба датчика ступени замкнуты, то мотор вправо до датчика 1
        if ((PINB & (1<<PM1_PIN)) | (PINB & (1<<PM2_PIN)) == 0) { //оба нажаты
            if (PINB & (1<<PM_CLOSE_PIN)) MOTOR_RIGHT();
            else MOTOR_STOP();
        }
    }
}

