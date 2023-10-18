/*
TIME -- маленькая библиотека
Тактовая частота процессора -- 16МГц!
*/

void TIME_INI(void);
uint32_t micros(void);

uint32_t micro_seconds = 0;

ISR(TIMER0_OVF_vect) {
    micro_seconds += 1024;
}

void TIME_INI() {
    //atmega128:
    // Fast PWM mode, OC0 pin as usual pin, prescaler 128
    TCCR0 = (0<<COM01)|
            (0<<COM00)|
            (1<<WGM01)|
            (1<<WGM00)|
            (1<<CS02)|
            (0<<CS01)|
            (1<<CS00);
    TCNT0 = 0;
    TIMSK &= ~(1<<OCIE0);
    TIMSK |= (1<<TOIE0);
    sei();
    //atmega328:
    /*TCCR0A = (0<<COM0A1)|
             (0<<COM0A0)|
             (0<<COM0B1)|
             (0<<COM0B0)|
             (1<<WGM01)|
             (1<<WGM00);
    TCCR0B |= (0<<CS02)|(1<<CS01)|(1<<CS00);
    TCNT0 = 0;
    TIMSK0 = 0b00000001;
    sei();*/
}

uint32_t micros() {
    uint32_t ans = micro_seconds;
    ans += ((uint32_t)TCNT0<<2);
    return ans;
}
