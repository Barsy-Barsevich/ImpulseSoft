#define EERE  0
#define EEWE  1
#define EEMWE 2
#define EEPM0 4
#define EEPM1 5

void EEPROM_write(uint16_t, uint8_t);

void EEPROM_write(uint16_t addr, uint8_t data) {
    while (EECR & (1<<EEWE)) {}
    // Установка адреса
    EEARH = ((uint8_t)(addr>>8)) & 0x0F;
    EEARL = (uint8_t)addr;
    EEDR = data;
    // Ждем очистки бита записи
    EECR = (1<<EEMWE);
    EECR = (1<<EEMWE)|(1<<EEWE);
}
