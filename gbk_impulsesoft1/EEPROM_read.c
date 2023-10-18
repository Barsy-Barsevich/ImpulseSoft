#define EERE  0
#define EEWE  1
#define EEMWE 2

uint8_t EEPROM_read(uint16_t);

uint8_t EEPROM_read(uint16_t addr) {
    // Ждем очистки бита записи
    while (EECR & (1<<EEWE)) {}
    // Установка адреса
    EEARH = (addr>>8) & 0x0F;
    EEARL = addr & 0xFF;
    // Установка бита чтения
    EECR = (1<<EERE);
    return (EEDR);
}
