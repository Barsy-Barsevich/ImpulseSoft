//******************************************************************************
// Внимание! Из-за произошедшей подставы невозможно использовать аппаратный SPI
// микроконтроллера. Программную реализацию надо переписать на ассемблер
//******************************************************************************
//<Команды W25Qxx>
#define W25_Write_Enable                0x06
#define W25_Volatile_SR_Write_Enable    0x50
#define W25_Write_Disable               0x04
#define W25_Read_SR1                    0x05
#define W25_Write_SR1                   0x01
#define W25_Read_SR2                    0x35
#define W25_Write_SR2                   0x31
#define W25_Read_SR3                    0x15
#define W25_Write_SR3                   0x11
#define W25_Chip_Erase                  0x60
#define W25_Power_Down                  0xB9
#define W25_Release_Power_Down_ID       0xAB
#define W25_Manufacturer_Device_ID      0x90
#define W25_JEDEC_ID                    0x9F
#define W25_Global_Block_Lock           0x7E
#define W25_Global_Block_Unlock         0x98
#define W25_Enter_QPI_Mode              0x38
#define W25_Enable_Reset                0x66
#define W25_Reset_Device                0x99
#define W25_Read_Unique_ID              0x4B
#define W25_Page_Program                0x02
#define W25_Quad_Page_Program           0x32
#define W25_Sector_Erase_4KB            0x20
#define W25_Sector_Erase_32KB           0x52
#define W25_Sector_Erase_64KB           0xD8
#define W25_Read_Data                   0x03
#define W25_Fast_Read                   0x0B
//<>
#define SS_PIN      PB1
#define MISO_PIN    PB2
#define MOSI_PIN    PB3
#define SCK_PIN     PB4
//******************************************************************************
//<>
inline void SS_DOWN(void);
inline void SS_UP(void);
void SPI_INI(void);
uint8_t SPI_ex(uint8_t);
void WRITE_DATA_TO_ROM(char[]);

//******************************************************************************
// Управление линией \CS
inline void SS_DOWN() {PORTB &= (~(1<<SS_PIN));};
inline void SS_UP() {PORTB |= (1<<SS_PIN);};
void SPI_INI() { // Инициализация программного SPI
    //Инициализация всех портов
    PORTB |= (1<<SS_PIN);
    PORTB &= ~(1<<MISO_PIN);
    DDRB |= (1<<SS_PIN)|(1<<MOSI_PIN)|(1<<SCK_PIN);
    DDRB &= ~(1<<MISO_PIN);
    //Special
    FlashADDR = 0;
}
uint8_t SPI_ex(uint8_t data) { // Обмен по SPI
    uint8_t out_data = 0;
    for (uint8_t i=0; i<8; i++) {
        //(1) Установка MOSI
        if (data & 0x80) PORTB |= (1<<MOSI_PIN);
        else             PORTB &= ~(1<<MOSI_PIN);
        data <<= 1;
        //_delay_us(1);
        //(2) Поднятие SCK
        PORTB |= (1<<SCK_PIN);
        //_delay_us(1);
        //(3) Чтение MISO
        out_data <<= 1;
        if (PINB & (1<<MISO_PIN)) out_data += 1;
        //_delay_us(1);
        //(4) Опускание SCK
        PORTB &= ~(1<<SCK_PIN);
        //_delay_us(1);
    }
    PORTB &= ~(1<<MOSI_PIN);
    return out_data;
}
//******************************************************************************
void WRITE_DATA_TO_ROM(char *page) {
    // Разрешение записи
    SS_DOWN();
    SPI_ex(W25_Write_Enable);
    SS_UP();
    // Отправка данных
    SS_DOWN();
    SPI_ex(W25_Page_Program);
    SPI_ex((FlashADDR>>16)&0xFF);
    SPI_ex((FlashADDR>>8)&0xFF);
    SPI_ex(FlashADDR & 0xFF);
    for (uint8_t i=0; i<128; i++) {
        SPI_ex(page[i]);
    }
    SS_UP();
    //_delay_ms(100);
    // Запрет записи
    SS_DOWN();
    SPI_ex(W25_Write_Disable);
    SS_UP();
    /*// Ждем окончания записи
    uint8_t flag = 1;
    while (flag == 1) {
        SS_DOWN();
        flag = SPI_ex(W25_Read_SR1) & 0x01;
        SS_UP();
    }*/
    //Special
    FlashADDR += 128;
}
