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
//#define SS_DDR      DDRB
//#define SS_PORT     PORTB
#define SS_PIN      PB4
//#define DDR_SPI     DDRB
#define DD_MOSI     PB2
#define DD_SCK      PB1

inline void SS_DOWN(void);
inline void SS_UP(void);
void SPI_INI(void);
void W25_Device_Erase(void);
void W25_Write_Page(char*, uint16_t, uint8_t, uint8_t);

//******************************************************************************
// Управление линией \CS
inline void SS_DOWN() {PORTB &= (~(1<<SS_PIN));};
inline void SS_UP() {PORTB |= (1<<SS_PIN);};

//******************************************************************************
// Инициализация SPI и FLASH
void SPI_INI() {
    // Set MOSI and SCK output, all others input
    DDRB = (1<<DD_MOSI)|(1<<DD_SCK);
    // Enable SPI, Master, set clock rate f_cpu/4
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0);
    // Инициализация порта Slave Select
    PORTB |= (1<<SS_PIN);
    DDRB |= (1<<SS_PIN);
}

//******************************************************************************
// Обмен по SPI
uint8_t SPI_ex(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1<<SPIF))) {}
    return SPDR;
}

//******************************************************************************
// Стирание всего чипа. Занимает ~200 секунд времени
void W25_Device_Erase() {
    SS_DOWN();
    SPI_ex(W25_Write_Enable);   // Разрешение записи
    SS_UP();
    SS_DOWN();
    SPI_ex(W25_Chip_Erase);     // Стирание чипа
    SS_UP();
    uint8_t flag = 1;
    while (flag == 1) {         // Ждем окончания стирания
        SS_DOWN();
        flag = SPI_ex(W25_Read_SR1) & 0x01;
        SS_UP();
    }
    SS_DOWN();
    SPI_ex(W25_Write_Disable);  // Запрет записи
    SS_UP();
}

//******************************************************************************
// Запись страницы памяти
void W25_Write_Page(char *page, uint16_t page_addr, uint8_t alku_addr, uint8_t num) {
    // Разрешение записи
    SS_DOWN();
    SPI_ex(W25_Write_Enable);
    SS_UP();
    // Отправка данных
    SS_DOWN();
    SPI_ex(W25_Page_Program);
    SPI_ex((uint8_t)page_addr >> 8);
    SPI_ex((uint8_t)page_addr & 0x00FF);
    SPI_ex(alku_addr);
    for (uint8_t i = alku_addr; i<num; i++) {
        SPI_ex(page[i]);
    }
    SS_UP();
    // Ждем окончания записи
    uint8_t flag = 1;
    while (flag == 1) {
        SS_DOWN();
        flag = SPI_ex(W25_Read_SR1) & 0x01;
        SS_UP();
    }
    // Запрет записи
    SS_DOWN();
    SPI_ex(W25_Write_Disable);
    SS_UP();
}


























