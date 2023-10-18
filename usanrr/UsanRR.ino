/* Прошивка приемной станции команды "Импульс", 25.06.2023
 */

//#define MOSI_PIN        11
//#define MISO_PIN        12
//#define SCK_PIN         13
#define SD_CS_PIN       10
#define FLASH_CS_PIN    9
#define RX1_PIN         5
#define TX1_PIN         4
#define LORA_AUX_PIN    6
#define LORA_M0_PIN     2
#define LORA_M1_PIN     3
#define K1_PIN          14+1//A1
#define K2_PIN          14+0//A0
#define K3_PIN          14+3//A3
#define K4_PIN          14+2//A2


//Пины A6 и А7 не мобут быть использованы как цифровые в ардуино-Нано!!!

#include <SPI.h>
#include <SD.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
//#include <SPIMemory.h>
#include <SoftwareSerial.h>

LiquidCrystal_I2C lcd(0x27,16,2);  // Устанавливаем дисплей
SoftwareSerial mySerial(RX1_PIN, TX1_PIN); // RX, TX
//SPIFlash flash(FLASH_CS_PIN);
File ReceiveFile;

uint8_t data_buffer[12];
uint16_t received_packets = 0;


void setup(){
  lcd.init();   
  //lcd.begin(16, 2);                  // Задаем размерность экрана
  lcd.backlight();
  pinMode(K1_PIN,INPUT_PULLUP);
  pinMode(K2_PIN,INPUT_PULLUP);
  pinMode(K3_PIN,INPUT_PULLUP);
  pinMode(K4_PIN,INPUT_PULLUP);
  pinMode(LORA_AUX_PIN, INPUT);
  pinMode(LORA_M0_PIN, OUTPUT);
  pinMode(LORA_M1_PIN, OUTPUT);
}



void loop(){
  delay(500);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("1-Rec  2-Command"));
  lcd.setCursor(0,1);
  lcd.print(F("3-ReadROM"));
  while (digitalRead(K1_PIN) & digitalRead(K2_PIN) & digitalRead(K3_PIN));
  if (!digitalRead(K1_PIN))      Mode1_Receive();
  else if (!digitalRead(K2_PIN)) Mode2_SendCommand();
  else if (!digitalRead(K3_PIN)) Mode3_ReadROM();
}
