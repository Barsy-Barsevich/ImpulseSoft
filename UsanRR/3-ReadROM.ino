void Mode3_ReadROM() {
  bool sd = SD.begin(SD_CS_PIN);
  bool fl = true;//flash.begin();
  if (sd && fl) Mode3_Standart();
  else if (!sd) Mode3_SD_Error();
  else Mode3_Flash_Error();
}

void Mode3_Standart() {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("> Read_EEPROM"));
    lcd.setCursor(2,1);
    lcd.print(F("Exit"));
    bool mode3_flag = true;
    uint8_t mode3_pointer = 0;
    while (mode3_flag) {
      delay(500);
      while (digitalRead(K1_PIN) & digitalRead(K2_PIN) & digitalRead(K3_PIN));
      if (!(digitalRead(K1_PIN) & digitalRead(K2_PIN))) { //down/up
        if (mode3_pointer == 0) {
          mode3_pointer = 1;
          lcd.setCursor(0,0);
          lcd.print(F(" "));
          lcd.setCursor(0,1);
          lcd.print(F(">"));
        }
        else {
          mode3_pointer = 0;
          lcd.setCursor(0,0);
          lcd.print(F(">"));
          lcd.setCursor(0,1);
          lcd.print(F(" "));
        }
      }
      else if (!digitalRead(K3_PIN)) { //ok
        if (mode3_pointer == 0) ReadEEPROM();
        mode3_flag = false;
      }
    }
}
void Mode3_SD_Error() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("SD init error"));
  lcd.setCursor(0,1);
  lcd.print(F("> Exit"));
  while (digitalRead(K3_PIN));
}
void Mode3_Flash_Error() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Flash init error"));
  lcd.setCursor(0,1);
  lcd.print(F("> Exit"));
  while (digitalRead(K3_PIN));
}
void ReadEEPROM() {
  /*RRfile = SD.open("ReadEEPROM.txt", FILE_WRITE);
  String inputStr;
  uint32_t strAddr = 0; 
  inputStr = flash.readStr(strAddr, inputStr);
  RRfile.print(inputStr);
  RRfile.close();//*/
}//*/
