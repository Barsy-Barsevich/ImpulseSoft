/* 
 * 0xFFFF - Address
 * REG0: 011 00 101 (0x65) - UART 9600 baud, 8N1, 19.2k air data rate
 * REG1: 01 0 000 00 (0x40) - 128 byte buffer, RSSI disable, 30dBm
 * REG2: 0x17 (23 channel - 433.1 MHz)
 * REG3: 0 0 0 1 0 000 (0x10) - LBT enable
 */
bool semaphore = false;

void Mode1_Receive() {
  lcd.clear();
  // initializing SD
  if (!SD.begin(SD_CS_PIN)) {
    lcd.setCursor(0,0);
    lcd.print(F("SD init error"));
    lcd.setCursor(0,1);
    lcd.print(F("> Exit"));
    while (digitalRead(K3_PIN));
  }
  else {
    ReceiveFile = SD.open("rf.txt", FILE_WRITE);
    ReceiveFile.print(F("Start receiving the data!\n"));
    mySerial.begin(38400);
    
    // go to idle mode
    digitalWrite(LORA_M0_PIN, HIGH);
    digitalWrite(LORA_M1_PIN, HIGH);
    delay(200);
    // write settings
    char pod[] = {0xC0,0x00,0x06,0xFF,0xFF,0x65,0x40,0x17,0x10};
    mySerial.write(pod,9);
    delay(20);
    // go to normal mode
    digitalWrite(LORA_M0_PIN, LOW);
    digitalWrite(LORA_M1_PIN, LOW);
    delay(20);

    //mySerial.begin(115200);
    attachInterrupt(22,LORA_REC_ALKU,FALLING);
    lcd.setCursor(0,0);
    lcd.print(F("Received:"));
    lcd.setCursor(0,1);
    lcd.print(F("> Exit"));

    // the main cycle
    bool receiver_mode = false;
    while (!receiver_mode) {
      delay(5);
      if (semaphore == true) {
        ReceiveFile.write(data_buffer,128);
        semaphore = false;
      }
      lcd.setCursor(10,0);
      lcd.print(received_packets);
      if (!digitalRead(K3_PIN)) receiver_mode = true;
    }
    ReceiveFile.close();
  }
}

void LORA_REC_ALKU() {
  uint8_t pointer = 0;
  while (!digitalRead(LORA_AUX_PIN)) {
    while (!mySerial.available());
    data_buffer[pointer] = mySerial.read();
    pointer += 1;
  }
  received_packets += 1;
  semaphore = true;
}
