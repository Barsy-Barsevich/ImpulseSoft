String choice_str[] = {"Stage1_sensors",
                       "Stage1_extract",
                       "Stage1_add",
                       "Stage1_stop",
                       "OBT_open",
                       "OBT_close",
                       "SSMS_open",
                       "SSMS_close",
                       "SSMS_stop",
                       "SAS_A+05",
                       "SAS_A-05",
                       "SAS_B+05",
                       "SAS_B-05",
                       "SAS_C+05",
                       "SAS_C-05",
                       "SAS_D+05",
                       "SAS_D-05",
                       "WARMODE",
                       "Exit"};
uint8_t win_addr = 0;
uint8_t num_of_strings = 19;
uint8_t pointer = 0;

void Mode2_SendCommand() {
  delay(1500);
  Serial.begin(1000000);
  win_addr = 0;
  pointer = 0;
  to_display();
  bool mode2_flag = true;
  while (mode2_flag) {
    while (digitalRead(K1_PIN) & digitalRead(K2_PIN) & digitalRead(K3_PIN));
    if (!digitalRead(K1_PIN))      step_down();
    else if (!digitalRead(K2_PIN)) step_up();
    else if (!digitalRead(K3_PIN)) mode2_flag = Mode2_Send();
    delay(500);
  }
}
void to_display() {
  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print(choice_str[win_addr]);
  lcd.setCursor(2,1);
  lcd.print(choice_str[win_addr+1]);
  if (win_addr == pointer) lcd.setCursor(0,0);
  else                     lcd.setCursor(0,1);
  lcd.print(">");  
}
void step_down() {
  pointer += 1;
  if (pointer >= num_of_strings) {
    pointer = 0;
    win_addr = 0;
  }
  else {
    if (pointer > win_addr+1) win_addr += 1;
  }
  to_display();
}
void step_up() {
  if (pointer == 0) {
    pointer = num_of_strings-1;
    win_addr = (num_of_strings - 2);
  }
  else {
    pointer -= 1;
    if (pointer < win_addr) win_addr = pointer;
  }
  to_display();
}


bool Mode2_Send() {
  switch (pointer) {
    case 0:
      Serial.print("0"); //Stage1_sensors
      break;
    case 1:
      Serial.print("1"); //Stage1_extract
      break;
    case 2:
      Serial.print("2"); //Stage1_add
      break;
    case 3:
      Serial.print("3"); //Stage1_stop
      break;
    case 4:
      Serial.print("4"); //OBT_open
      break;
    case 5:
      Serial.print("5"); //OBT_close
      break;
    case 6:
      Serial.print("6"); //SSMS_open
      break;
    case 7:
      Serial.print("7"); //SSMS_close
      break;
    case 8:
      Serial.print("8"); //SSMS_stop
      break;
    case 9:
      Serial.print("C"); //SAS_A+05
      break;
    case 10:
      Serial.print("D"); //SAS_A-05
      break;
    case 11:
      Serial.print("E"); //SAS_B+05
      break;
    case 12:
      Serial.print("F"); //SAS_B-05
      break;
    case 13:
      Serial.print("G"); //SAS_C+05
      break;
    case 14:
      Serial.print("H"); //SAS_C-05
      break;
    case 15:
      Serial.print("I"); //SAS_D+05
      break;
    case 16:
      Serial.print("J"); //SAS_D-05
      break;
    case 17:
      Serial.print("K"); //WARMODE
      break;    
    case 18:
      return false; //Exit
  }
  return true;
}
