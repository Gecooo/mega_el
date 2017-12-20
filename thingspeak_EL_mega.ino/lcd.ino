void LCD()
{
  unsigned long LCDMillis = millis();
  if (LCDMillis - prevMillisLCD >= 1000) {
        prevMillisLCD = LCDMillis;
  lcd.setCursor(0, 0);
  lcd.print("H.RAD:     "); //отсылка в сериал для Espeasy
  lcd.setCursor(6, 0);
  lcd.print(HRad,1);
  lcd.setCursor(0, 1);
  lcd.print("CRad:     ");
  lcd.setCursor(5, 1);
  lcd.print(Coldrad,1);
  
  //EEPROM_read(1, memoryDay);
  
  
  lcd.setCursor(0, 2);
  lcd.print("Tin:     ");
  lcd.setCursor(4, 2);
  lcd.print(hdc_t,1);
  lcd.setCursor(10, 2);
  lcd.print("Tout:     ");
  lcd.setCursor(15, 2);
  lcd.print(tempout,1);
  lcd.setCursor(0, 3);
  lcd.print("Hum:     ");
  lcd.setCursor(4, 3);
  lcd.print(hdc_h,1);
  lcd.setCursor(10, 3);
  lcd.print("AHum:     ");
  lcd.setCursor(15, 3);
  lcd.print(Abshum,1);
  
  lcd.setCursor(12, 0);
  lcd.print("IR:");
//  lcd.setCursor(12, 1);
//  lcd.print("DAY=");
//  lcd.setCursor(16, 1);
//  lcd.print(currentDay);

  
  }
  
  DateTime now = dt.now();  
   lcd.setCursor(12, 1);
   lcdDigits(now.hour()); lcd.print(":"); lcdDigits(now.minute()); lcd.print(":");lcdDigits(now.second());  
}

// время //
//    lcd.setCursor(15,3);
//    lcdDigits(now.hour()); lcd.print(":"); lcdDigits(now.minute()); //lcd.print(":");lcdDigits(now.second(), DEC);  


void lcdDigits(int digits){
   // Функция для красивого вывода времени. Выводит ноль перед
   // односимвольными числами. &amp;amp;quot;5&amp;amp;quot; будет выведено как &amp;amp;quot;05&amp;amp;quot;
   lcd.print("");
   if(digits < 10)
   lcd.print('0');
   lcd.print(digits);  
}
 
