void LCD()
{
  unsigned long LCDMillis = millis();
  if (LCDMillis - prevMillisLCD >= 1000) {
        prevMillisLCD = LCDMillis;
  lcd.setCursor(0, 0);
  lcd.print("H.RAD:      "); //отсылка в сериал для Espeasy
  lcd.setCursor(6, 0);
  lcd.print(HRad);
  lcd.setCursor(0, 1);
  lcd.print("C.Rad:        ");
  lcd.setCursor(6, 1);
  lcd.print(Coldrad);
  lcd.setCursor(0, 2);
  lcd.print("T.In:        ");
  lcd.setCursor(6, 2);
  lcd.print(hdc_t);
  lcd.setCursor(0, 3);
  lcd.print("Hum:         ");
  lcd.setCursor(6, 3);
  lcd.print(hdc_h);
  lcd.setCursor(10, 3);
  lcd.print("AHum:    ");
  lcd.setCursor(15, 3);
  lcd.print(Abshum);
  }
  
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
