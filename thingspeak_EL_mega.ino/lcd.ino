void LCD()
{
  unsigned long LCDMillis = millis();
  if (LCDMillis - prevMillisLCD >= 1000) {
        prevMillisLCD = LCDMillis;
  lcd.setCursor(0, 0);
  lcd.print("HOT RAD:       "); //отсылка в сериал для Espeasy
  lcd.setCursor(10, 0);
  lcd.print(HRad);
  lcd.setCursor(0, 1);
  lcd.print("COLD Rad:        ");
  lcd.setCursor(10, 1);
  lcd.print(Coldrad);
  lcd.setCursor(0, 2);
  lcd.print("TempIn:        ");
  lcd.setCursor(10, 2);
  lcd.print(t);
  lcd.setCursor(0, 3);
  lcd.print("Hum:           ");
  lcd.setCursor(10, 3);
  lcd.print(h);
  }
}
