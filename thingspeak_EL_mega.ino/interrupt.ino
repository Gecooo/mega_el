void myInterrupt() {
  static unsigned long millis_prev;
    if(millis()-250 > millis_prev){
      flag_work = !flag_work;
      flag_currentTime_day = true;
      if (flag_work) tone(buzzer_pin, 2000, 100);     //записываем в память день старта программы
      if (!flag_work) tone(buzzer_pin, 3000, 150);    //стираем день старта программы
      millis_prev = millis();
    }
}
