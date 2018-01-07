void PID_termostat()
{
  if (flag_work) // если нажата кнопка включить запускаем работу пелеть и ПИД регулятора
  {
    //if (currentDay < 1) Setpoint = 12;
//    if (currentDay >= 0 && hdc_t >= 13.00 && hdc_h >= 75.00) Setpoint = 0;
//    if (currentDay >= 0 && hdc_t < 13.00 && hdc_h < 72.00) Setpoint = 5;
   // else Setpoint = 0;
  Input = Coldrad;
  double gap = abs(Setpoint-Input); //distance away from setpoint
  if (gap < 1.00)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  myPID.Compute();
   
   if (HRad >= 45.0 + hist)
     {
      myPID.SetMode(MANUAL);
      pwmWrite(INT1, 50);
      digitalWrite(INT2, LOW);                                                       
      }
   else {
      myPID.SetMode(AUTOMATIC);
      pwmWrite(INT1, Output);
      digitalWrite(INT2, LOW);
    }
   }
  else
  {
    pwmWrite(INT1, 0);
      digitalWrite(INT2, LOW);
  }

  
}

