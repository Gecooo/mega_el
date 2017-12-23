void PID_termostat()
{
  if (flag_work) // если нажата кнопка включить запускаем работу пелеть и ПИД регулятора
  {
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
    DateTime now = dt.now();
    if (now.minute() >= 35 && now.minute() < 40 ) digitalWrite(vozduh_pin, HIGH); // включить воздух
    else digitalWrite(vozduh_pin, LOW); // воздух выкл
  }
  else
  {
    pwmWrite(INT1, 0);
      digitalWrite(INT2, LOW);
  }

  
}

