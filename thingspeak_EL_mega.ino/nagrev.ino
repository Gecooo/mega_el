void Nagrev ()
{
//  if (hdc_t < 12.5 && hdc_h > 74.0) {
//    digitalWrite(nagrev_pin, HIGH); 
//    }
//  if (hdc_t > 14.0 ||  hdc_h < 73.0 ) {
//    digitalWrite(nagrev_pin, LOW);
//  }
unsigned long now1 = millis();
Setpoint1 = 13;
Input1 = hdc_t;
 myPID1.Compute();

 if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (Output1 < (now1 - windowStartTime) && hdc_t < 12.5 && hdc_h > 60.0 && flag_work == true) digitalWrite(nagrev_pin, HIGH);
  else digitalWrite(nagrev_pin, LOW);
}

