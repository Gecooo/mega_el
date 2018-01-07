void Nagrev ()
{
//  if(currentDay < 1){
//    if(avtonagrev == true) {
//      if (hdc_t < 22.0 && hdc_h > 85.0) {
//        digitalWrite(nagrev_pin, HIGH); 
//      }
//      if (hdc_t > 22.5) {
//        digitalWrite(nagrev_pin, LOW);
//      }
//    }
//  }

  
  if (currentDay >= 0) {
    if(avtonagrev == true) {
      if (hdc_t < 11.5 && hdc_h > 75.0) {
        digitalWrite(nagrev_pin, HIGH); 
      }
      if (hdc_t > 13.0)       digitalWrite(nagrev_pin, LOW);
      //else if (hdc_h < 73.5)  digitalWrite(nagrev_pin, LOW);
    }
  }

  // Режим авттнагревва выключен для управления через mqtt вручную ///
if (avtonagrev == false && nagrevmqtt == true)  digitalWrite(nagrev_pin, HIGH);
if (avtonagrev == false && nagrevmqtt == false) digitalWrite(nagrev_pin, LOW);



  
//unsigned long now1 = millis();
//Setpoint1 = 13;
//
// myPID1.Compute();
//
// if (millis() - windowStartTime > WindowSize)
//  { //time to shift the Relay Window
//    windowStartTime = windowStartTime + WindowSize;
//    Input1 = hdc_t;
//  }
//  if (Output1 > (now1 - windowStartTime) && flag_work == true) digitalWrite(nagrev_pin, HIGH);
//  else digitalWrite(nagrev_pin, LOW);
}

