void Nagrev ()
{
  if (hdc_t < 12.0 && hdc_h > 60.0 || nagrevmqtt == true) {
    digitalWrite(nagrev_pin, HIGH); 
    }
  if (hdc_t > 13.0 ||  hdc_h < 56.0 || nagrevmqtt == false) {
    digitalWrite(nagrev_pin, LOW);
  }
 
}

