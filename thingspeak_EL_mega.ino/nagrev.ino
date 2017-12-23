void Nagrev ()
{
  if (hdc_t < 12.5 && hdc_h > 74.0) {
    digitalWrite(nagrev_pin, HIGH); 
    }
  if (hdc_t > 14.0 ||  hdc_h < 73.0 ) {
    digitalWrite(nagrev_pin, LOW);
  }
 
}

