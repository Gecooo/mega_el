void Nagrev ()
{
  if (hdc_t < 11.5 && hdc_h > 60.0) {
    digitalWrite(nagrev_pin, HIGH); 
    }
  if (hdc_t > 13.0 ||  hdc_h < 56.0 ) {
    digitalWrite(nagrev_pin, LOW);
  }
 
}

