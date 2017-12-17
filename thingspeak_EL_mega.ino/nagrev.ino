void Nagrev ()
{
  if (hdc_t < 12.0 && hdc_h > 60.0) {
    digitalWrite(nagrev, HIGH); 
    }
  if (hdc_t > 13.0 ||  hdc_h < 56.0) {
    digitalWrite(nagrev, LOW);
  }
}

