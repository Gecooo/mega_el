void Sens()
{
  unsigned long sensMillis = millis();
  unsigned long hdcMillis = millis();
  
  if (sensMillis - prevMillissens >= 1000) {
    // save the last time you blinked the LED
  prevMillissens = sensMillis;
  tempout = clock.getTemperature();
  sensors.requestTemperatures();
  HRad = sensors.getTempC(sensor4);
  Coldrad = sensors.getTempC(sensor3);
  h = dht.readHumidity();
  t = dht.readTemperature();
  }

  if (hdcMillis - prevMillishdc >= 3000) {
    prevMillishdc = hdcMillis;
    hdc_t = hdc1080.readTemperature();
    hdc_h = hdc1080.readHumidity();
  }
}

