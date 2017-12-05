void Sens()
{
  unsigned long sensMillis = millis();
  if (sensMillis - prevMillissens >= 1000) {
    // save the last time you blinked the LED
  prevMillissens = sensMillis;
  sensors.requestTemperatures();
  HRad = sensors.getTempC(sensor4);
  Coldrad = sensors.getTempC(sensor3);
  h = dht.readHumidity();
  t = dht.readTemperature();
  }
}

