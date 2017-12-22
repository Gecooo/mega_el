void Sens()
{
 //unsigned long sensMillis = millis();
 // unsigned long hdcMillis = millis();
    
  if (millis() - prevMillissens >= 1000) {
    // save the last time you blinked the LED
  
  tempout = clock.getTemperature();
  sensors.requestTemperatures();
  HRad = sensors.getTempC(sensor4);
  Coldrad = sensors.getTempC(sensor3);
  h = dht.readHumidity();
  t = dht.readTemperature();
  prevMillissens = millis();
  }

  if (millis() - prevMillishdc >= 3000) {
    hdc_t = hdc1080.readTemperature();
    hdc_h = hdc1080.readHumidity();
    prevMillishdc = millis();
  }
  if (millis() - abshummillis > 20000) {
    Abshum = (6.112*pow(2.718281828,(17.67*hdc_t)/(hdc_t+243.5))*hdc_h*2.1674)/(273.15+hdc_t);
    abshummillis = millis();
  }
}

