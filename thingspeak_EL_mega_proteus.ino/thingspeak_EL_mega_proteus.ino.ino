#include <PID_v1.h>
#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PWM.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <ELClient.h>
#include <ELClientRest.h>
#define DHTPIN 7     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);

ELClient esp(&Serial, &Serial);

#define ONE_WIRE_BUS 4

ELClientRest rest(&esp);

boolean wifiConnected = false;

float HRad;
float Coldrad;
float h;
float t;
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

double Input, Output, Setpoint;
double aggKp=8000.0, aggKi=400.0, aggKd=100.0;       //настройки PID-регулятора
//double consKp=8000, consKi=153.0, consKd=10.3;
double consKp=8000, consKi=53.0, consKd=10.3;

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, REVERSE);       //PID-регулятор элемента пелетье

// arrays to hold device addresses
DeviceAddress sensor4 = {0x28, 0xFF, 0x55, 0x6E, 0x80, 0x16, 0x5, 0x91};
DeviceAddress sensor3 = {0x28, 0xA7, 0x6E, 0x04, 0x00, 0x00, 0x80, 0xF6};

#define INT1 2 // H-bridge leg 1 ->INT1
#define INT2 3 // H-bridge leg 2 ->INT2
LiquidCrystal_I2C lcd(0x27,20,4);     //подключаем lcd
int hist = 3;
int32_t frequency = 25000; //frequency (in Hz)
unsigned long previousMillis = 0;
unsigned long prevMillissens = 0;
char *api_key = "EPY2NM6967MVDEM5";
// expand buffer size to your needs
#define BUFLEN 276


///////Memory////////
extern void *__brkval;
extern int __bss_end;
///////////////////////////

void wifiCb(void *response) {
  ELClientResponse *res = (ELClientResponse*)response;
  if (res->argc() == 1) {
    uint8_t status;
    res->popArg(&status, 1);

    if(status == STATION_GOT_IP) {
      Serial.println("WIFI CONNECTED");
      wifiConnected = true;
    } else {
      Serial.print("WIFI NOT READY: ");
      Serial.println(status);
      wifiConnected = false;
    }
  }
}

void setup() {
  lcd.init();                  //включаем lcd
  lcd.backlight(); 
  delay(15);
  Serial.begin(9600); 
  Serial.println("");
  Serial.println("EL-Client starting! V6");
   esp.wifiCb.attach(wifiCb); // wifi status change callback, optional (delete if not desired)
 bool ok;
  do {
    ok = esp.Sync();      // sync up with esp-link, blocks for up to 2 seconds
    if (!ok) {
      Serial.print("\nEL-Client sync failed! err: ");
      Serial.println(ok);
    }
  } while(!ok);
  Serial.println("EL-Client synced!");

  // Wait for WiFi to be connected. 
Serial.println("esp.GetWifiStatus()");
  esp.GetWifiStatus();
  ELClientPacket *packet;
  Serial.println("Waiting for WiFi ");
  if ((packet=esp.WaitReturn()) != NULL) {
    Serial.print(".");
    Serial.println(packet->value);
  }
  Serial.println("");

  // Set up the REST client to talk to api.thingspeak.com, this doesn't connect to that server,
  // it just sets-up stuff on the esp-link side
  // int err = rest.begin("api.thingspeak.com");
  int err = rest.begin("184.106.153.149");
  if (err != 0) {
    Serial.print("REST begin failed: ");
    Serial.println(err);
    while(1) ;
  }
  Serial.println("EL-REST ready");
   
   //InitTimersSafe();
   Timer3_Initialize();
  SetPinFrequencySafe(INT1, frequency);
  SetPinFrequencySafe(INT2, frequency);
  myPID.SetOutputLimits(30,250);    //лимит PID  
  dht.begin();
  sensors.begin();
  sensors.setResolution(sensor4, 10);
  sensors.setResolution(sensor3, 10);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Test Kamera"); //отсылка в сериал для Espeasy
  lcd.setCursor(7, 1);
  lcd.print("25 kHz");
    delay(500);                // ждем 0.5 секунды
    lcd.setCursor(0, 2);
     lcd.print("Loading");
     delay(500); 
     
  lcd.print(".");
    delay(500);                // ждем 0.5 секунды
  lcd.print(".");
    delay(2000);               // ждем 1 секунду
  lcd.clear();
Setpoint = 3.00;

}



void loop() {
 
 // PID_termostat ();
unsigned long sensMillis = millis();
  if (sensMillis - prevMillissens >= 1000) {
    // save the last time you blinked the LED
    prevMillissens = sensMillis;

    sensors.requestTemperatures();
 // Serial.println("DONE");
  HRad = sensors.getTempC(sensor4);
  Coldrad = sensors.getTempC(sensor3);
//Serial.print("Temp: ");
  // print the device information
  //Serial.println(HRad);

   h = dht.readHumidity();
   t = dht.readTemperature();

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
   else if (HRad <45.0) {
      myPID.SetMode(AUTOMATIC);
      pwmWrite(INT1, Output);
      digitalWrite(INT2, LOW);
  }
  lcd.setCursor(0, 0);
  lcd.print("HOT RAD:       "); //отсылка в сериал для Espeasy
  lcd.setCursor(10, 0);
  lcd.print(HRad);
  lcd.setCursor(0, 1);
  lcd.print("COLD Rad:        ");
  lcd.setCursor(10, 1);
  lcd.print(Coldrad);
  lcd.setCursor(0, 2);
  lcd.print("TempIn:        ");
  lcd.setCursor(10, 2);
  lcd.print(t);
  lcd.setCursor(0, 3);
  lcd.print("Hum:           ");
  lcd.setCursor(10, 3);
  lcd.print(h);

  }
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  //Serial.print("Requesting temperatures...");
  esp.Process();

  
 // if we're connected make an REST request
  if(wifiConnected) {
    unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 5000) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

   
      String memoryFreeString = String(memoryFree());
      const char *memoryFreeChar = memoryFreeString.c_str(); 

      String ColdradString = String(Coldrad);
      const char *ColdradChar = ColdradString.c_str(); 

      String hString = String(h);
      const char *hChar = hString.c_str();
      
      String tString = String(t);
      const char *tChar = tString.c_str(); 

      String OutputString = String(Output);
      const char *OutputChar = OutputString.c_str();

      String HRadString = String(HRad);
      const char *HRadChar = HRadString.c_str();
//
       
//        
    // Reserve a buffer for sending the data
    char path_data[BUFLEN];
    // Copy the path and API key into the buffer
    sprintf(path_data, "%s", "/update?api_key=");
    sprintf(path_data + strlen(path_data), "%s", api_key);
    
    // Copy the field number and value into the buffer
    // If you have more than one field to update,
    // repeat and change field1 to field2, field3, ...
    sprintf(path_data + strlen(path_data), "%s", "&field1=");
    sprintf(path_data + strlen(path_data), "%s", memoryFreeChar);

     sprintf(path_data + strlen(path_data), "%s", "&field2=");
     sprintf(path_data + strlen(path_data), "%s", ColdradChar);

    sprintf(path_data + strlen(path_data), "%s", "&field3=");
    sprintf(path_data + strlen(path_data), "%s", tChar);

    sprintf(path_data + strlen(path_data), "%s", "&field4=");
    sprintf(path_data + strlen(path_data), "%s", hChar);

    sprintf(path_data + strlen(path_data), "%s", "&field5=");
    sprintf(path_data + strlen(path_data), "%s", OutputChar);

    sprintf(path_data + strlen(path_data), "%s", "&field6=");
    sprintf(path_data + strlen(path_data), "%s", HRadChar);
    
    // Send PUT request to thingspeak.com
    rest.post(path_data,"");  

    // Reserve a buffer for the response from Thingspeak
    char response[BUFLEN];
    // Clear the buffer
    memset(response, 0, BUFLEN);
    // Wait for response from Thingspeak
    uint16_t code = rest.waitResponse(response, BUFLEN-1);
    // Check the response from Thingspeak
    if(code == HTTP_STATUS_OK){
      Serial.println("Thingspeak: POST successful:");
      Serial.print("Response: ");
      Serial.println(response);
    } else {
      Serial.print("Thingspeak: POST failed with error ");
      Serial.println(code);
      Serial.print("Response: ");
      Serial.println(response);
    }
    // Send next data in 20 seconds
    //delay(3000);
  }
  }
  

}
int memoryFree() {
  int freeValue;
  if ((int)__brkval == 0)
    freeValue = ((int)&freeValue) - ((int)&__bss_end);
  else
    freeValue = ((int)&freeValue) - ((int)__brkval);
  return freeValue;
}
