#include <EEPROM2.h>
#include <avr/wdt.h>
#include <ELClientCmd.h>
#include <ELClientMqtt.h>
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
// Initialize CMD client (for GetTime)
ELClientCmd cmd(&esp);

// Initialize the MQTT client
ELClientMqtt mqtt(&esp);
bool connected;
boolean wifiConnected = false;
const unsigned long SendThingspeak = 15000;
float HRad;
float Coldrad;
float h;
float t;
const int ledPin =  LED_BUILTIN;// the number of the LED pin
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

double Input, Output, Setpoint=3;
double aggKp=250.0, aggKi=25.2, aggKd=35.0;       //настройки PID-регулятора
//double consKp=8000, consKi=153.0, consKd=10.3;
double consKp=160, consKi=4.2, consKd=15.0;

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, REVERSE);       //PID-регулятор элемента пелетье

// arrays to hold device addresses
DeviceAddress sensor4 = {0x28, 0xFF, 0x55, 0x6E, 0x80, 0x16, 0x5, 0x91};
DeviceAddress sensor3 = {0x28, 0xA7, 0x6E, 0x04, 0x00, 0x00, 0x80, 0xF6};

#define INT1 11 // H-bridge leg 1 ->INT1
#define INT2 12 // H-bridge leg 2 ->INT2
LiquidCrystal_I2C lcd(0x27,20,4);     //подключаем lcd
int hist = 3;
int32_t frequency = 24900; //frequency (in Hz)

unsigned long previousMillis = 0;
unsigned long prevMillissens = 0;
unsigned long prevMillisLCD = 0;

char *api_key = "EPY2NM6967MVDEM5";
// expand buffer size to your needs
#define BUFLEN 266

#define IRPIN 22     // what pin we're connected to

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

// Callback when MQTT is connected
void mqttConnected(void* response) {
  Serial.println("MQTT connected!");
  mqtt.subscribe("/esp-link/setpoint");
  mqtt.subscribe("/hello/world/#");
  mqtt.subscribe("/esp-link/led");
  mqtt.subscribe("/esp-link/ir");
  //mqtt.subscribe("/esp-link/2", 1);
  //mqtt.publish("/esp-link/0", "test1");
  connected = true;
}

// Callback when MQTT is disconnected
void mqttDisconnected(void* response) {
  Serial.println("MQTT disconnected");
  connected = false;
}

// Callback when an MQTT message arrives for one of our subscriptions
void mqttData(void* response) {
  ELClientResponse *res = (ELClientResponse *)response;

  Serial.print("Received: topic=");
  String topic = res->popString();
  Serial.println(topic);

  Serial.print("data=");
  String data = res->popString();
  Serial.println(data);

   //String strTopic = String(topic);
  //String strPayload = String((char*)payload);
  //Serial.println(strPayload);
  
  if (topic == "/esp-link/led") {
    if (data == "1") {
      digitalWrite(ledPin, HIGH);
    }
    else if (data == "0") {
      digitalWrite(ledPin, LOW);
    }
  }

  if (topic == "/esp-link/ir") {
    if (data == "1") {
      digitalWrite(IRPIN, HIGH);
    }
    else if (data == "0") {
      digitalWrite(IRPIN, LOW);
    }
  }

  if (topic == "/esp-link/setpoint")
  {
    
   // (atof(myStr));
Setpoint = data.toDouble();
 EEPROM_write(10, Setpoint); 
  }
}

void mqttPublished(void* response) {
  Serial.println("MQTT published");
}

void setup() {
  
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
  wdt_enable (WDTO_8S); //взводим сторожевой таймер на 8 секунд.
  mqtt.connectedCb.attach(mqttConnected);
  mqtt.disconnectedCb.attach(mqttDisconnected);
  mqtt.publishedCb.attach(mqttPublished);
  mqtt.dataCb.attach(mqttData);
  mqtt.setup();
  Serial.println("EL-MQTT ready");

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
   
  InitTimersSafe();
  SetPinFrequencySafe(INT1, frequency);
  SetPinFrequencySafe(INT2, frequency);
  myPID.SetOutputLimits(80,255);    //лимит PID  
  dht.begin();
  sensors.begin();
  sensors.setResolution(sensor4, 10);
  sensors.setResolution(sensor3, 10);
  lcd.init();                  //включаем lcd
  lcd.backlight(); 
  delay(15);
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

EEPROM_read(10, Setpoint);
}

static int count;
static uint32_t last;

void loop() {
 int val ;
 
  Sens();
  LCD();
  PID_termostat ();
  
  esp.Process();

  
 // if we're connected make an REST request
  if(wifiConnected) {
    unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= SendThingspeak) {
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
    
  if (connected) {
    Serial.println("publishing");
    char buf[12];
    char buf1[8];
    val = digitalRead(ledPin);
   itoa(count++, buf, 10);        //Конвертирование целочисленных данных в строку, преобразование int
   itoa(val, buf1, 2);
   
    
   mqtt.publish("/esp-link/led", buf1);
    
   dtostrf(Setpoint, 2, 2, buf1);   //Конвертирование из float(double) в строку
   mqtt.publish("/esp-link/setpoint", buf1);

//   itoa(count+99, buf, 10);
//   mqtt.publish("/hello/world/arduino", buf);

//    uint32_t t = cmd.GetTime();
//    Serial.print("Time: "); Serial.println(t);
      }
    }
  
  }
  
wdt_reset();
}

int memoryFree() {
  int freeValue;
  if ((int)__brkval == 0)
    freeValue = ((int)&freeValue) - ((int)&__bss_end);
  else
    freeValue = ((int)&freeValue) - ((int)__brkval);
  return freeValue;
}
