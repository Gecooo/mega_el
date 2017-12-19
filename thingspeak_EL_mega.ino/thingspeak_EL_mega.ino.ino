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
#include "DS3231.h"
#include "ClosedCube_HDC1080.h"

ClosedCube_HDC1080 hdc1080;

/// Часы ////
DS3231 clock;
RTClib dt;

unsigned int currentTime_day;           //текущий день в юникс-формате
unsigned int currentDay;                // прошло дней с момента запуска
float temp3231;              //датчик температуры встроенный в часы
byte tMSB, tLSB;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
/////////////////////

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
bool nagrevmqtt = false;
boolean wifiConnected = false;
const unsigned long SendThingspeak = 15000;
float HRad;
float Coldrad;
float h;
float t;
float hdc_t;
float hdc_h;
float Abshum;           //абсолютная влажность в г/м3
float tempout;            // температруа снаружи, датчик в часах
const int ledPin =  LED_BUILTIN;// the number of the LED pin
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

double Input, Output, Setpoint=0;
double aggKp=160.0, aggKi=10.2, aggKd=8.0;       //настройки PID-регулятора
//double consKp=8000, consKi=153.0, consKd=10.3;
double consKp=100, consKi=4.0, consKd=5.5;

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
unsigned long prevMillishdc = 0;

volatile bool flag_work = 0; // флаг кнопки включения процесса

char *api_key = "EPY2NM6967MVDEM5";
// expand buffer size to your needs
#define BUFLEN 266

#define IRPIN 22     // what pin we're connected to
#define VKL 3 // 3 с подтяшкой к +5, кнопка включения процесса
#define nagrev 26   //нагреватель
#define vozduh 28   //воздух
#define buzzer_pin 9                   //пищалка
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
  mqtt.subscribe("/esp-link/vozduh");
  mqtt.subscribe("/esp-link/led");
  mqtt.subscribe("/esp-link/ir");
  mqtt.subscribe("/esp-link/vkl");
  mqtt.subscribe("/esp-link/nagrev");
  
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

  if (topic == "/esp-link/vozduh") {
    if (data == "1") {
      digitalWrite(vozduh, HIGH); // включить нагреватель
      //nagrevmqtt = true;
    }
    else if (data == "0") {
      digitalWrite(vozduh, LOW); // нагрев выкл
     // nagrevmqtt = false;
    }
  }

  if (topic == "/esp-link/ir") {
    if (data == "1") {
      digitalWrite(IRPIN, HIGH);
      lcd.setCursor(15, 0);
      lcd.print("On ");
    }
    else if (data == "0") {
      digitalWrite(IRPIN, LOW);
      lcd.setCursor(15, 0);
      lcd.print("Off");
    }
  }


//  if (topic == "/esp-link/setpoint")
//  {
//    
//   // (atof(myStr));
//Setpoint = data.toDouble();
// EEPROM_write(10, Setpoint); 
//  }

  if (topic == "/esp-link/vkl") {
    if (data == "1") {
      flag_work = 1;
      EEPROM_write(11, flag_work);      // записываем в память статус флага работы, чтобы при перзагрузки контроллера стартовал с этого же статуса
      EEPROM_write(1, currentTime_day); //записываем день старта программы
    }
    else if (data == "0") {
      flag_work = 0;
      EEPROM_write(11, flag_work);  // записываем в память статус флага работы, чтобы при перзагрузки контроллера стартовал с этого же статуса
      EEPROM_write(1, 0);           //когда выключили процесс, стираем дату начала процесса
    }
  }
  
}

void mqttPublished(void* response) {
  Serial.println("MQTT published");
}

void setup() {
  
  Serial.begin(9600);
  Wire.begin();
  hdc1080.begin(0x40); 
  hdc1080.setResolution(1, 01);
  pinMode (IRPIN, OUTPUT); //устанавливаем пин 5 как выход
  digitalWrite(IRPIN,LOW);
  //pinMode (VKL, INPUT);
  pinMode(nagrev, OUTPUT); 
  digitalWrite(nagrev, LOW); // нагреватель выключен
  pinMode(vozduh, OUTPUT); 
  digitalWrite(vozduh, LOW); // нагреватель выключен
  attachInterrupt(1, myInterrupt, FALLING); //подключить прерывания на первый таймер пин 3
  Serial.println("");
  Serial.println("EL-Client starting!");
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

//EEPROM_read(10, Setpoint);
EEPROM_read(11, flag_work);
if(flag_work) tone(buzzer_pin, 2000, 50);
}

static int count;
static uint32_t last;

void loop() {
 int val ;
unsigned int startDayUnixtime; // день старта в памяти

///////////////////
DateTime now = dt.now();
  currentTime_day = (now.unixtime() / 86400L);  // текущий день
  EEPROM_read(1, startDayUnixtime);  // здесь в каждом цикле идет запрос о дне работы 
  currentDay = (currentTime_day - startDayUnixtime);  //но нет смысла спрашивать каждый раз, можно настроить запрос по времени
// if (currentDay < 3) {   // если день работы менее 3 то влажность 85%
//// if (currentHour <= 72) {
//  //Setpoint = map(currentHour * 10, 0, 720, 170, 70);
//  Setpoint3 = 85.00;
// // Setpoint = Setpoint / 10;
//  Setpoint = 16.00;
//  }
/////////////////////////
  
  Sens();
  LCD();
  PID_termostat ();
  Nagrev ();
  
  esp.Process();

  
 // if we're connected make an REST request
  if(wifiConnected) {
    unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= SendThingspeak) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

   
      String tempoutString = String(tempout);
      const char *tempoutChar = tempoutString.c_str(); 

      String ColdradString = String(Coldrad);
      const char *ColdradChar = ColdradString.c_str(); 

      String hString = String(h);
      const char *hChar = hString.c_str();
      
      String tString = String(t);
      const char *tChar = tString.c_str(); 

//      String OutputString = String(Output);
//      const char *OutputChar = OutputString.c_str();

      String HRadString = String(HRad);
      const char *HRadChar = HRadString.c_str();
      
//      String SetpointString = String(Setpoint);
//      const char *SetpointChar = SetpointString.c_str();

      String hdc_tString = String(hdc_t);
      const char *hdc_tChar = hdc_tString.c_str();

      String hdc_hString = String(hdc_h);
      const char *hdc_hChar = hdc_hString.c_str();
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
    sprintf(path_data + strlen(path_data), "%s", tempoutChar);

//    sprintf(path_data + strlen(path_data), "%s", "&field1=");
//    sprintf(path_data + strlen(path_data), "%s", SetpointChar);

     sprintf(path_data + strlen(path_data), "%s", "&field2=");
     sprintf(path_data + strlen(path_data), "%s", ColdradChar);

    sprintf(path_data + strlen(path_data), "%s", "&field3=");
    sprintf(path_data + strlen(path_data), "%s", tChar);

    sprintf(path_data + strlen(path_data), "%s", "&field4=");
    sprintf(path_data + strlen(path_data), "%s", hChar);

//    sprintf(path_data + strlen(path_data), "%s", "&field5=");
//    sprintf(path_data + strlen(path_data), "%s", OutputChar);

    sprintf(path_data + strlen(path_data), "%s", "&field6=");
    sprintf(path_data + strlen(path_data), "%s", HRadChar);

    sprintf(path_data + strlen(path_data), "%s", "&field7=");
    sprintf(path_data + strlen(path_data), "%s", hdc_tChar);

    sprintf(path_data + strlen(path_data), "%s", "&field8=");
    sprintf(path_data + strlen(path_data), "%s", hdc_hChar);

    
    
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
   }

    if (connected && (millis()-last) > 4000) {
    Serial.println("publishing");
    //char buf[12];
    char buf1[8];
    val = digitalRead(ledPin);          //Конвертирование целочисленных данных в строку, преобразование int
    itoa(val, buf1, 2);
    mqtt.publish("/esp-link/led", buf1);

    val = digitalRead(nagrev);          //Конвертирование целочисленных данных в строку, преобразование int
    itoa(val, buf1, 2);
    mqtt.publish("/esp-link/nagrev", buf1);

    val = digitalRead(vozduh);
    itoa(val, buf1, 2);
    mqtt.publish("/esp-link/vozduh", buf1);

    val = flag_work;
    itoa(val, buf1, 2);
    mqtt.publish("/esp-link/vkl", buf1);

//    uint32_t t = cmd.GetTime();
//    Serial.print("Time: "); Serial.println(t);
     last = millis();
      }
  }
  
wdt_reset();
}

///// показывает свободную память ///////
int memoryFree() {
  int freeValue;
  if ((int)__brkval == 0)
    freeValue = ((int)&freeValue) - ((int)&__bss_end);
  else
    freeValue = ((int)&freeValue) - ((int)__brkval);
  return freeValue;
}

void myInterrupt() {
  flag_work = !flag_work;
  EEPROM_write(11, flag_work);    //записываем в память статус работы программы, чтобы при перезагрузки контроллера стартовать с этого же статуса
  if (flag_work) { EEPROM_write(1, currentTime_day);} //записываем в память день старта программы
  if (!flag_work){ EEPROM_write(1, 0);}                            //стираем день старта программы
}

//пищалка////////////////////////////////////////////////////////////////////////
void buzzer(int duration) {
  tone(buzzer_pin, 2000, duration);
}

