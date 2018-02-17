
 if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
const size_t bufferSize = 2*JSON_ARRAY_SIZE(2) + JSON_ARRAY_SIZE(3) + JSON_ARRAY_SIZE(4) + JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(8);//DynamicJsonBuffer jsonBuffer(bufferSize);
StaticJsonBuffer<bufferSize> jsonBuffer;
JsonObject& root = jsonBuffer.createObject();
root["sensor"] = "gps";
root["time"] = 1518894649;

JsonArray& temp_radiator = root.createNestedArray("temp radiator");
temp_radiator.add(48.08);
temp_radiator.add(20.3);

JsonArray& in = root.createNestedArray("in");
in.add(25.37);
in.add(35.15);
in.add(18.11);
in.add(21.55);

JsonArray& out = root.createNestedArray("out");
out.add(17.01);
out.add(16.05);
out.add(25.01);
root["abshum"] = 10.12;

JsonObject& mqtt = root.createNestedObject("mqtt");
mqtt["vkl"] = 0;
mqtt["ir"] = 1;
mqtt["vozduh"] = 1;
mqtt["nagrev"] = 0;

JsonArray& day = root.createNestedArray("day");
day.add(1518894649);
day.add(25);

root.printTo(Serial);
  Serial.println();


    previousMillis = currentMillis;
  }
  
}


{
  "sensor": "gps",
  "time": 1518894649,
  "temp radiator": [
    48.08,
    20.30
  ],
"in": [
25.37,
35.15,
18.11,
21.55
],
"out": [
17.01,
16.05,
25.01
],
"abshum": 10.12,
"mqtt": {
"vkl":0,
"ir":1,
"vozduh":1,
"nagrev":0
},
"day":[
1518894649,
25
]
}
