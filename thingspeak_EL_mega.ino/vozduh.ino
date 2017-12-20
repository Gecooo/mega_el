void vozduh() {
  DateTime now = dt.now();
  if (now.minute() >= 35 && now.minute() < 40 && flag_work == 1 || vozduhmqtt ==true ) {
    digitalWrite(vozduh_pin, HIGH); // включить воздух
    
  }
  else digitalWrite(vozduh_pin, LOW); // воздух выкл
}

