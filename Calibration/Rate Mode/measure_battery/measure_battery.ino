float ReadVoltage() {
  float voltage = (float)analogRead(15) * 4.88 * 3.3 / 1023;
  return voltage;
}

void setup() {
  pinMode(15, INPUT);
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Serial.begin(57600);
}

void loop() {
  Serial.println("Voltage: " + (String)readVoltage() + " V");
  Serial.println("Signal: " + (String)analogRead(15));
  Serial.println("");
  delay(50);
}