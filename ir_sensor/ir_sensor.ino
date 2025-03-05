const int irPin = A0;
float distance;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(irPin);
  float voltage = sensorValue * (5.0 / 1023.0);
  distance = 27.61 / pow(voltage, 1.17);
  Serial.println(distance);
  delay(100);
}
