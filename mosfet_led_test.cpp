// MOSFET LED Test - Blink On/Off

const int MOSFET_PIN = 6;  // D6 PWM pin

void setup() {
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, LOW);  // Start OFF
  
  Serial.begin(9600);
  Serial.println("=============================");
  Serial.println("MOSFET Test Starting");
  Serial.println("LED should blink every 2 sec");
  Serial.println("=============================");
}

void loop() {
  // Turn ON
  Serial.println("MOSFET: ON - LED should light up");
  digitalWrite(MOSFET_PIN, HIGH);
  delay(2000);
  
  // Turn OFF
  Serial.println("MOSFET: OFF - LED should turn off");
  digitalWrite(MOSFET_PIN, LOW);
  delay(2000);
}