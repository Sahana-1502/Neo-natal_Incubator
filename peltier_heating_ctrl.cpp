// Peltier Heating Control

const int PELTIER_PIN = 5;  // Using D5 instead of D6 (keep separate)

void setup() {
  pinMode(PELTIER_PIN, OUTPUT);
  digitalWrite(PELTIER_PIN, LOW);
  
  Serial.begin(9600);
  Serial.println("=============================");
  Serial.println("PELTIER HEATING TEST");
  Serial.println("Hot side should heat, cold side should stay cool");
  Serial.println("=============================");
  delay(3000);
}

void loop() {
  // Heat for 30 seconds
  Serial.println("PELTIER HEATING - Hot side warming up");
  digitalWrite(PELTIER_PIN, HIGH);
  
  for (int i = 30; i > 0; i--) {
    Serial.print("ON: ");
    Serial.print(i);
    Serial.println(" sec");
    delay(1000);
  }
  
  // Off for 30 seconds
  Serial.println("PELTIER OFF - Cooling down");
  digitalWrite(PELTIER_PIN, LOW);
  
  for (int i = 30; i > 0; i--) {
    Serial.print("OFF: ");
    Serial.print(i);
    Serial.println(" sec");
    delay(1000);
  }
}