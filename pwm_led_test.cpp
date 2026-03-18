// PWM Dimming Test

const int MOSFET_PIN = 6;

void setup() {
  pinMode(MOSFET_PIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("PWM Brightness Test");
}

void loop() {
  // Fade from dim to bright
  Serial.println("Fading UP (0% to 100%)");
  for (int brightness = 0; brightness <= 255; brightness += 5) {
    analogWrite(MOSFET_PIN, brightness);
    Serial.print("Power: ");
    Serial.print(brightness * 100 / 255);
    Serial.println("%");
    delay(100);
  }
  
  delay(1000);
  
  // Fade from bright to dim
  Serial.println("Fading DOWN (100% to 0%)");
  for (int brightness = 255; brightness >= 0; brightness -= 5) {
    analogWrite(MOSFET_PIN, brightness);
    Serial.print("Power: ");
    Serial.print(brightness * 100 / 255);
    Serial.println("%");
    delay(100);
  }
  
  delay(1000);
}