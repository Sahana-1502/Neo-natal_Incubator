#include <OneWire.h>
#include <DallasTemperature.h>

// Pin definitions  
#define TEMP_SENSOR_PIN 2
#define HEATER_PIN 6

// Temperature control parameters
const float TARGET_TEMP = 35.0;  // Target: 35°C
const float TOLERANCE = 0.5;     // ±0.5°C

// Setup sensor
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);

// Variables
float currentTemp = 0;
bool heaterState = false;
unsigned long lastPrintTime = 0;

void setup() {
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  
  sensors.begin();
  Serial.begin(9600);
  
  Serial.println("========================================");
  Serial.println("AUTOMATIC TEMPERATURE CONTROL");
  Serial.print("Target Temperature: ");
  Serial.print(TARGET_TEMP);
  Serial.println(" °C");
  Serial.print("Tolerance: ±");
  Serial.print(TOLERANCE);
  Serial.println(" °C");
  Serial.println("========================================");
  Serial.println("\nTime\tTemp\tTarget\tHeater\tStatus");
  Serial.println("----\t----\t------\t------\t------");
}

void loop() {
  // Read temperature
  sensors.requestTemperatures();
  currentTemp = sensors.getTempCByIndex(0);
  
  // Control logic with hysteresis (dead-band)
  if (currentTemp < TARGET_TEMP - TOLERANCE) {
    // Too cold - turn heater ON
    heaterState = true;
    digitalWrite(HEATER_PIN, HIGH);
  }
  else if (currentTemp > TARGET_TEMP + TOLERANCE) {
    // Too hot - turn heater OFF
    heaterState = false;
    digitalWrite(HEATER_PIN, LOW);
  }
  // If within tolerance, maintain current state (hysteresis)
  
  // Print status every 2 seconds
  if (millis() - lastPrintTime >= 2000) {
    lastPrintTime = millis();
    
    // Format output
    Serial.print(millis() / 1000);  // Time in seconds
    Serial.print("\t");
    Serial.print(currentTemp, 2);
    Serial.print("\t");
    Serial.print(TARGET_TEMP, 1);
    Serial.print("\t");
    Serial.print(heaterState ? "ON " : "OFF");
    Serial.print("\t");
    
    // Status message
    float error = currentTemp - TARGET_TEMP;
    if (abs(error) <= TOLERANCE) {
      Serial.print("✓ OK (±");
      Serial.print(abs(error), 2);
      Serial.println("°C)");
    } else if (error < 0) {
      Serial.print("↑ Heating (");
      Serial.print(error, 2);
      Serial.println("°C)");
    } else {
      Serial.print("↓ Cooling (");
      Serial.print(error, 2);
      Serial.println("°C)");
    }
  }
  
  delay(100);  // Small delay to prevent sensor overload
}