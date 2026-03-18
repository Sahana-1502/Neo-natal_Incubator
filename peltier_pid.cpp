#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <PID_v1.h>

// Pin definitions
#define PELTIER_PWM_PIN 6 // Must be a PWM-capable pin (has a ~ next to the number)

// Sensor setup (Using AHT20 for highly accurate temperature/humidity)
Adafruit_AHTX0 aht;

// PID Variables
double Setpoint, Input, Output;

// PID Tuning Parameters
// These are starting values. 
double Kp = 10.0;  // Proportional: How hard to push based on current error
double Ki = 0.5;   // Integral: Corrects steady-state error over time
double Kd = 2.0;   // Derivative: Dampens the response to prevent overshoot

// Initialize PID controller
// DIRECT means as Input goes down, Output goes up (Heating mode)
PID peltierPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(9600);
  
  // Setup MOSFET pin
  pinMode(PELTIER_PWM_PIN, OUTPUT);
  analogWrite(PELTIER_PWM_PIN, 0); // Ensure Peltier is OFF initially

  // Initialize AHT20 Sensor via I2C
  if (!aht.begin()) {
    Serial.println("Could not find AHT20 sensor! Check I2C wiring (SDA/SCL).");
    while (1) delay(10); // Halt if sensor isn't found
  }

  Serial.println("========================================");
  Serial.println("PID TEMPERATURE CONTROL");
  
  // Set the target temperature for the incubator
  Setpoint = 35.0; 
  
  Serial.print("Target Temperature: ");
  Serial.print(Setpoint);
  Serial.println(" °C");
  Serial.println("========================================");
  Serial.println("\nTime(s)\tTemp(C)\tTarget\tPWM(0-255)");
  Serial.println("-------\t-------\t------\t----------");

  // Setup PID Controller
  peltierPID.SetMode(AUTOMATIC);
  peltierPID.SetOutputLimits(0, 255); // Arduino 8-bit PWM range
}

void loop() {
  // Read temperature from AHT20
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  
  Input = temp.temperature;

  // Compute PID output
  // This automatically calculates the necessary PWM value based on Kp, Ki, Kd
  peltierPID.Compute();

  // Write the calculated PWM value to the MOSFET
  // 0 = Peltier completely OFF (cooling down via ambient air)
  // 255 = Peltier completely ON (heating up at max power)
  analogWrite(PELTIER_PWM_PIN, Output);

  // Print status every 2 seconds to the Serial Monitor
  if (millis() - lastPrintTime >= 2000) {
    lastPrintTime = millis();
    
    Serial.print(millis() / 1000);
    Serial.print("\t");
    Serial.print(Input, 2);
    Serial.print("\t");
    Serial.print(Setpoint, 1);
    Serial.print("\t");
    Serial.println(Output);
  }
  
  delay(100); // Small delay for system stability
}