// Includes the SoftwareWire library to emulate I2C on standard digital pins
#include <SoftwareWire.h>
// Includes the standard Arduino PID library for closed-loop control
#include <PID_v1.h>

// ==========================================
// 1. PIN DEFINITIONS & VIRTUAL BUSES
// ==========================================
#define PELTIER_PWM_PIN 6
#define TRANSDUCER_PIN 7

SoftwareWire bus1(2, 3);
SoftwareWire bus2(4, 5);
SoftwareWire bus3(8, 9);
SoftwareWire bus4(10, 11);
SoftwareWire bus5(12, 13);

SoftwareWire* sensorBuses[5] = {&bus1, &bus2, &bus3, &bus4, &bus5};

// ==========================================
// 2. CONTROL VARIABLES (TEMP & HUMIDITY)
// ==========================================
// Temperature (PID Control)
double Setpoint = 27.0; 
double Input, Output;
double Kp = 50.0, Ki = 1.0, Kd = 5.0;   
PID peltierPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Humidity (Bang-Bang Control)
const float TARGET_HUMIDITY = 60.0;  
const float TOLERANCE = 2.0;         
bool transducerState = false;
bool dehumidifierState = false;

// System Timers
unsigned long lastPrintTime = 0;
unsigned long lastHighlightTime = 0;

// ==========================================
// 3. DUAL-DATA SENSOR HELPER FUNCTION
// ==========================================
// Uses pass-by-reference (&) to return two values at once
bool readAHT20(SoftwareWire* bus, float &temperature, float &humidity) {
  bus->beginTransmission(0x38); 
  bus->write(0xAC);             
  bus->write(0x33);             
  bus->write(0x00);
  
  if (bus->endTransmission() != 0) return false; 

  delay(80); 

  bus->requestFrom(0x38, 6);
  
  if (bus->available() == 6) {
    bus->read(); // Skip status byte
    
    uint8_t b1 = bus->read(); 
    uint8_t b2 = bus->read(); 
    uint8_t b3 = bus->read(); 
    uint8_t b4 = bus->read(); 
    uint8_t b5 = bus->read(); 

    // Extract Humidity
    uint32_t rawHum = ((uint32_t)b1 << 12) | ((uint32_t)b2 << 4) | (b3 >> 4);
    humidity = ((float)rawHum / 1048576.0) * 100.0;

    // Extract Temperature
    uint32_t rawTemp = ((uint32_t)(b3 & 0x0F) << 16) | ((uint32_t)b4 << 8) | b5;
    temperature = ((float)rawTemp / 1048576.0) * 200.0 - 50.0;
    
    return true; 
  }
  return false; 
}

// ==========================================
// 4. SETUP ROUTINE 
// ==========================================
void setup() {
  Serial.begin(9600); 
  
  pinMode(PELTIER_PWM_PIN, OUTPUT);
  pinMode(TRANSDUCER_PIN, OUTPUT);
  analogWrite(PELTIER_PWM_PIN, 0); 
  digitalWrite(TRANSDUCER_PIN, LOW);

  for (int i = 0; i < 5; i++) {
    sensorBuses[i]->begin();
  }
  delay(100); 

  peltierPID.SetMode(AUTOMATIC); 
  peltierPID.SetOutputLimits(0, 255); 
  peltierPID.SetSampleTime(1000);

  Serial.println("=================================================================================================================");
  Serial.println("MASTER MULTI-SENSOR SYSTEM | PID TEMP & BANG-BANG HUMIDITY");
  Serial.println("=================================================================================================================");
  Serial.println("Time\tT1\tT2\tT3\tT4\tT5\tT_Avg\tPWM\t|\tH1\tH2\tH3\tH4\tH5\tH_Avg\tHumifier\tDehumidifier");
}

// ==========================================
// 5. MAIN LOOP 
// ==========================================
void loop() {
  float totalTemp = 0, totalHum = 0;       
  int activeSensors = 0;     
  float tReadings[5] = {0}; 
  float hReadings[5] = {0};
  float avgHum = 0;

  // --- A. DATA ACQUISITION PHASE ---
  for (int i = 0; i < 5; i++) {
    float temp = 0, hum = 0;
    if (readAHT20(sensorBuses[i], temp, hum)) {
      tReadings[i] = temp;
      hReadings[i] = hum;
      totalTemp += temp; 
      totalHum += hum;
      activeSensors++;                 
    } else {
      tReadings[i] = -999.0;
      hReadings[i] = -999.0;
    }
  }

  // --- B. CONTROL PHASE ---
  if (activeSensors > 0) {
    // 1. PID Temperature Control
    Input = totalTemp / activeSensors; 
    peltierPID.Compute(); 
    analogWrite(PELTIER_PWM_PIN, Output); 

    // 2. Bang-Bang Humidity Control
    avgHum = totalHum / activeSensors;
    if (avgHum < (TARGET_HUMIDITY - TOLERANCE)) {
      digitalWrite(TRANSDUCER_PIN, HIGH);
      transducerState = true;
      dehumidifierState = false;
    } else if (avgHum > (TARGET_HUMIDITY + TOLERANCE)) {
      digitalWrite(TRANSDUCER_PIN, LOW);
      transducerState = false;
      dehumidifierState = true;
    } else {
      digitalWrite(TRANSDUCER_PIN, LOW);
      transducerState = false;
      dehumidifierState = false;
    }
  } else {
    // Failsafe: Shut off all outputs if sensors disconnect
    Input = 0; Output = 0; avgHum = 0;
    analogWrite(PELTIER_PWM_PIN, 0);
    digitalWrite(TRANSDUCER_PIN, LOW);
    transducerState = false;
    dehumidifierState = false;
  }

  // --- C. TELEMETRY PHASE (Every 2 seconds) ---
  if (millis() - lastPrintTime >= 2000) {
    lastPrintTime = millis(); 
    
    Serial.print(millis() / 1000); 
    Serial.print("\t");
    
    // Print Temperatures
    for (int i = 0; i < 5; i++) {
      if (tReadings[i] != -999.0) Serial.print(tReadings[i], 1); 
      else Serial.print("ERR"); 
      Serial.print("\t");
    }
    
    if (activeSensors > 0) Serial.print(Input, 2);
    else Serial.print("ERR");
    Serial.print("\t");
    Serial.print(Output, 0); 
    Serial.print("\t|\t");

    // Print Humidities
    for (int i = 0; i < 5; i++) {
      if (hReadings[i] != -999.0) Serial.print(hReadings[i], 1); 
      else Serial.print("ERR"); 
      Serial.print("\t");
    }

    if (activeSensors > 0) Serial.print(avgHum, 1);
    else Serial.print("ERR");
    Serial.print("\t");

    // Print States
    Serial.print(transducerState ? "ON (5V)" : "OFF");
    Serial.print("\t\t");
    Serial.println(dehumidifierState ? "ON (Desiccant)" : "OFF");
  }

  // --- D. 5-MINUTE HIGHLIGHT PHASE ---
  if (millis() - lastHighlightTime >= 300000) {
    lastHighlightTime = millis();
    
    Serial.println("\n*****************************************************************************************************************");
    Serial.print("*** 5-MINUTE STATUS | Uptime: ");
    Serial.print(millis() / 60000);
    Serial.print(" min | Temp Avg: ");
    Serial.print(Input, 2);
    Serial.print(" Â°C | PID: ");
    Serial.print(Output, 0);
    Serial.print(" | Hum Avg: ");
    Serial.print(avgHum, 1);
    Serial.println(" %RH ***");
    Serial.println("*****************************************************************************************************************\n");
  }

  delay(100); 
}