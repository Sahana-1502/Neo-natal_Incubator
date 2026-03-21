// Includes the SoftwareWire library to emulate I2C on standard digital pins
#include <SoftwareWire.h>
// Includes the standard Arduino PID library for closed-loop control
#include <PID_v1.h>
// Add a second SoftwareSerial port to talk to Arduino 2
#include <SoftwareSerial.h>

// ==========================================
// 1. PIN DEFINITIONS & BUSES
// ==========================================
#define PELTIER_PWM_PIN 6
#define TRANSDUCER_PIN 7

SoftwareWire bus1(2, 3);
SoftwareWire bus2(4, 5);
SoftwareWire bus3(8, 9);
SoftwareWire bus4(10, 11);
SoftwareWire bus5(12, 13);
SoftwareWire* sensorBuses[5] = {&bus1, &bus2, &bus3, &bus4, &bus5};

// Communication Bridge: RX on A0, TX on A1
SoftwareSerial boardToBoard(A0, A1); 

// ==========================================
// 2. CONTROL VARIABLES
// ==========================================
double Setpoint = 34.0; // Default boot value
double Input, Output;
double Kp = 50.0, Ki = 1.0, Kd = 5.0;   
PID peltierPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

float targetHumidity = 65.0; // Default boot value, no longer a const!
const float TOLERANCE = 2.0;         

bool transducerState = false;
bool dehumidifierState = false;
bool old_dehumidifierState = false;

// Variables for the Non-Blocking Humidifier Pulse
unsigned long lastTransducerToggle = 0;
bool transducerPinState = LOW;

unsigned long lastPrintTime = 0;
unsigned long lastHighlightTime = 0;

// ==========================================
// 3. SENSOR HELPER
// ==========================================
bool readAHT20(SoftwareWire* bus, float &temperature, float &humidity) {
  bus->beginTransmission(0x38); 
  bus->write(0xAC);             
  bus->write(0x33);             
  bus->write(0x00);
  if (bus->endTransmission() != 0) return false; 
  delay(80); 
  bus->requestFrom(0x38, 6);
  
  if (bus->available() == 6) {
    bus->read(); 
    uint8_t b1 = bus->read(); 
    uint8_t b2 = bus->read(); 
    uint8_t b3 = bus->read(); 
    uint8_t b4 = bus->read(); 
    uint8_t b5 = bus->read(); 

    uint32_t rawHum = ((uint32_t)b1 << 12) | ((uint32_t)b2 << 4) | (b3 >> 4);
    humidity = ((float)rawHum / 1048576.0) * 100.0;

    uint32_t rawTemp = ((uint32_t)(b3 & 0x0F) << 16) | ((uint32_t)b4 << 8) | b5;
    temperature = ((float)rawTemp / 1048576.0) * 200.0 - 50.0;
    return true; 
  }
  return false; 
}

void setup() {
  Serial.begin(9600); 
  boardToBoard.begin(9600); // Start communication with Arduino 2
  
  pinMode(PELTIER_PWM_PIN, OUTPUT);
  pinMode(TRANSDUCER_PIN, OUTPUT);
  analogWrite(PELTIER_PWM_PIN, 0); 
  digitalWrite(TRANSDUCER_PIN, LOW);

  for (int i = 0; i < 5; i++) sensorBuses[i]->begin();
  delay(100); 

  peltierPID.SetMode(AUTOMATIC); 
  peltierPID.SetOutputLimits(0, 255); 
  peltierPID.SetSampleTime(1000);

  Serial.println("=========================================================================================================================================");
  Serial.println("MASTER CONTROLLER | LISTENING FOR UI TARGETS");
  Serial.println("=========================================================================================================================================");
}

void loop() {
  // FIX 1: Re-declared the missing arrays!
  float tReadings[5] = {0};
  float hReadings[5] = {0};

  // --- READ INCOMING TARGETS FROM UI ---
  if (boardToBoard.available() > 0) {
    if (boardToBoard.read() == '{') {
      float incomingT = boardToBoard.parseFloat();
      float incomingH = boardToBoard.parseFloat();
      if (boardToBoard.read() == '}') {
        Setpoint = incomingT;
        targetHumidity = incomingH;
        Serial.print("\n>>> NEW TARGETS RECEIVED: Temp = ");
        Serial.print(Setpoint);
        Serial.print("C | Hum = ");
        Serial.println(targetHumidity);

        Serial.println("\nTime\tT1\tT2\tT3\tT4\tT5\tT_Avg\tT_Tar\tPWM\t|\tH1\tH2\tH3\tH4\tH5\tH_Avg\tH_Tar\tHumi\tDehumi");
        Serial.println("----\t--\t--\t--\t--\t--\t-----\t-----\t---\t \t--\t--\t--\t--\t--\t-----\t-----\t----\t------");
      }
    }
  }

  // --- DATA ACQUISITION ---
  float totalTemp = 0, totalHum = 0;       
  int activeSensors = 0;     
  float avgHum = 0;

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

  // --- CONTROL PHASE ---
  if (activeSensors > 0) {
    Input = totalTemp / activeSensors; 
    peltierPID.Compute(); 
    analogWrite(PELTIER_PWM_PIN, Output); 

    avgHum = totalHum / activeSensors;
    old_dehumidifierState = dehumidifierState;
    
    // Bang-Bang Logic
    if (avgHum < (targetHumidity - TOLERANCE)) {
      transducerState = true;
      dehumidifierState = false;
      if (old_dehumidifierState != dehumidifierState) {
        Serial.println("////////////////////////////");
        Serial.println("/// CLOSE DESICCANT PATH ///");
        Serial.println("////////////////////////////");
      }
    } else if (avgHum > (targetHumidity + TOLERANCE)) {
      transducerState = false;
      dehumidifierState = true;
      if (old_dehumidifierState != dehumidifierState) {
        Serial.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
        Serial.println("%%% OPEN  DESICCANT PATH %%%");
        Serial.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
      }
    } else {
      transducerState = false;
      dehumidifierState = false;
      if (old_dehumidifierState != dehumidifierState) {
        Serial.println("////////////////////////////");
        Serial.println("/// CLOSE DESICCANT PATH ///");
        Serial.println("////////////////////////////");
      }
    }
  } else {
    Input = 0; Output = 0; avgHum = 0;
    analogWrite(PELTIER_PWM_PIN, 0);
    transducerState = false;
    dehumidifierState = false;
  }

  // FIX 2: NON-BLOCKING TRANSDUCER PULSE
  if (transducerState == true) {
    // Check if 500ms have passed since the last toggle
    if (millis() - lastTransducerToggle >= 500) {
      lastTransducerToggle = millis();
      transducerPinState = !transducerPinState; // Flip the state
      digitalWrite(TRANSDUCER_PIN, transducerPinState);
    }
  } else {
    digitalWrite(TRANSDUCER_PIN, LOW); // Force off if target is reached
    transducerPinState = LOW;
  }

  // --- TELEMETRY & TRANSMISSION PHASE ---
  if (millis() - lastPrintTime >= 2000) {
    lastPrintTime = millis(); 
    
    boardToBoard.print("<");
    boardToBoard.print(Input);
    boardToBoard.print(",");
    boardToBoard.print(avgHum);
    boardToBoard.println(">");

    // FIX 3: Perfectly Aligned Telemetry Printout
    Serial.print(millis() / 1000); 
    Serial.print("\t");
    
    // T1 - T5
    for (int i = 0; i < 5; i++) {
      if (tReadings[i] != -999.0) Serial.print(tReadings[i], 1); 
      else Serial.print("ERR"); 
      Serial.print("\t");
    }
    
    // T_Avg
    if (activeSensors > 0) Serial.print(Input, 1);
    else Serial.print("ERR");
    Serial.print("\t");

    // T_Target
    Serial.print(Setpoint, 1); 
    Serial.print("\t");

    // PWM
    Serial.print(Output, 0); 
    Serial.print("\t|\t");

    // H1 - H5
    for (int i = 0; i < 5; i++) {
      if (hReadings[i] != -999.0) Serial.print(hReadings[i], 1); 
      else Serial.print("ERR"); 
      Serial.print("\t");
    }

    // H_Avg
    if (activeSensors > 0) Serial.print(avgHum, 1);
    else Serial.print("ERR");
    Serial.print("\t");

    // H_Target
    Serial.print(targetHumidity, 1);
    Serial.print("\t");

    // Humi & Dehumi States
    Serial.print(transducerState ? "PULSING" : "OFF");
    Serial.print("\t");
    Serial.println(dehumidifierState ? "ON(BEADS)" : "OFF");
  }

  // --- D. 5-MINUTE HIGHLIGHT PHASE ---
  if (millis() - lastHighlightTime >= 300000) {
    lastHighlightTime = millis();
    
    Serial.println("\n*****************************************************************************************************************");
    Serial.print("*** 5-MINUTE STATUS | Uptime: ");
    Serial.print(millis() / 60000);
    Serial.print(" min | Temp Avg: ");
    Serial.print(Input, 2);
    Serial.print(" °C | PID: ");
    Serial.print(Output, 0);
    Serial.print(" | Hum Avg: ");
    Serial.print(avgHum, 1);
    Serial.println(" %RH ***");
    Serial.println("*****************************************************************************************************************\n");
  }
  delay(100); 
}
