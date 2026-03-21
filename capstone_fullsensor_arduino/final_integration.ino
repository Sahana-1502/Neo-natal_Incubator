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

// Alarms & Sensors
#define MUTE_BUTTON_PIN A2 
#define BUZZER_PIN A4
#define ALARM_LED_PIN A5    

SoftwareWire bus1(2, 3);
SoftwareWire bus2(4, 5);
SoftwareWire bus3(8, 9);
SoftwareWire bus4(10, 11);
SoftwareWire bus5(12, 13);
SoftwareWire* sensorBuses[5] = {&bus1, &bus2, &bus3, &bus4, &bus5};

// Communication Bridge: RX on A0, TX on A1
SoftwareSerial boardToBoard(A0, A1); 

// ==========================================
// 2. CONTROL VARIABLES & ALARMS
// ==========================================
double Setpoint = 28.0; 
double Input, Output;
double Kp = 50.0, Ki = 1.0, Kd = 5.0;   
PID peltierPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

float targetHumidity = 80.0; 
const float TOLERANCE = 2.0;         

bool transducerState = false;
bool dehumidifierState = false;
bool old_dehumidifierState = false;

// Hardware Safety Cutoffs
const float WARNING_TEMP = 38.0;   
const float CRITICAL_TEMP = 39.5;  

bool buzzerMuted = false;
bool lastButtonState = HIGH; 
unsigned long lastDebounceTime = 0;

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
  
  unsigned long startWait = millis();
  while(millis() - startWait < 80) {
      bool currentButtonState = digitalRead(MUTE_BUTTON_PIN);
      if (currentButtonState == LOW && lastButtonState == HIGH && (millis() - lastDebounceTime > 200)) {
        buzzerMuted = !buzzerMuted; 
        lastDebounceTime = millis();
      }
      lastButtonState = currentButtonState;
  }
  
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

// ==========================================
// 4. SETUP ROUTINE
// ==========================================
void setup() {
  Serial.begin(9600); 
  boardToBoard.begin(9600); 
  
  pinMode(PELTIER_PWM_PIN, OUTPUT);
  pinMode(TRANSDUCER_PIN, OUTPUT);
  pinMode(ALARM_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MUTE_BUTTON_PIN, INPUT_PULLUP); 
  
  analogWrite(PELTIER_PWM_PIN, 0); 
  digitalWrite(TRANSDUCER_PIN, LOW);

  for (int i = 0; i < 5; i++) sensorBuses[i]->begin();
  delay(100); 

  peltierPID.SetMode(AUTOMATIC); 
  peltierPID.SetOutputLimits(0, 255); 
  peltierPID.SetSampleTime(1000);

  Serial.println("=========================================================================================================================================");
  Serial.println("MASTER CONTROLLER | THERMAL SAFETY OVERRIDES ENABLED");
  Serial.println("=========================================================================================================================================");
  
  Serial.println("Time\tT1\tT2\tT3\tT4\tT5\tT_Avg\tT_Tar\tPWM\t|\tH1\tH2\tH3\tH4\tH5\tH_Avg\tH_Tar\tHumi\tDehumi\tStatus");
  Serial.println("----\t--\t--\t--\t--\t--\t-----\t-----\t---\t \t--\t--\t--\t--\t--\t-----\t-----\t----\t------\t------");
}

// ==========================================
// 5. MAIN LOOP
// ==========================================
void loop() {
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
        
        Serial.print("\n>>> NEW TARGETS: Temp = ");
        Serial.print(Setpoint);
        Serial.print("C | Hum = ");
        Serial.println(targetHumidity);

        Serial.println("\nTime\tT1\tT2\tT3\tT4\tT5\tT_Avg\tT_Tar\tPWM\t|\tH1\tH2\tH3\tH4\tH5\tH_Avg\tH_Tar\tHumi\tDehumi\tStatus");
        Serial.println("----\t--\t--\t--\t--\t--\t-----\t-----\t---\t \t--\t--\t--\t--\t--\t-----\t-----\t----\t------\t------");
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

  // --- CONTROL PHASE & SAFETY OVERRIDE ---
  bool isCriticalAlarm = false;
  bool isWarningAlarm = false;

  if (activeSensors > 0) {
    Input = totalTemp / activeSensors; 
    
    // SAFETY CHECK: Temperature Cutoff
    if (Input >= CRITICAL_TEMP) {
      isCriticalAlarm = true;
      Output = 0; 
      analogWrite(PELTIER_PWM_PIN, 0); // Physically cut power to heater
      
      // Force humidifier states to off
      transducerState = false;
      dehumidifierState = false;
    } 
    else {
      // Normal Operation
      if (Input >= WARNING_TEMP) isWarningAlarm = true;
      peltierPID.Compute(); 
      analogWrite(PELTIER_PWM_PIN, Output); 
      
      // Normal Bang-Bang Logic
      avgHum = totalHum / activeSensors;
      old_dehumidifierState = dehumidifierState;
      
      if (avgHum < (targetHumidity - TOLERANCE)) {
        transducerState = true;
        dehumidifierState = false;
      } else if (avgHum > (targetHumidity + TOLERANCE)) {
        transducerState = false;
        dehumidifierState = true;
      } else {
        transducerState = false;
        dehumidifierState = false;
      }
    }
  } else {
    // Failsafe: No sensors active
    Input = 0; Output = 0; avgHum = 0;
    analogWrite(PELTIER_PWM_PIN, 0);
    transducerState = false;
    dehumidifierState = false;
    isCriticalAlarm = true; 
  }

  // --- TRIGGER HARDWARE ALARMS ---
  if (isCriticalAlarm || isWarningAlarm) {
    digitalWrite(ALARM_LED_PIN, HIGH); 
    if (!buzzerMuted) {
      tone(BUZZER_PIN, 2000); 
    } else {
      noTone(BUZZER_PIN);
    }
  } else {
    digitalWrite(ALARM_LED_PIN, LOW);
    noTone(BUZZER_PIN);
    buzzerMuted = false; 
  }

  // NON-BLOCKING TRANSDUCER PULSE
  if (transducerState == true) {
    if (millis() - lastTransducerToggle >= 5000) {
      lastTransducerToggle = millis();
      transducerPinState = !transducerPinState; 
      digitalWrite(TRANSDUCER_PIN, transducerPinState);
    }
    //digitalWrite(TRANSDUCER_PIN, HIGH);
    //transducerPinState = !transducerPinState; 
  } else {
    digitalWrite(TRANSDUCER_PIN, LOW); 
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

    Serial.print(millis() / 1000); 
    Serial.print("\t");
    
    for (int i = 0; i < 5; i++) {
      if (tReadings[i] != -999.0) Serial.print(tReadings[i], 1); 
      else Serial.print("ERR"); 
      Serial.print("\t");
    }
    
    if (activeSensors > 0) Serial.print(Input, 1);
    else Serial.print("ERR");
    Serial.print("\t");

    Serial.print(Setpoint, 1); 
    Serial.print("\t");
    Serial.print(Output, 0); 
    Serial.print("\t|\t");

    for (int i = 0; i < 5; i++) {
      if (hReadings[i] != -999.0) Serial.print(hReadings[i], 1); 
      else Serial.print("ERR"); 
      Serial.print("\t");
    }

    if (activeSensors > 0) Serial.print(avgHum, 1);
    else Serial.print("ERR");
    Serial.print("\t");

    Serial.print(targetHumidity, 1);
    Serial.print("\t");

    Serial.print(transducerState ? "PULSING" : "OFF");
    Serial.print("\t");
    Serial.print(dehumidifierState ? "ON(BEADS)" : "OFF");
    Serial.print("\t");

    // Print Status Column priority check
    if (activeSensors == 0) Serial.println("SENSOR_FAIL");
    else if (Input >= CRITICAL_TEMP) Serial.println("CRITICAL_TEMP");
    else if (isWarningAlarm) Serial.println("HIGH_TEMP");
    else Serial.println("OK");
  }

  // --- 5-MINUTE HIGHLIGHT PHASE ---
  if (millis() - lastHighlightTime >= 300000) {
    lastHighlightTime = millis();
    
    // UPDATED: Added Uptime to the Highlight Banner
    Serial.print("\n*** 5-MINUTE HIGHLIGHT | Uptime: ");
    Serial.print(millis() / 60000);
    Serial.println(" Minutes | System Running Cleanly ***\n");
    
    Serial.println("Time\tT1\tT2\tT3\tT4\tT5\tT_Avg\tT_Tar\tPWM\t|\tH1\tH2\tH3\tH4\tH5\tH_Avg\tH_Tar\tHumi\tDehumi\tStatus");
    Serial.println("----\t--\t--\t--\t--\t--\t-----\t-----\t---\t \t--\t--\t--\t--\t--\t-----\t-----\t----\t------\t------");
  }
  
  // Keep checking the mute button at the very end of the loop
  unsigned long endWait = millis();
  while(millis() - endWait < 100) {
      bool currentButtonState = digitalRead(MUTE_BUTTON_PIN);
      if (currentButtonState == LOW && lastButtonState == HIGH && (millis() - lastDebounceTime > 200)) {
        buzzerMuted = !buzzerMuted; 
        lastDebounceTime = millis();
      }
      lastButtonState = currentButtonState;
  }
}
