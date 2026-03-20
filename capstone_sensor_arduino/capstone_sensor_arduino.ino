#include <SoftwareWire.h>
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

  Serial.println("==========================================================");
  Serial.println("MASTER CONTROLLER | LISTENING FOR UI TARGETS");
  Serial.println("==========================================================");
}

void loop() {
  // --- READ INCOMING TARGETS FROM UI ---
  // Format expected: {34.0,65.0}
  if (boardToBoard.available() > 0) {
    if (boardToBoard.read() == '{') {
      float incomingT = boardToBoard.parseFloat();
      float incomingH = boardToBoard.parseFloat();
      if (boardToBoard.read() == '}') {
        Setpoint = incomingT;
        targetHumidity = incomingH;
        Serial.print(">>> NEW TARGETS RECEIVED: Temp = ");
        Serial.print(Setpoint);
        Serial.print("C | Hum = ");
        Serial.println(targetHumidity);
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
      totalTemp += temp; 
      totalHum += hum;
      activeSensors++;                 
    } 
  }

  // --- CONTROL PHASE ---
  if (activeSensors > 0) {
    Input = totalTemp / activeSensors; 
    peltierPID.Compute(); 
    analogWrite(PELTIER_PWM_PIN, Output); 

    avgHum = totalHum / activeSensors;
    old_dehumidifierState = dehumidifierState;
    if (avgHum < (targetHumidity - TOLERANCE)) {
      digitalWrite(TRANSDUCER_PIN, HIGH);
      transducerState = true;
      dehumidifierState = false;
      if (old_dehumidifierState != dehumidifierState) {
        Serial.println("////////////////////////////");
        Serial.println("/// CLOSE DESICCANT PATH ///");
        Serial.println("////////////////////////////");
      }
    } else if (avgHum > (targetHumidity + TOLERANCE)) {
      digitalWrite(TRANSDUCER_PIN, LOW);
      transducerState = false;
      dehumidifierState = true;
      if (old_dehumidifierState != dehumidifierState) {
        Serial.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
        Serial.println("%%% OPEN  DESICCANT PATH %%%");
        Serial.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
      }
    } else {
      digitalWrite(TRANSDUCER_PIN, LOW);
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
    digitalWrite(TRANSDUCER_PIN, LOW);
  }

  // --- TELEMETRY & TRANSMISSION PHASE ---
  if (millis() - lastPrintTime >= 2000) {
    lastPrintTime = millis(); 
    
    // Send live data to Arduino 2 UI Panel
    // Format sent: <25.5,60.2>
    boardToBoard.print("<");
    boardToBoard.print(Input);
    boardToBoard.print(",");
    boardToBoard.print(avgHum);
    boardToBoard.println(">");

    Serial.print(millis() / 1000); 
    Serial.print("\t");
    
    // Print to PC for debugging
    Serial.print("CurT: "); Serial.print(Input, 1);
    Serial.print("\tTarT: "); Serial.print(Setpoint, 1);
    Serial.print("\tCurH: "); Serial.print(avgHum, 1);
    Serial.print("\tTarH: "); Serial.println(targetHumidity, 1);
  }
  delay(100); 
}




// TEST FUNCTION FOR REDUCING MIST RUNTIME AKA WATER SAVINGS 
void transducerPeriodic() {
  while (transducerState == true) {
    // "ON mode" means periodically turning on and off
    digitalWrite(TRANSDUCER_PIN, HIGH);
    delay(500);
    digitalWrite(TRANSDUCER_PIN, LOW);
    delay(500);
  }
}

// void dehumidifierPath() {
  // code to turn servo motor enough to move slider
// }