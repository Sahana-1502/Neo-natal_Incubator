#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <SoftwareSerial.h>

// Communication Bridge: RX on 10, TX on 11
SoftwareSerial boardToBoard(10, 11); 

LiquidCrystal_I2C lcd(0x27, 16, 2);

const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {9, 8, 7, 6};   
byte colPins[COLS] = {5, 4, 3, 2};   

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

float targetTemp = 34.0;
float targetHum  = 65.0;
float currentTemp = 0.0;
float currentHum  = 0.0;

unsigned long lastScreenUpdate = 0;

void clearDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
}

void showDefault() {
  lcd.setCursor(0, 0);
  lcd.print("Tar T:");
  lcd.print((int)targetTemp);
  lcd.print("C H:");
  lcd.print((int)targetHum);
  lcd.print("%  "); // Spaces to overwrite old digits

  lcd.setCursor(0, 1);
  lcd.print("Cur T:");
  lcd.print((int)currentTemp);
  lcd.print("C H:");
  lcd.print((int)currentHum);
  lcd.print("%  ");
}

void showInvalid() {
  clearDisplay();
  lcd.print("Invalid Input");
  delay(1000);
  clearDisplay();
  showDefault();
}

// Blast new targets to Arduino 1
void transmitTargets() {
  boardToBoard.print("{");
  boardToBoard.print(targetTemp);
  boardToBoard.print(",");
  boardToBoard.print(targetHum);
  boardToBoard.println("}");
}

int readNumber(const char* prompt) {
  clearDisplay();
  lcd.print(prompt);
  lcd.setCursor(0, 1);

  String input = "";

  while (true) {
    // Keep checking for live sensor updates while typing
    receiveSensorData();

    char key = keypad.getKey();
    if (key) {
      if (key >= '0' && key <= '9') {
        if (input.length() < 3) {
          input += key;
          lcd.setCursor(0, 1);
          lcd.print("                ");   
          lcd.setCursor(0, 1);
          lcd.print(input);
        }
      }
      else if (key == 'C') {   
        if (input.length() > 0) return input.toInt();
        return -2;   
      }
      else if (key == '*') {   
        return -1;
      }
    }
  }
}

// Catch live sensor telemetry from Arduino 1
void receiveSensorData() {
  if (boardToBoard.available() > 0) {
    if (boardToBoard.read() == '<') {
      float inTemp = boardToBoard.parseFloat();
      float inHum = boardToBoard.parseFloat();
      if (boardToBoard.read() == '>') {
        currentTemp = inTemp;
        currentHum = inHum;
        
        // Only refresh screen every 1 sec to prevent flickering
        if (millis() - lastScreenUpdate > 1000) {
          showDefault();
          lastScreenUpdate = millis();
        }
      }
    }
  }
}

void setup() {
  Serial.begin(9600);
  boardToBoard.begin(9600);
  
  lcd.init();
  lcd.backlight();

  clearDisplay();
  showDefault();
  
  // Push initial targets to Arduino 1 on boot
  delay(1000);
  transmitTargets();
}

void loop() {
  receiveSensorData();

  char key = keypad.getKey();
  if (key) {
    if (key == 'A') {
      int val = readNumber("Enter T (30-39):");
      if (val >= 30 && val <= 39) {
        targetTemp = val;
        transmitTargets(); // Send new target to controller
      } else if (val == -2 || val > 39 || (val != -1 && val < 30)) {
        showInvalid();
      }
      clearDisplay();
      showDefault();
    }
    else if (key == 'B') {
      int val = readNumber("Enter H (40-90):");
      if (val >= 40 && val <= 90) {
        targetHum = val;
        transmitTargets(); // Send new target to controller
      } else if (val == -2 || val > 90 || (val != -1 && val < 40)) {
        showInvalid();
      }
      clearDisplay();
      showDefault();
    }
  }
}