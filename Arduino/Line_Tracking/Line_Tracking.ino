/*
  MSE 4499: Line Tracking Code for Betta Feeder Robot
  Language: Arduino
  Authors: Michael Hoskins, Daniel Au, Soo Hyung Choe
  Date: April 2, 2018
*/

// Libraries
#include <EEPROMex.h> // Extended EEPROM library:  http://thijs.elenbaas.net/2012/07/extended-eeprom-library-for-arduino/
#include <SparkFun_TB6612.h>  // Motor Driver library
#include <QTRSensors.h> // IR Line Tracker library
#include <Wire.h> // Wire library for I2C
#include <LCD.h> // LCD library 
#include <Keypad.h> // Keypad library
#include <LiquidCrystal_I2C.h> // LCD I2C Library


// Function Declarations
void checkBattery();
int getBatteryPercentage();
void checkBumper();
void checkFluid();
void checkRowDetector();
void calibrateLineTracker();
void recallLineTracker();
void tunePID();


// Define digital and analog pins
const int Battery_Pin = A0;
const int LineTracker_5V_Pin = 53;
const int RowDetector_Pin = A1;
const int Bumper_Pin = 42;
const int Fluid_Pin = 11;
const int Pump_Pin = 12;
const int Status_LED = 13;
// Note: Motor driver and IR array have their pin definitions further below


// Define program activity flags
int LowBattery_Flag = 0;
int Collision_Flag = 0;
int LowFluid_Flag = 0;
int RowDetected_Flag = 0;


// Define variables
int Program_State = 0; // Program state. Flags cause the program to switch between states
int isWaiting = 0;
unsigned long WaitUntil = 0;
int Bumper_State = 1, Bumper_PrevState = 1; // Bumper switches are normally closed
int Fluid_State = 1, Fluid_PrevState = 1; // Fluid Level switch is normally closed, but kept low by the fluid tank
int RowDetector_Level = 0, RowDetector_PrevLevel = 0;
int Battery_Level = 1000, Battery_PrevLevel = 1000; // Acceptable battery level is 900-1024 (11-12.5V)
int Line_Position, Correction_Speed, RightMotor_Speed, LeftMotor_Speed;
float Kp = 6, Ki = 0, Kd = 0;
float Line_Position_Scaled, Error = 0, Error_Prev = 0, Error_Diff = 0, Error_Sum = 0;


// Define constant variables
int Base_Speed = 200; // Base speed of drive motors prior to PID corrections
const int Sample_Time = 50; // Sample time for PID loop
const int Line_Position_Max = 6000;


// ============================= Motor Driver Setup =============================

// Define motor driver pins
#define AIN1 46
#define BIN1 50
#define AIN2 44
#define BIN2 52
#define PWMA 3
#define PWMB 2
#define STBY 48

// Motor polarity reversal to account for backwards wiring. Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors
Motor RightMotor = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor LeftMotor = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

/*
    Notable motor driver information:
    Speed range is -255 to 255
    Motors can be driven with speed and duration: motor1.drive(255,1000);
    Braking function: motor1.brake(); or brake(motor1,motor2);
*/

// ============================= Line Tracker Setup =============================

// Define line tracker variables
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   51     // emitter is controlled by digital pin 51

// sensors 1 through 8 are connected to odd numbered digital pins 51 to 37, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  49, 47, 45, 43, 41, 39, 37, 35
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

/*
   Notable Line Tracker information:

   To get the calibrated line position under the sensor, from 0-5000:
   unsigned int position = qtrrc.readLine(sensorValues);

   To get the raw sensor values: qtrrc.read(sensorValues);
   To get specific sensor values: sensorValues[i]
*/

//EEPROM Addressing for calibration storage
#define addrCalibratedMinimumOn 0
#define addrCalibratedMaximumOn 100



// ============================= Keypad Setup =============================

const byte ROWS = 4; // Keypad # rows
const byte COLS = 4; // Keypad # cols
char hexaKeys[ROWS][COLS] = { // GUI map of keypad
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {40, 38, 36, 34};  // Keypad pins
byte colPins[COLS] = {32, 30, 28, 26};  // Keypad pins
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

// ============================= LCD Setup =============================

//LCD Variables
int stage = 0;
char keyInput;
int menuIndex = 0;
bool updateLCD = true;
int amount = 0;
int rows = 0;
int containers = 0;
int hundreds = 0;
int tens = 0;
int ones = 0;
int keyInputState = 0;

const int Buzzer_Pin = 33; // Buzzer pin
const int lcd_address = 0x3F; // LCD Address
const byte MAX_ELEM = 4;

LiquidCrystal_I2C lcd(lcd_address, 2, 1, 0, 4, 5, 6, 7);

char *menu_0[] = {
  "1. Prime/clean",
  "2. Start run",
  "3. Test run",
  "4. Settings",
};

char *menu_1[] = {
  "A to prime 1 sec",
  "B to prime 3 sec",
  "",
  "* to go back"
};


char *menu_2[] = {
  "1. Amount",
  "2. # rows",
  "3. Containers/row",
  "4. Start",
};

char *menu_3[] = {
  "1. Amount",
  "2. # rows",
  "3. Containers/row",
  "4. Start",
};

char *menu_4[] = {
  "1. Tuning",
  "2. Speed",
  "3. Dispense delay",
  "4. PID settings"
};

char *menu_5[] = {
  "Dispensing",
  "In Progress....",
  "",
  ""
};

char *menu_6[] = {
  "1. Tune row sensor",
  "2. Tune line sensor",
  "",
  "* to go back"
};

char *menu_7[] = {
  "1. Fast",
  "2. Medium",
  "3. Slow",
  "* to go back"
};

char *menu_8[] = {
  "1. High",
  "2. Medium",
  "3. Low",
  "* to go back"
};

char *menu_9[] = {
  "P (1+,4-)",
  "I (2+,5-)",
  "D (3+,6-)",
  "* to go back"
};

char *menu_10[] = {
  "Amount:",
  "",
  "",
  "* = back B = reset",
};

char *menu_11[] = {
  "Rows:",
  "",
  "",
  "* = back B = reset",
};

char *menu_12[] = {
  "Containers:",
  "",
  "",
  "* = back B = reset",
};

char **menu_array[] = {
  menu_0,
  menu_1,
  menu_2,
  menu_3,
  menu_4,
  menu_5,
  menu_6,
  menu_7,
  menu_8,
  menu_9,
  menu_10,
  menu_11,
  menu_12
};


void setup() {
  Serial.begin(9600);

  pinMode(LineTracker_5V_Pin, OUTPUT); // Use pin 53 for a 5V reference voltage to keep wires tidy.
  digitalWrite(LineTracker_5V_Pin, HIGH);

  pinMode(Battery_Pin, INPUT);
  pinMode(Bumper_Pin, INPUT);
  pinMode(Fluid_Pin, INPUT);
  pinMode(Status_LED, OUTPUT);
  pinMode(Pump_Pin, OUTPUT);
  pinMode(Buzzer_Pin, OUTPUT);


  lcd.begin (20, 4); // 16 x 2 LCD module
  lcd.setBacklightPin(3, POSITIVE); // BL, BL_POL
  lcd.setBacklight(HIGH);

  calibrateLineTracker(); // Activate 10s calibration process
  //recallLineTracker(); // Load previously calibrated values from EEPROM for Line Tracker

  digitalWrite(Status_LED, HIGH);
  delay(500);
  digitalWrite(Status_LED, LOW);

}

void loop() {

  checkBumper(); // Check for bumper collisions
  if (Collision_Flag == 1) {
    Serial.println("Collision Detected");
    //Program_State = 0; // If collision occurs, pause all action
    Program_State = ++Program_State % 2;  // Increment, roll over at n-1 (Currently switches between standby and driving mode)
    Collision_Flag = 0; // Reset flag after servicing
  }

  checkRowDetector();
  if (RowDetected_Flag == 1 && Program_State == 1) {
    if (isWaiting == 0) { // Check if currently waiting for timer
      WaitUntil = millis() + 250; // Specify waiting time
      isWaiting = 1; // Start waiting
    }
    if (WaitUntil <= millis()) {
      Program_State = 3;
      isWaiting = 0;
      RowDetected_Flag = 0;
    }

  }

  checkBattery(); // Check for low battery warning
  if (LowBattery_Flag == 1) {
    Program_State = 2;
    LowBattery_Flag = 0;
  }

  checkFluid(); // Check for low fluid tank level
  if (LowFluid_Flag == 1) {
    Program_State = 0;
    LowFluid_Flag = 0;
  }

  // LCD Loop Code:

  if (updateLCD) {
    updateDisplay(menu_array[menuIndex]);
  }

  keyInput = customKeypad.getKey();

  if (keyInput) {
    buzz();
    switch (stage) {
      case 0: // Main menu
        if (keyInput == '1') {
          setStage(1);
        } else if (keyInput == '2') {
          setStage(2);
        } else if (keyInput == '3') {
          setStage(3);
        } else if (keyInput == '4') {
          setStage(4);
        }
        break;
      case 1: // Prime / Clean pump
        if (keyInput == '*') {
          setStage(0);
        } else if (keyInput == 'A') {
          // Dispense for 3 seconds
        }
        break;
      case 2: // Start run
        if (keyInput == '*') {
          setStage(0);
        } else if (keyInput == '1') {
          setStage(10);
        } else if (keyInput == '2') {
          setStage(11);
        } else if (keyInput == '3') {
          setStage(12);
        } else if (keyInput == '4') {
          setStage(5);
        }
        break;
      case 3: // Test run
        if (keyInput == '*') {
          setStage(0);
        }
        break;
      case 4: // General settings
        if (keyInput == '*') {
          setStage(0);
        } else if (keyInput == '1') {
          setStage(6);
        } else if (keyInput == '2') {
          setStage(7);
        } else if (keyInput == '3') {
          setStage(8);
        } else if (keyInput == '4') {
          setStage(9);
        }
        break;
      case 5: // Running -- Display stats
        if (keyInput == '*') {
          setStage(2);
          Program_State = 0;
        }
        Program_State = 0;

        break;
    }


    switch (Program_State) {

      case 0: // Stationary Robot.

        brake(RightMotor, LeftMotor);
        analogWrite(Pump_Pin, 0);

        break;


      case 1: // Line Tracking

        delay(Sample_Time);

        Line_Position = qtrrc.readLine(sensorValues); // Get current position

        // Constrain max/min values
        if (Line_Position > Line_Position_Max) {
          Line_Position = Line_Position_Max;
        }
        else if (Line_Position < 0) {
          Line_Position = 0;
        }

        // Scale Line_Position from 0-Line_Position_Max to 0-1 and convert int to float
        Line_Position_Scaled = ((float) Line_Position / (float) Line_Position_Max);

        // Calculate Error for PID
        Error_Prev = Error;
        Error = 0.5 - Line_Position_Scaled;
        Error_Diff = (Error - Error_Prev) / Sample_Time;
        Error_Sum += (Error * Sample_Time);

        // Calculate Correction_Speed (bounded between 0-100) using PID
        Correction_Speed = (int)( (Kp * Error + Ki * Error_Sum + Kd * Error_Diff) * 100 );


        // Set drive motor speeds, constrained to forward motion
        RightMotor_Speed = Base_Speed - Correction_Speed;
        if (RightMotor_Speed < 50) {
          RightMotor_Speed = 50;
        } else if (RightMotor_Speed > 255) {
          RightMotor_Speed = 255;
        }

        LeftMotor_Speed = Base_Speed + Correction_Speed;
        if (LeftMotor_Speed < 50) {
          LeftMotor_Speed = 50;
        } else if (LeftMotor_Speed > 255) {
          LeftMotor_Speed = 255;
        }

        Battery_Level = analogRead(Battery_Pin);
        if (Battery_Level > 700) { // Only run motors if battery is connected.
          RightMotor.drive(RightMotor_Speed);
          LeftMotor.drive(LeftMotor_Speed);
        } else { // If battery is not connected, just print the specified motor speeds.

          Serial.print(Line_Position);
          Serial.print("     ");
          Serial.print(LeftMotor_Speed);
          Serial.print("      ");
          Serial.println(RightMotor_Speed);
        }
        break;

      case 2: // Low Battery State. Pause all actions and warn user of low battery
        brake(RightMotor, LeftMotor);

        Serial.print("Battery Level: ");
        Serial.print(Battery_Level);
        Serial.print("      Battery %: ");
        Serial.print(getBatteryPercentage());
        if ( Battery_Level < 900 ) {
          Serial.println("%   Battery critically low!");
        }
        else {
          Serial.println("%");
        }

        break;

      case 3: // Row detected, dispense liquid

        // Wait for 2 seconds to "dispense"
        brake(RightMotor, LeftMotor);
        
          Battery_Level = analogRead(Battery_Pin);
          if (Battery_Level > 700) { // Only run motors if battery is connected.
          analogWrite(Pump_Pin, 150);
          }
        
        delay(350); // 350 ms is the required time to dispense 5 ml of liquid food per container

        analogWrite(Pump_Pin, 0);

        Program_State = 1;
        break;

    }


  }


  // ============================= Functions =============================


  void checkBattery() {
    Battery_PrevLevel = Battery_Level;
    Battery_Level = analogRead(Battery_Pin);

    // If voltage drops to 11V, ie. Arduino reads 4.4V, set flag (75% depleted, nearing dump voltage)
    // 900/1024*5V = 4.39V
    if (Battery_Level < 900 && Battery_PrevLevel < 900) { // Check 2 values to denoise signal
      LowBattery_Flag = 1;
    }
  }

  int getBatteryPercentage() {  // Returns the battery level as a percentage of full charge (Rough estimate: not linearized)
    Battery_Level = analogRead(Battery_Pin);
    double Battery_Percentage = ((double) Battery_Level - 900) / (1024 - 900) * 100;
    return (int) Battery_Percentage;
  }

  void checkBumper() {
    Bumper_PrevState = Bumper_State;
    Bumper_State = digitalRead(Bumper_Pin);

    if (Bumper_PrevState != Bumper_State) { // If switch state changes
      if (Bumper_State == 0) { // Bumper switches are normally closed (Collision = 0)
        Collision_Flag = 1;
      }
    }
  }

  void checkFluid() {
    Fluid_PrevState = Fluid_State;
    Fluid_State = digitalRead(Fluid_Pin);

    if (Fluid_PrevState != Fluid_State) { // If switch state changes
      if (Fluid_State == 1) { // Fluid level switch is normally closed, and goes HIGH when tank gets empty.
        LowFluid_Flag = 1; // Flag is set if a full tank is placed and then removed. If no weight is placed on initially, flag will not be set.
      }
    }
  }

  void checkRowDetector() {

    RowDetector_PrevLevel = RowDetector_Level; // Store old moving average value (with offset)
    RowDetector_Level = analogRead(RowDetector_Pin);

    if (RowDetector_Level > 80 && RowDetector_Level < 650) { // 650 is upper bound filter, 80 is lower bound filter
      if (RowDetector_PrevLevel < 500 && RowDetector_Level > 500) { //350 = 15 cm +/- 2cm
        RowDetected_Flag = 1;
      }
    }
  }

  void calibrateLineTracker() {

    Serial.println("Calibrating Line Tracker. Sweeping array across line to identify contrast between line and floor...");
    delay(500);


    for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
    {
      qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    }

    Serial.println("Calibration complete.");

    Serial.println();
    Serial.println("Storing Calibration Data into EEPROM...");

    EEPROM.writeBlock<unsigned int>(addrCalibratedMinimumOn, qtrrc.calibratedMinimumOn, 8);
    EEPROM.writeBlock<unsigned int>(addrCalibratedMaximumOn, qtrrc.calibratedMaximumOn, 8);

    Serial.println("EEPROM Storage Complete");

    delay(1000);

  }

  void recallLineTracker() {
    Serial.println();
    Serial.println("Recalling Calibration Data from EEPROM...");

    qtrrc.calibrate();
    EEPROM.readBlock<unsigned int>(addrCalibratedMinimumOn, qtrrc.calibratedMinimumOn, 8);
    EEPROM.readBlock<unsigned int>(addrCalibratedMaximumOn, qtrrc.calibratedMaximumOn, 8);

    Serial.println("EEPROM Recall Complete");
  }

  void tunePID() {

    if (Serial.available())
    {
      char key = Serial.read();
      if (key == 'P')  {
        Kp += 0.1;
      }
      else if (key == 'p') {
        Kp -= 0.1;
      }
      else if (key == 'I') {
        Ki += 0.1;
      }
      else if (key == 'i') {
        Ki -= 0.1;
      }
      else if (key == 'D') {
        Kp += 0.1;
      }
      else if (key == 'd') {
        Kp -= 0.1;
      }
      else if (key == 'B') {
        Base_Speed += 10;
      }
      else if (key == 'b') {
        Base_Speed -= 10;
      }

      Serial.print("Base Speed: ");
      Serial.print(Base_Speed);
      Serial.print("    Kp: ");
      Serial.print(Kp);
      Serial.print("    Ki: ");
      Serial.print(Ki);
      Serial.print("    Kd: ");
      Serial.println(Kd);

    }

  }

// =================== U/I Functions ================================

  void updateDisplay(char **menu) {
    lcd.clear ();
    for (int i = 0; i < MAX_ELEM; i++) {
      lcd.print(menu[i]);
      lcd.setCursor(0, i + 1);
    }
    updateLCD = false;
  }

  void printLCD(int column, int row, String test) {
    lcd.clear ();
    lcd.print(test);
  }

  void setStage(int var) {
    updateLCD = true;
    menuIndex = var;
    stage = var;
  }

  void buzz() {
    tone(Buzzer_Pin, 1000); // Send 1KHz sound signal
    delay(150);        // 150ms
    noTone(Buzzer_Pin);     // Stop sound
  }



  void getInput(char keyInput) {
    switch (keyInputState) {
      case 0:
        lcd.setCursor(0, 1);
        lcd.print(keyInput);
        hundreds = keyInput - '0';
        keyInputState = 1;
        break;
      case 1:
        if (keyInput == '#') {
          calculateNumberAmount();
          keyInputState = 4;
          break;
        } else if (keyInput == 'B') {
          resetInput();
          break;
        }
        lcd.print(keyInput);
        keyInputState = 2;
        tens = keyInput - '0';
        break;
      case 2:
        if (keyInput == '#') {
          calculateNumberAmount();
          keyInputState = 4;
          break;
        } else if (keyInput == 'B') {
          resetInput();
          break;
        }
        lcd.print(keyInput);
        keyInputState = 3;
        ones = keyInput - '0';
        break;
      case 3:
        if (keyInput == '#') {
          keyInputState = 4;
          calculateNumberAmount();
          break;
        } else if (keyInput == 'B') {
          resetInput();
          break;
        }
        break;
      case 4:
        if (keyInput == 'B') {
          resetInput();
          break;
        }
        break;
    }
  }

  void getInputRows(char keyInput) {
    switch (keyInputState) {
      case 0:
        lcd.setCursor(0, 1);
        lcd.print(keyInput);
        hundreds = keyInput - '0';
        keyInputState = 1;
        break;
      case 1:
        if (keyInput == '#') {
          calculateNumberRows();
          keyInputState = 4;
          break;
        } else if (keyInput == 'B') {
          resetInput();
          break;
        }
        lcd.print(keyInput);
        keyInputState = 2;
        tens = keyInput - '0';
        break;
      case 2:
        if (keyInput == '#') {
          calculateNumberRows();
          keyInputState = 4;
          break;
        } else if (keyInput == 'B') {
          resetInput();
          break;
        }
        lcd.print(keyInput);
        keyInputState = 3;
        ones = keyInput - '0';
        break;
      case 3:
        if (keyInput == '#') {
          keyInputState = 4;
          calculateNumberRows();
          break;
        } else if (keyInput == 'B') {
          resetInput();
          break;
        }
        break;
      case 4:
        if (keyInput == 'B') {
          resetInput();
          break;
        }
        break;
    }
  }

  void calculateNumberAmount() {
    if (ones == -1 && tens == -1) {
      amount = hundreds;
    } else if (ones == -1) {
      amount = (10 * hundreds) + tens;
    } else {
      amount = (100 * hundreds) + (10 * tens) + ones;
    }

    hundreds = -1;
    tens = -1;
    ones = -1;

    lcd.setCursor(0, 1);
    lcd.print(amount);
    lcd.setCursor(0, 2);
    lcd.print("Saved");
  }

  void calculateNumberRows() {
    if (ones == -1 && tens == -1) {
      rows = hundreds;
    } else if (ones == -1) {
      rows = (10 * hundreds) + tens;
    } else {
      rows = (100 * hundreds) + (10 * tens) + ones;
    }

    hundreds = -1;
    tens = -1;
    ones = -1;

    lcd.setCursor(0, 1);
    lcd.print(rows);
    lcd.setCursor(0, 2);
    lcd.print("Saved");
  }

  void resetInput() {
    hundreds = -1;
    tens = -1;
    ones = -1;

    clearLCDRow(1);
    clearLCDRow(2);
    lcd.setCursor(0, 1);
    keyInputState = 0;
  }

  void clearLCDRow(int rowNumber) {
    lcd.setCursor(0, rowNumber);
    lcd.print("                   ");
  }


