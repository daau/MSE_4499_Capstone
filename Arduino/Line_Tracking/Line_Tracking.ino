;''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''/*

  MSE 4499 Line Tracking Code for Betta Feeder Robot
  Language: Arduino
  Authors: Michael Hoskins
  Date: March 26 2018

*/

// Libraries
#include <EEPROMex.h> // Extended EEPROM library:  http://thijs.elenbaas.net/2012/07/extended-eeprom-library-for-arduino/
#include <SparkFun_TB6612.h>  // Motor Driver library
#include <QTRSensors.h> // IR Line Tracker library


// Function Declarations
void checkBattery();
int getBatteryPercentage();
void checkBumper();
void checkFluid();
void checkRowDetector(); 
void calibrateRowDetector(); 
void calibrateLineTracker();
void recallLineTracker();
void tunePID();


// Define digital and analog pins
const int Battery_Pin = A0;
const int RowDetector_Pin = A1; 
const int Bumper_Pin = 33;
const int Fluid_Pin = 27;
const int Status_LED = 25;


// Define program flags
int LowBattery_Flag = 0;
int Collision_Flag = 0;
int LowFluid_Flag = 0;
int RowDetected_Flag = 0;


// Define variables
int Debug_Variable; // Misc variable for debugging sensors
int Program_State = 0; // Program state. Flags cause the program to switch between states
int Bumper_State = 1, Bumper_PrevState = 1; // Bumper switches are normally closed
int Fluid_State = 0, Fluid_PrevState = 0; // Fluid Level switch is normally closed, but kept low by the fluid tank
int RowDetector_Level[] = { 0,0,0,0,0,0 }; // IR sensor has range between 80-500 (10-80 cm)
int RowDetector_Index = 0, RowDetector_Total = 0, RowDetector_Avg = 0, RowDetector_PrevAvg = 0;
int Battery_Level = 1000, Battery_PrevLevel = 1000; // Acceptable battery level is 900-1024 (11-12.5V)
int Line_Position, Correction_Speed, RightMotor_Speed, LeftMotor_Speed;
float Line_Position_Scaled, Error = 0, Error_Prev = 0, Error_Diff = 0, Error_Sum = 0;



// Define constant variables
float Kp = 5;
float Ki = 0;
float Kd = 0;
int Base_Speed = 200;
const int Sample_Time = 50;
const int Line_Position_Max = 6000;


// Define motor driver pins
#define AIN1 42
#define BIN1 46
#define AIN2 40
#define BIN2 48
#define PWMA 3
#define PWMB 2
#define STBY 44

// Motor polarity reversal to account for backwards wiring. Value can be 1 or -1
const int offsetA = 1;
const int offsetB = -1;

// Initializing motors
Motor RightMotor = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor LeftMotor = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

/*
    Notable motor driver information:
    Speed range is -255 to 255
    Motors can be driven with speed and duration: motor1.drive(255,1000);
    Braking function: motor1.brake(); or brake(motor1,motor2);
*/


// Define line tracker variables
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   53     // emitter is controlled by digital pin 53

// sensors 1 through 8 are connected to odd numbered digital pins 51 to 37, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  51, 49, 47, 45, 43, 41, 39, 37
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

void setup() {
  Serial.begin(9600);

  pinMode(Battery_Pin, INPUT);
  pinMode(Bumper_Pin, INPUT);
  pinMode(Fluid_Pin, INPUT);
  pinMode(Status_LED, OUTPUT);

  //calibrateLineTracker();
  recallLineTracker();
  
  digitalWrite(Status_LED, HIGH);
  delay(500);
  digitalWrite(Status_LED, LOW);

}

void loop() {

  // Checking program flags, switching state if required (No time slicing implemented yet)

  checkBumper(); // Check for bumper collisions
  if (Collision_Flag == 1) {
    Serial.println("Collision Detected");
    //Program_State = 0; // If collision occurs, pause all action
    Program_State = ++Program_State % 3;  // Increment, roll over at n-1 (for testing)
    Collision_Flag = 0; // Reset flag after servicing
  }

  //checkRowDetector();
  if (RowDetected_Flag == 1) {
    Program_State = 2;
    RowDetected_Flag = 0;
  }

 //checkBattery(); // Check for low battery warning
  if (LowBattery_Flag == 1) {
    Program_State = 2;
    LowBattery_Flag = 0;
  }

  //checkFluid();
  if(LowFluid_Flag == 1) {
    Program_State = 2;
    LowFluid_Flag = 0;
  }

  if (Program_State == 2) {
    digitalWrite(Status_LED, HIGH);
  } else {
    digitalWrite(Status_LED, LOW);
  }


  switch (Program_State) {

    case 0: // Stationary Robot.

      brake(RightMotor, LeftMotor);

Serial.print("Case 0.   ");
Serial.print(RowDetector_PrevAvg);
Serial.print("   ");
Serial.println(RowDetector_Avg);

     // tunePID();

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
      if (RightMotor_Speed < 0) {
        RightMotor_Speed = 0;
      } else if (RightMotor_Speed > 255) {
        RightMotor_Speed = 255;
      }

      LeftMotor_Speed = Base_Speed + Correction_Speed;
      if (LeftMotor_Speed < 0) {
        LeftMotor_Speed = 0;
      } else if (LeftMotor_Speed > 255) {
        LeftMotor_Speed = 255;
      }
      
Battery_Level = analogRead(Battery_Pin);
if (Battery_Level > 500) { // Only run motors if battery is connected. 
      RightMotor.drive(RightMotor_Speed);
      LeftMotor.drive(LeftMotor_Speed);
} else { // If battery is not connected, just display the specified speed. 
      
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
      if ( Battery_Level < 762 ) {
        Serial.println("%   Battery critically low!");
      }
      else {
        Serial.println("%");
      }

      break;


  }


}

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
      LowFluid_Flag = 1;
    }
  }
}

void checkRowDetector() {

RowDetector_Total = RowDetector_Total - RowDetector_Level[RowDetector_Index]; // Subtract old level
RowDetector_Level[RowDetector_Index] = analogRead(RowDetector_Pin); // Read new level 
RowDetector_Total = RowDetector_Total + RowDetector_Level[RowDetector_Index]; // Add new level
RowDetector_Index = ++ RowDetector_Index % 6; // Increment index and wrap around (0-5)

RowDetector_PrevAvg = RowDetector_Avg - 10; // Store old moving average value (with offset)
RowDetector_Avg = RowDetector_Total / 6; // Calculate new moving average value



if (RowDetector_PrevAvg > RowDetector_Avg) { // If distance measured gets much further away
  RowDetected_Flag = 1;
}

  
}

void calibrateRowDetector() {

  Serial.println("Calibrating Row Detector...");
  delay(500);


 // CODE HERE

  Serial.println("Calibration complete.");
  delay(1000);

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


