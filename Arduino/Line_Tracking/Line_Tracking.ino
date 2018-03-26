/*

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
void calibrateLineTracker();
void storeLineTracker();
void recallLineTracker();


// Define digital and analog pins
const int Battery_Pin = A0;
const int Bumper_Pin = 33;


// Define program flags
int LowBattery_Flag = 0;
int Collision_Flag = 0;


// Define variables
int Debug_Variable; // Misc variable for debugging sensors
int Program_State = 0; // Program state. Flags cause the program to switch between states.
int Bumper_State = 1, Bumper_PrevState = 1; // Bumper switches are normally closed
int Battery_Level;
int Line_Position;
float Line_Position_Scaled;
float Error = 0, Error_Prev = 0, Error_Diff = 0, Error_Sum = 0;
int Base_Speed = 75, Correction_Speed, RightMotor_Speed, LeftMotor_Speed;


// Define constant variables
const float Kp = 0.8;
const float Ki = 0;
const float Kd = 0;
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

  calibrateLineTracker();

}

void loop() {

  // Checking program flags, switching state if required (No time slicing implemented yet)

  checkBumper();
  if (Collision_Flag == 1) {
    //Program_State = 0; // If collision occurs, pause all action
    Program_State = ++Program_State % 2;  // Increment, roll over at n-1
    Collision_Flag = 0; // Reset flag after servicing

  }

  //checkBattery();
  if (LowBattery_Flag == 1) { // If battery is low, pause all action
    Program_State = 3;
    LowBattery_Flag = 0;
  }



  switch (Program_State) {

    case 0: // Stationary Robot. Used for alarm flags, user interaction, and startup.
      //Serial.println("Case 0");
      brake(RightMotor, LeftMotor);

      break;

    case 2:
      //Serial.println("case 1");
      // forward(RightMotor, LeftMotor, 100);
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
      Error_Diff = (Error - Error_Prev) / Sample_Time;  // = (Error-Error_Prev)/(Sample_Time);
      Error_Sum += (Error * Sample_Time);  // += (Error * Sample_Time);

      // Calculate Correction_Speed (bounded between 0-100) using PID
      Correction_Speed = (int)( (Kp * Error + Ki * Error_Sum + Kd * Error_Diff) * 100 );

      Serial.println(Correction_Speed);
      // Set drive motor speeds, constrained to forward motion
      RightMotor_Speed = Base_Speed - Correction_Speed;
      if (RightMotor_Speed < 0) {
        RightMotor_Speed = 0;
      }

      LeftMotor_Speed = Base_Speed + Correction_Speed;
      if (LeftMotor_Speed < 0) {
        LeftMotor_Speed = 0;
      }

      RightMotor.drive(RightMotor_Speed);
      LeftMotor.drive(LeftMotor_Speed);

      break;


    case 3: // Low Battery State. Pause all actions and warn user of low battery

      if ( Battery_Level < 762 ) {
        Serial.println("Battery critically low!!!");
      }
      else Serial.println(getBatteryPercentage());


      break;


  }


}

void checkBattery() {
  Battery_Level = analogRead(Battery_Pin);
  // If voltage drops to 9.3V, ie. Arduino reads 3.72V, set flag
  if (Battery_Level < 762) { // 762/1024*5V = 3.721V
    LowBattery_Flag = 1;
  }
  else {
    LowBattery_Flag = 0;
  }
}

int getBatteryPercentage() {  // Returns the battery level as a percentage of full charge (Rough estimate: not linearized)
  Battery_Level = analogRead(Battery_Pin);
  double Battery_Percentage = ((double) Battery_Level - 762) / (1024 - 762) * 100;
  return (int) Battery_Percentage;
}

void checkBumper() {
  Bumper_PrevState = Bumper_State;
  Bumper_State = digitalRead(Bumper_Pin);

  if (Bumper_PrevState != Bumper_State) { // If switch state changes
    if (Bumper_State == 0) { // Bumper switches are normally closed (Collision = 0)
      // Serial.println("Bumper Pressed")
      Collision_Flag = 1;
    }
    else {
      // Serial.println("Bumper Released");
      // Collision_Flag = 0;
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

  Serial.println();
  Serial.println("Calibration complete.");
  delay(1000);

}

void storeLineTracker() {
  Serial.println();
  Serial.println("Storing Calibration Data into EEPROM...");

  EEPROM.writeBlock<unsigned int>(addrCalibratedMinimumOn, qtrrc.calibratedMinimumOn, 8);
  EEPROM.writeBlock<unsigned int>(addrCalibratedMaximumOn, qtrrc.calibratedMaximumOn, 8);

  Serial.println("EEPROM Storage Complete");
}

void recallLineTracker() {
  Serial.println();
  Serial.println("Recalling Calibration Data from EEPROM...");

  qtrrc.calibrate(); 
  EEPROM.readBlock<unsigned int>(addrCalibratedMinimumOn, qtrrc.calibratedMinimumOn, 8);
  EEPROM.readBlock<unsigned int>(addrCalibratedMaximumOn, qtrrc.calibratedMaximumOn, 8);

  Serial.println("EEPROM Recall Complete");
}


