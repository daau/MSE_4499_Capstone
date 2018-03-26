/*

  MSE 4499 Line Tracking Code for Betta Feeder Robot
  Language: Arduino
  Authors: Michael Hoskins
  Date: March 26 2018

*/

// Libraries
#include <SparkFun_TB6612.h>  // Motor Driver library
#include <QTRSensors.h> // IR Line Tracker library


// Function Declarations
void checkBattery();
void checkBumper();
void calibrateLineTracker();


// Define digital and analog pins
const int Battery_Pin = A0;
const int Bumper_Pin = 33;


// Define program flags
int LowBattery_Flag = 0;
int Collision_Flag = 0;


// Define variables
int Debug_Variable;
int State = 0; // Program state. Flags cause the program to switch between states.
int Bumper_State = 1, Bumper_PrevState = 1; // Bumper switches are normally closed



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
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

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

// sensors 1 through 8 are connected to digital pins 3 through 10, respectively
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


void setup() {
  Serial.begin(9600);

  pinMode(Battery_Pin, INPUT);
  pinMode(Bumper_Pin, INPUT);

  //calibrateLineTracker();

}

void loop() {

  // Checking program flags, switching state if required (No time slicing implemented yet)

  checkBumper();
  if (Collision_Flag == 1) {
    //State = 0; // If collision occurs, pause all action
    State = ++State % 2;  // Increment, roll over at n-1
    delay(500);
  }

  //checkBattery();
  if (LowBattery_Flag == 1) { // If battery is low, pause all action
    State = 0;
  }



  switch (State) {

    case 0: // Stationary Robot. Used for alarm flags, user interaction, and startup.
      // Serial.println("Case 0");
      brake(motor1, motor2);


      break;

    case 1:
      // Serial.println("case 1");
      forward(motor1, motor2, 100); // Drive forward with specified speed


      break;

  }


}

void checkBattery() {
  int batteryLevel = analogRead(Battery_Pin);
  // If voltage drops to 9.3V, ie. Arduino reads 3.72V, set flag
  if (batteryLevel < 762) { // 762/1024*5V = 3.721V
    LowBattery_Flag = 1;
  }
  else {
    LowBattery_Flag = 0;
  }
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
      Collision_Flag = 0;
    }
  }
}


void calibrateLineTracker() {

  delay(500);
  // We are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  // We are through with calibration


  // print the calibration minimum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

}


