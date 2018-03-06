/*

 MSE 4499 Line Tracking Code for Betta Feeder Robot
 Language: Arduino
 Authors: Michael Hoskins
 Date: Jan 30 2018
 
 */
#include <SparkFun_TB6612.h>  // Motor Driver library
#include <QTRSensors.h> // IR Line Tracker library



// Motor Driver Port Pin Definitions
#define AIN1 2
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

// Motor polarity reversal to account for backwards wiring. Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing Motors. 
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

/* 
 *  Notable motor driver information:
 *  Speed range is -255 to 255
 *  Motors can be driven with speed and duration: motor1.drive(255,1000);
 *  Braking function: motor1.brake(); or brake(motor1,motor2);
 */


// Line Tracker Definitions
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

// sensors 1 through 8 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {3, 4, 5, 6, 7, 8, 9, 10},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

/*
 * Notable Line Tracker information:
 * 
 * To get the calibrated line position under the sensor, from 0-5000:
 * unsigned int position = qtrrc.readLine(sensorValues);
 * 
 * To get the raw sensor values: qtrrc.read(sensorValues);
 * To get specific sensor values: sensorValues[i]
 */










void setup() {

Serial.begin(9600);






void loop() {


}

calibrateLineTracker(){
  
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
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


