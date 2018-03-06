/**********************************************************
   Pump and Drive Motor Test for Betta Fish Feeder
   Language: Arduino
   Author: Michael Hoskins
   Date: March 5 2018

   This code uses a potentiometer and switch to control the pump
   and the two drive motors. It uses the potentiometer to set a
   shared PWM duty cycle, and the switch to change which actuator
   is powered.

   Reducing CPU Load:
   The checkSwitch() and checkPot() functions will
   read the digital and analog pins respectively, and only return
   1 if there is a detected change in state. In this way, by calling
   a checkX() function in an if statement, the CPU will only perform
   the action of checking sensors every loop, while the resulting
   changes to the actuators (such as changing the motor speed)
   only happen as needed.

 **********************************************************/

#include "SparkFun_TB6612.h"

// Pins for all motor driver inputs, keep in mind the PWM defines must be on PWM pins
#define AIN1 2
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


// Pin Definitions
int pot_pin = A0;
int pump_pin = 11;
int switch_pin = 13;

// Sensor Variables
int pin_state = 0;
int prev_pin_state;
int sensor_value = 0;
int prev_sensor_value;

// Other Variables
float pwm_ratio = 0.249; //  = 255/1024

// Functions
int checkSwitch();
int checkPot();

void setup()
{
  Serial.begin(9600);
  pinMode(switch_pin, INPUT);
  pinMode(pump_pin, OUTPUT);
}

void loop() {

  delay(5);

  if (checkSwitch() || checkPot()) {


    if (pin_state == 1) { // Drive Motor Activated

      analogWrite(pump_pin, 0); // Deactivate Pump

      int drive_pwm = (int)(sensor_value * pwm_ratio);

      if (sensor_value <= 25) { // Switch off motors if knob is almost fully dialed left
        brake(motor1, motor2);
      }
      else forward(motor1, motor2, drive_pwm);
    }

    else { // Pump Activated

      brake(motor1, motor2); // Deactivate Drive Motors

      int pump_pwm = (int)(sensor_value * pwm_ratio);

      if (sensor_value <= 25) {
        analogWrite(pump_pin, 0);
      }
      else analogWrite(pump_pin, pump_pwm);
    }

  }
}



int checkSwitch() {  // Reads the digital switch state. Returns 1 if the state has changed.

  prev_pin_state = pin_state;
  pin_state = digitalRead(switch_pin);

  if (prev_pin_state != pin_state) {

    if (pin_state == HIGH) {
      Serial.println("Switch state HIGH - Motors activated. ");
    }
    else {
      Serial.println("Switch state LOW - Pump activated.");
    }

    return 1;
  }
  else return 0;
}

int checkPot() { // Reads the analog potentiometer value. Returns 1 if the value has changed.

  prev_sensor_value = sensor_value;
  sensor_value = analogRead(pot_pin);

  if (prev_sensor_value > (sensor_value + 1) || prev_sensor_value < (sensor_value - 1)) { // +/- 1 threshold to de-noise.

    Serial.print("Potentiometer changed to ");
    Serial.print(sensor_value);
    Serial.print(", PWM duty cycle changed to ");
    Serial.println((float)(sensor_value) / 1024);

    return 1;
  }
  else return 0;
}



