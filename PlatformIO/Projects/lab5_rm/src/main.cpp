#include <Arduino.h>
#include <VL53L0X.h>
// standard libraries
#include <stdio.h>
#include <time.h>
#include <math.h>
// external libraries
#include <PID_v1.h>
#include <pio_rotary_encoder.h>


// Motor encoder output pulses per 360 degree revolution
#define ENC_COUNT_REV 3840
// initialise static encoder variables for both motors
int RotaryEncoder::rotation_motor_a = 0;

int RotaryEncoder::rotation_motor_b = 0;
// create motor object
RotaryEncoder encoder_left(2, 3, MOTOR_A_SM);
RotaryEncoder encoder_right(26, 27, MOTOR_B_SM);


#define PIN_INPUT 0
#define PIN_OUTPUT 8

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=1, Ki=0.1, Kd=0.2;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  // initialise encoders
  encoder_left.set_rotation(0);
  encoder_right.set_rotation(0);
  SerialUSB.begin(115200);
  delay(1000);

  Input = encoder_left.get_rotation()/ 4;
  Setpoint = Input + 30;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}
void loop() {
// print encode readings in ticks
  SerialUSB.print("enc_left_ticks:= ");
  SerialUSB.println(encoder_left.get_rotation()/ 4);
  SerialUSB.print("enc_right_ticks:= ");
  SerialUSB.println(encoder_right.get_rotation()/ 4);
  
  Input = encoder_left.get_rotation()/ 4;
  Setpoint = Input + 30; 
  myPID.Compute();
  if (Input < 500)
    analogWrite(PIN_OUTPUT, Output);
  else
    analogWrite(PIN_OUTPUT, 0);
  delay(250);
}
