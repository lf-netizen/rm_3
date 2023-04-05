#include <Arduino.h>
#include <VL53L0X.h>
// standard libraries
#include <stdio.h>
#include <time.h>
#include <math.h>
// external libraries
#include <PID_v1.h>
#include <pio_rotary_encoder.h>
#include <stdlib.h>
// #include <cmath>


#define LOOP_DEL 250
// Motor encoder output pulses per 360 degree revolution
#define ENC_COUNT_REV 3840
// initialise static encoder variables for both motors
int RotaryEncoder::rotation_motor_a = 0;

int RotaryEncoder::rotation_motor_b = 0;
// create motor object
RotaryEncoder encoder_left(2, 3, MOTOR_A_SM);
RotaryEncoder encoder_right(26, 27, MOTOR_B_SM);

#define PIN_RIGHT_BACKWARD 8 // TICKS +
#define PIN_LEFT_FORWARD 10 // TICKS +
#define PIN_RIGHT_FORWARD 9
#define PIN_LEFT_BACKWARD 11

//Define Variables we'll be connecting to

int velocity_l = 3;
int velocity_r = 3;

double Setpoint_l, Input_l, Output_l, v_l;
double Setpoint_r, Input_r, Output_r, v_r;
//Specify the links and initial tuning parameters
double Kp=1, Ki=0.1, Kd=0.2;
PID PID_l(&v_l, &Output_l, &Setpoint_l, Kp, Ki, Kd, DIRECT);
PID PID_r(&v_r, &Output_r, &Setpoint_r, Kp, Ki, Kd, DIRECT);

double to_rad(double enc) {
  return enc / ENC_COUNT_REV * 2 * M_PI;
}
double inv_rad(double rad) {
  return ENC_COUNT_REV / (rad * 2 * M_PI);
}

void setup() {
  // initialise encoders
  encoder_left.set_rotation(0);
  encoder_right.set_rotation(0);
  SerialUSB.begin(115200);
  delay(1000);

  //turn the PID on
  PID_l.SetMode(AUTOMATIC);
  PID_r.SetMode(AUTOMATIC);

  // PID_l.SetOutputLimits(-1.0, 1.0);
  // PID_r.SetOutputLimits(-1.0, 1.0);
  PID_r.SetSampleTime(LOOP_DEL);
  PID_l.SetSampleTime(LOOP_DEL);
}

double Input_l_prev = 0;
double Input_r_prev = 0;
void loop() {
// print encode Input_lreadings in ticks  
  
  Input_l = abs(to_rad(encoder_left.get_rotation()));
  Input_r = abs(to_rad(encoder_right.get_rotation()));

  v_l = (Input_l_prev - Input_l) * 1000.0 / LOOP_DEL;
  v_r = (Input_r_prev - Input_r) * 1000.0 / LOOP_DEL;

  Setpoint_l = abs(velocity_l);
  Setpoint_r = abs(velocity_r);

  PID_l.Compute();
  PID_r.Compute();

  Output_l = inv_rad(Output_l);
  Output_r = inv_rad(Output_r);


  if (velocity_l > 0)
    analogWrite(PIN_LEFT_FORWARD, Output_l);
  else
    analogWrite(PIN_LEFT_BACKWARD, Output_l);

  if (velocity_r > 0)
    analogWrite(PIN_RIGHT_FORWARD, Output_r);
  else
    analogWrite(PIN_RIGHT_BACKWARD, Output_r);

  SerialUSB.print("Rad left motor:");
  SerialUSB.println(Input_l);
  SerialUSB.print("Rad right motor:");
  SerialUSB.println(Input_r);
  
  SerialUSB.print("Rad left vel:");
  SerialUSB.println(v_l);
  SerialUSB.print("Rad right vel:");
  SerialUSB.println(v_r);
  SerialUSB.print("Rad left vel:");
  SerialUSB.println(Output_l);
  SerialUSB.print("Rad right vel:");
  SerialUSB.println(Output_r);
  SerialUSB.print("======================\n");
  // SerialUSB.println(v_r);
  

  // if ( abs(Input_l) < 40 && abs(Input_r) < 40) {
    
    if (velocity_l > 0)
      analogWrite(PIN_LEFT_FORWARD, Output_l);
    else
      analogWrite(PIN_LEFT_BACKWARD, Output_l);

    if (velocity_r > 0)
      analogWrite(PIN_RIGHT_FORWARD, Output_r);
    else
      analogWrite(PIN_RIGHT_BACKWARD, Output_r);
  // }
  // else
  // {
  //   SerialUSB.print("SUCCESS\n");
  //   analogWrite(PIN_RIGHT_FORWARD, 0);
  //   analogWrite(PIN_RIGHT_BACKWARD, 0);
  //   analogWrite(PIN_LEFT_FORWARD, 0);
  //   analogWrite(PIN_LEFT_BACKWARD, 0);
  // }
  Input_l_prev = Input_l;
  Input_r_prev = Input_r;
  delay(LOOP_DEL);
}