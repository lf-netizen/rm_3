#include <Arduino.h>
#include <VL53L0X.h>
#include <Servo.h>



// #define buzzer 22
int io = 22;
VL53L0X sensor;

Servo servo;  // create servo object to control a servo

int pos ;    // variable to store the servo position



void setup()
{
  //inicjalizacja uarta
  Serial.begin(9600);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  //inicjalizacja czujnika
  sensor.startContinuous();
  //serwomechanizm 
  servo.attach(12);
}

void read_sensor()
{
  Serial.print(sensor.readRangeContinuousMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();


}

void loop()
{
  for (pos = 0; pos <= 180; pos += 1) { // rotate from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(pos);                   // tell servo to go to position in variable 'pos'
    read_sensor();
    delay(20);                          // waits 10ms for the servo to reach the position
  }

  for (pos = 180; pos >= 0; pos -= 1) { // rotate from 180 degrees to 0 degrees
    servo.write(pos);                   // tell servo to go to position in variable 'pos'
    read_sensor();
    delay(20);                          // waits 10ms for the servo to reach the position
  }

}


  