// Do not remove the include below
#include "AdversaryLaserDetection.h"

#include <AFMotor.h>

AF_DCMotor motor(4);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");


  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(13, OUTPUT);

  // turn on motor
  motor.setSpeed(200);

  motor.run(RELEASE);
}

void loop() {
  uint8_t i;
 int j=0;
  //Serial.print("tick");

/*
  motor.run(FORWARD);
  for (i=0; i<255; i++) {
    motor.setSpeed(i);
    Serial.println(i);
    delay(50);
 }
  delay(1000);
  for (i=255; i!=0; i--) {
    motor.setSpeed(i);
    Serial.println(i);
    delay(50);
 }
  delay(1000);
  Serial.print("tock");

  motor.run(BACKWARD);
  for (i=0; i<255; i++) {
    motor.setSpeed(i);
    Serial.println(i);
    delay(50);
 }
  delay(1000);
  for (i=255; i!=0; i--) {
    motor.setSpeed(i);
    Serial.println(i);
    delay(50);
 }


  Serial.print("tech");
  motor.run(RELEASE);
  delay(2000);
*/

  motor.run(FORWARD);
    motor.setSpeed(75);

    int sensorVal2 = digitalRead(2);
    int sensorVal3 = digitalRead(3);
    Serial.print("sensorVal2= ");
    Serial.print(sensorVal2);
    Serial.print("  sensorVal3= ");
    Serial.println(sensorVal3);

    if (sensorVal2 == HIGH) {
        digitalWrite(13, LOW);
      } else {
        digitalWrite(13, HIGH);
      }
}
