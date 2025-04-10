#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOSTOP 380
#define SERVODERECHA 440
#define SERVOIZQUIERDA 320

#define servo_left 0
#define servo_right 1

void setup() 
{
  pwm.begin();
  pwm.setPWMFreq(60);
}

void loop() 
{
  pwm.setPWM(servo_left,0,SERVOSTOP);
  pwm.setPWM(servo_right,0,SERVOSTOP);
}


