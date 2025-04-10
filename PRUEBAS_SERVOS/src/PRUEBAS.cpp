#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOSTOP 380
#define SERVODERECHA 440
#define SERVOIZQUIERDA 320

#define servo_left 0
#define servo_right 1
#define IR_left 2
#define IR_right 3

void setup() 
{
  pwm.begin();
  pwm.setPWMFreq(60);
  pinMode(IR_left,INPUT);
  pinMode(IR_right,INPUT);
  Serial.begin(9600);
}

void loop() 
{
  /*
  Blanco = 1
  Negro = 0
  */
  //Con este bloque imprimo una linea entera con el valor de los IR en dicho momento
  /////////////////////////////////////////////
  Serial.print("IR_Izquierdo: ");
  Serial.print(digitalRead(IR_left));
  Serial.print("      IR_Derecho: ");
  Serial.println(digitalRead(IR_right));
  /////////////////////////////////////////////

  //if(IR_left == 1)
  //{
    pwm.setPWM(servo_left,0,SERVOIZQUIERDA);
    pwm.setPWM(servo_right,0,SERVODERECHA);
  //}
  /*if(IR_left == 0)
  {
    Serial.println("Segundo");
    pwm.setPWM(servo_left,0,SERVOIZQUIERDA);
    pwm.setPWM(servo_right,0,SERVODERECHA);
  }*/ 
 
}