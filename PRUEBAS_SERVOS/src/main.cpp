#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//MOVIMIENTO DE LAS RUEDAS
#define SERVOSTOP 380
#define SERVODERECHA 390
#define SERVOIZQUIERDA 370

//RUEDAS
#define servo_left 0
#define servo_right 1

//INFRAROJOS
#define IR_left 2
#define IR_right 3

//GIRO POR ANGULOS
#define SERVO_90GRADOS 420
int servo_180 = 2; //El servo está conectado al canal 2

  //////////////////////////////////////////////////
 ////        INICIO DECLARACIÓN FUNCIONES      ////
//////////////////////////////////////////////////
void Correcion(int I, int D)
{
  if(I == 1 && D == 0)
  {
    //Con esto le digo que si el izquierdo ve blanco se vaya un poco a la derecha
    pwm.setPWM(servo_left,0,360);
    pwm.setPWM(servo_right,0,SERVOSTOP);
  }
  else if(I == 0 && D == 1)
  {
    //Con esto le digo que si el izquierdo ve blanco se vaya un poco a la derecha
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,400);
  }
}
  //////////////////////////////////////////////////
 ////        FIN DECLARACIÓN FUNCIONES         ////
//////////////////////////////////////////////////

void setup() 
{
  pinMode(IR_left,INPUT);
  pinMode(IR_right,INPUT);
  pwm.begin();
  pwm.setPWMFreq(60);
  Serial.begin(9600);

  pwm.setPWM(servo_left,0,SERVOSTOP);
  pwm.setPWM(servo_right,0,SERVOSTOP);

}

void loop() 
{
  int IR_Izquierdo = digitalRead(IR_left);
  int IR_Derecho = digitalRead(IR_right);

  /*
    Variables para empezar la correción,
    las variables almacenan lo último que vio 
  */
  int I;
  int D;
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

  /*
    Con esta parte de codigo puede ir hacia atras
    pwm.setPWM(servo_left,0,SERVODERECHA);
    pwm.setPWM(servo_right,0,SERVOIZQUIERDA);
  */
  
  /*
    Con esta parte de código hago que vaya hacia adelante
    pwm.setPWM(servo_left,0,SERVOIZQUIERDA);
    pwm.setPWM(servo_right,0,SERVODERECHA);
  */
  

  /*
    Con esta parte hago que gire hacia la derecha
    pwm.setPWM(servo_left,0,SERVOIZQUIERDA);
    pwm.setPWM(servo_right,0,SERVOSTOP);
  */
  
  /*
    Con esta parte hago que gire hacia la izquierda
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVODERECHA);
  */

  //Con esto hago que siempre vaya hacia adelante
  pwm.setPWM(servo_left,0,SERVOIZQUIERDA);
  pwm.setPWM(servo_right,0,SERVODERECHA);
  
  if(IR_Izquierdo == 1 && IR_Derecho == 0)
  {
    //Con esto le digo que si el izquierdo ve blanco se vaya un poco a la derecha
    pwm.setPWM(servo_left,0,360);
    pwm.setPWM(servo_right,0,SERVOSTOP);
    
    I = IR_Izquierdo;
    D = IR_Derecho;
  }
  else if(IR_Izquierdo == 0 && IR_Derecho == 1)
  {
    //Con esto le digo que si el izquierdo ve blanco se vaya un poco a la derecha
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,400);

    I = IR_Izquierdo;
    D = IR_Derecho;
  }

  //Si detecta blanco en ambos, se va a ir a la función de corrección
  else if(IR_Izquierdo == 1 && IR_Derecho == 1)
  {
    Correcion(I,D);
  } 
  
}

