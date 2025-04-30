#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//MOVIMIENTO DE LAS RUEDAS
#define SERVOSTOP 380
#define SERVORIGHT 440
#define SERVOLEFT 320

//RUEDAS
#define servo_left 0
#define servo_right 1

//LEDS
#define red_led 12
#define green_led 9 

//INFRAROJOS
#define IR_left 2
#define IR_right 3

//GIRO POR ANGULOS
#define SERVO_90deg 420
int servo_180 = 2; //El servo está conectado al canal 2

//PULSADOR
#define Buttonpin 11

//Variables globales de corrección
int I;
int D;

//Variable para función de cambio de sentido
//Según la documentación de ardino que se encuentra en el siguiente enlace: https://www.arduino.cc/reference/cs/language/functions/external-interrupts/attachinterrupt/?_x_tr_hist=true
//La variable tiene que ser declarada como volatil para estar seguros de que las variables compartidas entre un ISR y el programa principal se actualizan correctamente
volatile boolean b = false;

  //////////////////////////////////////////////////
 ////        FUNCIÓN CAMBIO DE SENTIDO         ////
//////////////////////////////////////////////////
void direction_change()
{
  b = true;
}

void setup() 
{
  Serial.begin(9600);
  //Variables de corrección

  
  //Infrarrojo izquierdo
  pinMode(IR_left,INPUT);

  //Infrarojo derecho
  pinMode(IR_right,INPUT);
  
 

  //Leds
  pinMode(red_led,OUTPUT);
  pinMode(green_led,OUTPUT);
 
  pwm.begin();
  pwm.setPWMFreq(60);
  
  //Pulsador
  pinMode(Buttonpin, INPUT);
  attachInterrupt(digitalPinToInterrupt(Buttonpin),direction_change,RISING);
}

void loop() 
{
 int blanco = 1;
 int negro = 0;

  while(b)
  {
    digitalWrite(green_led,HIGH);
    pwm.setPWM(servo_left,0,400);
    pwm.setPWM(servo_right,0,SERVOSTOP); 
    if(digitalRead(IR_right)==negro)
    {
      b = false;
      pwm.setPWM(servo_left,0,SERVOSTOP);
      pwm.setPWM(servo_right,0,SERVOSTOP);
      digitalWrite(red_led,HIGH);
      digitalWrite(green_led,LOW);
    }
  }
  
  Serial.print("Estado del booleano: ");
  Serial.println(b);
  int IR_Izquierdo = digitalRead(IR_left);
  int IR_Derecho = digitalRead(IR_right);
  int luz = analogRead(A0);
  int correct;

 

  if(luz < 200)
  {
    correct = 10;
  }
  else if(luz >=200 && luz <= 500)
  {
    correct = 60;
  }
  else if(luz >500)
  {
    correct = 30;
  }
  //Con este bloque imprimo una linea entera con el valor de los IR en dicho momento
  /////////////////////////////////////////////
  Serial.print("IR_Izquierdo: ");
  Serial.print(digitalRead(IR_left));
  Serial.print("      IR_Derecho: ");
  Serial.println(digitalRead(IR_right));
  /////////////////////////////////////////////

  if((IR_Izquierdo == blanco && IR_Derecho == negro) || (IR_Izquierdo == negro && IR_Derecho == blanco))
  {
    pwm.setPWM(servo_left,0,SERVOSTOP - correct);
    pwm.setPWM(servo_right,0,SERVOSTOP + correct);
    I = IR_Izquierdo;
    D = IR_Derecho;
  }
  //Si ambos detectan negro
  if(IR_Izquierdo == negro && IR_Derecho == negro)
  {
    //Si empieza a ver negro y lo último que vió fue negro blanco tiene que irse a la derecha
    if(I == negro && D == blanco)
    {
      pwm.setPWM(servo_left,0,SERVOSTOP-correct);
      pwm.setPWM(servo_right,0,SERVOSTOP);
    }
    if(I == blanco && D == negro)
    {
      pwm.setPWM(servo_left,0,SERVOSTOP);
      pwm.setPWM(servo_right,0,SERVOSTOP+correct);
    }

    I = IR_Izquierdo;
    D = IR_Derecho;
  }
  if(IR_Izquierdo == blanco && IR_Derecho == blanco)
  {
    
    //Si lo último que vió fue blanco, negro tiene que ir a la derecha
    if(I == blanco && D == negro)
    {
      pwm.setPWM(servo_left,0,SERVOSTOP-correct);
      pwm.setPWM(servo_right,0,SERVOSTOP);
    }
    if(I == negro && D == blanco)
    {
      pwm.setPWM(servo_left,0,SERVOSTOP);
      pwm.setPWM(servo_right,0,SERVOSTOP+correct);
    }
    I = IR_Izquierdo;
    D = IR_Derecho;
  }

}

  /*
    Con esta parte de codigo puede ir hacia atras
    pwm.setPWM(servo_left,0,SERVORIGHT);
    pwm.setPWM(servo_right,0,SERVOLEFT);
  */
  
  /*
    Con esta parte de código hago que vaya hacia adelante
    pwm.setPWM(servo_left,0,SERVOLEFT);
    pwm.setPWM(servo_right,0,SERVORIGHT);
  */
  

  /*
    Con esta parte hago que gire hacia la derecha
    pwm.setPWM(servo_left,0,SERVOLEFT);
    pwm.setPWM(servo_right,0,SERVOSTOP);
  */
  
  /*
    Con esta parte hago que gire hacia la izquierda
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVORIGHT);
  */