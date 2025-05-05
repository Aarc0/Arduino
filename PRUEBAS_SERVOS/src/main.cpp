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

//PULSADOR
#define Buttonpin 11

//Variables globales de corrección
int I;
int D;

//Variables para sensor de proximidad
#define pingPin 5
#define Buzzpin 13
#define C4 262
long cm;

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
  int IR_Izquierdo = digitalRead(IR_left);
  int IR_Derecho = digitalRead(IR_right);
  int luz = analogRead(A0);
  int correct;
  int blanco = 1;
  int negro = 0;
  long duracion;

  if(b)
  {
    pwm.setPWM(servo_left,0,320);
    pwm.setPWM(servo_right,0,360);
    delay(500);
    while(digitalRead(IR_right) == blanco)
    {
      digitalWrite(red_led,LOW);
      digitalWrite(green_led,HIGH);
      pwm.setPWM(servo_left,0,360);
      pwm.setPWM(servo_right,0,360);
    }
    if(digitalRead(IR_right)==negro)
    {
        digitalWrite(red_led,HIGH);
        digitalWrite(green_led,LOW);
        pwm.setPWM(servo_left,0,SERVOSTOP);
        pwm.setPWM(servo_right,0,SERVOSTOP);
        b=false;
    }
  }

  //DISPARADOR DE SONIDO
  //////////////////////////////////////////////////////////
  pinMode(pingPin,OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(10);
  digitalWrite(pingPin,HIGH); //Dispara el sonido
  delayMicroseconds(10);
  digitalWrite(pingPin,LOW); //Cierra el disparador
  pinMode(pingPin,INPUT);
  duracion = pulseIn(pingPin,INPUT);
  cm = duracion /29 /2;
  Serial.println("CM: ");
  Serial.println(cm);
  //////////////////////////////////////////////////////////
  //Con esta parte de código solo mido la distancia al objeto en cm


  
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

  //La condición es que si la distancia al objeto que está en frente es menor a 10 cm, rodee el objeto
  if(cm<10)
  { 
    //Primero quiero que una vez haya visto que el objeto está cerca, se pare
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);

    //Ahora la idea es que gire hacia la derecha (Ya que usualmente a la derecha queda el centro de la pista, así no tiramos al robot por el vacio)
    //Ya que el tiempo de giro varía con la velocidad, creo que es mejor idea que la velocidad de giro sea constante a que cambie dependiendo de la luz
    pwm.setPWM(servo_left,0,320);
    pwm.setPWM(servo_right,0,SERVOSTOP);
    delay(2000);

    //Una vez que el robot ha girado, queremos que vuelva a pararse
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);

    //Luego queremos que avance hacia adelante para que evite el obstáculo, ¿Durante cuanto? Pues más o menos unos 3 segundos calculando que el objeto mide unos 10cm
    pwm.setPWM(servo_left,0,320);
    pwm.setPWM(servo_right,0,440);
    delay(3000);
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);

    //Una vez tenemos el robot mirando hacia el centro, queremos que mire hacia la izquierda
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,440);
    delay(2000);
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);

    //Luego para que evite totalmente al obstáculo y ya quede detrás de él, hacemos que vaya hacia adelante durante 3 segundos
    pwm.setPWM(servo_left,0,320);
    pwm.setPWM(servo_right,0,440);
    delay(4000);
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);

    //Para que mire está vez para afuera de la mesa hacemos que gire a la izquierda
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,440);
    delay(2000);
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);

    while(digitalRead(IR_left) != negro || digitalRead(IR_right) != negro)
    {
      //Con esto le decimos que lea constantemente los valores de los sensores infrarojos
      //Y que mientras uno de ellos no lea negro, siga adelante, una vez uno de ellos lea negro tenemos algunas opciones
      pwm.setPWM(servo_left,0,320);
      pwm.setPWM(servo_right,0,440);
    }
    
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);
    
    //Primero tenemos la opción de que encuentre la linea por los dos sensores, por lo que el robot se quedaría parado sin hacer nada
    //Para ello tenemos que hacer que llegue a la condición de ir hacia adelante
    //La opción de ver la linea por los dos sensores es Negro-Negro
    if(digitalRead(IR_left) == negro && digitalRead(IR_right) == negro)
    {
      //La cosa es que quiero que mientras el sensor derecho lea negro, se vaya un poco hacia la derecha
      while(digitalRead(IR_right) != blanco)
      {
        //Hago que vaya lento para no pasarme mucho vaya
        pwm.setPWM(servo_left,0,370);
        pwm.setPWM(servo_right,0,390);
      }

      if(digitalRead(IR_left) == negro && digitalRead(IR_right) == blanco)
      {
        pwm.setPWM(servo_left,0,SERVOSTOP);
        pwm.setPWM(servo_right,0,SERVOSTOP);
      }
    }
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