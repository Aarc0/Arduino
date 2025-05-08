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
#define BuzzPinn 13
#define C4 262
long cm;

//Variable para función de cambio de sentido
//Según la documentación de ardino que se encuentra en el siguiente enlace: https://www.arduino.cc/reference/cs/language/functions/external-interrupts/attachinterrupt/?_x_tr_hist=true
//La variable tiene que ser declarada como volatil para estar seguros de que las variables compartidas entre un ISR y el programa principal se actualizan correctamente
volatile boolean b = false;

//Variable para que empiece a buscar linea
int line_found = 0;

  //////////////////////////////////////////////////
 ////        FUNCIÓN CAMBIO DE SENTIDO         ////
//////////////////////////////////////////////////
void direction_change()
{
  b = true;
}

int m_distance()
{
  long duration;
  pinMode(pingPin, OUTPUT);  //Hago que el pin se configure como salida 
  digitalWrite(pingPin, LOW); //Hago que el pin pingPin esté en estado bajo
  delayMicroseconds(10); 
  digitalWrite(pingPin, HIGH); //Esto es como un disparo
  delayMicroseconds(10); 
  digitalWrite(pingPin, LOW); //Esto es como si guardamos con lo que disparamos, pero el disparo sigue viajando
  pinMode(pingPin, INPUT); 	//se cambia el pin 5 a entrada, para poder recibir el disparo que devuelve el sensor después de que el pulso rebota en un objeto.
  duration = pulseIn(pingPin, HIGH); //Esta función mide cuánto tiempo permanece el pin en estado HIGH. Es decir, mide cuánto tarda en llegar el eco desde que fue enviado el pulso.
  cm = duration /29 /2;
  Serial.print("CM: ");
  Serial.println(cm);
  return cm;
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
  long startTime = millis();
  int IR_Izquierdo = digitalRead(IR_left);
  int IR_Derecho = digitalRead(IR_right);
  int luz = analogRead(A0);
  int correct;
  int blanco = 1;
  int negro = 0;

  int distance = m_distance();
  if(distance<15)
  {
    tone(BuzzPinn,C4);
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);
    delay(500);

    //PRIMERO GIRA A LA DERECHA
    pwm.setPWM(servo_left,0,SERVOLEFT);
    pwm.setPWM(servo_right,0,SERVOSTOP);
    delay(1200);
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);

    //LUEGO CUANDO ESTÁ MIRANDO A LA DERECHA TIENE QUE IR PARA ADELANTE POR 0.3 SEGUNDOS
    pwm.setPWM(servo_left,0,SERVOLEFT);
    pwm.setPWM(servo_right,0,SERVORIGHT);
    delay(300);
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);

    //LUEGO TIENE QUE GIRAR A LA IZQUIERDA
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVORIGHT);
    delay(1200);
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);

    //MIENTRAS QUE NO DETECTE NEGRO TIENE QUE IR HACIA ADELANTE
    while(digitalRead(IR_left) != negro || digitalRead(IR_right) != negro)
    {
      pwm.setPWM(servo_left,0,320);
      pwm.setPWM(servo_right,0,440);
    }

    //Si se para significa que por uno de los dos sensores leyó negro
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);
    //En los casos de que uno detecte negro y el otro blanco ya se contemplan 
    //fuera de la condición, por eso no hace falta tratarlas aquí dentro

    //Pero aún queda la condición de que los dos detecten negro en cuanto vayan para adelante
    if(digitalRead(IR_left) == negro && digitalRead(IR_right) == negro)
    {
      //La cosa es que quiero que mientras el sensor derecho lea negro, se vaya un poco hacia la derecha
      while(digitalRead(IR_right) != blanco)
      {
        //Hago que vaya lento para no pasarme mucho vaya
        pwm.setPWM(servo_left,0,370);
        pwm.setPWM(servo_right,0,390);
      }

      //En este caso significa que el izquierdo lea negro y el derecho blanco que es lo que quiero
      //Cuando llega a este punto hago que se pare para que luego entre en la primera condición 
      //De los if de afuera para que vaya hacia adelante
      if(digitalRead(IR_left) == negro && digitalRead(IR_right) == blanco)
      {
        pwm.setPWM(servo_left,0,SERVOSTOP);
        pwm.setPWM(servo_right,0,SERVOSTOP);
      }
    }
    noTone(BuzzPinn);
  }
  
  if(digitalRead(IR_left) == blanco && digitalRead(IR_right) == blanco && line_found == 0)
  {
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);
    delay(5000);
    
    /////////////////////////////////////////////////////////////////////
    pwm.setPWM(servo_left,0,SERVOLEFT);
    pwm.setPWM(servo_right,0,SERVOSTOP);
    delay(100);
    if(digitalRead(IR_left)==negro || digitalRead(IR_right)==negro)
    {
      pwm.setPWM(servo_left,0,SERVOSTOP);
      pwm.setPWM(servo_right,0,SERVOSTOP);
    }
    
    pwm.setPWM(servo_left,0,SERVOLEFT);
    pwm.setPWM(servo_right,0,SERVOSTOP);
    delay(100);
    if(digitalRead(IR_left)==negro || digitalRead(IR_right)==negro)
    {
      pwm.setPWM(servo_left,0,SERVOSTOP);
      pwm.setPWM(servo_right,0,SERVOSTOP);
    }
    /////////////////////////////////////////////////////////////////////
    if(digitalRead(IR_left)==negro && digitalRead(IR_right)==negro)
    {
      while(digitalRead(IR_right)!=blanco)
      {
        pwm.setPWM(servo_left,0,370);
        pwm.setPWM(servo_right,0,SERVOSTOP);
      }
      if(digitalRead(IR_left)==negro && digitalRead(IR_right)==blanco)
      {
        pwm.setPWM(servo_left,0,SERVOSTOP);
        pwm.setPWM(servo_right,0,SERVOSTOP);
      }
    }
    line_found++;
  }

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
  long endtime = millis();
  long duration = endtime-startTime;
  Serial.println();
  Serial.print("Duración del loop: ");
  Serial.print(duration);
  Serial.println(" Milisegundos");
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