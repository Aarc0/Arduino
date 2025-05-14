#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//WHEEL MOVEMENT
#define SERVOSTOP 380
#define SERVORIGHT 440
#define SERVOLEFT 320

//WHEELS
#define servo_left 0
#define servo_right 1

//LEDS
#define red_led 12
#define green_led 9 

//INFRARED
#define IR_left 2
#define IR_right 3

//BUTTON
#define Buttonpin 11

//GLOBAL VARIABLES OF CORRECTION
int I;
int D;

//VARIABLES FOR THE PROXIMITY SENSOR
#define pingPin 5
#define BuzzPinn 13
#define C4 262
long cm;

//Servo 180
#define servo_45deg 420
int servo_180 = 2;

//Variable for change of direction function
//According to the ardino documentation found at the following link: https://www.arduino.cc/reference/cs/language/functions/external-interrupts/attachinterrupt/?_x_tr_hist=true
//Variable has to be declared as volatile to be sure that the variables shared between an ISR and the main program are updated correctly.
volatile boolean b = false;

//Variable to start searching for line
int line_found = 0;

//CHECK TERRAIN
boolean check = false;

void direction_change()
{
  b = true;
}

void check_straight()
{
  int black = 0;
  pwm.setPWM(servo_left,0,SERVOLEFT);
  pwm.setPWM(servo_right,0,SERVORIGHT);
  delay(100);
  if(digitalRead(IR_left)==black||digitalRead(IR_right)==black)
  {
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);  
    check = true;
  }
  else
  {
    delay(100);
    if(digitalRead(IR_left)==black||digitalRead(IR_right)==black)
    {
      pwm.setPWM(servo_left,0,SERVOSTOP);
      pwm.setPWM(servo_right,0,SERVOSTOP);  
      check = true;
    }
    else
    {
      delay(100);
      if(digitalRead(IR_left)==black||digitalRead(IR_right)==black)
      {
        pwm.setPWM(servo_left,0,SERVOSTOP);
        pwm.setPWM(servo_right,0,SERVOSTOP);  
        check = true;
      }
    }
  }
}

void check_turn_left()
{
  int black = 0;
  pwm.setPWM(servo_left,0,SERVOSTOP);
  pwm.setPWM(servo_right,0,SERVORIGHT);
  delay(100);
  if(digitalRead(IR_left)==black||digitalRead(IR_right)==black)
  {
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);  
    check = true;
  }
  else
  {
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVORIGHT);
    delay(100);
    if(digitalRead(IR_left)==black||digitalRead(IR_right)==black)
    {
      pwm.setPWM(servo_left,0,SERVOSTOP);
      pwm.setPWM(servo_right,0,SERVOSTOP);
      check = true;      
    }
    else
    {
      pwm.setPWM(servo_left,0,SERVOSTOP);
      pwm.setPWM(servo_right,0,SERVORIGHT);
      delay(100);
      if(digitalRead(IR_left)==black||digitalRead(IR_right)==black)
      {
        pwm.setPWM(servo_left,0,SERVOSTOP);
        pwm.setPWM(servo_right,0,SERVOSTOP);  
        check = true;    
      }
      else
      {
        pwm.setPWM(servo_left,0,SERVOSTOP);
        pwm.setPWM(servo_right,0,SERVORIGHT);
        delay(100);
        if(digitalRead(IR_left)==black||digitalRead(IR_right)==black)
        {
          pwm.setPWM(servo_left,0,SERVOSTOP);
          pwm.setPWM(servo_right,0,SERVOSTOP);  
          check = true;    
        }
        else
        {
          pwm.setPWM(servo_left,0,SERVOSTOP);
          pwm.setPWM(servo_right,0,SERVORIGHT);
          delay(100);
          if(digitalRead(IR_left)==black||digitalRead(IR_right)==black)
          {
            pwm.setPWM(servo_left,0,SERVOSTOP);
            pwm.setPWM(servo_right,0,SERVOSTOP); 
            check = true;     
          }
          else
          {
            pwm.setPWM(servo_left,0,SERVOSTOP);
            pwm.setPWM(servo_right,0,SERVORIGHT);
            delay(100);
            if(digitalRead(IR_left)==black||digitalRead(IR_right)==black)
            {
              pwm.setPWM(servo_left,0,SERVOSTOP);
              pwm.setPWM(servo_right,0,SERVOSTOP); 
              check = true;     
            }
            else
            {
              pwm.setPWM(servo_left,0,SERVOSTOP);
              pwm.setPWM(servo_right,0,SERVORIGHT);
              delay(100);
              if(digitalRead(IR_left)==black||digitalRead(IR_right)==black)
              {
                pwm.setPWM(servo_left,0,SERVOSTOP);
                pwm.setPWM(servo_right,0,SERVOSTOP);   
                check = true;   
              }
              else
              {
                pwm.setPWM(servo_left,0,SERVOSTOP);
                pwm.setPWM(servo_right,0,SERVORIGHT);
                delay(100);
                if(digitalRead(IR_left)==black||digitalRead(IR_right)==black)
                {
                  pwm.setPWM(servo_left,0,SERVOSTOP);
                  pwm.setPWM(servo_right,0,SERVOSTOP);
                  check = true;      
                }
                else
                {
                  pwm.setPWM(servo_left,0,SERVOSTOP);
                  pwm.setPWM(servo_right,0,SERVORIGHT);
                  delay(100);
                  if(digitalRead(IR_left)==black||digitalRead(IR_right)==black)
                  {
                    pwm.setPWM(servo_left,0,SERVOSTOP);
                    pwm.setPWM(servo_right,0,SERVOSTOP);  
                    check = true;    
                  }
                  else
                  {
                    pwm.setPWM(servo_left,0,SERVOSTOP);
                    pwm.setPWM(servo_right,0,SERVORIGHT);
                    delay(100);
                    if(digitalRead(IR_left)==black||digitalRead(IR_right)==black)
                    {
                      pwm.setPWM(servo_left,0,SERVOSTOP);
                      pwm.setPWM(servo_right,0,SERVOSTOP);  
                      check = true;    
                    }
                    else
                    {
                      pwm.setPWM(servo_left,0,SERVOSTOP);
                      pwm.setPWM(servo_right,0,SERVORIGHT);
                      delay(100);
                      if(digitalRead(IR_left)==black||digitalRead(IR_right)==black)
                      {
                        pwm.setPWM(servo_left,0,SERVOSTOP);
                        pwm.setPWM(servo_right,0,SERVOSTOP); 
                        check = true;     
                      }
                      else
                      {
                        pwm.setPWM(servo_left,0,SERVOSTOP);
                        pwm.setPWM(servo_right,0,SERVORIGHT);
                        delay(100);
                        if(digitalRead(IR_left)==black||digitalRead(IR_right)==black)
                        {
                          pwm.setPWM(servo_left,0,SERVOSTOP);
                          pwm.setPWM(servo_right,0,SERVOSTOP);  
                          check = true;    
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}

int m_distance()
{
  long duration;
  pinMode(pingPin, OUTPUT);  //Make pin configured as output 
  digitalWrite(pingPin, LOW); //I make the pingPin pin in low state
  delayMicroseconds(10); 
  digitalWrite(pingPin, HIGH); //This is like a gunshot
  delayMicroseconds(10); 
  digitalWrite(pingPin, LOW); //This is as if we save with what we shot, but the shot still travels.
  pinMode(pingPin, INPUT); 	//pin 5 is changed to input, in order to receive the trigger returned by the sensor after the pulse bounces off an object.
  duration = pulseIn(pingPin, HIGH); //This function measures how long the pin remains in HIGH state. That is, it measures how long it takes for the echo to arrive since the pulse was sent.
  cm = duration /29 /2;
  Serial.print("CM: ");
  Serial.println(cm);
  return cm;
}

void setup() 
{
  Serial.begin(9600);

  
  //LEFT INFRARED
  pinMode(IR_left,INPUT);

  //RIGHT INFRARED
  pinMode(IR_right,INPUT);
  
  //Leds
  pinMode(red_led,OUTPUT);
  pinMode(green_led,OUTPUT);
 
  pwm.begin();
  pwm.setPWMFreq(60);
  
  //BUTTON
  pinMode(Buttonpin, INPUT);
  attachInterrupt(digitalPinToInterrupt(Buttonpin),direction_change,RISING);
}

void loop() 
{
  pwm.setPWM(servo_180,0,servo_45deg);
  long startTime = millis();
  int IR_Izquierdo = digitalRead(IR_left);
  int IR_Derecho = digitalRead(IR_right);
  int light = analogRead(A0);
  int correct;
  int white = 1;
  int black = 0;

  int distance = m_distance();
  if(distance<=10)
  {
    check = false;
    tone(BuzzPinn,C4);
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);
    delay(500);

    //FIRST TURN RIGHT
    pwm.setPWM(servo_left,0,SERVOLEFT);
    pwm.setPWM(servo_right,0,SERVOSTOP);
    delay(1200);
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);

    //THEN WHEN YOU ARE LOOKING TO THE RIGHT YOU HAVE TO GO FORWARD FOR 0.2 SECONDS
    check_straight();
    if(check ==false)
    {
      pwm.setPWM(servo_left,0,SERVOSTOP);
      pwm.setPWM(servo_right,0,SERVOSTOP);
      check_turn_left();
      if(check == false)
      {
        pwm.setPWM(servo_left,0,SERVOSTOP);
        pwm.setPWM(servo_right,0,SERVOSTOP);
        check_straight();
        if(check == false)
        {
          pwm.setPWM(servo_left,0,SERVOSTOP);
          pwm.setPWM(servo_right,0,SERVOSTOP);
          check_turn_left();
          if(check==false)
          {
            while(digitalRead(IR_left)==white && digitalRead(IR_right)==white)
            {
              pwm.setPWM(servo_left,0,SERVOLEFT);
              pwm.setPWM(servo_right,0,SERVORIGHT);
            }
              pwm.setPWM(servo_left,0,SERVOSTOP);
              pwm.setPWM(servo_right,0,SERVOSTOP);
          }
        }
      }
    }
    
    

    //But there is still the condition that both detect black as soon as they go forward.
    if(digitalRead(IR_left) == black && digitalRead(IR_right) == black)
    {
      //The thing is that I want that while the right sensor reads black, it goes a little to the right.
      while(digitalRead(IR_right) != white)
      {
        //I make it go slow so that I don't go too far.
        pwm.setPWM(servo_left,0,360);
        pwm.setPWM(servo_right,0,SERVOSTOP);
      }

      //In this case it means that the left reads black and the right reads white which is what I want.
      //When it gets to this point I make it stop so that it then enters the first condition. 
      //Of the if outside so that it goes forward
      if(digitalRead(IR_left) == black && digitalRead(IR_right) == white)
      {
        pwm.setPWM(servo_left,0,SERVOSTOP);
        pwm.setPWM(servo_right,0,SERVOSTOP);
      }
    }
    noTone(BuzzPinn);
  }
  
  if(digitalRead(IR_left) == white && digitalRead(IR_right) == white && line_found == 0)
  {
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVOSTOP);
    delay(5000);
    
    /////////////////////////////////////////////////////////////////////
    pwm.setPWM(servo_left,0,SERVOLEFT);
    pwm.setPWM(servo_right,0,SERVOSTOP);
    delay(100);
    if(digitalRead(IR_left)==black || digitalRead(IR_right)==black)
    {
      pwm.setPWM(servo_left,0,SERVOSTOP);
      pwm.setPWM(servo_right,0,SERVOSTOP);
    }
    
    pwm.setPWM(servo_left,0,SERVOLEFT);
    pwm.setPWM(servo_right,0,SERVOSTOP);
    delay(100);
    if(digitalRead(IR_left)==black || digitalRead(IR_right)==black)
    {
      pwm.setPWM(servo_left,0,SERVOSTOP);
      pwm.setPWM(servo_right,0,SERVOSTOP);
    }
    /////////////////////////////////////////////////////////////////////
    if(digitalRead(IR_left)==black && digitalRead(IR_right)==black)
    {
      while(digitalRead(IR_right)!=white)
      {
        pwm.setPWM(servo_left,0,370);
        pwm.setPWM(servo_right,0,SERVOSTOP);
      }
      if(digitalRead(IR_left)==black && digitalRead(IR_right)==white)
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
    while(digitalRead(IR_right) == white)
    {
      digitalWrite(red_led,LOW);
      digitalWrite(green_led,HIGH);
      pwm.setPWM(servo_left,0,360);
      pwm.setPWM(servo_right,0,360);
    }
    if(digitalRead(IR_right)==black)
    {
        digitalWrite(red_led,HIGH);
        digitalWrite(green_led,LOW);
        pwm.setPWM(servo_left,0,SERVOSTOP);
        pwm.setPWM(servo_right,0,SERVOSTOP);
        b=false;
    }
  }
  
  if(light < 200)
  {
    correct = 20;
  }
  else if(light >=200 && light <= 500)
  {
    correct = 60;
  }
  else if(light >500)
  {
    correct = 40;
  }
  //With this block I print a whole line with the value of the IR at that moment.
  /////////////////////////////////////////////
  Serial.print("IR_Izquierdo: ");
  Serial.print(digitalRead(IR_left));
  Serial.print("      IR_Derecho: ");
  Serial.println(digitalRead(IR_right));
  /////////////////////////////////////////////

  if((IR_Izquierdo == white && IR_Derecho == black) || (IR_Izquierdo == black && IR_Derecho == white))
  {
    pwm.setPWM(servo_left,0,SERVOSTOP - correct);
    pwm.setPWM(servo_right,0,SERVOSTOP + correct);
    I = IR_Izquierdo;
    D = IR_Derecho;
  }
  
  //If both detect black
  if(IR_Izquierdo == black && IR_Derecho == black)
  {
    //If you start seeing black and the last thing you saw was black white you have to go to the right.
    if(I == black && D == white)
    {
      pwm.setPWM(servo_left,0,SERVOSTOP-correct);
      pwm.setPWM(servo_right,0,SERVOSTOP);
    }
    if(I == white && D == black)
    {
      pwm.setPWM(servo_left,0,SERVOSTOP);
      pwm.setPWM(servo_right,0,SERVOSTOP+correct);
    }

    I = IR_Izquierdo;
    D = IR_Derecho;
  }
  if(IR_Izquierdo == white && IR_Derecho == white)
  {
    //If the last thing you saw was white, black has to go to the right.
    if(I == white && D == black)
    {
      pwm.setPWM(servo_left,0,SERVOSTOP-correct);
      pwm.setPWM(servo_right,0,SERVOSTOP);
    }
    if(I == black && D == white)
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
    With this part of the code you can go backwards
    pwm.setPWM(servo_left,0,SERVORIGHT);
    pwm.setPWM(servo_right,0,SERVOLEFT);
  */
  
  /*
    With this part of code I make it go forward
    pwm.setPWM(servo_left,0,SERVOLEFT);
    pwm.setPWM(servo_right,0,SERVORIGHT);
  */
  

  /*
    With this part I make it turn clockwise
    pwm.setPWM(servo_left,0,SERVOLEFT);
    pwm.setPWM(servo_right,0,SERVOSTOP);
  */
  
  /*
    With this part I make it rotate to the left
    pwm.setPWM(servo_left,0,SERVOSTOP);
    pwm.setPWM(servo_right,0,SERVORIGHT);
  */