#include <Arduino.h>

#define C4 262
#define Buttonpin 11
#define Buzzpin 13
#define led_rojo 12
#define led_verde 9

volatile bool alarma = false;

void Desactivar_Alarma()
{
    alarma = !alarma;
}

void setup() 
{
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(Buttonpin),Desactivar_Alarma,RISING);
  pinMode(led_rojo,OUTPUT);
  pinMode(led_verde,OUTPUT);
}

void loop() 
{
    interrupts();
    int Nivel_luz = analogRead(A0);
    if(Nivel_luz>400) alarma = !alarma;
    
    if(alarma)
    {
        //Si el nivel de luz es mayor a 400 activa la alarma, led...
        tone(Buzzpin,C4);
        digitalWrite(led_rojo, HIGH);
        digitalWrite(led_verde, LOW);
        delay(100);
        digitalWrite(led_rojo,LOW);
        digitalWrite(led_verde,HIGH);
        delay(100);
    }   
    else 
    {
        digitalWrite(led_verde,HIGH);
        digitalWrite(led_rojo,HIGH);
    }     

}
    


