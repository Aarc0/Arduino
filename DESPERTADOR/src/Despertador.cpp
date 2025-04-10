#include <Arduino.h>

#define C4 262
#define Buttonpin 11
#define Buzzpin 13
#define led_rojo 12
#define led_verde 9

volatile bool alarma = false;

void Desactivar_Alarma()
{
    alarma = false;
    //Esta función vuelve la variable "alarma" en falso si es verdadera, es decir, desactiva la alarma
}

void setup() 
{
  Serial.begin(9600);
  pinMode(led_rojo,OUTPUT);
  pinMode(led_verde,OUTPUT);
  pinMode(Buttonpin, INPUT);
  attachInterrupt(digitalPinToInterrupt(Buttonpin),Desactivar_Alarma,RISING); //Si presiono al boton llamo a la función del medio
}

void loop() 
{
    int Nivel_luz = analogRead(A0);
    Serial.println(Nivel_luz);
    if(Nivel_luz > 120) alarma = true;

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
        //Si quiero desactivar la alarma tengo que presionar el boton (Linea 19)
    }   
    else 
    {
        noTone(Buzzpin);
        digitalWrite(led_verde,HIGH);
        digitalWrite(led_rojo,HIGH);
    }     

}