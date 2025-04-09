#include <Arduino.h>

#define pingPin 5
#define Buzzpin 13
#define C4 262


void setup() 
{
  Serial.begin(9600);
}

void loop() 
{
  long duracion, cm;
  pinMode(pingPin, OUTPUT);  
  digitalWrite(pingPin,LOW);
  delayMicroseconds(10);
  digitalWrite(pingPin,HIGH); //Abre el disparador y dispara el sonido
  delayMicroseconds(10);
  digitalWrite(pingPin,LOW); //Cierra el disparador
  pinMode(pingPin, INPUT); //Hacemos que ahora reciba cosas, en este caso va a recibir la distancia a la que está el objeto más cercano  
  duracion = pulseIn(pingPin,HIGH);
  cm = duracion / 29 / 2;
  Serial.print(cm);
  Serial.print(" Cm ");

  if(cm < 20)
  {
    tone(Buzzpin,C4);
  }
  else
  {
    noTone(Buzzpin);
  }
}