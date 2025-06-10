/*
#--------------------------------------
#
#   IDPA Rollende-Kugel
#   Autor: MuedeHydra
#   Datum: 23.01.2024
#   Version: 0.2
#
#--------------------------------------
*/

#include <Arduino.h>

const int m1_pin = 4;   //D2
const int m2_pin = 14;  //D5
const int m3_pin = 12;  //D6

int Eingabe = '0';
byte var = 0;
int m1 = 0;
int m2 = 0;
int m3 = 0;

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(2);

  pinMode(m1_pin, OUTPUT);
  pinMode(m2_pin, OUTPUT);
  pinMode(m3_pin, OUTPUT);
  
  analogWriteFreq(330);

  analogWrite(m2_pin, 128);
  analogWrite(m3_pin, 128);
  analogWrite(m1_pin, 128);
}

int kontrolle(int m)
{
  if (m < 80){return(80);}
  else if (m > 175){return(175);}
  else {return(m);}
}

void loop() {

  if (Serial.available() > 0) {

    Eingabe = (Serial.parseInt());
    //Serial.print("Eingabe "); Serial.println(Eingabe);

    if (Eingabe == 500)
    {
      var++;
    }
    else if (var == 1)
    {
      m1 = Eingabe;
      var++;
    }
    else if (var == 2)
    {
      m2 = Eingabe;
      var++;
    }
    else if (var == 3)  
    {
      m3 = Eingabe;
      var = 0;
      analogWrite(m1_pin, kontrolle(m1));
      analogWrite(m2_pin, kontrolle(m2));
      analogWrite(m3_pin, kontrolle(m3));

      Serial.print(m1);
      Serial.print(" : ");
      Serial.print(m2);
      Serial.print(" : ");
      Serial.println(m3);
    }    
  }
}
