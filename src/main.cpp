#include <Arduino.h>
#include <PRIZM.h>

PRIZM prizm;


void setup() 
{
  prizm.PrizmBegin();

  prizm.setMotorPower(1, 100);
  
}

void loop() 
{
  
}
