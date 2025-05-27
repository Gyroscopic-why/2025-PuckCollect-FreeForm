#include <Arduino.h>
#include <PRIZM.h>

#include "../include/Devices.h"
#include "../include/DriveLogic/MainDrive.h"


PRIZM prizm;


void setup() 
{
    Serial.begin(115200);
    //  Initialize serial speed
    
    BeginDevices();
    //  Reset devices
    
    StartDrive();
    //  Testing forward drive (5 seconds)
}

void loop() 
{
    
    
}
