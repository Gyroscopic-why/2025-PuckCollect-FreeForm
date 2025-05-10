#pragma once

#include <Arduino.h>
#include "Devices.h"
#include "../Utils/ElapsedTime.h"
#include "MovementSamples.h"



void StartDrive()
{
    Timer timer;
    timer.Reset();


    while (timer.TimeSeconds(false) < 5)
    {
        Drive(0);

        
    }
}