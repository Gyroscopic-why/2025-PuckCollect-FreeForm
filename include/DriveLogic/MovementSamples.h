#pragma once

#include <Arduino.h>
#include "../Drivers/DcMotor.h"
#include "../Drivers/DistanceSensor.h"
#include "../Utils/ElapsedTime.h"
#include "Devices.h"

#define MAX_SPEED 1.0f

void Drive(float turn)
{
    float encoderTL = motorTL.getCurrentPosition();
    float encoderBL = motorBL.getCurrentPosition();
    //  Get left wheel motors encoders

    float encoderTR = motorTR.getCurrentPosition();
    float encoderBR = motorBR.getCurrentPosition();
    //  Get right wheel motors encoders


    float errorLS = encoderTL - encoderBL;
    float errorRS = encoderTR - encoderBR;
    //  Errors for the right and left sides of the robor wheels
    //  When the motor speed is not perfect and differs


    float maxError = (  max ( abs(errorLS),  abs(errorRS) )  + abs(turn) ) / 2.0f;
    //  Calculate the max error for maintaining the max speed


    //-------------  Apply power  ---------------------------------//
    motorTL.setPower(MAX_SPEED - maxError + (errorLS + turn) / 2.0f);
    motorBL.setPower(MAX_SPEED - maxError - (errorLS + turn) / 2.0f);
    //  Left wheel

    motorTR.setPower(MAX_SPEED - maxError + (errorRS + turn) / 2.0f);
    motorBR.setPower(MAX_SPEED - maxError - (errorRS + turn) / 2.0f);
    //  Right wheel
}