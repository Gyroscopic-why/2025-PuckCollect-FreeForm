#pragma once

#include <Arduino.h>
#include "Devices.h"
#include "../Utils/ElapsedTime.h"
#include "../Utils/PDRegulator.h"
#include "MovementSamples.h"

#define P_STOCK_COEFFICIENT 0.8f
#define D_STOCK_COEFFICIENT 2.0f

void StartDrive()
{
    Timer timer;
    timer.Reset();


    PDRegulator rwPD, lwPD, mainPD;
    //  right wheel, left wheel, main robot direction PD regulators for the errors

      rwPD.Reset(P_STOCK_COEFFICIENT, D_STOCK_COEFFICIENT);
      lwPD.Reset(P_STOCK_COEFFICIENT, D_STOCK_COEFFICIENT);
    mainPD.Reset(P_STOCK_COEFFICIENT, D_STOCK_COEFFICIENT);
    


    while (timer.TimeSeconds(false) < 5)
    {
        float errorTL = driveMotorTL.getCurrentPosition();
        float errorBL = driveMotorBL.getCurrentPosition();

        float errorTR = driveMotorTR.getCurrentPosition();
        float errorBR = driveMotorBR.getCurrentPosition();
        //  All drive motors errors by encoders


        float leftWheelError  =   rwPD.UpdateCorrection(errorTL - errorBL);
        float rightWheelError =   lwPD.UpdateCorrection(errorTR - errorBR);
        //  Individual wheel errors

        float directionError  = mainPD.UpdateCorrection(errorTL + errorBL - errorTR - errorBR);
        //  Main direction of our robot error
        
        DriveSample(0, directionError, leftWheelError, rightWheelError);
        
        
    }
}