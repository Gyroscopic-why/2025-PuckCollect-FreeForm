#pragma once

#include <Arduino.h>
#include "../Drivers/DcMotor.h"
#include "../Drivers/DistanceSensor.h"
#include "../Utils/ElapsedTime.h"
#include "Devices.h"

#define MAX_SPEED 1.0f
#define ERROR_VALUE_KOEFFICIENT 0.8f


enum wheelType
{
    left = true,
    right = false
};


void DriveSample(float turnDir, float dirError, float lwError, float rwError)
{
    //  turnDir  = Turn  in    direction
    //  dirError = error for   the current robot direction
    //  lwError  = left  wheel error
    //  rwError  = right wheel error


    float tlError = (  lwError + dirError + turnDir )  /  2.0f;
    float blError = ( -lwError + dirError - turnDir )  /  2.0f;
    //  ---  Top Left and Bottom Left motor errors  ---  //
    //  For the drifting of a single wheel 
    //  (proposed to the straight moving of the other wheel)

    float trError = (  rwError - dirError + turnDir )  /  2.0f;
    float brError = ( -rwError - dirError - turnDir )  /  2.0f;
    //  ---  Top Right and Bottom Right motor errors  ---  //
    //  For the drifting of a single wheel 
    //  (proposed to the straight moving of the other wheel)


    
    float maxError = max(  max(tlError, blError),  max(trError, brError)  );
    //  Calculate the max error for maintaining the max speed



    //-------------  Apply power  ---------------------------------//

    driveMotorTL.setPower( MAX_SPEED - maxError + tlError );
    driveMotorBL.setPower( MAX_SPEED - maxError + blError );
    //  Left wheel

    driveMotorTR.setPower( MAX_SPEED - maxError + trError );
    driveMotorBR.setPower( MAX_SPEED - maxError + brError );
    //  Right wheel


    //  Note: 
    //  - Even though or robot is using a double swerve
    //    The motors control the movement inverse in reference to the real swerve behaviour:
    //      ^^ Same wheel motors spin in the same direction      (++ or --)  = forward 1 wheel
    //      <> Same wheel motors spin in the opposite direction  (+- or -+)  = turning 1 wheel
}


float GetDriveWheelError(wheelType driveWheel)
{
    float error;

    if (driveWheel)  //  If the left wheel is selected for the error calculation
    {
        float encoderTL = driveMotorTL.getCurrentPosition();
        float encoderBL = driveMotorBL.getCurrentPosition();
        //  Get left wheel motors encoders

        error = encoderTL - encoderBL;
    }

    else             //  Right wheel is selected
    {
        float encoderTR = driveMotorTR.getCurrentPosition();
        float encoderBR = driveMotorBR.getCurrentPosition();
        //  Get right wheel motors encoders

        error = encoderTR - encoderBR;
    }

    return error;
}

