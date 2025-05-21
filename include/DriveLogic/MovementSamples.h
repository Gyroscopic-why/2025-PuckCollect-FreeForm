#pragma once

#include <Arduino.h>
#include "../Drivers/DcMotor.h"
#include "../Drivers/DistanceSensor.h"
#include "../Utils/ElapsedTime.h"
#include "Devices.h"

#define MAX_SPEED 1.0f
#define ERROR_VALUE_KOEFFICIENT 0.8f



void Drive(float speed, float turnDir, float dirError, float lwError, float rwError)
{
    //  speed - max speed for either of the wheels, can be set between -1 and 1 (float)
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
    

    float highestError = max(   max(  abs(tlError), abs(blError)  ),   max(  abs(trError), abs(brError)  )  );
    highestError *= speed > 0 ? 1 : -1;
    //  Calculate the max error for maintaining the max set speed (with the sign)



    //-------------  Apply power  ---------------------------------//

    driveMotorTL.setPower( speed - highestError + tlError );
    driveMotorBL.setPower( speed - highestError + blError );
    //  Left wheel

    driveMotorTR.setPower( speed - highestError + trError );
    driveMotorBR.setPower( speed - highestError + brError );
    //  Right wheel


    //  Note: 
    //  - Even though or robot is using a double swerve
    //    The motors control the movement inverse in reference to the real swerve behaviour:
    //      ^^ Same wheel motors spin in the same direction      (++ or --)  = forward 1 wheel
    //      <> Same wheel motors spin in the opposite direction  (+- or -+)  = turning 1 wheel
}


void DriveForwardAlongWall();

void DriveForwardUntilDistance();

void DriveForwardUntilAnyDistance();

void DriveForwardSeconds();

void TurnInMotionByEncoder();

void TurnInMotionByGyro();

void TurnInPlaceByEncoder();

void TurnInPlaceByGyro();

void DriveAndTurnInLineByEncoder();

void DriveAndTurnInLineByGyro();



void GetCurrentWheelsAngle();

void GetRobotCoordinates();