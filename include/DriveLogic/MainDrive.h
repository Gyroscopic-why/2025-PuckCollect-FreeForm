#pragma once

#include <Arduino.h>
#include "Devices.h"
#include "../Utils/ElapsedTime.h"
#include "../Utils/PDRegulator.h"
#include "../Utils/Queue.h"

#include "MovementSamples.h"
#include "DriveSample.h"

#include "Configs.h"

#define P_STOCK_COEFFICIENT 1.0f
#define D_STOCK_COEFFICIENT 1.0f


Queue<DriveSample *> driveAlgorithm;
Timer mainTimer;



////  TEMPORARY 
static void Drive(float speed, float turnDir, float dirError, float lwError, float rwError)
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



void InitDrive()
{
    mainTimer.Reset();


    PDRegulator<float> rwPD, lwPD, mainPD;
    //  right wheel, left wheel, main robot direction PD regulators for the errors

      rwPD.Reset(P_STOCK_COEFFICIENT, D_STOCK_COEFFICIENT);
      lwPD.Reset(P_STOCK_COEFFICIENT, D_STOCK_COEFFICIENT);
    mainPD.Reset(P_STOCK_COEFFICIENT, D_STOCK_COEFFICIENT);
    


    while (mainTimer.TimeFastSeconds(fastReset) < 5)
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
        
        Drive(MAX_DRIVE_POWER, 0.0f, directionError, leftWheelError, rightWheelError);
      
    }


    //  Main drive logic test placeholder
    //  Desabled whilst unfinished
    if (false)
    {
        if (HAS_GYROSCOPE) TurnResetGyro(45);

        driveAlgorithm.enqueue(new ForwardUntilFrontDistance(mainPD, lwPD, rwPD, 20));

    }
}



void StartDrive()
{
    mainTimer.Reset();

    if ( !driveAlgorithm.isEmpty() )
    {
        driveAlgorithm.front()->Start();
        driveAlgorithm.front()->ResetPd();
    }
}

void UpdateDrive()
{
    //  Executing the current movement (in the 'if' statement), without reinitialising it

    if ( driveAlgorithm.front()->Execute() && (  EXECUTION_LIMIT - mainTimer.TimeAccurateSeconds(dontReset)  ) > 0  )
    {   //  .Execute()  will return true if the movement is finished

        delete driveAlgorithm.frontAndDequeue();

        //  If the are still moves left
        if ( !driveAlgorithm.isEmpty() ) 
        {
            driveAlgorithm.front()->Start();
            driveAlgorithm.front()->ResetPd();
            //  Initialize next movement

            //  -> If the movement is "DriveMS" 
            //     the drive timer will be used (reset once inside the sample)
            //  No need to keep the usage state since .Start() is called only once
        }
    }

}