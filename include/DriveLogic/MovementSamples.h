#pragma once

#include <Arduino.h>
#include "../Drivers/DcMotor.h"
#include "../Drivers/DistanceSensor.h"
#include "../Utils/ElapsedTime.h"
#include "Devices.h"

#include "DriveSample.h"

#define MAX_SPEED 0.5f
#define ERROR_VALUE_COEFFICIENT 0.8f
#define CM_TO_FLOAT_COEFFICIENT 0.01f


enum SonarPosition
{
    front,

    right,
    left,
    
    back
};

/*
 *   Naming logic here:
 *   - 2 categories of moves
 *         + Navigate using gyroscope
 *         + Navigate using encoder (if the gyro is not present )
 * 
 *   - 2 categories of turning
 *         + Adding turning value to our current rotation
 *         + Reseting our current rotation to set value
 * 
 *   - Turn in Line: turning our robot while moving in a line
 * 
 *     Example:
 *                 FRONT
 *          ___________________
 *         |___________________|
 *         |___________________|
 *      L  |_/`HH`\_____/~HH~\_|  R
 *      E  |_|_HH_|_____|_HH_|_|  I
 *      F  |_\_HH_/_____\_HH_/_|  G
 *      T  |___________________|  H
 *         |___________________|  T
 *         |___________________|   
 *                 BACK            
 * 
 *                   ^
 *                   | 
 * 
 *                 FRONT
 *          ___________________
 *         |______/`HH`\_______|
 *      L  |______|_HH_|_______|  R
 *      E  |______\_HH_/_______|  I
 *      F  |___________________|  G
 *      T  |______/`HH`\_______|  H
 *         |______|_HH_|_______|  T
 *         |______\_HH_/_______|
 *                 BACK            
 *         
 * 
 * 
*/





class BackwardUntilBackDistance : public DriveSample
{
private:
    uint16_t _distanceMM;

public:
    BackwardUntilBackDistance(PDRegulator<float> &PDmain, PDRegulator<float> &PDlw, PDRegulator<float> &PDrw, 
        uint16_t maxDistanceMM) : DriveSample(PDmain, PDlw, PDrw)
    {
        //   NEED TO CALIBRATE THIS
        mainReg_p = 1.0f;
        mainReg_d = 1.0f;

        lwReg_p = 1.0f;
        lwReg_d = 1.0;

        rwReg_p = 1.0f;
        rwReg_d = 1.0f;


        _distanceMM = maxDistanceMM;
    }

    bool Execute() override
    { 
        // энкодеры сбрасываются, все норм. ПД тоже сбрасывается

        if (backSonarFilt.getCurrentValue() > _distanceMM)
        {
            float errorTL = driveMotorTL.getCurrentPosition();
            float errorBL = driveMotorBL.getCurrentPosition();

            float errorTR = driveMotorTR.getCurrentPosition();
            float errorBR = driveMotorBR.getCurrentPosition();
            //  All drive motors errors by encoders


            float leftWheelError  = PDrw  ->  UpdateCorrection(errorTL - errorBL);
            float rightWheelError = PDlw  ->  UpdateCorrection(errorTR - errorBR);
            //  Individual wheel errors

            float directionError  = PDmain -> UpdateCorrection(errorTL + errorBL - errorTR - errorBR);
            //  Main direction of our robot error

            Drive(MAX_SPEED, 0, directionError, leftWheelError, rightWheelError);
            return false;
        }

        return true;
    }
};


class ForwardUntilFrontDistance : public DriveSample
{
private:
    uint16_t _distanceMM;

public:
    ForwardUntilFrontDistance(PDRegulator<float> &PDmain, PDRegulator<float> &PDlw, PDRegulator<float> &PDrw, 
        uint16_t maxDistanceMM) : DriveSample(PDmain, PDlw, PDrw)
    {
        //   NEED TO CALIBRATE THIS
        mainReg_p = 1.0f;
        mainReg_d = 1.0f;

        lwReg_p = 1.0f;
        lwReg_d = 1.0;

        rwReg_p = 1.0f;
        rwReg_d = 1.0f;


        _distanceMM = maxDistanceMM;
    }

    bool Execute() override
    { 
        // энкодеры сбрасываются, все норм. ПД тоже сбрасывается

        if (frontSonarFilt.getCurrentValue() > _distanceMM)
        {
            float errorTL = driveMotorTL.getCurrentPosition();
            float errorBL = driveMotorBL.getCurrentPosition();

            float errorTR = driveMotorTR.getCurrentPosition();
            float errorBR = driveMotorBR.getCurrentPosition();
            //  All drive motors errors by encoders


            float leftWheelError  = PDrw  ->  UpdateCorrection(errorTL - errorBL);
            float rightWheelError = PDlw  ->  UpdateCorrection(errorTR - errorBR);
            //  Individual wheel errors

            float directionError  = PDmain -> UpdateCorrection(errorTL + errorBL - errorTR - errorBR);
            //  Main direction of our robot error

            Drive(MAX_SPEED, 0, directionError, leftWheelError, rightWheelError);
            return false;
        }

        return true;
    }
};


class ForwardUntilAnyDistance   : public DriveSample
{
private:
    uint16_t _distanceMM;

public:
    ForwardUntilAnyDistance(PDRegulator<float> &PDmain, PDRegulator<float> &PDlw, PDRegulator<float> &PDrw, 
        uint16_t maxDistanceMM) : DriveSample(PDmain, PDlw, PDrw)
    {
        //   NEED TO CALIBRATE THIS
        mainReg_p = 1.0f;
        mainReg_d = 1.0f;

        lwReg_p = 1.0f;
        lwReg_d = 1.0;

        rwReg_p = 1.0f;
        rwReg_d = 1.0f;


        _distanceMM = maxDistanceMM;
    }

    bool Execute() override
    {
        // энкодеры сбрасываются, все норм. ПД тоже сбрасывается

        if (frontSonarFilt.getCurrentValue() > _distanceMM
        &&   leftSonarFilt.getCurrentValue() > _distanceMM
        &&  rightSonarFilt.getCurrentValue() > _distanceMM)
        {
            float errorTL = driveMotorTL.getCurrentPosition();
            float errorBL = driveMotorBL.getCurrentPosition();

            float errorTR = driveMotorTR.getCurrentPosition();
            float errorBR = driveMotorBR.getCurrentPosition();
            //  All drive motors errors by encoders


            float leftWheelError  = PDrw  ->  UpdateCorrection(errorTL - errorBL);
            float rightWheelError = PDlw  ->  UpdateCorrection(errorTR - errorBR);
            //  Individual wheel errors

            float directionError  = PDmain -> UpdateCorrection(errorTL + errorBL - errorTR - errorBR);
            //  Main direction of our robot error

            Drive(MAX_SPEED, 0, directionError, leftWheelError, rightWheelError);
            return false;
        }

        return true;
    }
};




void DriveAlongWall(PDRegulator<float> &PDreg, float holdDistance, SonarPosition sonarPlace);

void DriveSeconds(float timeSeconds);



void CurveAddEncoder(float degrees, float maxDistanceMM = 0);

void CurveAddGyro();

void TurnAddEncoder();

void TurnAddGyro(float degrees);

void TurnResetEncoder(float degrees);

void TurnResetGyro(float degrees);



void TurnInLineAddEncoder  (float degrees);

void TurnInLineAddGyro     (float degrees);

void TurnInLineResetEncoder(float degrees);

void TurnInLineResetGyro   (float degrees);



void ReturnToBaseEncoder   ();

void ReturnToBaseGyro      ();

void TryReturnToBaseEncoder();

void TryReturnToBaseGyro   ();



void BlockEnemyBase    ();

void TryBlockEnemyBase();


void GetCurrentWheelsAngle();

void GetRobotCoordinates();