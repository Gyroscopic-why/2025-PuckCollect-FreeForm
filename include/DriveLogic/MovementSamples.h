#pragma once

#include <Arduino.h>
#include "../Drivers/DcMotor.h"
#include "../Drivers/DistanceSensor.h"
#include "../Utils/ElapsedTime.h"
#include "Devices.h"

#include "DriveSample.h"


#define MAX_SPEED 0.5f
#define ERROR_VALUE_COEFFICIENT 0.8f
#define DEGREES_TO_DRIVEPOWER_COEFFICIENT 0.01f

Timer driveTimer;

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

        dropProcess();
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

        dropProcess();
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

        dropProcess();
        return true;
    }
};


class DriveMS                   : public DriveSample
{
private:
    float _timeMS;
    Direction _driveDirection;
    
public:
    DriveMS(PDRegulator<float> &PDmain, PDRegulator<float> &PDlw, PDRegulator<float> &PDrw, 
        float timeMS, TimerUsage &driveTimerUsageTemp, Direction driveDirection = forward) : DriveSample(PDmain, PDlw, PDrw)
    {
        //   NEED TO CALIBRATE THIS
        mainReg_p = 1.0f;
        mainReg_d = 1.0f;

        lwReg_p = 1.0f;
        lwReg_d = 1.0;

        rwReg_p = 1.0f;
        rwReg_d = 1.0f;


        _timeMS = timeMS;
        _driveDirection = driveDirection;

        if (!driveTimerUsageTemp) driveTimerUsageTemp = startUse;
        else driveTimerUsageTemp = activeUse;
    }

    bool Execute() override
    { 
        // энкодеры сбрасываются, все норм. ПД тоже сбрасывается

        if (driveTimer.TimeFastMilliseconds(dontReset) < _timeMS)
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

            if (_driveDirection == forward) Drive(MAX_SPEED, 0, directionError, leftWheelError, rightWheelError);
            else Drive(-MAX_SPEED, 0, directionError, leftWheelError, rightWheelError);

            return false;
        }

        dropProcess();
        return true;
    }
};



void DriveAlongWall(PDRegulator<float> &PDreg, float holdDistance, SonarPosition sonarPlace);




void CurveAddEncoder(float degrees, float maxDistanceMM = 0);

void CurveAddGyro();


class TurnAddEncoder : public DriveSample
{
private:
    float _degrees;

public:
    TurnAddEncoder(PDRegulator<float> &PDmain, PDRegulator<float> &PDlw, PDRegulator<float> &PDrw, 
        float &degrees) : DriveSample(PDmain, PDlw, PDrw)
    {
        //   NEED TO CALIBRATE THIS
        mainReg_p = 1.0f;
        mainReg_d = 1.0f;

        lwReg_p = 1.0f;
        lwReg_d = 1.0;

        rwReg_p = 1.0f;
        rwReg_d = 1.0f;


        _degrees = degrees;
         degrees -= MAX_SPEED * MAX_SPEED * DEGREES_TO_DRIVEPOWER_COEFFICIENT;
    }

    bool Execute() override
    { 
        // энкодеры сбрасываются, все норм. ПД тоже сбрасывается

        if ( MAX_SPEED * MAX_SPEED * DEGREES_TO_DRIVEPOWER_COEFFICIENT < abs(_degrees) )
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
            

            _degrees *= DEGREES_TO_DRIVEPOWER_COEFFICIENT;
            if (_degrees > 1.0f) _degrees = 1.0f;
            else if (_degrees < 1.0f) _degrees = -1.0f;
            //  Calculate turn (simplified)

            float CurrentAngle = (WHEEL_DIAMETER * PI) / SINGLE_ENCODER_STEP * (errorTL + errorBL);

            float angle = (Angle / DISTANCE_BETWEEN_WHEELS) / PI * 180;
            angle = chopDegrees(angle);


            float deltaDegrees = chopDegrees(_degrees - angle);
            

            if (abs(error) > ANGLE_ERROR)
            {
                Drive(stop, ROBOT_SPEED * sgn(error));
                return false;
            }


            Drive(MAX_SPEED, _degrees, directionError, leftWheelError, rightWheelError);
            return false;
        }

        dropProcess();
        return true;
    }
};

void TurnAddGyro(float degrees);

void TurnResetGyro(float degrees);



void TurnOnlyWheelsEncoder (float degrees);



void TurnInLineAddEncoder  (float degrees);

void TurnInLineAddGyro     (float degrees);

void TurnInLineResetEncoder(float degrees);

void TurnInLineResetGyro   (float degrees);



void ReturnToBaseEncoder   ();

void ReturnToBaseGyro      ();

void TryReturnToBaseEncoder();

void TryReturnToBaseGyro   ();



void BlockEnemyBase   ();

void TryBlockEnemyBase();



void CalibrateRobotCoordinates ();

void CalibrateRobotRotation    ();

void GetCurrentRobotCoordinates();

void GetCurrentRobotRotation   ();


void UpdateGameMap();

void CalculateBestDirection();

void TransformPredictionsToMovements();



void GetWheelsRotation();