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

class DriveUntilFrontDistance : public DriveSample
{

private:
    uint16_t _distanceMM;

public:
    DriveUntilFrontDistance(PDRegulator<float> &PDmain, PDRegulator<float> &PDrw, PDRegulator<float> &PDlw, 
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

void DriveAlongWall(PDRegulator<float> &PDreg, float holdDistance, SonarPosition sonarPlace);

void DriveUntilAnyDistance(float maxDistanceMM);

void DriveSeconds(float timeSeconds);

void CurveAddByEncoder(float degrees, float maxDistanceMM = 0);

void CurveAddByGyro();

void TurnAddByEncoder();

void TurnAddByGyro(float degrees);

void CurveResetByEncoder();

void CurveResetByGyro(float degrees);

void TurnResetByEncoder(float degrees);

void TurnResetByGyro(float degrees);

void DriveTurnInLineByEncoder(float degrees);

void DriveTurnInLineByGyro(float degrees);



void GetCurrentWheelsAngle();

void GetRobotCoordinates();