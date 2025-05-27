#pragma once

#include <Arduino.h>
#include "../Drivers/DcMotor.h"
#include "../Drivers/DistanceSensor.h"
#include "../Utils/ElapsedTime.h"
#include "Devices.h"

#include "DriveSample.h"

#define MAX_SPEED 0.5f
#define ERROR_VALUE_KOEFFICIENT 0.8f


enum SonarPosition
{
    front,

    right,
    left,
    
    back
};

class DriveUntilFrontDistance : public DriveSample
{
    uint8_t _distance;

public:
    DriveUntilFrontDistance(PDRegulator<float> &PDreg, uint8_t maxDistanceCM) : DriveSample(PDreg)
    {
        _distance = maxDistanceCM;
        coef_p = 1.0f; // нужны норм каэфициенты!
        coef_d = 1.0f;
    }

    bool Execute() override
    { 
        // энкодеры сбрасываются, все норм. ПД тоже сбрасывается

        if (frontSonarFilt.getCurrentValue() > _distance)
        {
            //Drive(forward, PDreg->Update(rightMotor.readCurrentPosition() - leftMotor.readCurrentPosition()));
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