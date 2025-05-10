#pragma once

#include <Arduino.h>
#include "Devices.h"
#include "Configs.h"
#include "Utils/PDRegulator.h"
#include "Utils/ElapsedTime.h"
#include "Utils/Sign.h"

PDRegulator _separatorRegulator(SEPARATOR_P, SEPARATOR_D);
int32_t _targetSeparatorPos = 0;

enum ColorType
{
    RED,
    BLUE,
    WHITE
};

ColorType floorColor = WHITE, puckColor = WHITE, ourColor = RED;

ElapseTime _clampTimer, _separartorDefendTimer;

void intakeBegin()
{
    separatorMotor.setMaxPower(SEPARATOR_MAX_POWER);

    brushServoLeft.write(90);
    brushServoRight.write(90);

    brushMotor.setPower(0.0);
}

void intakeStart()
{
    separatorMotor.resetEncoder();
    clampServo.write(CLAMP_SERVO_CALMP_POS);
    _separatorRegulator.start();
    _clampTimer.reset();
    _separartorDefendTimer.reset();

    brushServoLeft.write(90 + BRUSH_SERVO_SPEED);
    brushServoRight.write(90 - BRUSH_SERVO_SPEED);
    brushMotor.setPower(BRUSH_MOTOR_POWER);
}

void updateColorSensors()
{
    Color clampColor = clampColorSenor.readRawRGB();
    Color separatorColor = separatorColorSensor.readRawRGB();

    if (clampColor.r - max(clampColor.g, clampColor.b) > COLOR_SENSOR_CLAMP_SENS)
        floorColor = RED;
    else if (clampColor.b - max(clampColor.g, clampColor.r) > COLOR_SENSOR_CLAMP_SENS)
        floorColor = BLUE;
    else
        floorColor = WHITE;

    if (floorColor != WHITE)
        puckColor = WHITE;
    else
    {
        if (separatorColor.r - max(separatorColor.g, separatorColor.b) > COLOR_SENSOR_SEPARARTOR_SENS)
            puckColor = RED;
        else if (separatorColor.b - max(separatorColor.g, separatorColor.r) > COLOR_SENSOR_SEPARARTOR_SENS)
            puckColor = BLUE;
        else
            puckColor = WHITE;
    }
}

void detectFloor()
{
    updateColorSensors();
    ourColor = floorColor;
}

void intakeUpdate()
{
    updateColorSensors();

    int32_t separatorErr = _targetSeparatorPos - separatorMotor.getCurrentPosition();

    if (abs(separatorErr) > SEPARATOR_SENS)
    {
        separatorMotor.setPower(_separatorRegulator.update(separatorErr));

        if (_separartorDefendTimer.seconds() > BRUSH_DEFEND_TIMER)
        {
            _targetSeparatorPos -= sgn(separatorErr) * SEPARATOR_MOTOR_STEP;
            _separartorDefendTimer.reset();
        }
    }
    else
    {
        _separartorDefendTimer.reset();
        separatorMotor.setPower(0.0f);
    }

    if (abs(separatorErr) < SEPARATOR_SENS && puckColor != WHITE)
    {
        if (puckColor == ourColor)
            _targetSeparatorPos += SEPARATOR_MOTOR_STEP;
        else
            _targetSeparatorPos -= SEPARATOR_MOTOR_STEP;
    }

    if (floorColor == ourColor)
    {
        if (_clampTimer.seconds() > CLAMP_OPEN_TIMER)
            clampServo.write(CLAMP_SERVO_UNCLAMP_POS);
        else
            clampServo.write(CLAMP_SERVO_CALMP_POS);
    }
    else
    {
        _clampTimer.reset();

        clampServo.write(CLAMP_SERVO_CALMP_POS);
    }
}
