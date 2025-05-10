#pragma once

#include <Arduino.h>
#include "Drivers/DcMotor.h"
#include "Drivers/MultiWire.h"
#include "Drivers/Gyro.h"
#include "Drivers/ColorSensor.h"
#include "Drivers/DistanceSensor.h"
#include "Drivers/Button.h"
#include <Servo.h>

HardwareWire hardwareWire;
SoftwareWire softwareWire(2, 3);

DcExpansion dcExpansion1(1, &hardwareWire);
DcExpansion dcExpansion3(3, &hardwareWire);

DcMotor leftMotor(&dcExpansion1, 1);
DcMotor rightMotor(&dcExpansion1, 2);

DcMotor brushMotor(&dcExpansion3, 1);
DcMotor separatorMotor(&dcExpansion3, 2);

TCS34725ColorSensor separatorColorSensor(&hardwareWire);
TCS34725ColorSensor clampColorSenor(&softwareWire);

DistanceSensor forwardDistanceSensor(4, 5);
DistanceSensor leftDistanceSensor(6, 7);
DistanceSensor rightDistanceSensor(8, 9);

BNO055Gyro gyro(&hardwareWire);

Button startButton(2);

Servo clampServo;
Servo brushServoLeft, brushServoRight;

void devicesBegin(){
    // clampServo.attach(11);

    // brushServoLeft.attach(12);
    // brushServoRight.attach(13);

    hardwareWire.begin();
    // softwareWire.begin();

    dcExpansion1.begin();
    dcExpansion3.begin();

    // separatorColorSensor.begin();
    // clampColorSenor.begin();

    // forwardDistanceSensor.begin();
    // leftDistanceSensor.begin();
    // rightDistanceSensor.begin();

    startButton.begin();

    // gyro.begin();

    leftMotor.begin();
    brushMotor.begin();
    separatorMotor.begin();
    rightMotor.begin();
}