#pragma once

#include <Arduino.h>
#include <Servo.h>

#include "Drivers/DcMotor.h"
#include "Drivers/MultiWire.h"
#include "Drivers/Gyro.h"
#include "Drivers/ColorSensor.h"
#include "Drivers/DistanceSensor.h"
#include "Drivers/Button.h"

HardwareWire hardwareWire;
SoftwareWire softwareWire(2, 3);

DcExpansion dcExpansion1(1, &hardwareWire);
DcExpansion dcExpansion3(3, &hardwareWire);

DcMotor brushMotor(&dcExpansion3, 1);
DcMotor separatorMotor(&dcExpansion3, 2);

DcMotor motorTL(&dcExpansion1, 1);
DcMotor motorBL(&dcExpansion1, 2);
//  TL - top left motor
//  BL - bottom left motor

DcMotor motorTR(&dcExpansion3, 1);
DcMotor motorBR(&dcExpansion3, 2);
//  TR - top right motor
//  BR - bottom right motor



TCS34725ColorSensor separatorColorSensor(&hardwareWire);
TCS34725ColorSensor floorColorSenor(&softwareWire);

DistanceSensor forwardDistanceSensor(4, 5);
DistanceSensor leftDistanceSensor(6, 7);
DistanceSensor rightDistanceSensor(8, 9);
DistanceSensor backwardDistanceSensor(10, 11);

BNO055Gyro gyro(&hardwareWire);

Button startButton(2);

Servo clampServo;
Servo brushServoLeft, brushServoRight;

void DevicesBegin(){
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

    motorTL.begin();
    motorBL.begin();

    motorTR.begin();
    motorBR.begin();


    brushMotor.begin();
    separatorMotor.begin();
}