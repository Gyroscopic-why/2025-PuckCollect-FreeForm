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


//  Init hardware logic modules
//  Main prizm also acts like an expansion
DcExpansion dcPrizm1   (5, &hardwareWire);
DcExpansion dcExp1port2(2, &hardwareWire);
DcExpansion dcExp2port3(3, &hardwareWire);


//  Fill prizm (main / expansion 1)
DcMotor brushMotor    (&dcPrizm1, 1);
DcMotor separatorMotor(&dcPrizm1, 2);


//  Fill expansion No1 (port 2)
DcMotor motorTL(&dcExp1port2, 1);
DcMotor motorBL(&dcExp1port2, 2);
//  TL - top left motor
//  BL - bottom left motor


//  Fill expansion No2 (port 3)
DcMotor motorTR(&dcExp2port3, 1);
DcMotor motorBR(&dcExp2port3, 2);
//  TR - top right motor
//  BR - bottom right motor



TCS34725_ColorSensor separatorColorSensor(&hardwareWire);
TCS34725_ColorSensor floorColorSenor(&softwareWire);


/*  Sonars on the robot

               Front
                
        ________0-0________
          0_____________0
Left    0/_______________\0    Right
        ___________________
        ___________________
        ___________________
        ________0-0________

               Back               */


HCSR04_DistanceSensor frontSonar(4, 5);
HCSR04_DistanceSensor rightSonar(6, 7);
HCSR04_DistanceSensor  leftSonar(8, 9);
HCSR04_DistanceSensor  backSonar(10, 11);


BNO055_Gyro gyroscope(&hardwareWire);

Button startButton(2);



Servo clampServo;
Servo brushServoLeft, brushServoRight;


void DevicesBegin()
{
    // clampServo.attach(11);

    // brushServoLeft.attach(12);
    // brushServoRight.attach(13);

    
    // separatorColorSensor.Begin();
    // floorColorSensor.Begin();

    // frontSonar.Begin();
    // rigthSonar.Begin();
    //  leftSonar.Begin();
    //  backSonar.Begin();

    // gyroscope.begin();

    
    // softwareWire.begin();
    hardwareWire.begin();


    dcExp1port2.begin();
    dcExp2port3.begin();

    startButton.begin();



    //  Reset motor encoders
    motorTL.begin();
    motorBL.begin();
        //  Left wheel

    motorTR.begin();
    motorBR.begin();
        //  Right wheel



    brushMotor.begin();
    separatorMotor.begin();
}