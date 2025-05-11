#pragma once

#include <Arduino.h>
#include <Servo.h>

#include "Drivers/DcMotor.h"
#include "Drivers/MultiWire.h"
#include "Drivers/BNO055_Gyroscope.h"
#include "Drivers/ColorSensor.h"
#include "Drivers/DistanceSensor.h"
#include "Drivers/Button.h"



HardwareWire hardwareWire;
SoftwareWire softwareWire(2, 3);
//  ????????????????????????????


//  Init hardware logic modules
//  Main prizm also acts like an expansion
DcExpansion dcPrizm    (5, &hardwareWire);
DcExpansion dcExp1port2(2, &hardwareWire);
DcExpansion dcExp2port3(3, &hardwareWire);


//  Fill prizm (main / expansion 1)
DcMotor brushMotor(&dcPrizm, 1);
DcMotor splitter  (&dcPrizm, 2);
//  Splitter - motor that separates the pucks


//  Drive motors on expansion No1 (port 2)
DcMotor driveMotorTL(&dcExp1port2, 1);
DcMotor driveMotorBL(&dcExp1port2, 2);
//  TL - top left motor
//  BL - bottom left motor


//  Drive motors expansion No2 (port 3)
DcMotor driveMotorTR(&dcExp2port3, 1);
DcMotor driveMotorBR(&dcExp2port3, 2);
//  TR - top    right motor
//  BR - bottom right motor



TCS34725_ColorSensor puckColScanner (&hardwareWire);
TCS34725_ColorSensor groundColScanner(&softwareWire);
//  Color sensors - color scanners


/*  Sonars on the robot

               Front
                
        ________0-0________
          0_____________0
Left    0/_______________\0    Right
        ___________________
        ___________________
        ___________________
        ________0-0________

               Back               

//  Note: 
      - Even though or robot is using a double swerve
        The motors control the movement inverse in reference to the real swerve behaviour:
          ^^ Same wheel motors spin in the same dirrection    = forward 1 wheel
          <> Same wheel motors spin in the opposite direction = turning 1 wheel  */



HCSR04_DistanceSensor frontSonar(4, 5);
HCSR04_DistanceSensor rightSonar(6, 7);
HCSR04_DistanceSensor  leftSonar(8, 9);
HCSR04_DistanceSensor  backSonar(10, 11);


BNO055_Gyroscope gyro(&hardwareWire);

Button startButton(2);



Servo dispenser;
//  Servo responsible for openning the (correct) puck gate


void DevicesBegin()
{
    // clampServo.attach(11);

    // puckColScanner.Begin();
    // groundColScanner.Begin();

    // frontSonar.Begin();
    // rightSonar.Begin();
    //  leftSonar.Begin();
    //  backSonar.Begin();

    // gyro.Begin();

    
    // softwareWire.begin();
    hardwareWire.begin();


    dcExp1port2.begin();
    dcExp2port3.begin();

    startButton.begin();



    //----------  Reset DC Motors encoders  --------//


    driveMotorTL.begin();
    driveMotorBL.begin();
        //  Left drive wheel

    driveMotorTR.begin();
    driveMotorBR.begin();
        //  Right drive wheel


    brushMotor.begin();
    splitter.begin();
    //  Brushes and puck separator
}