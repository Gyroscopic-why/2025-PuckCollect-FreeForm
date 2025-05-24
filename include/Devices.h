#pragma once

#include <Arduino.h>
#include <Servo.h>


#include "Drivers/DcMotor.h"
#include "Drivers/MultiWire.h"

#include "Drivers/ColorSensor.h"
#include "Drivers/DistanceSensor.h"
#include "Drivers/Button.h"

#include "Drivers/BNO055_Gyroscope.h"
#include "Drivers/MPU9250_Gyroscope.h"


#include "Utils/ElapsedTime.h"
#include "Utils/MedianFilter.h"


#define START_BUTTON_PIN 2
#define DISPENSER_SERVO_PIN 1



HardwareWire hardwareWire; // 4 and 5 ports by default
SoftwareWire softwareWire(2, 3); 
// Our version of an i2c system, custom and slower used for the second color sensor


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
//  Put on separate i2c systems, because 1x i2c bugs and cannot handle 2 color sensors


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


MedianFilter<uint16_t> frontSonarFilt(0);
MedianFilter<uint16_t> rightSonarFilt(0);
MedianFilter<uint16_t>  leftSonarFilt(0);
MedianFilter<uint16_t>  backSonarFilt(0);


BNO055_Gyroscope  BNO_gyro(&hardwareWire);
//MPU9250_Gyroscope MPU_gyro(&hardwareWire);


Button startButton(START_BUTTON_PIN);


Servo dispenser;
//  Servo responsible for openning the (correct) puck gate


void BeginDevices()
{
    // dispenser.attach(DISPENSER_SERVO_PIN);

    // puckColScanner.Begin();
    // groundColScanner.Begin();

    // frontSonar.Begin();
    // rightSonar.Begin();
    //  leftSonar.Begin();
    //  backSonar.Begin();

    BNO_gyro.begin();

    
    softwareWire.begin();
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


Timer deviceUpdateTimer;

void UpdateDevices()
{
    if (deviceUpdateTimer.TimeAccurateMicroseconds(accurateReset) > 6)
    {
        frontSonarFilt.update(frontSonar.readDistance());
        rightSonarFilt.update(frontSonar.readDistance());
         leftSonarFilt.update( leftSonar.readDistance());
         backSonarFilt.update( backSonar.readDistance());
    }
}