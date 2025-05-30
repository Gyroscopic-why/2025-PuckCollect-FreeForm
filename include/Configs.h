#pragma once
#include <Arduino.h>


//  For the median filter
#define DEFAULT_MEDIAN_BUFFER_LENGHT 5



//  For the drive test
#define EXECUTION_LIMIT 30



//  Intake
#define CLAMP_SERVO_CLAMP_POS 0
#define CLAMP_SERVO_UNCLAMP_POS 20

#define CLAMP_OPEN_TIMER 0.5f

#define SEPARATOR_P 0.1f
#define SEPARATOR_D 0.0f
#define SEPARATOR_SENS 50

#define COLOR_SENSOR_CLAMP_SENS 10
#define COLOR_SENSOR_SEPARARTOR_SENS 10

#define BRUSH_DEFEND_TIMER 1.0f



//  Robot parameters

      //  Power
#define MAX_DRIVE_POWER 0.5f

#define BRUSH_SERVO_SPEED 45
#define BRUSH_MOTOR_POWER 0.5f

#define SEPARATOR_MAX_POWER 0.7
#define SEPARATOR_MOTOR_STEP 720 / 3

      //  Drive
#define ERROR_VALUE_COEFFICIENT 0.8f
#define DEGREES_TO_DRIVEPOWER_COEFFICIENT 0.0055f

      //  Turn
#define WHEEL_DIAMETER           1   //  Needs tuning
#define SINGLE_ENCODER_STEP     (20 * 24)
#define DISTANCE_BETWEEN_WHEELS  1   //  Needs tuning
#define ROBOT_TURN_STEP         (DISTANCE_BETWEEN_WHEELS * SINGLE_ENCODER_STEP)
#define MAX_TURN_ERROR_DEGREES_FOR_ENCODER_TURN 5

      //  Robot configuration
#define HAS_GYROSCOPE false
#define HAS_SONAR true