#pragma once

#include <Arduino.h>
#include "Devices.h"
#include "Configs.h"
#include "utils/PDRegulator.h"
#include "utils/Sign.h"
#include "utils/ElapsedTime.h"

class DriveSample
{
protected:
    enum Direction
    {
        backward = -1,
        stop = 0,
        forward = 1
    };

    void Drive(float speed, float turnDir, float dirError, float lwError, float rwError)
    {
        //  speed - max speed for either of the wheels, can be set between -1 and 1 (float)
        //  turnDir  = Turn  in    direction
        //  dirError = error for   the current robot direction
        //  lwError  = left  wheel error
        //  rwError  = right wheel error


        float tlError = (  lwError + dirError + turnDir )  /  2.0f;
        float blError = ( -lwError + dirError - turnDir )  /  2.0f;
        //  ---  Top Left and Bottom Left motor errors  ---  //
        //  For the drifting of a single wheel 
        //  (proposed to the straight moving of the other wheel)

        float trError = (  rwError - dirError + turnDir )  /  2.0f;
        float brError = ( -rwError - dirError - turnDir )  /  2.0f;
        //  ---  Top Right and Bottom Right motor errors  ---  //
        //  For the drifting of a single wheel 
        //  (proposed to the straight moving of the other wheel)
        

        float highestError = max(   max(  abs(tlError), abs(blError)  ),   max(  abs(trError), abs(brError)  )  );
        highestError *= speed > 0 ? 1 : -1;
        //  Calculate the max error for maintaining the max set speed (with the sign)



        //-------------  Apply power  ---------------------------------//

        driveMotorTL.setPower( speed - highestError + tlError );
        driveMotorBL.setPower( speed - highestError + blError );
        //  Left wheel

        driveMotorTR.setPower( speed - highestError + trError );
        driveMotorBR.setPower( speed - highestError + brError );
        //  Right wheel


        //  Note: 
        //  - Even though or robot is using a double swerve
        //    The motors control the movement inverse in reference to the real swerve behaviour:
        //      ^^ Same wheel motors spin in the same direction      (++ or --)  = forward 1 wheel
        //      <> Same wheel motors spin in the opposite direction  (+- or -+)  = turning 1 wheel
    }


    float GetOriantation()
    {
        return BNO_gyro.getOrientation().x;
    }

    void encoderReset()
    {
        driveMotorTL.softwareEncoderReset();
        driveMotorBL.softwareEncoderReset();

        driveMotorTR.softwareEncoderReset();
        driveMotorBR.softwareEncoderReset();
    }

    void dropProcess()
    {
        Drive(stop, 0, 0, 0, 0);
        encoderReset();
    }

    PDRegulator<float> *PDmain;
    PDRegulator<float> *PDlw;
    PDRegulator<float> *PDrw;

public:
    float mainReg_p = 1.0f;
    float mainReg_d = 1.0f;

    float lwReg_p = 1.0f;
    float lwReg_d = 1.0f;

    float rwReg_p = 1.0f;
    float rwReg_d = 1.0f;


    DriveSample() {};   //  For the QUEUE system

    DriveSample(PDRegulator<float> &PDRegmain)
    {
        PDmain = &PDRegmain;
    }

    DriveSample(PDRegulator<float> &PDRegmain, PDRegulator<float> &PDReglw, PDRegulator<float> &PDRegrw)
    {
        PDmain = &PDRegmain;
        PDlw = &PDReglw;
        PDrw = &PDRegrw;
    }

    virtual ~DriveSample() {}

    virtual void Start()
    {
        encoderReset();

        PDmain->Reset(0, 0);
        if (PDlw != nullptr) PDlw->Reset(0, 0);
        if (PDrw != nullptr) PDrw->Reset(0, 0);
    }

    virtual bool Execute()
    {
        return true;
    }

    void ResetPd()
    {
        PDmain->Reset(mainReg_p, mainReg_d);
        if (PDlw != nullptr) PDlw->Reset(lwReg_p, lwReg_d);
        if (PDrw != nullptr) PDrw->Reset(rwReg_p, rwReg_d);
    }
};