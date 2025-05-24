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

    PDRegulator<float> *PDreg;

public:
    float coef_p = 1.0f;
    float coef_d = 1.0f;

    DriveSample() {}; // для очереди

    DriveSample(PDRegulator<float> &PDr)
    {
        PDreg = &PDr;
    }

    virtual ~DriveSample() {}

    virtual void Start()
    {
        encoderReset();

        PDreg->Reset(0, 0);
    }

    virtual bool Execute()
    {
        return true;
    }

    void ResetPd()
    {
        PDreg->Reset(coef_p, coef_d);
    }
};