#pragma once

#include <Arduino.h>
#include "MultiWire.h"
#include "utils/Sign.h"

#define BNO055_ADDRESS 0x28 //0x29

#define BNO055_CONFIG_MODE 0X00
#define BNO055_OPERATION_MODE 0x0C
#define BNO055_MODE_SWAP 0X3D

#define BNO055_POWER_MODE_SWAP 0X3E
#define BNO055_POWER_MODE_NORMAL 0x00

#define BNO055_PAGE_ID 0x007
#define BNO055_SET_USE_EXTRA_CRYSTAL 0X3F
#define BNO055_USE_EXTRA_CRYSTAL 0x80
#define BNO055_NOT_USE_EXTRA_CRYSTAL 0x00

#define BNO055_SET_UNITS 0X3B

#define BNO055_REQUEST_ORIENTATION 0X1A

#define BNO055_REQUEST_CHIP_ID 0x00
#define BNO055_CHIP_ID 0xA0
 
float chopDegrees(float val){
    float chopedVal = val;

    while (abs(chopedVal) > 180.0f)
        chopedVal -= 360.0f * sign(chopedVal);
        
    return chopedVal;
}

float chopRadians(float val){
    float chopedVal = val;

    while (abs(chopedVal) > PI)
        chopedVal -= 2.0f * PI * sign(chopedVal);
        
    return chopedVal;
}

enum AngleUnit{
    RADIANS = 0,
    DEGREES = 1
};

struct Oriantation{
    float x, y, z;

    Oriantation(float x, float y, float z){
        this->x = x;
        this->y = y;
        this->z = z;
    }

    Oriantation(){
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }
};

class BNO055_Gyro
{
private:
    IWire *_wire;
    AngleUnit _angleUnit;
    Oriantation _startOrientation;

    Oriantation getRawOrientation(){
        _wire->write8(BNO055_ADDRESS, BNO055_REQUEST_ORIENTATION);

        _wire->requestFrom(BNO055_ADDRESS, 6);
        uint8_t buffer[6];
        _wire->readBytes(buffer, 6);

        int16_t x, y, z;

        x = ((int16_t)buffer[0]) | (((int16_t) buffer[1]) << 8);
        y = ((int16_t)buffer[2]) | (((int16_t) buffer[3]) << 8);
        z = ((int16_t)buffer[4]) | (((int16_t) buffer[5]) << 8);

        return Oriantation(x / 16.0f, y / 16.0f, z / 16.0f);
    }

public:
    BNO055_Gyro(IWire *wire, AngleUnit angleUnit = DEGREES){
        _wire = wire;
        _angleUnit = angleUnit;
    }

    bool isConnected(){
        _wire->write8(BNO055_ADDRESS, BNO055_REQUEST_CHIP_ID);

        if(_wire->read() == BNO055_CHIP_ID)
            return true;

        return false;
    }

    void begin(){
        uint8_t unitBit = (_angleUnit == RADIANS ? 1 : 0);
        uint8_t unitData = (1 << 7) | (0 << 4) | (unitBit << 2) | (unitBit << 1) | (0 << 0);

        _wire->beginTransmission(BNO055_ADDRESS);
        
        _wire->write(BNO055_MODE_SWAP);
        _wire->write(BNO055_CONFIG_MODE);

        _wire->write(BNO055_POWER_MODE_SWAP);
        _wire->write(BNO055_POWER_MODE_NORMAL);

        _wire->write(BNO055_PAGE_ID);
        _wire->write(0x0);

        _wire->write(BNO055_SET_UNITS);
        _wire->write(unitData);

        _wire->write(BNO055_SET_USE_EXTRA_CRYSTAL);
        _wire->write(BNO055_USE_EXTRA_CRYSTAL);

        _wire->write(BNO055_MODE_SWAP);
        _wire->write(BNO055_OPERATION_MODE);

        _wire->endTransmission();

        _startOrientation = getRawOrientation();
    }
    
    void resetTo(Oriantation oriantation){
        _startOrientation = getRawOrientation();
        
        _startOrientation.x += oriantation.x;
        _startOrientation.y += oriantation.y;
        _startOrientation.z += oriantation.z;

        if(_angleUnit == RADIANS){
            _startOrientation.x = chopRadians(_startOrientation.x);
            _startOrientation.y = chopRadians(_startOrientation.y);
            _startOrientation.z = chopRadians(_startOrientation.z);
        }
        else{
            _startOrientation.x = chopDegrees(_startOrientation.x);
            _startOrientation.y = chopDegrees(_startOrientation.y);
            _startOrientation.z = chopDegrees(_startOrientation.z);
        }
    }

    void reset(){
        resetTo(Oriantation());
    }

    Oriantation getOrientation(){
        Oriantation orientation = getRawOrientation();

        orientation.x -= _startOrientation.x;
        orientation.y -= _startOrientation.y;
        orientation.z -= _startOrientation.z;

        if(_angleUnit == RADIANS){
            orientation.x = chopRadians(orientation.x);
            orientation.y = chopRadians(orientation.y);
            orientation.z = chopRadians(orientation.z);
        }
        else{
            orientation.x = chopDegrees(orientation.x);
            orientation.y = chopDegrees(orientation.y);
            orientation.z = chopDegrees(orientation.z);
        }

        return orientation;
    }
};
