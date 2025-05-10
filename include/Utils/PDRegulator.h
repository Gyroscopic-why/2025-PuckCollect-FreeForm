#pragma once

#include <Arduino.h>

class PDRegulator{
private:
    float _kp, _kd;

    int32_t _oldErr = 0;

    uint64_t _lastUpdateTime = 0;
public:
    PDRegulator(float kp, float kd){
        _kp = kp;
        _kd = kd;
    }

    float update(int32_t err){
        float uP = err * _kp;

        float uD = (err - _oldErr) / ((float)(millis()) - _lastUpdateTime) * _kd;

        _oldErr = err;

        _lastUpdateTime = millis();

        return uP + uD;
    }

    void start(){
        _lastUpdateTime = millis();
    }

    void reset(float kp, float kd){
        _kp = kp;
        _kd = kd;
        start();
        _oldErr = 0.0f;
    }
};