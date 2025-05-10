#pragma once

#include <Arduino.h>

class Timer
{
  private:
    uint64_t _lastMicros = 0;

  public:
    void Reset()
    {
        _lastMicros = micros();
    }
    //  Reset the last saved time to current

    uint64_t TimeMicroseconds(bool reset)
    {
        //  Calculate delta time (elapsed)
        uint64_t elapsedMicroseconds = micros() - _lastMicros;
        
        //  Reset last time save (optional)
        if (reset) Reset();

        return elapsedMicroseconds;
    }
    //  Return elapsed microseconds

    float TimeMilliseconds(bool reset)
    {
        //  Calculate delta time (elapsed)
        uint64_t elapsedMicroseconds = micros() - _lastMicros;
        
        //  Reset last time save (optional)
        if (reset) Reset();
        
        return elapsedMicroseconds / 1000.0f;
    }
    //  Return elapsed milliseconds

    float TimeSeconds(bool reset)
    {
        //  Calculate delta time (elapsed)
        uint64_t elapsedMicroseconds = micros() - _lastMicros;
        
        //  Reset last time save (optional)
        if (reset) Reset();
        
        return elapsedMicroseconds / 1000000.0f;
    }
    //  Return elapsed seconds
};