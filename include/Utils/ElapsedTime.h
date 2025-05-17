#pragma once

#include <Arduino.h>


enum ResetState
{
    dontReset = false,
    accurateReset = true,
    fastReset = true
};


class Timer
{
  private:
    uint64_t _lastMicroseconds = 0;

  public:
    void Reset(ResetState resetType = accurateReset, uint64_t alreadyCalculated = 0)
    {
        if (alreadyCalculated == 0) 
        {
            if (resetType == accurateReset) alreadyCalculated = micros();
            else alreadyCalculated = millis() * 1000.0f;
        }
        else if (resetType == fastReset) alreadyCalculated *= 1000.0f;

        _lastMicroseconds = alreadyCalculated;
    }


    //  Reset the last saved time to current

    uint64_t TimeAccurateMicroseconds(ResetState reset)
    {
        //  Calculate delta time (elapsed)
        uint64_t newMicroseconds = micros();
        
        //  Reset last time save (optional)
        if (reset) Reset(accurateReset, newMicroseconds);

        return newMicroseconds - _lastMicroseconds;
    }

    uint64_t TimeFastMicroseconds(ResetState reset)
    {
        //  Calculate delta time (elapsed)
        uint64_t newMilliseconds = millis();
        
        //  Reset last time save (optional)
        if  (reset == accurateReset) Reset(reset);
        else if (reset == fastReset) Reset(fastReset, newMilliseconds);

        return newMilliseconds * 1000.0f - _lastMicroseconds;
    }

    

    float TimeAccurateMilliseconds(ResetState reset)
    {
        //  Calculate delta time (elapsed)
        uint64_t newMicroseconds = micros();
        
        //  Reset last time save (optional)
        if  (reset) Reset(accurateReset, newMicroseconds);
        
        return ( newMicroseconds - _lastMicroseconds )   /   1000.0f;
    }

    float TimeFastMilliseconds(ResetState reset)
    {
        //  Calculate delta time (elapsed)
        uint64_t newMilliseconds = millis();
        
        //  Reset last time save (optional)
        if  (reset == accurateReset) Reset(accurateReset);
        else if (reset == fastReset) Reset(fastReset, newMilliseconds);
        
        return newMilliseconds - _lastMicroseconds / 1000.0f;
    }
    


    float TimeAccurateSeconds(ResetState reset)
    {
        //  Calculate delta time (elapsed)
        uint64_t newMicroseconds = micros();
        
        //  Reset last time save (optional)
        if  (reset) Reset(accurateReset, newMicroseconds);
        
        return ( newMicroseconds - _lastMicroseconds ) / 1000000.0f;
    }

    float TimeFastSeconds(ResetState reset)
    {
        //  Calculate delta time (elapsed)
        uint64_t newMilliseconds  = millis();
        
        //  Reset last time save (optional)
        if  (reset == accurateReset) Reset(accurateReset);
        else if (reset == fastReset) Reset(fastReset, newMilliseconds);
    
        
        return  ( newMilliseconds * 1000.0f  -  _lastMicroseconds )  /  1000000.0f;
    }
    
};