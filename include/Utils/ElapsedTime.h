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

        _lastMicroseconds = alreadyCalculated;
    }


    //  Reset the last saved time to current

    uint64_t TimeAccurateMicroseconds(ResetState reset)
    {
        //  Calculate delta time (elapsed)
        uint64_t deltaMicroseconds = micros() - _lastMicroseconds;
        
        //  Reset last time save (optional)
        if (reset) Reset(accurateReset, deltaMicroseconds);


        return deltaMicroseconds;
    }

    uint64_t TimeFastMicroseconds(ResetState reset)
    {
        //  Calculate delta time (elapsed)
        uint64_t deltaMicroseconds = millis() * 1000.0f - _lastMicroseconds;
        
        //  Reset last time save (optional)
        if  (reset == accurateReset) Reset(reset);
        else if (reset == fastReset) Reset(fastReset, deltaMicroseconds + _lastMicroseconds);


        return deltaMicroseconds;
    }

    


    float TimeAccurateMilliseconds(ResetState reset)
    {
        //  Calculate delta time (elapsed)
        uint64_t deltaMicroseconds = micros() - _lastMicroseconds;
        
        //  Reset last time save (optional)
        if  (reset) Reset(accurateReset, deltaMicroseconds + _lastMicroseconds);
        

        //  Transform McrS to MS
        uint64_t elapsedMilliseconds = deltaMicroseconds / 1000.0f;

        return elapsedMilliseconds;
    }

    float TimeFastMilliseconds(ResetState reset)
    {
        //  Calculate delta time (elapsed)
        uint64_t deltaMicroseconds = millis() * 1000.0f - _lastMicroseconds;
        
        //  Reset last time save (optional)
        if  (reset == accurateReset) Reset(accurateReset);
        else if (reset == fastReset) Reset(fastReset, deltaMicroseconds + _lastMicroseconds);
        
        
        //  Transform McrS to MS
        uint64_t elapsedMilliseconds = deltaMicroseconds / 1000.0f;

        return elapsedMilliseconds;
    }
    



    float TimeAccurateSeconds(ResetState reset)
    {
        //  Calculate delta time (elapsed)
        uint64_t deltaMicroseconds = micros() - _lastMicroseconds;
        
        //  Reset last time save (optional)
        if  (reset) Reset(accurateReset, deltaMicroseconds + _lastMicroseconds);
        

        //  Transform MS to Sec
        uint64_t elapsedSeconds = deltaMicroseconds / 1000000.0f;

        return elapsedSeconds;
    }

    float TimeFastSeconds(ResetState reset)
    {
        //  Calculate delta time (elapsed)
        uint64_t deltaMicroseconds = millis() * 1000.0f - _lastMicroseconds;
        
        //  Reset last time save (optional)
        if  (reset == accurateReset) Reset(accurateReset);
        else if (reset == fastReset) Reset(fastReset, deltaMicroseconds + _lastMicroseconds);
        

        //  Transform MS to Sec
        uint64_t elapsedSeconds = deltaMicroseconds / 1000000.0f;
        
        return elapsedSeconds;
    }
    
};