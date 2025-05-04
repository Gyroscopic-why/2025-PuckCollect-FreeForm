#pragma once

#include <Arduino.h>

class Button{
private:
    bool _isReversed;

    uint8_t _pin;
public:
    Button(uint8_t buttonPin, bool reversed = false){
        _pin = buttonPin;
        _isReversed = reversed;
    }

    void begin(){
        pinMode(_pin, INPUT_PULLUP);
    }

    void setReversed(bool reversed){
        _isReversed = reversed;
    }

    bool readState(){
        bool state = digitalRead(_pin);

        return _isReversed ? !state : state;
    }
};