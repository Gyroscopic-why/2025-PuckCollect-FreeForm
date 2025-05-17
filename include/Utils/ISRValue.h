#pragma once

template <typename T>
class ISRValue{
private:
    T _ISRValue;
    T _currentValue;

public:
    ISRValue(){}

    void ISRSet(T value){
        _ISRValue = value;
    }

    void update(){
        _currentValue = _ISRValue;
    }

    T getValue(){
        return _currentValue;
    }
};