#pragma once

#include <Arduino.h>
#include "Configs.h"
#include "Swap.h"

template <typename T>
class MedianFilter
{
private:
    uint8_t _bufferLenght;
    T *_buffer;
    T *_copyedBuffer;
    uint8_t currentBuffLenght = 0;
    T _currentValue;

    T quickMedianRecursion(T* arr, uint8_t left, uint8_t right, uint8_t k) { //дип сик рулит
        if (left == right) return arr[left];
    
        uint8_t pivotIndex = left + (right - left) / 2;
        T pivotValue = arr[pivotIndex];
        swap(&arr[pivotIndex], &arr[right]);

        pivotIndex = left;
    
        for (uint8_t i = left; i < right; ++i) {
            if (arr[i] < pivotValue) {
                swap(&arr[pivotIndex], &arr[i]);

                pivotIndex++;
            }
        }

        swap(&arr[right], &arr[pivotIndex]);
    
        if (k == pivotIndex)
            return arr[k];
        else if (k < pivotIndex)
            return quickMedianRecursion(arr, left, pivotIndex - 1, k);
        else
            return quickMedianRecursion(arr, pivotIndex + 1, right, k);
    }

    T quickMedian(T *arr, uint8_t lenght){
        for(uint8_t i = 0; i < lenght; i++)
            _copyedBuffer[i] = arr[i];

        return quickMedianRecursion(_copyedBuffer, 0, lenght - 1, (lenght - 1) / 2);
    }

public:
    MedianFilter(T defaultVal, uint8_t bufferLenght = DEFAULT_MEDIAN_BUFFER_LENGHT){
        _currentValue = defaultVal;
        _bufferLenght = bufferLenght + ((bufferLenght + 1) % 2);

        _buffer = new T[_bufferLenght];
        _copyedBuffer = new T[_bufferLenght];
    }

    ~MedianFilter(){
        delete []_buffer;
    }

    T update(const T value){
        if(currentBuffLenght < _bufferLenght){
            _buffer[currentBuffLenght] = value;
            currentBuffLenght++;

            _currentValue = quickMedian(_buffer, currentBuffLenght);
            return _currentValue;
        }
        else{
            for(uint8_t i = 0; i < _bufferLenght - 1; i++)
                _buffer[i] = _buffer[i + 1];

            _buffer[_bufferLenght - 1] = value;

            _currentValue = quickMedian(_buffer, _bufferLenght);
            return _currentValue;
        }
    }

    T getCurrentValue(){
        return _currentValue;
    }
};
