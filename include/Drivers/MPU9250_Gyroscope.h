#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class MPU9250_Gyroscope
{
  private:
    uint8_t mpuAddress = 0x68;
    uint8_t magAddress = 0x0C;  //  Adress of the magnitometer AK8963

    uint32_t prev_time = 0;
    
    //  Calibration parameters (NEED TO BRUTEFORCE YOUR OWN)
    float magCalibration[3] = {1.0, 1.0, 1.0};
    float magScale[3] = {1.0, 1.0, 1.0};
    float magBias [3] = {0.0, 0.0, 0.0};

    float gyroBias[3] = {0};
    int calibrationSamples = 1000;
    
    //  Madgwick filter parameters
    float beta = 0.1f;  //  Filter coefficient
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; 
    float invSampleFreq = 1.0f / 200.0f;

    
    
    //  Sensor registers
    enum Registers {
        PWR_MGMT_1 = 0x6B,
        INT_PIN_CFG = 0x37,
        ACCEL_XOUT_H = 0x3B,
        GYRO_XOUT_H = 0x43,
        EXT_SENS_DATA_00 = 0x49
    };

    void WriteRegister(uint8_t device, uint8_t reg, uint8_t value) 
    {
        Wire.beginTransmission(device);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }

    void ReadRegisters(uint8_t device, uint8_t reg, uint8_t count, uint8_t* data) 
    {
        Wire.beginTransmission(device);
        Wire.write(reg);
        Wire.endTransmission(false);

        Wire.requestFrom(device, count);
        for(uint8_t i = 0; i < count; i++) 
        {
            data[i] = Wire.read();
        }
    }

    void ReadAccel(float& ax, float& ay, float& az) 
    {
        uint8_t buffer[6];

        ReadRegisters(mpuAddress, ACCEL_XOUT_H, 6, buffer);

        ax = (int16_t)(buffer[0] << 8 | buffer[1]) * 2.0f / 32768.0f;
        ay = (int16_t)(buffer[2] << 8 | buffer[3]) * 2.0f / 32768.0f;
        az = (int16_t)(buffer[4] << 8 | buffer[5]) * 2.0f / 32768.0f;
    }

    void ReadGyro(float& gx, float& gy, float& gz) 
    {
        uint8_t buffer[6];

        ReadRegisters(mpuAddress, GYRO_XOUT_H, 6, buffer);

        gx = (int16_t)(buffer[0] << 8 | buffer[1]) * 500.0f / 32768.0f;
        gy = (int16_t)(buffer[2] << 8 | buffer[3]) * 500.0f / 32768.0f;
        gz = (int16_t)(buffer[4] << 8 | buffer[5]) * 500.0f / 32768.0f;
    }

    void ReadMag(float& mx, float& my, float& mz) 
    {
        uint8_t buffer[7];

        WriteRegister(magAddress, 0x0A, 0x01);
        //  One time reading mode


        delay(10);
        ReadRegisters(magAddress, 0x03, 7, buffer);
        
        mx = (int16_t)(buffer[1] << 8 | buffer[0]) * 0.15f * magCalibration[0] + magBias[0];
        my = (int16_t)(buffer[3] << 8 | buffer[2]) * 0.15f * magCalibration[1] + magBias[1];
        mz = (int16_t)(buffer[5] << 8 | buffer[4]) * 0.15f * magCalibration[2] + magBias[2];
    }

    void MadgwickUpdate(
    float gx, float gy, float gz,  //  Угловые скорости (рад/с)
    float ax, float ay, float az,  //  Акселерометр (нормализованный)
    float mx, float my, float mz,  //  Магнитометр (нормализованный)
    float q0, float q1, float q2, float q3,  //  Текущий кватернион
    float beta,  //  Коэффициент фильтра
    float dt,    //  Временной шаг (сек)
    float& q0_out, float& q1_out, float& q2_out, float& q3_out) 
    {
        //  Новый кватернион


        // 1. Инициализация переменных
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        
        // 2. Вычисление производной кватерниона от гироскопа
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

        // 3. Нормализация акселерометра и магнитометра
        recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        recipNorm = InvSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        // 4. Вспомогательные вычисления
        float _2q0mx = 2.0f * q0 * mx;
        float _2q0my = 2.0f * q0 * my;
        float _2q0mz = 2.0f * q0 * mz;
        float _2q1mx = 2.0f * q1 * mx;
        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        //float _2q0q2 = 2.0f * q0 * q2;
        //float _2q2q3 = 2.0f * q2 * q3;
        float q0q0 = q0 * q0;
        float q0q1 = q0 * q1;
        float q0q2 = q0 * q2;
        float q0q3 = q0 * q3;
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q3q3 = q3 * q3;


        // 5. Расчет эталонного магнитного поля
        float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + 
                _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - 
                my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        float _2bx = sqrtf(hx * hx + hy * hy);
        float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + 
                _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;


        // Удвоение для оптимизации
        float _4bx = 2.0f * _2bx;
        float _4bz = 2.0f * _2bz;


        // 6. Вычисление градиента
        s0 = -_2q2*(2.0f*(q1q3 - q0q2) - ax) + 
            _2q1*(2.0f*(q0q1 + q2q3) - ay) + 
            -_2bz*q2*(2.0f*(q0q2 + q1q3) - ax) + 
            (-_2bx*q3 + _2bz*q1)*(2.0f*(q1q2 + q0q3) - ay) + 
            _2bx*q2*(2.0f*(q0q1 - q2q3) - az);

        s1 = _2q3*(2.0f*(q1q3 - q0q2) - ax) + 
            _2q0*(2.0f*(q0q1 + q2q3) - ay) + 
            -4.0f*q1*(1 - 2.0f*q1q1 - 2.0f*q2q2 - az) + 
            _2bz*q3*(2.0f*(q0q2 + q1q3) - ax) + 
            (_2bx*q2 + _2bz*q0)*(2.0f*(q1q2 + q0q3) - ay) + 
            (_2bx*q3 - _4bz*q1)*(2.0f*(q0q1 - q2q3) - az);

        s2 = -_2q0*(2.0f*(q1q3 - q0q2) - ax) + 
            _2q3*(2.0f*(q0q1 + q2q3) - ay) + 
            -4.0f*q2*(1 - 2.0f*q1q1 - 2.0f*q2q2 - az) + 
            (-_4bx*q2 - _2bz*q0)*(2.0f*(q0q2 + q1q3) - ax) + 
            (_2bx*q1 + _2bz*q3)*(2.0f*(q1q2 + q0q3) - ay) + 
            (_2bx*q0 - _4bz*q2)*(2.0f*(q0q1 - q2q3) - az);

        s3 = _2q1*(2.0f*(q1q3 - q0q2) - ax) + 
            _2q2*(2.0f*(q0q1 + q2q3) - ay) + 
            (-_4bx*q3 + _2bz*q1)*(2.0f*(q0q2 + q1q3) - ax) + 
            (-_2bx*q0 + _2bz*q2)*(2.0f*(q1q2 + q0q3) - ay) + 
            _2bx*q1*(2.0f*(q0q1 - q2q3) - az);

        // 7. Нормализация градиента
        recipNorm = InvSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // 8. Применение обратной связи
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;

        // 9. Интегрирование по времени
        q0 += qDot1 * dt;
        q1 += qDot2 * dt;
        q2 += qDot3 * dt;
        q3 += qDot4 * dt;

        // 10. Нормализация кватерниона
        recipNorm = InvSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0_out = q0 * recipNorm;
        q1_out = q1 * recipNorm;
        q2_out = q2 * recipNorm;
        q3_out = q3 * recipNorm;
}


    //  QUAKE III FAST INVERSE SQUARE ROOT LETS GOOOO (original name = Q_rsqrt)
    float InvSqrt ( float number ) // Inverse square root =  1 / Sqrt(2)
    {
        long i;
        float x2, y; //  x2 - half number
        const float threehalves = 1.5f;

        x2 = number * 0.5f;
        y  = number;
        i  = * ( long * ) &y;                       //  evil floating point bit hack
        i  = 0x5F3759DF - ( i >> 1 );               //  what the fuck?
        y  = * ( float * ) &i;
        y  = y * ( threehalves - ( x2 * y * y ) );  //  1st iteration
    //  y  = y * ( threehalves - ( x2 * y * y ) );  //  2nd iteration, can be removed

        return y;
    }


    void Calibrate(uint16_t samples = 1000) 
    {
        float maxVal[3] = {-10000, -10000, -10000};
        float minVal[3] = {10000, 10000, 10000};
        
        //  Change the mode to continious update with frequency 8Hz
        WriteMagRegister(0x0A, 0x16);
        delay(100);

        //Serial.println("Начинаем калибровку магнитометра...");
        //Serial.println("Вращайте датчик во всех направлениях");

        for(uint16_t i = 0; i < samples; i++) 
        {
            uint8_t buffer[7];
            ReadMagRegisters(0x03, 7, buffer);
            
            //  Read raw data (little endian)
            int16_t mx = (buffer[1] << 8) | buffer[0];
            int16_t my = (buffer[3] << 8) | buffer[2];
            int16_t mz = (buffer[5] << 8) | buffer[4];
            
            //  Update min max values
            minVal[0] = min(minVal[0], mx);
            maxVal[0] = max(maxVal[0], mx);
            minVal[1] = min(minVal[1], my);
            maxVal[1] = max(maxVal[1], my);
            minVal[2] = min(minVal[2], mz);
            maxVal[2] = max(maxVal[2], mz);
            
            if(i % 100 == 0) {
                //Serial.print("Прогресс: ");
                //Serial.print(i * 100 / samples);
                //Serial.println("%");
            }
            delay(20);
        }

        //  Calculate the parameters of the calibration process
        for(uint8_t i = 0; i < 3; i++) 
        {
            magBias[i] = (maxVal[i] + minVal[i]) / 2.0f;
            magScale[i] = (maxVal[i] - minVal[i]) / 2.0f;
            
            //  Average calculation
            float avg_scale = (magScale[0] + magScale[1] + magScale[2]) / 3.0f;
            magScale[i] = avg_scale / magScale[i];
        }
    }


    void WriteMagRegister(uint8_t reg, uint8_t value) 
    {
        Wire.beginTransmission(magAddress);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }

    void ReadMagRegisters(uint8_t reg, uint8_t count, uint8_t* data) 
    {
        Wire.beginTransmission(magAddress);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(magAddress, count);

        for(uint8_t i = 0; i < count; i++) data[i] = Wire.read();
    }

  public:
    
    MPU9250_Gyroscope() {}
    //  Constructor

    void Initialize() 
    {
        Wire.begin();
        
        //----  Initializing MPU9250 settings ----//

        WriteRegister(mpuAddress, PWR_MGMT_1, 0x00);
        WriteRegister(mpuAddress, 0x1B, 0x08);  
        //  Gyroscope ±500dps

        WriteRegister(mpuAddress, 0x1C, 0x10);  
        //  Acelerometer ±8g

        WriteRegister(mpuAddress, INT_PIN_CFG, 0x22); 
        //  Bypass mode for the magnitometer



        //  Calibrating the magnitomoter AK8963
        WriteRegister(magAddress, 0x0A, 0x0F); 
        //  Self-calibrating


        delay(100);
        CalibrateGyro();
        //  Calibrating the magnitometer
    }

    void Update() 
    {
        // 1. Чтение сырых данных
        float gx_raw, gy_raw, gz_raw;  // Гироскоп (градусы/с)
        float ax_raw, ay_raw, az_raw;  // Акселерометр (сырые значения)
        float mx_raw, my_raw, mz_raw;  // Магнитометр (сырые значения)

        ReadAccel(ax_raw, ay_raw, az_raw);
        ReadGyro (gx_raw, gy_raw, gz_raw);
        ReadMag  (mx_raw, my_raw, mz_raw);
        
        // 2. Конвертация гироскопа в радианы/с
        float gx = gx_raw * (M_PI / 180.0f);
        float gy = gy_raw * (M_PI / 180.0f);
        float gz = gz_raw * (M_PI / 180.0f);

        // 3. Нормализация акселерометра и магнитометра
        float acc_norm = sqrt(ax_raw*ax_raw + ay_raw*ay_raw + az_raw*az_raw);
        float ax = ax_raw / acc_norm;
        float ay = ay_raw / acc_norm;
        float az = az_raw / acc_norm;

        float mag_norm = sqrt(mx_raw*mx_raw + my_raw*my_raw + mz_raw*mz_raw);
        float mx = mx_raw / mag_norm;
        float my = my_raw / mag_norm;
        float mz = mz_raw / mag_norm;


        uint32_t current_time = millis();

        float dt  = ( current_time - prev_time ) * 0.001f;  //  Convert to seconds
        prev_time =   current_time;


        // 5. Вызов фильтра Madgwick
        float new_q0, new_q1, new_q2, new_q3;

        MadgwickUpdate(
            gx, gy, gz,      //  Угловые скорости в рад/с
            ax, ay, az,      //  Нормализованный акселерометр
            mx, my, mz,      //  Нормализованный магнитометр
            q0, q1, q2, q3,  //  Текущий кватернион
            beta,            //  Коэффициент фильтрации
            dt,              //  Временной шаг в секундах
            new_q0, new_q1, new_q2, new_q3  //  Выходные значения
        );

        // 6. Обновление кватерниона
        q0 = new_q0;
        q1 = new_q1;
        q2 = new_q2;
        q3 = new_q3;
    }

    void GetAngles(float& roll, float& pitch, float& yaw) 
    {
        roll = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
        pitch = asin(-2.0f * (q1*q3 - q0*q2));
        yaw = atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
        
        // Convert to degrees
        roll *= (180.0f / M_PI);
        pitch *= (180.0f / M_PI);
        yaw *= (180.0f / M_PI);
    }

    void CalibrateGyro() 
    {
        float sum[3] = {0};
        
        for(int i = 0; i < calibrationSamples; i++) {
            float gx, gy, gz;
            ReadGyro(gx, gy, gz); 
            
            sum[0] += gx;
            sum[1] += gy;
            sum[2] += gz;
            
            delay(5);
        }
        
        gyroBias[0] = sum[0] / calibrationSamples;
        gyroBias[1] = sum[1] / calibrationSamples;
        gyroBias[2] = sum[2] / calibrationSamples;
    }

    void ApplyGyroCalibration(float& gx, float& gy, float& gz) 
    {
        gx -= gyroBias[0];
        gy -= gyroBias[1];
        gz -= gyroBias[2];
    }
};