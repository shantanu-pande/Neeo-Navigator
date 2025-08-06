#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

#define MPU_I2C_SPEED 400000
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H   0x41
#define TEMP_OUT_L   0x42
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48
#define DLPF_REGISTER 0x1A
#define DLPF_CFG     0x05
#define GYRO_CONFIG  0x1B
#define GYRO_FS_SEL  0x08 
#define ACCEL_CONFIG 0x1C
#define ACCEL_FS_SEL 0x10


class MPU6050 {
    private:
        TwoWire _LocalWire;
        int16_t _accelX, _accelY, _accelZ;
        int16_t _temperature;
        int16_t _gyroX, _gyroY, _gyroZ;

        float _angleX, _angleY, _angleZ;

        float _accX_g, _accY_g, _accZ_g;
        float _rateX, _rateY, _rateZ;

        float _rateCalibrationRoll, _rateCalibrationPitch, _rateCalibrationYaw;
        int _rateCalibrationNumber;

        void readSensor();

    public:
        MPU6050(TwoWire LocalWire);
        ~MPU6050();
        void begin();
        void caliberate();
        void update();
        
        int16_t getAccelX();
        int16_t getAccelY();
        int16_t getAccelZ();

        int16_t getTemperature();

        int16_t getGyroX();
        int16_t getGyroY();
        int16_t getGyroZ();

        float getAngleX();
        float getAngleY();
        float getAngleZ();

};


#endif