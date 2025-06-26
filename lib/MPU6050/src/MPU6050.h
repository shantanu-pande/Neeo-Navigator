#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <math.h>

#define MPU6050_ADDR 0x68

// MPU6050 register addresses
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

class MPU6050 {
    private:
        TwoWire &_wire;
        int16_t accelX, accelY, accelZ;
        int16_t temperature;
        int16_t gyroX, gyroY, gyroZ;
        int16_t kalmanX, kalmanY, kalmanZ;

        float qAngle = 0.001f; // Process noise variance for the accelerometer
        float qBias = 0.003f;  // Process noise variance for the gyroscope
        float rMeasure = 0.03f; // Measurement noise variance
        float angleX = 0.0f;   // Angle around the X-axis
        float angleY = 0.0f;   // Angle around the Y-axis
        float angleZ = 0.0f;   // Angle around the Z-axis
        float biasX = 0.0f;    // Bias around the X-axis
        float biasY = 0.0f;    // Bias around the Y-axis
        float biasZ = 0.0f;    // Bias around the Z-axis
        float kalmanGainX = 0.0f; // Kalman gain for X-axis
        float kalmanGainY = 0.0f; // Kalman gain for Y
        float kalmanGainZ = 0.0f; // Kalman gain for Z-axis
        float P[3][3] = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} }; // Error covariance matrix
        float dt = 0.01f; // Time step in seconds
        void writeRegister(uint8_t reg, uint8_t value);
        uint8_t readRegister(uint8_t reg);
        void readRegisters(uint8_t reg, uint8_t *buffer, size_t length);
        void updateKalmanFilter(float accel, float gyro, float &angle, float &bias, float &kalmanGain);
        readMPU6050();
    public:
        MPU6050(TwoWire &wire = Wire) : _wire(wire);
        void begin();
        void readSensorData();
        int16_t getAccelX() const { return accelX; }
        int16_t getAccelY() const { return accelY; }
        int16_t getAccelZ() const { return accelZ; }
        int16_t getTemperature() const { return temperature; }
        int16_t getGyroX() const { return gyroX; }
        int16_t getGyroY() const { return gyroY; }
        int16_t getGyroZ() const { return gyroZ; }
        int16_t getKalmanX() const { return kalmanX; }
        int16_t getKalmanY() const { return kalmanY; }
        int16_t getKalmanZ() const { return kalmanZ; }
        void printSensorData(Stream &stream = Serial);
        void calibrate();
        void setKalmanFilter(float q, float r);
        void updateSensorData();
        void reset();
};

#endif // MPU6050_H