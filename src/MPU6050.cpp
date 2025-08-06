#include <MPU6050.h>

MPU6050 :: MPU6050(TwoWire LocalWire)
{
    _LocalWire = LocalWire;
    _accelX = 0;
    _accelY = 0;
    _accelZ = 0;
    _temperature = 0;
    _gyroX = 0;
    _gyroY = 0;
    _gyroZ = 0;
    _angleX = 0.0f;
    _angleY = 0.0f;
    _angleZ = 0.0f;
    _accX_g = 0.0f;
    _accY_g = 0.0f;
    _accZ_g = 0.0f;
    _rateX = 0.0f;
    _rateY = 0.0f;
    _rateZ = 0.0f;
    _rateCalibrationRoll = 0.0f;
    _rateCalibrationPitch = 0.0f;
    _rateCalibrationYaw = 0.0f;
    _rateCalibrationNumber = 0;
}

MPU6050 :: ~MPU6050()
{

}

void MPU6050 :: begin()
{
    _LocalWire.setClock(MPU_I2C_SPEED);
    _LocalWire.begin();

    // Wake up the MPU6050 by clearing the sleep mode bit
    _LocalWire.beginTransmission(MPU6050_ADDR);
    _LocalWire.write(PWR_MGMT_1);
    _LocalWire.write(0); 
    _LocalWire.endTransmission(true);

    // Set the DLPF (Digital Low Pass Filter) configuration
    _LocalWire.beginTransmission(MPU6050_ADDR);
    _LocalWire.write(DLPF_REGISTER);
    _LocalWire.write(DLPF_CFG);
    _LocalWire.endTransmission(true);

    // gyroscope configuration
    _LocalWire.beginTransmission(MPU6050_ADDR);
    _LocalWire.write(GYRO_CONFIG);
    _LocalWire.write(GYRO_FS_SEL);
    _LocalWire.endTransmission(true);

    // accelerometer configuration
    _LocalWire.beginTransmission(MPU6050_ADDR);
    _LocalWire.write(ACCEL_CONFIG);
    _LocalWire.write(ACCEL_FS_SEL);
    _LocalWire.endTransmission(true);

}

void MPU6050 :: caliberate()
{
    
}

void MPU6050 :: readSensor()
{
    // Request Accelerometer data
    _LocalWire.beginTransmission(MPU6050_ADDR);
    _LocalWire.write(ACCEL_XOUT_H);
    _LocalWire.endTransmission(false);

    // read 6 bytes
    _LocalWire.requestFrom(MPU6050_ADDR, 6, true);
  
    // Read accelerometer data
    _accelX = _LocalWire.read() << 8 | _LocalWire.read();
    _accelY = _LocalWire.read() << 8 | _LocalWire.read();
    _accelZ = _LocalWire.read() << 8 | _LocalWire.read();
    
    // Request temperature data 
    _LocalWire.beginTransmission(MPU6050_ADDR);
    _LocalWire.write(TEMP_OUT_H);
    _LocalWire.endTransmission(false);

    // Read 2 bytes for temperature
    _LocalWire.requestFrom(MPU6050_ADDR, 2, true);

    // Read temperature data
    _temperature = _LocalWire.read() << 8 | _LocalWire.read();

    // Request gyroscope data
    _LocalWire.beginTransmission(MPU6050_ADDR);
    _LocalWire.write(GYRO_XOUT_H);
    _LocalWire.endTransmission(false);

    // Read 6 bytes for gyroscope
    _LocalWire.requestFrom(MPU6050_ADDR, 6, true);

    // Read gyroscope data
    _gyroX = _LocalWire.read() << 8 | _LocalWire.read();
    _gyroY = _LocalWire.read() << 8 | _LocalWire.read();
    _gyroZ = _LocalWire.read() << 8 | _LocalWire.read();

    _accX_g = _accelX / 4096.0f;
    _accY_g = _accelY / 4096.0f;
    _accZ_g = _accelZ / 4096.0f;

    _rateX = _gyroX / 65.5f;
    _rateY = _gyroY / 65.5f;
    _rateZ = _gyroZ / 65.5f;

    _angleX = atan(_accY_g/sqrt(_accX_g * _accX_g + _accZ_g * _accZ_g)) * 180.0f / PI;
    _angleY = atan(-_accX_g/sqrt(_accY_g * _accY_g + _accZ_g * _accZ_g)) * 180.0f / PI;
    _angleZ += _rateZ * 0.01f; // Assuming update is called every 10ms
}

void MPU6050 :: update()
{
    readSensor();
}

float MPU6050 :: getAngleX()
{
    return _angleX;
}

float MPU6050 :: getAngleY()
{
    return _angleY;
}

float MPU6050 :: getAngleZ()
{
    return _angleZ;
}

int16_t MPU6050 :: getAccelX()
{
    return _accelX;
}

int16_t MPU6050 :: getAccelY()
{
    return _accelY;
}

int16_t MPU6050 :: getAccelZ()
{
    return _accelZ;
}

int16_t MPU6050 :: getTemperature()
{
    return _temperature;
}

int16_t MPU6050 :: getGyroX()
{
    return _gyroX;
}

int16_t MPU6050 :: getGyroY()
{
    return _gyroY;
}

int16_t MPU6050 :: getGyroZ()
{
    return _gyroZ;
}

