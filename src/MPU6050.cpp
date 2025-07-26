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
    _angleX = 0;
    _angleY = 0;
    _angleZ = 0;
}

MPU6050 :: ~MPU6050()
{

}

void MPU6050 :: begin()
{
    _LocalWire.begin();
    _LocalWire.beginTransmission(MPU6050_ADDR);
    _LocalWire.write(PWR_MGMT_1);
    _LocalWire.write(0); 
    _LocalWire.endTransmission(true);
}

void MPU6050 :: readSensor()
{
    _LocalWire.beginTransmission(MPU6050_ADDR);
    _LocalWire.write(ACCEL_XOUT_H);
    _LocalWire.endTransmission(false);
    _LocalWire.requestFrom(MPU6050_ADDR, 14, true);
  
    // Read accelerometer data
    _accelX = _LocalWire.read() << 8 | _LocalWire.read();
    _accelY = _LocalWire.read() << 8 | _LocalWire.read();
    _accelZ = _LocalWire.read() << 8 | _LocalWire.read();
    
    // Read temperature data
    _temperature = _LocalWire.read() << 8 | _LocalWire.read();

    // Read gyroscope data
    _gyroX = _LocalWire.read() << 8 | _LocalWire.read();
    _gyroY = _LocalWire.read() << 8 | _LocalWire.read();
    _gyroZ = _LocalWire.read() << 8 | _LocalWire.read();
}

void MPU6050 :: update()
{
    readSensor();
}

int16_t MPU6050 :: getAngleX()
{
    return _angleX;
}

int16_t MPU6050 :: getAngleY()
{
    return _angleY;
}

int16_t MPU6050 :: getAngleZ()
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

