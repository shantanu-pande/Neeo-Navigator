#include <Arduino.h>
#include "MPU6050.h"

TwoWire Wire3(PB4, PA8); 

MPU6050 imu(Wire3);

void setup()
{
    Serial1.begin(115200);
    delay(100);
    imu.begin();
    delay(100);

}

void loop()
{
    imu.update();
    // Serial1.print("Accelerometer: ");
    Serial1.print(">AccelX:"); Serial1.println(imu.getAccelX());
    Serial1.print(">AccelY:"); Serial1.println(imu.getAccelY());
    Serial1.print(">AccelZ:"); Serial1.println(imu.getAccelZ());

    // Serial1.print("Gyroscope:     ");
    Serial1.print(">GyroX:"); Serial1.println(imu.getGyroX());
    Serial1.print(">GyroY:"); Serial1.println(imu.getGyroY());
    Serial1.print(">GyroZ:"); Serial1.println(imu.getGyroZ());

    // Serial1.print("Angles:        ");
    Serial1.print(">AngleX:"); Serial1.println(imu.getAngleX());
    Serial1.print(">AngleY:"); Serial1.println(imu.getAngleY());
    Serial1.print(">AngleZ:"); Serial1.println(imu.getAngleZ());
    // Serial1.print("Temperature:   ");
    Serial1.print(">Temp:"); Serial1.println(imu.getTemperature() / 340.0 + 36.53);
    // Serial1.println(" °C");

    delay(50);
}
 