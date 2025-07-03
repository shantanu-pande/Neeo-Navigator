#include <Arduino.h>
#include <Wire.h>

// MPU6050 I2C address
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

// Variables to store sensor data
int16_t accelX, accelY, accelZ;
int16_t temperature;
int16_t gyroX, gyroY, gyroZ;
int16_t kalmanX, kalmanY, kalmanZ;


// Create I2C3 instance
TwoWire Wire3(PB4, PA8); // SDA=PB4, SCL=PA8

void readMPU6050();

void setup() {
  // Initialize Serial1 on pins A9 (TX) and A10 (RX)
  Serial1.begin(115200);
  
  // Initialize I2C3 communication with custom pins
  Wire3.begin();
  
  // Wait for serial connection
  delay(2000);
  
  Serial1.println("MPU6050 Reader - I2C3 (SDA=PB4, SCL=PA8)");
  Serial1.println("========================================");
  
  // Wake up MPU6050 (it starts in sleep mode)
  Wire3.beginTransmission(MPU6050_ADDR);
  Wire3.write(PWR_MGMT_1);
  Wire3.write(0); // Set to zero (wakes up the MPU6050)
  Wire3.endTransmission(true);
  
  delay(100);
  Serial1.println("MPU6050 initialized successfully on I2C3!");
}

void loop() {
  // Read accelerometer and gyroscope data
  readMPU6050();
  
  // Print accelerometer data
  Serial1.println("\n--- MPU6050 Data ---");
  Serial1.print("Accelerometer: ");
  Serial1.print("X="); Serial1.print(accelX);
  Serial1.print(" Y="); Serial1.print(accelY);
  Serial1.print(" Z="); Serial1.println(accelZ);
  
  // Print gyroscope data
  Serial1.print("Gyroscope:     ");
  Serial1.print("X="); Serial1.print(gyroX);
  Serial1.print(" Y="); Serial1.print(gyroY);
  Serial1.print(" Z="); Serial1.println(gyroZ);
  
  // Print temperature
  Serial1.print("Temperature:   ");
  Serial1.print(temperature / 340.0 + 36.53);
  Serial1.println(" °C");
  
  delay(500); // Read every 500ms
}

void readMPU6050() {
  // Request 14 bytes from MPU6050 starting from ACCEL_XOUT_H
  Wire3.beginTransmission(MPU6050_ADDR);
  Wire3.write(ACCEL_XOUT_H);
  Wire3.endTransmission(false);
  Wire3.requestFrom(MPU6050_ADDR, 14, true);
  
  // Read accelerometer data
  accelX = Wire3.read() << 8 | Wire3.read();
  accelY = Wire3.read() << 8 | Wire3.read();
  accelZ = Wire3.read() << 8 | Wire3.read();
  
  // Read temperature data
  temperature = Wire3.read() << 8 | Wire3.read();
  
  // Read gyroscope data
  gyroX = Wire3.read() << 8 | Wire3.read();
  gyroY = Wire3.read() << 8 | Wire3.read();
  gyroZ = Wire3.read() << 8 | Wire3.read();
}