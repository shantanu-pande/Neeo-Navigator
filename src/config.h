#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

#define CONFIG_VERSION "1.0.0"
#define CONFIG_AUTHOR "Shantanu Pande"
#define CONFIG_PROJECT_NAME "NEEO-V2"

#define LF_Kp 1
#define LF_Ki 0.1
#define LF_Kd 0.01

#define LED_PIN PC13
#define BUTTON_1_PIN PA13
#define BUTTON_2_PIN PA14
#define BUZZER_PIN PB5

#define I2C_PIN_SDA PB4
#define I2C_PIN_SCL PA8
#define I2C_SPEED 400000 // 400kHz

#define MPU6050_ADDRESS 0x68 // Default I2C address for MPU6050

#define VL53L0X_DEFAULT_ADDRESS 0x29 // Default I2C address for VL53L0X
#define VL53L0X_LEFT_ADDRESS 0x30 // Left sensor address
#define VL53L0X_RIGHT_ADDRESS 0x31 // Left sensor address
#define VL53L0X_FRONT_ADDRESS 0x32 // Front sensor address
#define X_RIGHT PB2
#define X_LEFT PB10
#define X_FRONT PB12

#define TX1_PIN PA9
#define RX1_PIN PA10

#define IR1_PIN A2
#define IR2_PIN A3
#define IR3_PIN A4
#define IR4_PIN A5
#define IR5_PIN A6
#define IR6_PIN A7
#define IR7_PIN A8
#define IR8_PIN A9

#define ENC_R_A PA0
#define ENC_R_B PA1
#define ENC_L_A PA15
#define ENC_L_B PB3

#define RIGHT_CLOCKWISE PB9
#define RIGHT_ANTICLOCKWISE PB8
#define LEFT_CLOCKWISE PB7
#define LEFT_ANTICLOCKWISE PB6

#endif// CONFIG_H