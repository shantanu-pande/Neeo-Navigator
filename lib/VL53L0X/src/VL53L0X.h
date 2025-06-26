#ifndef VL53L0X_H
#define VL53L0X_H

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

// VL53L0X register addresses
#define SYSRANGE_START                              0x00
#define SYSTEM_THRESH_HIGH                          0x0C
#define SYSTEM_THRESH_LOW                           0x0E
#define SYSTEM_SEQUENCE_CONFIG                      0x01
#define SYSTEM_RANGE_CONFIG                         0x09
#define SYSTEM_INTERMEASUREMENT_PERIOD              0x04
#define SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A
#define GPIO_HV_MUX_ACTIVE_HIGH                     0x84
#define SYSTEM_INTERRUPT_CLEAR                      0x0B
#define RESULT_INTERRUPT_STATUS                     0x13
#define RESULT_RANGE_STATUS                         0x14
#define I2C_SLAVE_DEVICE_ADDRESS                    0x8A
#define VL53L0X_EXPECTED_DEVICE_ID                  0xEE
#define VL53L0X_IDENTIFICATION_MODEL_ID             0xC0


class VL53L0X {
    private:
        uint8_t _address;
        TwoWire* _wire;

        void writeReg(uint8_t reg, uint8_t value);
        uint8_t readReg(uint8_t reg);
        void writeReg16(uint8_t reg, uint16_t value);
        uint16_t readReg16(uint8_t reg);
    public:
        VL53L0X(TwoWire* wire = &Wire, uint8_t address = 0x29);
        bool begin();
        uint16_t readRangeSingleMillimeters();
        uint16_t readRangeContinuousMillimeters();
        void startContinuous(uint32_t period_ms = 150);
        void stopContinuous();
        void setAddress(uint8_t new_address);
        uint8_t getAddress() const;
};

#endif