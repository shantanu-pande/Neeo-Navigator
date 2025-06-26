#include "VL53L0X.h"

VL53L0X::VL53L0X(TwoWire* wire, uint8_t address) {
    _wire = wire;
    _address = address;
}

bool VL53L0X::begin() {
    // Check if device is present
    uint8_t deviceId = readReg(VL53L0X_IDENTIFICATION_MODEL_ID);
    if (deviceId != VL53L0X_EXPECTED_DEVICE_ID) {
        return false;
    }
    
    // VL53L0X initialization sequence
    writeReg(0x88, 0x00);
    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    writeReg(0x91, 0x3c);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);
    
    // Set I2C standard mode
    writeReg(0x88, 0x00);
    
    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    writeReg(0x91, 0x3C);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);
    
    return true;
}

uint16_t VL53L0X::readRangeSingleMillimeters() {
    // Start single ranging measurement
    writeReg(SYSRANGE_START, 0x01);
    
    // Wait for measurement to complete
    uint8_t status;
    do {
        status = readReg(RESULT_RANGE_STATUS);
        delay(1);
    } while ((status & 0x01) == 0);
    
    // Read distance
    uint16_t distance = readReg16(RESULT_RANGE_STATUS + 10);
    
    // Clear interrupt
    writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
    
    return distance;
}

uint16_t VL53L0X::readRangeContinuousMillimeters() {
    // In continuous mode, just read the latest measurement
    uint8_t status = readReg(RESULT_RANGE_STATUS);
    
    // Check if new measurement is available
    if ((status & 0x01) == 0) {
        return 0; // No new measurement available
    }
    
    // Read distance
    uint16_t distance = readReg16(RESULT_RANGE_STATUS + 10);
    
    // Clear interrupt
    writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
    
    return distance;
}

void VL53L0X::startContinuous(uint32_t period_ms) {
    // Set intermeasurement period (in ms)
    if (period_ms < 33) period_ms = 33; // Minimum period
    writeReg16(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
    
    // Start continuous ranging
    writeReg(SYSRANGE_START, 0x02);
}

void VL53L0X::stopContinuous() {
    // Stop continuous ranging
    writeReg(SYSRANGE_START, 0x01);
}

void VL53L0X::setAddress(uint8_t new_address) {
    // Validate new address (must be 7-bit address, so between 0x08 and 0x77)
    if (new_address < 0x08 || new_address > 0x77) {
        return; // Invalid address
    }
    
    // Write the new I2C address to the device
    writeReg(I2C_SLAVE_DEVICE_ADDRESS, new_address);
    
    // Update internal address
    _address = new_address;
    
    // Small delay to ensure the change takes effect
    delay(10);
}

uint8_t VL53L0X::getAddress() const {
    return _address;
}

// Private helper functions
void VL53L0X::writeReg(uint8_t reg, uint8_t value) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->write(value);
    _wire->endTransmission();
}

uint8_t VL53L0X::readReg(uint8_t reg) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_address, (uint8_t)1);
    return _wire->read();
}

void VL53L0X::writeReg16(uint8_t reg, uint16_t value) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->write((value >> 8) & 0xFF);
    _wire->write(value & 0xFF);
    _wire->endTransmission();
}

uint16_t VL53L0X::readReg16(uint8_t reg) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_address, (uint8_t)2);
    return (_wire->read() << 8) | _wire->read();
}