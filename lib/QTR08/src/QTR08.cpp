#include "QTR08.h"

// Constructor with individual pin parameters
QTR08::QTR08(uint8_t D1, uint8_t D2, uint8_t D3, uint8_t D4, uint8_t D5, uint8_t D6, uint8_t D7, uint8_t D8) {
    _sensorPins[0] = D1;
    _sensorPins[1] = D2;
    _sensorPins[2] = D3;
    _sensorPins[3] = D4;
    _sensorPins[4] = D5;
    _sensorPins[5] = D6;
    _sensorPins[6] = D7;
    _sensorPins[7] = D8;
    
    _debug = false;
    _calibrated = false;
    _calibrationDelay = 5000;
    _serialPort = &Serial;
    
    // Initialize arrays
    for (int i = 0; i < 8; i++) {
        _sensorValues[i] = 0;
        _calibratedValues[i] = 0;
    }
}

// Constructor with pin array parameter
QTR08::QTR08(uint8_t *sensorPins) {
    for (int i = 0; i < 8; i++) {
        _sensorPins[i] = sensorPins[i];
        _sensorValues[i] = 0;
        _calibratedValues[i] = 0;
    }
    
    _debug = false;
    _calibrated = false;
    _calibrationDelay = 5000;
    _serialPort = &Serial;
}

// Destructor
QTR08::~QTR08() {
    // Nothing to clean up
}

// Set sensor pins
void QTR08::setSensorPins(uint8_t *sensorPins) {
    for (int i = 0; i < 8; i++) {
        _sensorPins[i] = sensorPins[i];
    }
}

// Initialize the sensor array
void QTR08::begin(bool debug, Stream &serialPort) {
    _debug = debug;
    _serialPort = &serialPort;
    
    // Set all sensor pins as inputs
    for (int i = 0; i < 8; i++) {
        pinMode(_sensorPins[i], INPUT);
    }

    #ifdef STM32F4
    // Special handling for STM32F4
    for (int i = 0; i < 8; i++) {
        pinMode(_sensorPins[i], INPUT_ANALOG);
    } 
    
    if (_debug) {
        _serialPort->println("QTR08 Sensor Array Initialized");
        _serialPort->print("Sensor pins: ");
        for (int i = 0; i < 8; i++) {
            _serialPort->print(_sensorPins[i]);
            if (i < 7) _serialPort->print(", ");
        }
        _serialPort->println();
    }
}

// Private method to read raw sensor values
void QTR08::readSensors(uint16_t *sensorValues) {
    for (int i = 0; i < 8; i++) {
        sensorValues[i] = analogRead(_sensorPins[i]);
    }
}

// Public method to read sensor values (with calibration if available)
void QTR08::read(uint16_t *sensorValues) {
    // Read raw sensor values
    readSensors(_sensorValues);
    
    // Copy to output array
    for (int i = 0; i < 8; i++) {
        sensorValues[i] = _sensorValues[i];
    }
    
    if (_debug) {
        _serialPort->print("Raw sensor values: ");
        for (int i = 0; i < 8; i++) {
            _serialPort->print(sensorValues[i]);
            if (i < 7) _serialPort->print(", ");
        }
        _serialPort->println();
    }
}

// Calibrate the sensor array
void QTR08::calibrate(uint16_t delay) {
    _calibrationDelay = delay;
    uint16_t minValues[8];
    uint16_t maxValues[8];
    uint16_t tempValues[8];
    
    // Initialize min and max arrays
    for (int i = 0; i < 8; i++) {
        minValues[i] = 1023;
        maxValues[i] = 0;
    }
    
    if (_debug) {
        _serialPort->print("Starting calibration for ");
        _serialPort->print(_calibrationDelay);
        _serialPort->println(" ms");
        _serialPort->println("Move sensors over light and dark surfaces...");
    }
    
    unsigned long startTime = millis();
    
    // Collect calibration data
    while (millis() - startTime < _calibrationDelay) {
        readSensors(tempValues);
        
        // Update min and max values
        for (int i = 0; i < 8; i++) {
            if (tempValues[i] < minValues[i]) {
                minValues[i] = tempValues[i];
            }
            if (tempValues[i] > maxValues[i]) {
                maxValues[i] = tempValues[i];
            }
        }
        
        // Print progress every 500ms
        if (_debug && (millis() - startTime) % 500 == 0) {
            _serialPort->print("Calibrating... ");
            _serialPort->print((millis() - startTime) / 1000);
            _serialPort->println("s");
        }
        
        delay(10);
    }
    
    // Store calibration data
    for (int i = 0; i < 8; i++) {
        _calibratedValues[i] = (minValues[i] + maxValues[i]) / 2;
    }
    
    _calibrated = true;
    
    if (_debug) {
        _serialPort->println("Calibration complete!");
        _serialPort->print("Min values: ");
        for (int i = 0; i < 8; i++) {
            _serialPort->print(minValues[i]);
            if (i < 7) _serialPort->print(", ");
        }
        _serialPort->println();
        
        _serialPort->print("Max values: ");
        for (int i = 0; i < 8; i++) {
            _serialPort->print(maxValues[i]);
            if (i < 7) _serialPort->print(", ");
        }
        _serialPort->println();
        
        _serialPort->print("Threshold values: ");
        for (int i = 0; i < 8; i++) {
            _serialPort->print(_calibratedValues[i]);
            if (i < 7) _serialPort->print(", ");
        }
        _serialPort->println();
    }
}