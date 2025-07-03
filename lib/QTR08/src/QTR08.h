#ifndef QTR08_H
#define QTR08_H

#include <Arduino.h>

class QTR08 {

private:
    uint8_t _sensorPins[8]; 
    uint16_t _sensorValues[8];
    uint16_t _calibratedValues[8];
    uint16_t _calibrationDelay;
    bool _debug;
    bool _calibrated;
    Stream *_serialPort;

    void readSensors(uint16_t *sensorValues);

public:
    QTR08(uint8_t D1, uint8_t D2, uint8_t D3, uint8_t D4, uint8_t D5, uint8_t D6, uint8_t D7, uint8_t D8);
    QTR08(uint8_t *sensorPins);
    ~QTR08();
    void setSensorPins(uint8_t *sensorPins);
    void read(uint16_t *sensorValues);
    void begin(bool debug = false, Stream &serialPort = Serial);
    void calibrate(uint16_t delay = 5000);
};

#endif