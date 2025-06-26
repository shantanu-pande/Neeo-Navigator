#ifndef QTR08_H
#define QTR08_H

#include <Arduino.h>

class QTR08 {

private:
    uint8_t _D1, _D2, _D3, _D4, _D5, _D6, _D7, _D8;
    bool _debug;
    Stream *_serialPort;

    void readSensors(uint16_t *sensorValues);
    void calibrateSensors(uint16_t *sensorValues);

    public:
    QTR08(uint8_t D1, uint8_t D2, uint8_t D3, uint8_t D4, uint8_t D5, uint8_t D6, uint8_t D7, uint8_t D8);
    void begin(bool debug = false, Stream &serialPort = Serial);
    void calibrate(uint16_t delay = 5000);
};

#endif