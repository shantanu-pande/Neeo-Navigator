#include <Arduino.h>


const int adcPins[] = {A2, A3, A4, A5, A6, A7, A8, A9};
const int numChannels = 8;


uint16_t minValues[numChannels];
uint16_t maxValues[numChannels];
bool isCalibrated = false;

// PID controller constants
const float KP = 55.00;     // Proportional gain
const float KI = 0.00;    // Integral gain
const float KD = 35;      // Derivative gain
const int BASE_SPEED = 50; // Base motor speed (PWM value)

// Global variables for line tracking
float lastValidPosition = 3.5;  // Start assuming center position
bool lineWasLost = false;


// Function prototypes
void performCalibration(unsigned long duration);
void readSensors(uint16_t *values);
void readCalibratedSensors(uint16_t *values);
void getDigitalReadings(uint16_t *calibratedValues, bool *digitalValues, uint16_t threshold = 500);
void printDigitalPattern(bool *digitalValues);
void printSensorBar(uint16_t *values);
float calculatePosition(uint16_t *values);

void move(int left_speed, int right_speed);


void setup() {
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
 