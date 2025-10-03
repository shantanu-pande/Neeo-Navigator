#include <Arduino.h>


const int adcPins[] = {A2, A3, A4, A5, A6, A7, A8, A9};
const int numChannels = 8;


uint16_t minValues[numChannels];
uint16_t maxValues[numChannels];
bool isCalibrated = false;

// PID controller constants
const float KP = 55.00;     // Proportional gain
const float KI = 0.00;    // Integral gain
const float KD = 45;      // Derivative gain
const int BASE_SPEED =44; // Base motor speed (PWM value)

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
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, LOW); // Turn off LED
    Serial1.println("Starting");


    // Pin Setup
    pinMode(PB6, OUTPUT);
    pinMode(PB7, OUTPUT);
    pinMode(PB8, OUTPUT);
    pinMode(PB9, OUTPUT);

    
    // Configure ADC resolution (12-bit for STM32)
    analogReadResolution(12);
    
    // Initialize with default values
    for (int i = 0; i < numChannels; i++) {
      minValues[i] = 4095; // Maximum possible 12-bit ADC value
      maxValues[i] = 0;    // Minimum possible value
    }
    
    delay(5000);
    // move(255, 255);
    // delay(5000);
    // move(-100, -100);
    // delay(1000);
    move(0,0);

    performCalibration(3000);
  }

// PID control variables
float lastError = 0;
float integral = 0;

void loop() {
  uint16_t values[numChannels];
  bool digitalValues[numChannels];
        
  readCalibratedSensors(values);
  getDigitalReadings(values, digitalValues);
        
  // Calculate position of line
  float position = calculatePosition(values);
  
  // Calculate PID control values
  float error = position - 3.5; // Center position is 3.5 (between sensors 3 and 4)
  
  // If line is lost, use previous error to maintain direction
  if (lineWasLost) {
    digitalWrite(PC13, HIGH);  // Turn on LED when line is lost
    error = lastError;         // Use previous error to maintain trajectory
  } else {
    digitalWrite(PC13, LOW);   // Turn off LED when line is detected
  }
  
  // Integral term with anti-windup
  integral = integral + error;
  integral = constrain(integral, -100, 100);  // Prevent excessive buildup
  
  // Derivative term
  float derivative = error - lastError;
  
  // Only update lastError if we have a valid line reading
  if (!lineWasLost) {
    lastError = error;
  }
  
  // Calculate motor adjustment using PID
  float motorAdjustment = KP * error + KI * integral + KD * derivative;
  
  // Apply motor speeds with PID adjustment
  int leftSpeed = BASE_SPEED - motorAdjustment;
  int rightSpeed = BASE_SPEED + motorAdjustment;
  
  // Display debug information
  // Serial1.print("Position: ");
  // Serial1.print(position);
  // Serial1.print(" | Error: ");
  // Serial1.print(error);
  // Serial1.print(" | Adjustment: ");
  // Serial1.print(motorAdjustment);
  // Serial1.print(" | Motors L:");
  // Serial1.print(leftSpeed);
  // Serial1.print(" R:");
  // Serial1.println(rightSpeed);
  
  // Print visual representation of line position
  printDigitalPattern(digitalValues);
  
  if (lineWasLost) {
    Serial1.println("Line lost! Using previous error.");
  }
  
  // Move robot according to PID output
  move(leftSpeed, rightSpeed);
  
  delay(10);  // Faster loop for better control
}


void move(int left_speed, int right_speed) {
    if (left_speed > 255) left_speed = 255;
    if (left_speed < -255) left_speed = -255;
    if (right_speed > 255) right_speed = 255;
    if (right_speed < -255) right_speed = -255;

    // Left motor control
    if (left_speed > 0) {
        analogWrite(PB6, left_speed);  // Forward
        analogWrite(PB7, 0);
    } else if (left_speed < 0) {
        analogWrite(PB6, 0);
        analogWrite(PB7, -left_speed); // Backward
    } else {
        analogWrite(PB6, 0);
        analogWrite(PB7, 0);           // Stop
    }

    // Right motor control
    if (right_speed > 0) {
        analogWrite(PB9, right_speed);  // Forward
        analogWrite(PB8, 0);
    } else if (right_speed < 0) {
        analogWrite(PB9, 0);
        analogWrite(PB8, -right_speed); // Backward
    } else {
        analogWrite(PB9, 0);
        analogWrite(PB8, 0);            // Stop
    }
}


void readSensors(uint16_t *values) {
    for (int i = 0; i < numChannels; i++) {
        values[i] = analogRead(adcPins[i]);
    }
}

// Read calibrated (normalized) sensor values
void readCalibratedSensors(uint16_t *values) {
    uint16_t rawValues[numChannels];
    readSensors(rawValues);
    
    for (int i = 0; i < numChannels; i++) {
        // Constrain the value to be within calibrated range
        uint16_t value = constrain(rawValues[i], minValues[i], maxValues[i]);
        
        // Map to 0-1000 range (inverted so black/high reflectance = 1000)
        values[i] = map(value, minValues[i], maxValues[i], 0, 1000);
    }
}

// Perform calibration routine
void performCalibration(unsigned long duration) {
    // Reset calibration values
    for (int i = 0; i < numChannels; i++) {
        minValues[i] = 4095; // Maximum 12-bit ADC value
        maxValues[i] = 0;
    }
    
    // Set start time
    unsigned long startTime = millis();
    
    // LED blink during calibration (fast)
    const int blinkInterval = 100; // ms
    unsigned long lastBlink = 0;
    bool ledState = false;
    
    Serial1.println("Calibrating...");
    move(-50, 50);
    delay(500);
    while (millis() - startTime < duration) {
        // Blink LED to indicate calibration in progress
        if (millis() - lastBlink > blinkInterval) {
            ledState = !ledState;
            digitalWrite(PC13, ledState ? HIGH : LOW);
            lastBlink = millis();
        }
        
        // Read current values
        uint16_t currentValues[numChannels];
        readSensors(currentValues);
        
        // Update min and max values
        for (int i = 0; i < numChannels; i++) {
            if (currentValues[i] < minValues[i]) {
                minValues[i] = currentValues[i];
            }
            if (currentValues[i] > maxValues[i]) {
                maxValues[i] = currentValues[i];
            }
        }
        
        // Short delay between readings
        delay(10);
    }
    
    // Turn off LED when calibration is complete
    digitalWrite(PC13, LOW);
    move(0,0);
    delay(500);
    // Display calibration results
    Serial1.println("Calibration complete!");
    Serial1.println("Calibrated values:");

    
    for (int i = 0; i < numChannels; i++) {
        Serial1.print("Sensor ");
        Serial1.print(i);
        Serial1.print(": Min = ");
        Serial1.print(minValues[i]);
        Serial1.print(", Max = ");
        Serial1.println(maxValues[i]);
    }
    
    isCalibrated = true;
}

// Convert calibrated values to digital (0 or 1)
void getDigitalReadings(uint16_t *calibratedValues, bool *digitalValues, uint16_t threshold) {
    for (int i = 0; i < numChannels; i++) {
        // If the calibrated value is above threshold, consider it a line detection (1)
        // Otherwise, it's not detecting a line (0)
        digitalValues[i] = (calibratedValues[i] > threshold);
    }
}

// Function to print the digital values as a simple binary pattern
void printDigitalPattern(bool *digitalValues) {
    // Serial1.print("Digital pattern: [");
    // for (int i = 0; i < numChannels; i++) {
    //     Serial1.print(digitalValues[i] ? "1" : "0");
    //     if (i < numChannels - 1) Serial1.print(" ");
    // }
    // Serial1.println("]");
    
    // Visual representation
    Serial1.print("Line position:  ");
    for (int i = 0; i < numChannels; i++) {
        Serial1.print(digitalValues[i] ? "■" : "□");
        Serial1.print(" ");
    }
    Serial1.println();
}

// Calculate weighted position of the line
float calculatePosition(uint16_t *values) {
    float sum = 0;
    float weightedSum = 0;
    
    for (int i = 0; i < numChannels; i++) {
        sum += values[i];
        weightedSum += i * values[i];
    }
    
    // Check if line is lost (using threshold to determine if any significant readings)
    if (sum < 500) {  // Adjust threshold based on your sensor sensitivity
        lineWasLost = true;
        // Serial1.println("Line lost! Using previous position.");
        return lastValidPosition;  // Return last valid position when line is lost
    }
    
    // Calculate weighted average (center of line)
    float position = weightedSum / sum;
    
    // Store valid position for future reference
    lastValidPosition = position;
    lineWasLost = false;
    
    return position;
}

// Print a graphical bar representing sensor values
void printSensorBar(uint16_t *values) {
    Serial1.println("Sensor bar representation:");
    
    // Determine the scale factor for printing
    uint16_t maxVal = 0;
    for (int i = 0; i < numChannels; i++) {
        if (values[i] > maxVal) maxVal = values[i];
    }
    
    const int barWidth = 50;  // Maximum bar length
    float scale = (float)barWidth / maxVal;
    
    // Print bars for each sensor
    for (int i = 0; i < numChannels; i++) {
        Serial1.print("S");
        Serial1.print(i);
        Serial1.print(": ");
        
        int barLength = (int)(values[i] * scale);
        for (int j = 0; j < barLength; j++) {
            Serial1.print("█");
        }
        Serial1.print(" ");
        Serial1.println(values[i]);
    }
    Serial1.println();
}