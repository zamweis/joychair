#include <Wire.h>
#include <Joystick.h>
#include <MPU6050.h>
#include <algorithm>

MPU6050 mpu;
int16_t accelerationX;
int16_t accelerationY;
int16_t accelerationZ;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;

int16_t accelerationXOffset;
int16_t accelerationYOffset;
int16_t accelerationZOffset;
int16_t gyroXOffset;
int16_t gyroYOffset;
int16_t gyroZOffset;

Joystick_ joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                   7, 0,                // 7 Buttons, no Hat Switch
                   true, true, false,   // X and Y Axes (used for left-right and forward-backward)
                   false, false, true); // No Rx, No Ry, Rz for Rotation (used for rotating left-right)

const int numReadings = 3000;
const int gyroThreshold = 6000;
const int jumpThreshold = 10000;

// Declaring acceleration threshold variable
const int accelerationThreshold = 6000;

// Deadzone variables
float accelerationXDeadzone;
float accelerationYDeadzone;
float accelerationZDeadzone;

void calibrateSensors();
void checkForMovement();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  Serial.println("Initializing...");
  
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1); // Halt if connection failed
  }
  
  calibrateSensors(); // Perform sensor calibration
  
  joystick.begin();
  Serial.println("Device is calibrated and ready!");
}

void loop() {
  static unsigned long lastMillis = 0;
  if (millis() - lastMillis >= 10) {
    lastMillis = millis();
    
    mpu.getMotion6(&accelerationX, &accelerationY, &accelerationZ, &gyroX, &gyroY, &gyroZ);

    accelerationX -= accelerationXOffset;
    accelerationY -= accelerationYOffset;
    accelerationZ -= accelerationZOffset;
    gyroX -= gyroXOffset;
    gyroY -= gyroYOffset;
    gyroZ -= gyroZOffset;

    checkForMovement();
  }
}

void checkForMovement() {
    // Map movements to joystick controls
    joystick.setXAxis(abs(accelerationX) > accelerationXDeadzone ? accelerationX : 0);
    joystick.setYAxis(abs(accelerationY) > accelerationYDeadzone ? accelerationY : 0);
    joystick.setRzAxis(abs(gyroZ) > gyroThreshold ? gyroZ : 0);

    // Handle specific buttons for rotational moves
    joystick.setButton(1, gyroX > gyroThreshold ? 1 : 0);    // Rotate right
    joystick.setButton(2, gyroX < -gyroThreshold ? 1 : 0);   // Rotate left

    // Directions based on accelerometer thresholds
    joystick.setButton(3, accelerationX > accelerationThreshold ? 1 : 0);   // Move right
    joystick.setButton(4, accelerationX < -accelerationThreshold ? 1 : 0);  // Move left
    joystick.setButton(5, accelerationY > accelerationThreshold ? 1 : 0);   // Move backward
    joystick.setButton(6, accelerationY < -accelerationThreshold ? 1 : 0);  // Move forward

    // Output detected movements to the console or update game state
    Serial.print("Smoothed Ax: "); Serial.print(accelerationX); Serial.print(", ");
    Serial.print("Smoothed Ay: "); Serial.print(accelerationY); Serial.print(", ");
    Serial.print("Smoothed Az: "); Serial.print(accelerationZ); Serial.print(", ");
    Serial.print("Smoothed Gx: "); Serial.print(gyroX); Serial.print(", ");
    Serial.print("Smoothed Gy: "); Serial.print(gyroY); Serial.print(", ");
    Serial.print("Smoothed Gz: "); Serial.println(gyroZ);
}

void calibrateSensors() {
    int16_t accelerationXReading, accelerationYReading, accelerationZReading;
    int16_t gyroXReading, gyroYReading, gyroZReading;

    Serial.println("Calibrating sensors, make sure the device is stationary and flat...");

    // Discard initial readings to eliminate potential drift
    for (int i = 0; i < 100; i++) {
        mpu.getMotion6(&accelerationXReading, &accelerationYReading, &accelerationZReading, &gyroXReading, &gyroYReading, &gyroZReading);
        delay(10);
    }

    // Collect sufficient data for calibration
    int16_t accelerationXReadings[numReadings];
    int16_t accelerationYReadings[numReadings];
    int16_t accelerationZReadings[numReadings];

    for (int i = 0; i < numReadings; i++) {
        mpu.getMotion6(&accelerationXReading, &accelerationYReading, &accelerationZReading, &gyroXReading, &gyroYReading, &gyroZReading);
        
        // Accumulate sensor readings
        accelerationXReadings[i] = accelerationXReading; 
        accelerationYReadings[i] = accelerationYReading; 
        accelerationZReadings[i] = accelerationZReading;
    }

    // Sort the readings
    std::sort(accelerationXReadings, accelerationXReadings + numReadings);
    std::sort(accelerationYReadings, accelerationYReadings + numReadings);
    std::sort(accelerationZReadings, accelerationZReadings + numReadings);

    // Calculate the 90th percentile
    int index90th = numReadings * 0.9;
    accelerationXDeadzone = abs(accelerationXReadings[index90th] - accelerationXOffset);
    accelerationYDeadzone = abs(accelerationYReadings[index90th] - accelerationYOffset);
    accelerationZDeadzone = abs(accelerationZReadings[index90th] - accelerationZOffset);

    Serial.println("Calibration complete.");
    Serial.print("Offsets: ");
    Serial.print(accelerationXOffset); Serial.print(", ");
    Serial.print(accelerationYOffset); Serial.print(", ");
    Serial.print(accelerationZOffset); Serial.print(", ");
    Serial.print(gyroXOffset); Serial.print(", ");
    Serial.print(gyroYOffset); Serial.print(", ");
    Serial.println(gyroZOffset);
    Serial.print("Dead Zones: ");
    Serial.print(accelerationXDeadzone); Serial.print(", ");
    Serial.print(accelerationYDeadzone); Serial.print(", ");
    Serial.println(accelerationZDeadzone);
}