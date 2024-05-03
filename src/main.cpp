#include <Wire.h>
#include <Joystick.h>
#include <MPU6050.h>

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

const int numReadings = 1000;
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
    /*
    // Output detected movements to the console or update game state
    Serial.print("Smoothed Ax: "); Serial.print(accelerationX); Serial.print(", ");
    Serial.print("Smoothed Ay: "); Serial.print(accelerationY); Serial.print(", ");
    Serial.print("Smoothed Az: "); Serial.print(accelerationZ); Serial.print(", ");
    Serial.print("Smoothed Gx: "); Serial.print(gyroX); Serial.print(", ");
    Serial.print("Smoothed Gy: "); Serial.print(gyroY); Serial.print(", ");
    Serial.print("Smoothed Gz: "); Serial.println(gyroZ);
    */

    // Compute movement detection flags
    bool moveRight = accelerationX > accelerationThreshold;
    bool moveLeft = accelerationX < -accelerationThreshold;
    bool moveBackward = accelerationY > accelerationThreshold;
    bool moveForward = accelerationY < -accelerationThreshold;
    bool rotateRight = gyroY > gyroThreshold;  // Adjusted for rotation around y-axis
    bool rotateLeft = gyroY < -gyroThreshold; // Adjusted for rotation around y-axis
    bool jumpDetected = accelerationZ > jumpThreshold; // Adjusted for jump detection along z-axis

    // Map movements to joystick controls
    joystick.setXAxis(moveLeft ? -accelerationX : (moveRight ? accelerationX : 0));
    joystick.setYAxis(moveForward ? -accelerationY : (moveBackward ? accelerationY : 0));
    joystick.setRzAxis(rotateLeft ? -gyroY : (rotateRight ? gyroY : 0)); // Adjusted for rotation around y-axis

    // Handle specific buttons for rotational moves
    joystick.setButton(1, rotateRight);    // Rotate right
    joystick.setButton(2, rotateLeft);     // Rotate left

    // Directions based on accelerometer thresholds
    joystick.setButton(3, moveRight);      // Move right
    joystick.setButton(4, moveLeft);       // Move left
    joystick.setButton(5, moveBackward);   // Move backward
    joystick.setButton(6, moveForward);    // Move forward

    // Output detected movements to the console
    if (moveRight) {
        Serial.println("Moving right");
    } else if (moveLeft) {
        Serial.println("Moving left");
    }
    if (moveBackward) {
        Serial.println("Moving backward");
    } else if (moveForward) {
        Serial.println("Moving forward");
    }
    if (rotateRight) {
        Serial.println("Rotating right");
    } else if (rotateLeft) {
        Serial.println("Rotating left");
    }
    if (jumpDetected) {
        Serial.println("Jump detected");
    }
}

void calibrateSensors() {
    int32_t accelerationXSum = 0, accelerationYSum = 0, accelerationZSum = 0;
    int32_t gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
    int16_t accelerationXReading, accelerationYReading, accelerationZReading;
    int16_t gyroXReading, gyroYReading, gyroZReading;

    Serial.println("Calibrating sensors, make sure the device is stationary and flat...");

    // Discard initial readings to eliminate potential drift
    for (int i = 0; i < 100; i++) {
        mpu.getMotion6(&accelerationXReading, &accelerationYReading, &accelerationZReading, &gyroXReading, &gyroYReading, &gyroZReading);
        delay(10);
    }

    // Collect sufficient data for calibration
    for (int i = 0; i < numReadings; i++) {
        mpu.getMotion6(&accelerationXReading, &accelerationYReading, &accelerationZReading, &gyroXReading, &gyroYReading, &gyroZReading);
        
        // Accumulate sensor readings
        accelerationXSum += accelerationXReading; 
        accelerationYSum += accelerationYReading; 
        accelerationZSum += accelerationZReading;
        gyroXSum += gyroXReading; 
        gyroYSum += gyroYReading; 
        gyroZSum += gyroZReading;
    }

    // Calculate average readings
    accelerationXOffset = accelerationXSum / numReadings;
    accelerationYOffset = accelerationYSum / numReadings;
    accelerationZOffset = accelerationZSum / numReadings;
    gyroXOffset = gyroXSum / numReadings;
    gyroYOffset = gyroYSum / numReadings;
    gyroZOffset = gyroZSum / numReadings;

    // Calculate standard deviation for dead zone
    float accelerationXVariance = 0.0, accelerationYVariance = 0.0, accelerationZVariance = 0.0;
    for (int i = 0; i < numReadings; i++) {
        mpu.getMotion6(&accelerationXReading, &accelerationYReading, &accelerationZReading, &gyroXReading, &gyroYReading, &gyroZReading);
        
        // Calculate variance for each axis
        accelerationXVariance += sq(accelerationXReading - accelerationXOffset); 
        accelerationYVariance += sq(accelerationYReading - accelerationYOffset); 
        accelerationZVariance += sq(accelerationZReading - accelerationZOffset);    
    }

    // Update dead zone thresholds based on standard deviation
    accelerationXDeadzone = sqrt(accelerationXVariance / numReadings);
    accelerationYDeadzone = sqrt(accelerationYVariance / numReadings);
    accelerationZDeadzone = sqrt(accelerationZVariance / numReadings);

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

