#include <Wire.h>
#include <Joystick.h>
#include <MPU6050.h>

MPU6050 mpu;
int16_t accelerationX, accelerationY, accelerationZ;
int16_t gyroX, gyroY, gyroZ;

int16_t accelerationXOffset, accelerationYOffset, accelerationZOffset;
int16_t gyroXOffset, gyroYOffset, gyroZOffset;

Joystick_ joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                   7, 0,                // 7 Buttons, no Hat Switch
                   true, true, true,    // X, Y, and Z Axes
                   true, true, true);   // Rx, Ry, and Rz for Rotation

const int numReadings = 1000;
const int gyroThreshold = 6000;
const int jumpThreshold = 10000;
const int accelerationThreshold = 6000;

float accelerationXDeadzone, accelerationYDeadzone, accelerationZDeadzone;

void calibrateSensors();
void checkForMovement();
void applyOffsets();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1); // Halt if connection failed
  }
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  calibrateSensors();
  joystick.begin();
  Serial.println("Device is calibrated and ready!");
}

void loop() {
  static unsigned long lastMillis = 0;
  if (millis() - lastMillis >= 10) {
    lastMillis = millis();
    
    mpu.getMotion6(&accelerationX, &accelerationY, &accelerationZ, &gyroX, &gyroY, &gyroZ);
    applyOffsets();
    checkForMovement();
  }
}

void applyOffsets() {
    accelerationX -= accelerationXOffset;
    accelerationY -= accelerationYOffset;
    accelerationZ -= accelerationZOffset;
    gyroX -= gyroXOffset;
    gyroY -= gyroYOffset;
    gyroZ -= gyroZOffset;
}

void checkForMovement() {
    // Use smoothing or filtering to reduce noise and improve control
    float smoothFactor = 0.9;  // Adjust based on responsiveness vs. smoothness
    static int lastX = 0, lastY = 0, lastZ = 0;
    static int lastRx = 0, lastRy = 0, lastRz = 0;

    int filteredX = smoothFactor * lastX + (1 - smoothFactor) * accelerationX;
    int filteredY = smoothFactor * lastY + (1 - smoothFactor) * accelerationY;
    int filteredZ = smoothFactor * lastZ + (1 - smoothFactor) * accelerationZ;
    int filteredRx = smoothFactor * lastRx + (1 - smoothFactor) * gyroX;
    int filteredRy = smoothFactor * lastRy + (1 - smoothFactor) * gyroY;
    int filteredRz = smoothFactor * lastRz + (1 - smoothFactor) * gyroZ;

    lastX = filteredX;
    lastY = filteredY;
    lastZ = filteredZ;
    lastRx = filteredRx;
    lastRy = filteredRy;
    lastRz = filteredRz;

    joystick.setXAxis(filteredX);
    joystick.setYAxis(filteredY);
    joystick.setZAxis(filteredZ);
    joystick.setRxAxis(filteredRx);
    joystick.setRyAxis(filteredRy);
    joystick.setRzAxis(filteredRz);

    
    // Enhanced button logic using thresholds and direction
    // Filter out jumping because of inaccurancy
    if (accelerationZ > jumpThreshold) {
        joystick.setButton(0, 1);
        Serial.println("Jump detected with height: " + String(accelerationZ));
    } else {
        joystick.setButton(0, 0);
        joystick.setButton(1, gyroY > gyroThreshold);
        joystick.setButton(2, gyroY < -gyroThreshold);
        joystick.setButton(3, accelerationX > accelerationThreshold);
        joystick.setButton(4, accelerationX < -accelerationThreshold);
        joystick.setButton(5, accelerationY > accelerationThreshold);
        joystick.setButton(6, accelerationY < -accelerationThreshold);
    }

    // Detailed output for each movement detection
    if (accelerationX > accelerationThreshold) {
        Serial.println("Moving right with intensity: " + String(filteredX));
    } else if (accelerationX < -accelerationThreshold) {
        Serial.println("Moving left with intensity: " + String(filteredX));
    }
    if (accelerationY > accelerationThreshold) {
        Serial.println("Moving forward with intensity: " + String(filteredY));
    } else if (accelerationY < -accelerationThreshold) {
        Serial.println("Moving backward with intensity: " + String(filteredY));
    }
    if (gyroY > gyroThreshold) {
        Serial.println("Rotating right with speed: " + String(filteredRz));
    } else if (gyroY < -gyroThreshold) {
        Serial.println("Rotating left with speed: " + String(filteredRz));
    }
}

void calibrateSensors() {
    int32_t accelerationXSum = 0, accelerationYSum = 0, accelerationZSum = 0;
    int32_t gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
    int16_t accelerationXReading, accelerationYReading, accelerationZReading;
    int16_t gyroXReading, gyroYReading, gyroZReading;

    Serial.println("Calibrating sensors, make sure the device is stationary and flat...");

    // Discard the first 100 readings to allow the sensor to stabilize
    for (int i = 0; i < 100; i++) {
        mpu.getMotion6(&accelerationXReading, &accelerationYReading, &accelerationZReading, &gyroXReading, &gyroYReading, &gyroZReading);
        delay(10);
    }

    // Collect data for averaging
    for (int i = 0; i < numReadings; i++) {
        mpu.getMotion6(&accelerationXReading, &accelerationYReading, &accelerationZReading, &gyroXReading, &gyroYReading, &gyroZReading);

        accelerationXSum += accelerationXReading; 
        accelerationYSum += accelerationYReading; 
        accelerationZSum += accelerationZReading;
        gyroXSum += gyroXReading; 
        gyroYSum += gyroYReading; 
        gyroZSum += gyroZReading;
    }

    // Calculate average offset values
    accelerationXOffset = accelerationXSum / numReadings;
    accelerationYOffset = accelerationYSum / numReadings;
    accelerationZOffset = accelerationZSum / numReadings;
    gyroXOffset = gyroXSum / numReadings;
    gyroYOffset = gyroYSum / numReadings;
    gyroZOffset = gyroZSum / numReadings;

    // Output the calculated offsets
    Serial.println("Calibration complete.");
    Serial.println("Offsets:");
    Serial.print("    AX: "); Serial.println(accelerationXOffset);
    Serial.print("    AY: "); Serial.println(accelerationYOffset);
    Serial.print("    AZ: "); Serial.println(accelerationZOffset);
    Serial.print("    GX: "); Serial.println(gyroXOffset);
    Serial.print("    GY: "); Serial.println(gyroYOffset);
    Serial.print("    GZ: "); Serial.println(gyroZOffset);
}