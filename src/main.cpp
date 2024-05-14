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

const int numReadings = 1500;
const int gyroThreshold = 1000; // Adjusted from 5000 based on new gyro offset and sensitivity
const int jumpThreshold = 2000; // Adjusted from 10000, consider the new sensitivity
const int accelerationThreshold = 1000; // Adjusted from 5000
const float leanThreshold = 100.0; // Degrees, reduced from 45 for sensitivity

// mapping range variables
int16_t rangeGyro = 7000;
int16_t rangeAccelXY = 1000;
int16_t rangeAccelZ = 4000;

bool isPlayerPresent = false;

float accelerationXDeadzone, accelerationYDeadzone, accelerationZDeadzone;
float forwardCosine = 1.0;  // Default-value (no rotation)
float forwardSine = 0.0;    // Default-value (no rotation)
float alpha = 0.98;         // Complementary filter coefficient

// Sensor fusion variables
float pitch = 0, roll = 0;

void calibrateSensors();
void checkForMovement();
void checkForSignificantLean();
void applyOffsets();
void applySensorFusion();
void detectPlayer();
void defineAxis();
void playInitSound();
void playSuccessSound();
void playSetupCompleteSound();

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1); // Halt if connection failed
    }
     // Configure MPU6050 settings for accelerometer and gyroscope
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16); // Set accelerometer to +/- 16g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000); // Set gyroscope to +/- 2000 deg/s
    pinMode(8, OUTPUT);     // Set digital pin 8 as an OUTPUT for buzzer
    calibrateSensors();
    Serial.println("Device is calibrated.");

    //detectPlayer();             // Ensure player is detected before proceeding
    //checkForSignificantLean();  // Wait for player to lean forward
    //defineAxis();               // Then define the axis based on the player's orientation

    playSetupCompleteSound();
    Serial.println("Setup complete. Device is ready to use!");
    
    joystick.begin();
}

void loop() {
    static unsigned long lastMillis = 0;
    if (millis() - lastMillis >= 10) {
        lastMillis = millis();
        mpu.getMotion6(&accelerationX, &accelerationY, &accelerationZ, &gyroX, &gyroY, &gyroZ);
        
        accelerationX = map(accelerationX, -rangeAccelXY, rangeAccelXY, 0, 1023);
        accelerationY = map(accelerationY, -rangeAccelXY, rangeAccelXY, 0, 1023);
        accelerationZ = map(accelerationZ, -rangeAccelZ, rangeAccelZ, 0, 1023);
        gyroX = map(gyroX, -rangeGyro, rangeGyro, 0, 1023);
        gyroY = map(gyroY, -rangeGyro, rangeGyro, 0, 1023);
        gyroZ = map(gyroZ, -rangeGyro, rangeGyro, 0, 1023);

        //applySensorFusion();
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

    joystick.setXAxis(accelerationX);
    joystick.setYAxis(accelerationY);
    joystick.setZAxis(accelerationZ);
    joystick.setRxAxis(gyroX);
    joystick.setRyAxis(gyroY);
    joystick.setRzAxis(gyroZ);
}

void calibrateSensors() {
    delay(1000);
    playInitSound();

    Serial.println("Calibrating...");
    int32_t accelerationXSum = 0, accelerationYSum = 0, accelerationZSum = 0;
    int32_t gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
    int16_t accelerationXReading, accelerationYReading, accelerationZReading;
    int16_t gyroXReading, gyroYReading, gyroZReading;

    Serial.println("Ensure the device is stationary and flat on a level surface...");

    for (int i = 0; i < numReadings; i++) {
        mpu.getMotion6(&accelerationXReading, &accelerationYReading, &accelerationZReading,
                       &gyroXReading, &gyroYReading, &gyroZReading);

        accelerationX = map(accelerationX, -1000, 1000, 0, 1023);
        accelerationY = map(accelerationY, -1000, 1000, 0, 1023);
        accelerationZ = map(accelerationZ, -8000, 8000, 0, 1023);
        gyroX = map(gyroX, -7000, 7000, 0, 1023);
        gyroY = map(gyroY, -7000, 7000, 0, 1023);
        gyroZ = map(gyroZ, -7000, 7000, 0, 1023);
        accelerationXSum += accelerationXReading;
        accelerationYSum += accelerationYReading;
        accelerationZSum += accelerationZReading;
        gyroXSum += gyroXReading;
        gyroYSum += gyroYReading;
        gyroZSum += gyroZReading;
        delay(5);
    }

    accelerationXOffset = -accelerationXSum / numReadings;
    accelerationYOffset = -accelerationYSum / numReadings;
    accelerationZOffset = -accelerationZSum / numReadings;
    gyroXOffset = -gyroXSum / numReadings;
    gyroYOffset = -gyroYSum / numReadings;
    gyroZOffset = -gyroZSum / numReadings;

    Serial.println("Calibration complete.");
    Serial.println("Offsets:");
    Serial.print("    AX: "); Serial.println(accelerationXOffset);
    Serial.print("    AY: "); Serial.println(accelerationYOffset);
    Serial.print("    AZ: "); Serial.println(accelerationZOffset);
    Serial.print("    GX: "); Serial.println(gyroXOffset);
    Serial.print("    GY: "); Serial.println(gyroYOffset);
    Serial.print("    GZ: "); Serial.println(gyroZOffset);

    playSuccessSound();
}

void checkForSignificantLean() {
    Serial.println("Waiting for a significant lean to define the forward direction...");
    playInitSound();
    bool significantLeanDetected = false;

    while (!significantLeanDetected) {
        mpu.getMotion6(&accelerationX, &accelerationY, &accelerationZ, &gyroX, &gyroY, &gyroZ);
        applyOffsets();
        applySensorFusion(); // Ensure sensor fusion is applied to update pitch and roll

        if (fabs(pitch) > leanThreshold || fabs(roll) > leanThreshold) {
            Serial.println("Significant lean detected. Proceeding to define the axis...");
            significantLeanDetected = true;
            playSuccessSound();
            break;
        }
        delay(10);
    }
}

void defineAxis() {
    Serial.println("Defining the forward axis based on the average lean direction.");
    Serial.println("Please hold the position for a few seconds...");
    unsigned long startTime = millis();
    const int defineDuration = 5000;
    float sumCosX = 0, sumSinY = 0;
    int count = 0;

    while (millis() - startTime < defineDuration) {
        mpu.getMotion6(&accelerationX, &accelerationY, &accelerationZ, &gyroX, &gyroY, &gyroZ);
        applyOffsets();
        applySensorFusion(); // Ensure pitch and roll are updated

        // Debugging outputs
        Serial.print("Pitch: "); Serial.println(pitch);
        Serial.print("Roll: "); Serial.println(roll);

        // Trigonometric calculations with checks
        if (roll * PI / 180 < PI && roll * PI / 180 > -PI) {
            sumCosX += cos(roll * PI / 180) * cos(pitch * PI / 180); // Calculate forward direction based on stabilized pitch and roll
            sumSinY += sin(roll * PI / 180);
            count++;
        } else {
            Serial.println("Roll value out of bounds for trigonometric functions.");
        }

        delay(10);
    }

    if (count > 0) {
        forwardCosine = sumCosX / count;
        forwardSine = sumSinY / count;
    } else {
        forwardCosine = 0; // Default or error handling
        forwardSine = 0;
    }

    Serial.print("Forward direction defined with cosine: ");
    Serial.println(forwardCosine);
    Serial.print("and sine: ");
    Serial.println(forwardSine);
    playSuccessSound();
}

void detectPlayer() {
    delay(1000);
    playInitSound();
    int stableReadings = 0;
    const int requiredStableReadings = 10;
    float accelerationZThreshold = 5000; // Lowered threshold based on sensitivity

    Serial.println("Waiting for player to sit down...");

    while (!isPlayerPresent) {
        mpu.getMotion6(&accelerationX, &accelerationY, &accelerationZ, &gyroX, &gyroY, &gyroZ);
        applyOffsets();

        if (abs(accelerationZ - 2048) > accelerationZThreshold) { // Adjusted for 16g sensitivity, assuming middle value around 2048
            stableReadings++;
            if (stableReadings >= requiredStableReadings) {
                isPlayerPresent = true;
                Serial.println("Player detected on chair.");
                break;
            }
        } else {
            stableReadings = 0;
        }
        delay(10);
    }
    playSuccessSound();
}

void playInitSound() {
    tone(8, 1000);
    delay(150);
    noTone(8);
    delay(150);
    tone(8, 1000);
    delay(150);
    noTone(8);
}

void playSuccessSound() {
    tone(8, 1000);
    delay(150);
    tone(8, 1200);
    delay(150);
    tone(8, 1400);
    delay(150);
    noTone(8);
}

void playSetupCompleteSound() {
    tone(8, 1000);
    delay(300);
    noTone(8);
    delay(300);
    tone(8, 1000);
    delay(300);
    noTone(8);
    delay(300);
    tone(8, 1200);
    delay(300);
    noTone(8);
}
