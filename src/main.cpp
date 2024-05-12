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

const int numReadings = 2000;
const int gyroThreshold = 5000;
const int jumpThreshold = 10000;
const int accelerationThreshold = 5000;

bool isPlayerPrensent = false;

float accelerationXDeadzone, accelerationYDeadzone, accelerationZDeadzone;
float forwardCosine = 1.0;  // Default-value (no rotation)
float forwardSine = 0.0;    // Default-value (no rotation)

void calibrateSensors();
void checkForMovement();
void checkForSignificantLean();
void applyOffsets();
void applyRotation();
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
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    pinMode(8, OUTPUT);     // Set digital pin 8 as an OUTPUT for buzzer
    calibrateSensors();
    joystick.begin();

    Serial.println("Device is calibrated. Waiting for player...");

    detectPlayer();             // Ensure player is detected before proceeding
    checkForSignificantLean();  // Wait for player to lean forward
    defineAxis();               // Then define the axis based on the player's orientation

    playSetupCompleteSound();
    Serial.println("Setup complete. Device is ready to use!");
}

void loop() {
    static unsigned long lastMillis = 0;
    if (millis() - lastMillis >= 10) {
        lastMillis = millis();
        
        mpu.getMotion6(&accelerationX, &accelerationY, &accelerationZ, &gyroX, &gyroY, &gyroZ);
        applyOffsets();
        applyRotation();
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

long mapLong(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
    delay(1000);
    playInitSound();

    Serial.println("Calibrating...");
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
        delay(3);
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
    
    playSuccessSound();
}

void defineAxis() {
    Serial.println("Defining the forward axis based on the average lean direction...");
    unsigned long startTime = millis();
    const int defineDuration = 5000; // Duration to collect data for averaging
    int16_t sumX = 0, sumY = 0, count = 0;

    while (millis() - startTime < defineDuration) {
        mpu.getMotion6(&accelerationX, &accelerationY, &accelerationZ, &gyroX, &gyroY, &gyroZ);
        applyOffsets();

        sumX += accelerationX;
        sumY += accelerationY;
        count++;
        delay(10); // Short delay between measurements
    }

    // Calculate the average components
    float avgX = float(sumX) / count;
    float avgY = float(sumY) / count;
    float forwardAngle = atan2(avgY, avgX);  // Calculate angle based on average

    forwardCosine = cos(forwardAngle);
    forwardSine = sin(forwardAngle);

    Serial.print("Forward direction defined at average lean angle: ");
    Serial.println(forwardAngle * 180 / PI);
    playSuccessSound();
}

void checkForSignificantLean() {
    delay(1000);
    Serial.println("Waiting for a significant lean to define the forward direction...");
    playInitSound();  // Signal the start of detection
    bool significantLeanDetected = false;

    while (!significantLeanDetected) {
        mpu.getMotion6(&accelerationX, &accelerationY, &accelerationZ, &gyroX, &gyroY, &gyroZ);
        applyOffsets();

        // Check for extreme or invalid readings
        if (accelerationX == 0 && accelerationY == 0) {
            Serial.println("Warning: Zero or invalid sensor readings detected.");
            continue;  // Skip this loop iteration if readings are zero
        }

        float tiltMagnitudeSquared = float(accelerationX) * accelerationX + float(accelerationY) * accelerationY;
        if (tiltMagnitudeSquared < 0) {
            Serial.println("Error: Negative value calculated for tilt magnitude squared.");
            continue;  // Skip this iteration if calculated square is negative
        }

        float tiltMagnitude = sqrt(tiltMagnitudeSquared);
        Serial.println("Tilt Magnitude: " + String(tiltMagnitude));  // Debug output

        if (tiltMagnitude > 5000) {  // Threshold can be adjusted as needed
            Serial.println("Significant lean detected. Proceeding to define the axis...");
            significantLeanDetected = true;
            playSuccessSound();
            break;  // Exit the loop
        }
        delay(10);  // Short delay for stability
    }
}


void detectPlayer() {
    delay(1000);
    playInitSound();
    int stableReadings = 0;
    const int requiredStableReadings = 10;  // Reduced for more sensitivity
    float accelerationZThreshold = 17000;   // Define a specific threshold for Z-axis

    Serial.println("Waiting for player to sit down...");

    while (!isPlayerPrensent) {
        mpu.getMotion6(&accelerationX, &accelerationY, &accelerationZ, &gyroX, &gyroY, &gyroZ);
        applyOffsets();

        // Debugging output to monitor Z-axis values
        // Serial.print("Z-Axis reading: ");
        // Serial.println(accelerationZ);

        // Check if the absolute value of Z-axis acceleration is less than a threshold
        if (abs(accelerationZ) > accelerationZThreshold) {
            stableReadings++;
            if (stableReadings >= requiredStableReadings) {
                isPlayerPrensent = true;
                Serial.println("Player detected on chair.");
                break;  // Exit the loop once the player is detected
            }
        } else {
            stableReadings = 0;  // Reset if readings are not stable
        }
        delay(10);  // Short delay to debounce the detection
    }
    playSuccessSound();
}

void applyRotation() {
    // Rotate the axes to adjust to the new forward direction
    float newX = accelerationX * forwardCosine - accelerationY * forwardSine;
    float newY = accelerationX * forwardSine + accelerationY * forwardCosine;
    accelerationX = newX;
    accelerationY = newY;
}

void playInitSound() {
    // Buzzer
    tone(8, 1000);  // Play tone at 1000 Hz
    delay(150);     // Continue for 1 second
    noTone(8);      // Stop the tone
    delay(150);
    tone(8, 1000);
    delay(150);
    noTone(8);
}

void playSuccessSound() {
    tone(8, 1000);  // Play tone at 1000 Hz
    delay(150);     // Continue for 0.75 seconds
    tone(8, 1200);  // Play tone at 1000 Hz
    delay(150);
    tone(8, 1400);
    delay(150);
    noTone(8);
}

void playSetupCompleteSound() {
    // Play a simple ascending tone sequence to indicate completion
    tone(8, 800);   // Play tone at 800 Hz
    delay(150);     // Continue for 0.2 seconds
    tone(8, 1000);  // Increase tone to 1000 Hz
    delay(150);     // Continue for 0.2 seconds
    tone(8, 1200);  // Increase tone further to 1200 Hz
    delay(150);     // Continue for 0.2 seconds
    noTone(8);      // Stop the tone

    // Add a short pause
    delay(150);

    // Play a final high tone to definitively signal readiness
    tone(8, 1500);  // Play a higher tone at 1500 Hz
    delay(300);     // Continue for 0.3 seconds
    noTone(8);      // Stop the tone
}
