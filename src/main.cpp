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
void applyOffsets();
void applyRotation();
void detectPlayer();
void defineAxis();
void playSuccessSound();

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

    detectPlayer();  // Ensure player is detected before proceeding
    defineAxis();    // Then define the axis based on the player's orientation
    Serial.println("Setup complete. Device is ready to use!");
}

void loop() {
    static unsigned long lastMillis = 0;
    if (millis() - lastMillis >= 10) {
        lastMillis = millis();
        
        mpu.getMotion6(&accelerationX, &accelerationY, &accelerationZ, &gyroX, &gyroY, &gyroZ);
        //applyOffsets();
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

    // Buzzer
    tone(8, 1000);  // Play tone at 1000 Hz
    delay(200);     // Continue for 1 second
    noTone(8);      // Stop the tone
    tone(8, 1000);
    delay(200);
    noTone(8);

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
    Serial.println("Lean forward to define the forward direction.");
    Serial.println("Please hold the position for a few seconds...");

    // Give the player some time
    // TODO: implement proper direction dection function
    delay(250);

    unsigned long startTime = millis();
    const int defineDuration = 5000; // Duration to capture the forward tilt in milliseconds
    float maxTiltMagnitude = 360.0; // To capture the maximum tilt magnitude
    int16_t bestX = 0, bestY = 0;

    // Buzzer
    tone(8, 500);

    // TODO: improve the forward detection by calculation average leaning direction
    while (millis() - startTime < defineDuration) {
        mpu.getMotion6(&accelerationX, &accelerationY, &accelerationZ, &gyroX, &gyroY, &gyroZ);
        applyOffsets();

        // Calculate the magnitude of tilt in the XY plane
        float tiltMagnitude = sqrt(accelerationX * accelerationX + accelerationY * accelerationY);

        // Check if the current tilt magnitude is the maximum detected
        if (tiltMagnitude < maxTiltMagnitude) {
            maxTiltMagnitude = tiltMagnitude;
            bestX = accelerationX;
            bestY = accelerationY;
        }

        delay(10); // Short delay between measurements
    }

    // Calculate the angle for the most significant forward direction
    float forwardAngle = atan2(bestY, bestX);

    // Buzzer
    noTone(8); // Stop the tone
    
    Serial.print("Forward direction defined at maximum lean angle: ");
    Serial.println(forwardAngle * 180 / PI);

    // Save the cosine and sine values for the global transformation
    forwardCosine = cos(forwardAngle);
    forwardSine = sin(forwardAngle);
}

void detectPlayer() {
    int stableReadings = 0;
    const int requiredStableReadings = 10;  // Reduced for more sensitivity
    float accelerationZThreshold = 17000;   // Define a specific threshold for Z-axis

    Serial.println("Waiting for player to sit down...");

    while (!isPlayerPrensent) {
        mpu.getMotion6(&accelerationX, &accelerationY, &accelerationZ, &gyroX, &gyroY, &gyroZ);
        //applyOffsets();

        // Debugging output to monitor Z-axis values
        // Serial.print("Z-Axis reading: ");
        // Serial.println(accelerationZ);

        // Check if the absolute value of Z-axis acceleration is less than a threshold
        if (abs(accelerationZ) > accelerationZThreshold) {
            stableReadings++;
            if (stableReadings >= requiredStableReadings) {
                isPlayerPrensent = true;
                Serial.println("Player detected on chair.");
                playSuccessSound();
                break;  // Exit the loop once the player is detected
            }
        } else {
            stableReadings = 0;  // Reset if readings are not stable
        }
        delay(10);  // Short delay to debounce the detection
    }
}

void applyRotation() {
    // Rotate the axes to adjust to the new forward direction
    float newX = accelerationX * forwardCosine - accelerationY * forwardSine;
    float newY = accelerationX * forwardSine + accelerationY * forwardCosine;
    accelerationX = newX;
    accelerationY = newY;
}

void playSuccessSound(){
    tone(8, 1000);  // Play tone at 1000 Hz
    delay(250);     // Continue for 0.75 seconds
    tone(8, 1200);  // Play tone at 1000 Hz
    delay(250);
    tone(8, 1400);
    delay(250);
    noTone(8);
}