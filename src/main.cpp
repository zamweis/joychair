#include <Wire.h>
#include <Joystick.h>
#include <MPU6050.h>

MPU6050 mpu;
int16_t accelerationX, accelerationY, accelerationZ;
int16_t gyroX, gyroY, gyroZ;
int16_t accelerationXOffset, accelerationYOffset, accelerationZOffset;
int16_t gyroXOffset, gyroYOffset, gyroZOffset;

Joystick_ joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD, 7, 0, true, true, true, true, true, true);

const int numReadings = 2000;
const float alpha = 0.98;  // Complementary filter coefficient
const float leanThreshold = 100.0; // Degrees
float pitch = 0, roll = 0;

void calibrateSensors();
void applySensorFusion();
void applyOffsets();
void checkForMovement();

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    pinMode(8, OUTPUT);  // Buzzer pin

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1); // Halt if connection failed
    }

    calibrateSensors();
    joystick.begin();
    Serial.println("Setup complete. Device is ready to use!");
}

void loop() {
    static unsigned long lastMillis = 0;
    if (millis() - lastMillis >= 10) {
        lastMillis = millis();
        mpu.getMotion6(&accelerationX, &accelerationY, &accelerationZ, &gyroX, &gyroY, &gyroZ);
        applyOffsets();
        applySensorFusion();
        checkForMovement();
    }
}

void applySensorFusion() {
    float dt = 0.01; // Time step
    float gyroRateX = gyroX / 131.0;
    float gyroRateY = gyroY / 131.0;
    float accelPitch = atan2(accelerationY, sqrt(accelerationX * accelerationX + accelerationZ * accelerationZ)) * 180 / PI;
    pitch = alpha * (pitch + gyroRateX * dt) + (1 - alpha) * accelPitch;
    float accelRoll = atan2(-accelerationX, accelerationZ) * 180 / PI;
    roll = alpha * (roll + gyroRateY * dt) + (1 - alpha) * accelRoll;
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

    // Example button logic for tilts
    joystick.setButton(0, (fabs(pitch) > leanThreshold));
}

void calibrateSensors() {
    Serial.println("Calibrating sensors, please ensure the device is stationary and flat...");
    int32_t sumAX = 0, sumAY = 0, sumAZ = 0, sumGX = 0, sumGY = 0, sumGZ = 0;
    for (int i = 0; i < numReadings; i++) {
        mpu.getMotion6(&accelerationX, &accelerationY, &accelerationZ, &gyroX, &gyroY, &gyroZ);
        sumAX += accelerationX; 
        sumAY += accelerationY; 
        sumAZ += accelerationZ;
        sumGX += gyroX; 
        sumGY += gyroY; 
        sumGZ += gyroZ;
        delay(3);
    }

    accelerationXOffset = sumAX / numReadings;
    accelerationYOffset = sumAY / numReadings;
    accelerationZOffset = sumAZ / numReadings;
    gyroXOffset = sumGX / numReadings;
    gyroYOffset = sumGY / numReadings;
    gyroZOffset = sumGZ / numReadings;

    Serial.println("Calibration complete.");
}
