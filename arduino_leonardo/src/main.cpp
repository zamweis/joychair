#include <Wire.h>
#include <Joystick.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Function and variable declarations
void playInitSound();
void playSuccessSound();
void playSetupCompleteSound();
void playErrorSound();
void printRollPitchYawAccelZ();
void printJoystickValues();
int applyDeadzone(int value, int deadzone);
float calculateTilt(float pitch, float roll);
bool checkTilt(float pitch, float roll, float tolerance);
float calculateRelativeAngle(float accelX, float accelY, float accelZ);
bool initializeMPU6050();

uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];  // yaw, pitch, roll
float yaw;
float pitch;
float roll;
float accelX;
float accelY;
float accelZ;
float relativeAngle; // Angle of sensor rotation

MPU6050 mpu;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                   1, 0,                   // Number of buttons, Hat Switch Count
                   true, true, true,       // X, Y, and Z axis
                   true, true, true,       // Rx, Ry, and Rz
                   false, false,           // Rudder and Throttle
                   false, false, false);   // Accelerator, Brake, and Steering

int joystickX;
int joystickY;
int joystickZ;
int joystickRx;
int joystickRy;
int joystickRz;

float cosAngle;
float sinAngle;

int jumpThreshold = 3000;
int buzzerPin = 6;

bool forwardDirectionDefined = false;
bool sittingCheckComplete = false;
bool mpuInitialized = false;
unsigned long lastTiltCheckTime = 0;
unsigned long lastJumpTime = 0; // Track the last jump time
const unsigned long jumpCooldown = 500; // 500ms cooldown between jumps
const unsigned long calibrationDuration = 5000; // Calibration duration in milliseconds
const float tiltTolerance = 5; // Tolerance for tilt check in radiants
const float sittingAccelZThreshold = 4000; // Threshold for detecting sitting

void setup() {
  playInitSound();
  Serial.begin(115200);
  Wire.begin();
  pinMode(buzzerPin, OUTPUT); // Set digital pin as output for the buzzer
  Joystick.begin(false);

  if (!initializeMPU6050()) {
    Serial.println("Failed to initialize MPU6050. Check connections and restart.");
    while (true) {
      playErrorSound();
      delay(1000);
    }
  }

  Serial.println("Waiting for player to sit down...");
  playInitSound();
}

void loop() {
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection lost. Trying to reconnect...");
    playErrorSound();
    unsigned long startAttemptTime = millis();
    while (!mpu.testConnection() && millis() - startAttemptTime < 10000) { // 10 second timeout
      delay(500);
      Serial.print(".");
      if (!mpuInitialized) {
        initializeMPU6050();
      }
    }

    if (mpu.testConnection()) {
      Serial.println("MPU6050 reconnected.");
      playSuccessSound();
    } else {
      Serial.println("Failed to reconnect MPU6050.");
      playErrorSound();
      delay(5000); // Wait 5 seconds before trying again
      return; // Skip the rest of the loop if not reconnected
    }
  }

  long currentMillis = millis();
  // Check for new data from DMP
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // Get Quaternion data
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    // Get gravity vector
    mpu.dmpGetGravity(&gravity, &q);
    // Get Yaw, Pitch, and Roll
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    accelZ = mpu.getAccelerationZ() - 16383;

    // Convert to degrees
    yaw = ypr[0] * 180 / M_PI;
    pitch = ypr[1] * 180 / M_PI;
    roll = ypr[2] * 180 / M_PI;
/*
    Serial.print("Tilt: ");
    Serial.print(calculateTilt(pitch, roll));
    Serial.print("Accelz: ");
    Serial.println(accelZ);
*/

    // Wait for player to sit
    if (!sittingCheckComplete) {
      if (accelZ > sittingAccelZThreshold) {
        playSuccessSound();
        sittingCheckComplete = true;
        Serial.println("Player detected.");
        Serial.println("Tilt forward and hold for a few seconds...");
      }
      return;
    }

    // Check for continuous tilt of 5 seconds
    if (!forwardDirectionDefined) {
      if (checkTilt(pitch, roll, tiltTolerance)) {
        if (lastTiltCheckTime == 0) {
          lastTiltCheckTime = millis(); // Start timing when tilt is sufficient
          playInitSound();
        } else if (currentMillis - lastTiltCheckTime >= calibrationDuration) {
          // Calculate relative angle
          relativeAngle = -calculateRelativeAngle(mpu.getAccelerationX(), mpu.getAccelerationY(), mpu.getAccelerationZ());
          Serial.print("Relative Angle: ");
          Serial.println(relativeAngle);

          forwardDirectionDefined = true;
          playSetupCompleteSound();
          Serial.println("Forward direction defined!");
          Serial.println("Setup Complete!");
        }
      } else {
        if (lastTiltCheckTime != 0) { // Play error sound only once
          playErrorSound();
        }
        lastTiltCheckTime = 0; // Reset timing if tilt is not sufficient
      }
      return;
    }

    float cosAngle = -cos(relativeAngle);
    float sinAngle = -sin(relativeAngle);

    // Rotate Roll and Pitch values according to the relative angle
    float newRoll = pitch * sinAngle + roll * cosAngle;
    float newPitch = pitch * cosAngle - roll * sinAngle;

    // Map new Roll and Pitch values to joystick axes
    joystickX = map(newRoll, -10, 10, 0, 1023);
    joystickY = map(newPitch, -10, 10, 0, 1023);
    joystickZ = map(accelZ, -32767, 32767, 0, 1023);
    joystickRx = map(newRoll, -90, 90, 0, 1023);
    joystickRy = map(newPitch, -90, 90, 0, 1023);
    joystickRz = map(yaw, -180, 180, 0, 1023); // Use Yaw for Rz

    // Apply deadzones to X and Y axes
    joystickX = applyDeadzone(joystickX, 5);
    joystickY = applyDeadzone(joystickY, 5);

    Joystick.setXAxis(joystickX);
    Joystick.setYAxis(joystickY);
    Joystick.setZAxis(joystickZ);
    Joystick.setRxAxis(joystickRx);
    Joystick.setRyAxis(joystickRy);
    Joystick.setRzAxis(joystickRz);

    if (accelZ > jumpThreshold && currentMillis - lastJumpTime >= jumpCooldown) {
      Joystick.setButton(0, 1);
      Serial.println("Jump detected!");
      lastJumpTime = currentMillis; // Update the last jump time
    } else {
      Joystick.setButton(0, 0);
    }

    Joystick.sendState(); // Send updated state to host
  }
}

bool initializeMPU6050() {
  Serial.println("Initializing...");

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    playErrorSound();
    mpuInitialized = false;
    return false;
  }

  // Initialize DMP
  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP ready!");
    mpuInitialized = true;
  } else {
    Serial.print("DMP Initialization failed (code ");
    playErrorSound();
    Serial.print(devStatus);
    Serial.println(")");
    mpuInitialized = false;
    return false;
  }

  mpu.CalibrateAccel();
  mpu.CalibrateGyro();
  Serial.println("Calibration complete!");
  playSuccessSound();
  delay(1000);
  return true;
}

int applyDeadzone(int value, int deadzone) {
  // Calculate center of output range
  const int center = 511; // Center for range [0, 1023]
  int distance = value - center;

  // If the distance from the center is less than the deadzone, set output to center
  if (abs(distance) < deadzone) {
    return center;
  } else {
    // If outside the deadzone, adjust value linearly from the edge of the deadzone
    return (distance > 0) ? map(distance, deadzone, 511, center + deadzone, 1023) :
                            map(distance, -511, -deadzone, 0, center - deadzone);
  }
}

float calculateRelativeAngle(float accelX, float accelY, float accelZ) {
  // Normalize acceleration values
  float norm = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
  accelX /= norm;
  accelY /= norm;
  accelZ /= norm;

  // Calculate projection on XY-plane (ground projection)
  float proj_x = accelX;
  float proj_y = accelY;

  // Calculate angle in XY-plane relative to forward direction (positive y-axis)
  float angle = atan2(proj_y, proj_x);

  return angle;
}

bool checkTilt(float pitch, float roll, float tolerance) {
  return (abs(pitch) > tolerance || abs(roll) > tolerance);
}

float calculateTilt(float pitch, float roll){
  return max(abs(roll), abs(pitch));
}

void printRollPitchYawAccelZ() {
  Serial.print(F("roll: ")); Serial.print(roll);
  Serial.print(F(", pitch: ")); Serial.print(pitch);
  Serial.print(F(", yaw: ")); Serial.print(yaw);
  Serial.print(F(", accelZ: ")); Serial.println(accelZ);
}

void printJoystickValues(){
  Serial.print(F("joystickX: ")); Serial.print(joystickX);
  Serial.print(F(", joystickY: ")); Serial.print(joystickY);
  Serial.print(F(", joystickZ: ")); Serial.print(joystickZ);
  Serial.print(F(", joystickRx: ")); Serial.print(joystickRx);
  Serial.print(F(", joystickRy: ")); Serial.print(joystickRy);
  Serial.print(F(", joystickRz: ")); Serial.println(joystickRz);
}

void playInitSound() {
  tone(buzzerPin, 1000);  // Play tone at 1000 Hz
  delay(150); // Continue for 150 ms
  noTone(buzzerPin); // Stop tone
  delay(150);
  tone(buzzerPin, 1000);
  delay(150);
  noTone(buzzerPin);
}

void playErrorSound() {
  // The buzzer emits a distinct error sound by using a lower tone
  tone(buzzerPin, 500); // Play tone at 500 Hz
  delay(250); // Continue for 250 ms
  noTone(buzzerPin); // Stop tone
  delay(100); // Short pause
  tone(buzzerPin, 500); // Repeat tone at 500 Hz
  delay(250); // Continue for 250 ms
  noTone(buzzerPin); // Stop tone
}

void playSuccessSound() {
  tone(buzzerPin, 1000); // Play tone at 1000 Hz
  delay(150); // Continue for 150 ms
  tone(buzzerPin, 1200); // Increase tone to 1200 Hz
  delay(150);
  tone(buzzerPin, 1400);
  delay(150);
  noTone(buzzerPin);
}

void playSetupCompleteSound() {
  // Play a simple ascending tone to indicate completion
  tone(buzzerPin, 800); // Play tone at 800 Hz
  delay(150); // Continue for 150 ms
  tone(buzzerPin, 1000); // Increase tone to 1000 Hz
  delay(150); // Continue for 150 ms
  tone(buzzerPin, 1200); // Further increase tone to 1200 Hz
  delay(150); // Continue for 150 ms
  noTone(buzzerPin); // Stop tone

  // Add a short pause
  delay(150);

  // Play final high tone to signal readiness
  tone(buzzerPin, 1500); // Play higher tone at 1500 Hz
  delay(300); // Continue for 300 ms
  noTone(buzzerPin); // Stop tone
}
