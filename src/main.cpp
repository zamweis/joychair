#include <Wire.h>
#include <Joystick.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

void playInitSound();
void playSuccessSound();
void playSetupCompleteSound();
void playErrorSound();
void printRollPitchYawAccelZ();
void printJoystickValues();
int applyDeadzone(int value, int deadzone);
bool checkTilt(float pitch, float roll, float tolerance);

uint16_t packetSize;
uint8_t fifoBuffer[64];

long previousMillis = 0;  // will store last time LED was updated
long interval = 250;  // interval at which to blink (milliseconds)

Quaternion q;
VectorFloat gravity;
float ypr[3];  // yaw, pitch, roll
float yaw;
float pitch;
float roll;
float accelZ;

MPU6050 mpu;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                   1, 0,                   // Button Count, Hat Switch Count
                   true, true, true,       // X and Y, and Z Axis
                   true, true, true,       // Rx, Ry, and Rz
                   false, false,           // rudder and throttle
                   false, false, false);   // accelerator, brake, and steering

int joystickX;
int joystickY;
int joystickZ;
int joystickRx;
int joystickRy;
int joystickRz;

int jumpThreshold = 800;
int buzzerPin = 6;

bool calibrationComplete = false;
bool tiltCheckComplete = false;
float calibrationYaw, calibrationPitch, calibrationRoll;
unsigned long calibrationStartTime;
unsigned long tiltCheckStartTime;
const unsigned long calibrationDuration = 5000; // Calibration duration in milliseconds
const unsigned long tiltCheckDuration = 3000; // Duration to check for tilt in milliseconds
const float tiltTolerance = 10.0; // Tolerance for tilt check in degrees

void setup() {
  Serial.begin(115200);
  Wire.begin();
  playInitSound();
  Serial.println("Initializing...");

  pinMode(buzzerPin, OUTPUT);      // Set digital pin as an OUTPUT for buzzer 
  Joystick.begin(false);

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    playErrorSound();
    while (1);
  }
  
  // Initialize DMP
  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP ready!");
  } else {
    Serial.print("DMP Initialization failed (code ");
    playErrorSound();
    Serial.print(devStatus);
    Serial.println(")");
  }

  mpu.CalibrateAccel();
  mpu.CalibrateGyro();
  
  playSuccessSound();
  tiltCheckStartTime = millis();
}

void loop() {
  long currentMillis = millis();
  // Check if there's new data from the DMP
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // Get the quaternion data
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    // Get the gravity vector
    mpu.dmpGetGravity(&gravity, &q);
    // Get yaw, pitch, and roll
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    accelZ = mpu.getAccelerationZ() - 16383; 

    // Convert to degrees
    yaw = ypr[0] * 180/M_PI;
    pitch = ypr[1] * 180/M_PI;
    roll = ypr[2] * 180/M_PI;

    if (!tiltCheckComplete) {
      if (checkTilt(pitch, roll, tiltTolerance)) {
        if (currentMillis - tiltCheckStartTime >= tiltCheckDuration) {
          tiltCheckComplete = true;
          calibrationStartTime = millis();
          playInitSound();
          Serial.println("Tilt check complete. Starting calibration...");
        }
      } else {
        tiltCheckStartTime = millis();
      }
      return;
    }

    if (!calibrationComplete) {
      if (currentMillis - calibrationStartTime >= calibrationDuration) {
        calibrationYaw = yaw;
        calibrationPitch = pitch;
        calibrationRoll = roll;
        calibrationComplete = true;
        playSetupCompleteSound();
        Serial.println("Calibration complete!");
        Serial.print("Calibration yaw: "); Serial.println(calibrationYaw);
        Serial.print("Calibration pitch: "); Serial.println(calibrationPitch);
        Serial.print("Calibration roll: "); Serial.println(calibrationRoll);
      }
      return;
    }

    // Adjust the angles based on calibration
    float adjustedYaw = yaw - calibrationYaw;
    float adjustedPitch = pitch;
    float adjustedRoll = roll;

    // Apply the rotation matrix based on calibration (if necessary)
    float tempX = adjustedRoll * cos(calibrationYaw) - adjustedPitch * sin(calibrationYaw);
    float tempY = adjustedRoll * sin(calibrationYaw) + adjustedPitch * cos(calibrationYaw);
    adjustedRoll = tempX;
    adjustedPitch = tempY;

    // Linearly map angles and accelerations to joystick ranges
    joystickX = map(adjustedRoll, -45, 45, 0, 1023);
    joystickZ = map(accelZ, -32767, 32767, 0, 1023);
    joystickRx = map(adjustedPitch, -90, 90, 0, 1023);
    joystickRy = map(adjustedYaw, -180, 180, 0, 1023) + 11; // Sensor got error of 14
    joystickRz = map(adjustedRoll, -90, 90, 0, 1023);
    joystickY = map(adjustedPitch, -45, 45, 0, 1023);

    // Deadzones for X-and Y-Axis
    joystickX = applyDeadzone(joystickX, 11);
    joystickY = applyDeadzone(joystickY, 11);

    Joystick.setXAxis(joystickX);
    Joystick.setYAxis(joystickY);
    Joystick.setZAxis(joystickZ);
    Joystick.setRxAxis(joystickRx);
    Joystick.setRyAxis(joystickRy);
    Joystick.setRzAxis(joystickRz);

    if (joystickZ > jumpThreshold) {
      Joystick.setButton(0, 1);
      Serial.println("Jump detected with height: " + String(joystickZ));
    } else {
      Joystick.setButton(0, 0);
    }

    Joystick.sendState(); // Send the updated state to the host
  }
}

int applyDeadzone(int value, int deadzone) {
    // Calculate the center point of the output range
    const int center = 511;  // Center for the range [0, 1023]
    int distance = value - center;
    
    // If the distance from center is less than the deadzone, set output to center
    if (abs(distance) < deadzone) {
        return center;
    } else {
        // If outside the deadzone, adjust the value linearly from the edge of the deadzone
        return (distance > 0) ? map(distance, deadzone, 511, center + deadzone, 1023) : 
                                map(distance, -511, -deadzone, 0, center - deadzone);
    }
}

bool checkTilt(float pitch, float roll, float tolerance) {
  return (abs(pitch) > tolerance || abs(roll) > tolerance);
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
  delay(150);     // Continue for 1 second
  noTone(buzzerPin);      // Stop the tone
  delay(150);
  tone(buzzerPin, 1000);
  delay(150);
  noTone(buzzerPin);
}

void playErrorSound() {
  // Buzzer emits a distinct error sound, using a lower tone
  tone(buzzerPin, 500);   // Play tone at 500 Hz
  delay(250);     // Continue for 0.25 seconds
  noTone(buzzerPin);      // Stop the tone
  delay(100);     // Short pause
  tone(buzzerPin, 500);   // Repeat tone at 500 Hz
  delay(250);     // Continue for 0.25 seconds
  noTone(buzzerPin);      // Stop the tone
}

void playSuccessSound() {
  tone(buzzerPin, 1000);  // Play tone at 1000 Hz
  delay(150);     // Continue for 0.75 seconds
  tone(buzzerPin, 1200);  // Play tone at 1000 Hz
  delay(150);
  tone(buzzerPin, 1400);
  delay(150);
  noTone(buzzerPin);
}

void playSetupCompleteSound() {
  // Play a simple ascending tone sequence to indicate completion
  tone(buzzerPin, 800);   // Play tone at 800 Hz
  delay(150);     // Continue for 0.2 seconds
  tone(buzzerPin, 1000);  // Increase tone to 1000 Hz
  delay(150);     // Continue for 0.2 seconds
  tone(buzzerPin, 1200);  // Increase tone further to 1200 Hz
  delay(150);     // Continue for 0.2 seconds
  noTone(buzzerPin);      // Stop the tone

  // Add a short pause
  delay(150);

  // Play a final high tone to definitively signal readiness
  tone(buzzerPin, 1500);  // Play a higher tone at 1500 Hz
  delay(300);     // Continue for 0.3 seconds
  noTone(buzzerPin);      // Stop the tone
}
