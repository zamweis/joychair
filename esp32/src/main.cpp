/*
 * This example turns the ESP32 into a Bluetooth LE gamepad that presses buttons and moves axis
 *
 * At the moment we are using the default settings, but they can be canged using a BleGamepadConfig instance as parameter for the begin function.
 *
 * Possible buttons are:
 * BUTTON_1 through to BUTTON_16
 * (16 buttons by default. Library can be configured to use up to 128)
 *
 * Possible DPAD/HAT switch position values are:
 * DPAD_CENTERED, DPAD_UP, DPAD_UP_RIGHT, DPAD_RIGHT, DPAD_DOWN_RIGHT, DPAD_DOWN, DPAD_DOWN_LEFT, DPAD_LEFT, DPAD_UP_LEFT
 * (or HAT_CENTERED, HAT_UP etc)
 *
 * bleGamepad.setAxes sets all axes at once. There are a few:
 * (x axis, y axis, z axis, rx axis, ry axis, rz axis, slider 1, slider 2)
 *
 * Library can also be configured to support up to 5 simulation controls
 * (rudder, throttle, accelerator, brake, steering), but they are not enabled by default.
 *
 * Library can also be configured to support different function buttons
 * (start, select, menu, home, back, volume increase, volume decrease, volume mute)
 * start and select are enabled by default
 */

#include <BleGamepad.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Function and variable declarations
void blinkLED();
void playInitSound();
int getBatteryLevel();
void playErrorSound();
void playSuccessSound();
bool initializeMPU6050();
void printJoystickValues();
void playSetupCompleteSound();
void printRollPitchYawAccelZ();
void setLEDColor(String color);
int applyDeadzone(int value, int deadzone);
float calculateTilt(float pitch, float roll);
bool checkTilt(float pitch, float roll, float tolerance);
float calculateRelativeAngle(float accelX, float accelY, float accelZ);

// Define pin numbers for the RGB LED
int redPin = 25;
int greenPin = 26;
int bluePin = 27;
int buzzerPin = 13;
long lastBlink = 0;  // will store last time LED was updated
long blinkInterval = 500;  // interval at which to blink (milliseconds)
bool blinkRed = false;
bool blinkBlue = false;
bool blinkGreen = false;

// Battery
int batteryPin = 36; // ADC1 channel 0 is GPIO36
float minBatteryVoltage = 3.0; // Minimum expected battery voltage (discharged LiPo)
float maxBatteryVoltage = 4.2; // Maximum expected battery voltage (fully charged LiPo)

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

BleGamepad bleGamepad("Joychair", "iXperienceLab", 100);

int joystickX;
int joystickY;
int joystickZ;
int joystickRx;
int joystickRy;
int joystickRz;

float cosAngle;
float sinAngle;

int jumpThreshold = 3000;

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
  blinkLED();
  Wire.begin();

  Serial.begin(115200);

  // initialize the digital pin as an output.
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Initialize the battery pin as an input
  pinMode(batteryPin, INPUT);

  Serial.println("Starting BLE work!");
  bleGamepad.begin();

  // Initialize MPU6050
  while(!initializeMPU6050()) {
    Serial.println("Failed to initialize MPU6050.");
    playErrorSound();
    delay(1000);
  }

  delay(500);
  Serial.println("Waiting for player to sit down...");
  playInitSound();
}


void loop() {
  long currentMillis = millis();

  // Check the battery level
  int batteryLevel = getBatteryLevel();

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
      blinkLED();
      delay(5000); // Wait 5 seconds before trying again
      return; // Skip the rest of the loop if not reconnected
    }
  }

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
      
      blinkLED();
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
      
      blinkLED();
      return;
    }

    float cosAngle = -cos(relativeAngle);
    float sinAngle = -sin(relativeAngle);

    // Rotate Roll and Pitch values according to the relative angle
    float newRoll = pitch * sinAngle + roll * cosAngle;
    float newPitch = pitch * cosAngle - roll * sinAngle;

    // Map new Roll and Pitch values to joystick axes
    joystickX = map(newRoll, -10, 10, 0, 32767);
    joystickY = map(newPitch, -10, 10, 0, 32767);
    joystickZ = map(accelZ, -32767, 32767, 0, 32767);
    joystickRx = map(newRoll, -90, 90, 0, 32767);
    joystickRy = map(newPitch, -90, 90, 0, 32767);
    joystickRz = map(yaw, -180, 180, 0, 32767); // Use Yaw for Rz

    // Apply deadzones to X and Y axes
    joystickX = applyDeadzone(joystickX, 5);
    joystickY = applyDeadzone(joystickY, 5);

    if (bleGamepad.isConnected()) {
      if (accelZ > jumpThreshold && currentMillis - lastJumpTime >= jumpCooldown) {
        Serial.println("Jump detected!");
        bleGamepad.press(BUTTON_5);
        lastJumpTime = currentMillis; // Update the last jump time
      } else {
        bleGamepad.release(BUTTON_5);
      }
      bleGamepad.setAxes(joystickX, joystickY, joystickZ, joystickRx, joystickRy, joystickRz, 16383, 16383);
      bleGamepad.setBatteryLevel(batteryLevel);
    }
  }
  blinkLED();
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
  setLEDColor("blue");
  tone(buzzerPin, 1000);  // Play tone at 1000 Hz
  delay(150); // Continue for 150 ms
  noTone(buzzerPin); // Stop tone
  delay(150);
  tone(buzzerPin, 1000);
  delay(150);
  noTone(buzzerPin);
}

void playErrorSound() {
  setLEDColor("red");
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
  setLEDColor("green");
  tone(buzzerPin, 1000); // Play tone at 1000 Hz
  delay(150); // Continue for 150 ms
  tone(buzzerPin, 1200); // Increase tone to 1200 Hz
  delay(150);
  tone(buzzerPin, 1400);
  delay(150);
  noTone(buzzerPin);
}

void playSetupCompleteSound() {
  setLEDColor("green");
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

void blinkLED() {
  unsigned long currentMillis = millis();

  // Non-blocking LED blink code
  if (currentMillis - lastBlink >= blinkInterval) {
    // save the last time you blinked the LED
    lastBlink = currentMillis;

    // Toggle the LED states based on the control variables
    if (blinkRed) {
      digitalWrite(redPin, !digitalRead(redPin));
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, LOW);
    } else if (blinkGreen) {
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, HIGH);
      digitalWrite(bluePin, LOW);
    } else if (blinkBlue) {
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, !digitalRead(bluePin));
    } else {
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, LOW);
    }
  }
}

void setLEDColor(String color) {
  // Set all blink flags to false initially
  blinkRed = false;
  blinkGreen = false;
  blinkBlue = false;

  // Check the input string and set the appropriate flag to true
  if (color == "red") {
    blinkRed = true;
  } else if (color == "green") {
    blinkGreen = true;
  } else if (color == "blue") {
    blinkBlue = true;
  }
}

int getBatteryLevel() {
  // Read the raw analog value from the battery pin
  int rawValue = analogRead(batteryPin);

  // Debugging output for raw ADC value
  Serial.print("Raw ADC Value: ");
  Serial.println(rawValue);

  // If the raw ADC value is 0, there's likely an issue with the hardware connection
  if (rawValue == 0) {
    Serial.println("Error: ADC read value is 0. Check battery connection and pin configuration.");
    return 0;
  }

  // Convert the raw value to a voltage level
  // ESP32 ADC resolution is 12 bits (4095 levels) and default reference voltage is 3.3V
  float batteryVoltage = (rawValue / 4095.0) * 3.3;

  // Debugging output for voltage
  Serial.print("Battery Voltage: ");
  Serial.println(batteryVoltage);

  // Adjust the voltage based on the known maximum ADC input voltage
  // SparkFun ESP32 Thing has a voltage divider that scales down the voltage by a factor of 2
  batteryVoltage *= 2;

  // Debugging output for adjusted voltage
  Serial.print("Adjusted Battery Voltage: ");
  Serial.println(batteryVoltage);

  // Calculate the battery percentage
  int batteryPercentage = (int)((batteryVoltage - minBatteryVoltage) / (maxBatteryVoltage - minBatteryVoltage) * 100);

  // Ensure the percentage is within 0-100%
  if (batteryPercentage < 0) {
    batteryPercentage = 0;
  } else if (batteryPercentage > 100) {
    batteryPercentage = 100;
  }

  // Debugging output for battery percentage
  Serial.print("Battery Percentage: ");
  Serial.println(batteryPercentage);

  return batteryPercentage;
}
