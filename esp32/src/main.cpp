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
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

// Define the number of buttons and hat switches
#define numOfButtons 1 // No physical buttons, no virtual buttons needed
#define numOfHatSwitches 0 // No hat switches

// Enable axes based on MPU6050 capabilities
#define enableX true
#define enableY true
#define enableZ true
#define enableRZ true
#define enableRX true
#define enableRY true
#define enableSlider1 false
#define enableSlider2 false
#define enableRudder false
#define enableThrottle false
#define enableAccelerator false
#define enableBrake false
#define enableSteering false

// Function and variable declarations
void blinkLED();
void recalibrate();
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
float calculateRelativeAngle(float accelX, float accelY, float accelZ);

// Define pin numbers for the RGB LED and button
int redPin = 26;
int greenPin = 25;
int bluePin = 27;
int buzzerPin = 13;
int bootButtonPin = 0;  // Boot button is typically connected to GPIO 0
long lastBlink = 0;  // will store last time LED was updated
long blinkInterval = 500;  // interval at which to blink (milliseconds)
bool isConnected = false;
bool blinkRed = false;
bool blinkGreen = false;
bool blinkYellow = false;


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
float maxTiltAngle; // Maximal tilt angle of player

MPU6050 mpu;

SFE_MAX1704X lipo; // Defaults to the MAX17043

BleGamepad bleGamepad("Joychair", "iXperienceLab", 50);

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
const float tiltMinimum = 5; // Tolerance for tilt check in radiants
const float sittingAccelZThreshold = 4000; // Threshold for detecting sitting
const float maxTiltTolerance = 45; // maximal estimated possible tilt 

void setup() {
  Serial.begin(115200);
  Wire.begin(4, 17);  
  playInitSound();

  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, HIGH);

  if (lipo.begin() == false) {
    Serial.println(F("MAX17043 not detected. Please check wiring. Freezing."));
    while (1)
      ;
  }
  
  lipo.quickStart();

  // initialize the digital pin as an output.
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Initialize the battery pin as an input
  pinMode(batteryPin, INPUT);

  // Initialize the boot button pin as input with internal pull-up resistor
  pinMode(bootButtonPin, INPUT_PULLUP);

  Serial.println("Starting BLE work!");
  // Initialize BLE Gamepad
  BleGamepadConfiguration bleGamepadConfig;
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD);
  bleGamepadConfig.setButtonCount(numOfButtons);
  bleGamepadConfig.setHatSwitchCount(numOfHatSwitches);
  bleGamepadConfig.setWhichAxes(enableX, enableY, enableZ, enableRX, enableRY, enableRZ, enableSlider1, enableSlider2);
  bleGamepadConfig.setWhichSimulationControls(enableRudder, enableThrottle, enableAccelerator, enableBrake, enableSteering);
  bleGamepadConfig.setModelNumber("1.0");
  bleGamepadConfig.setSoftwareRevision("Software Rev 1");
  bleGamepadConfig.setSerialNumber("1337");
  bleGamepadConfig.setFirmwareRevision("2.0");
  bleGamepadConfig.setHardwareRevision("4.0");

  bleGamepad.begin(&bleGamepadConfig);
  if (!initializeMPU6050()) {
    Serial.println("Failed to initialize MPU6050. Check connections and restart.");
    while (true) {
      delay(500);
      playErrorSound();
      delay(500);
    }
  }

  Serial.println("Waiting for player to sit down...");
  playInitSound();
}

void loop() {
  long currentMillis = millis();
  float soc = lipo.getSOC();
  // Set LED color based on battery level
  if (soc < 20.0) {
    setLEDColor("red");
  } else if (soc < 50.0) {
    setLEDColor("yellow");
  } else {
    setLEDColor("green");
  }  

  static unsigned long lastButtonPressTime = 0; 
  static bool lastButtonState = HIGH; 
  bool currentButtonState = digitalRead(bootButtonPin); 

  if (currentButtonState == LOW && lastButtonState == HIGH && currentMillis - lastButtonPressTime >= 1000) {
    recalibrate();  
    lastButtonPressTime = currentMillis; 
  }
  lastButtonState = currentButtonState; 

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection lost. Trying to reconnect...");
    playErrorSound();
    unsigned long startAttemptTime = millis();
    while (!mpu.testConnection() && millis() - startAttemptTime < 10000) { 
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
      delay(5000); 
      return;
    }
  }

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    accelZ = mpu.getAccelerationZ() - 16383;
    yaw = ypr[0] * 180 / M_PI;
    pitch = ypr[1] * 180 / M_PI;
    roll = ypr[2] * 180 / M_PI;

    if (!sittingCheckComplete) {
      if (accelZ > sittingAccelZThreshold) {
        playSuccessSound();
        sittingCheckComplete = true;
        Serial.println("Player detected.");
        Serial.println("Tilt forward and hold for a few seconds...");
        setLEDColor("green");
      }
      blinkLED();
      return;
    }

    if (!forwardDirectionDefined) {
      if (calculateTilt(pitch, roll) >= tiltMinimum) {
        if (lastTiltCheckTime == 0) {
          lastTiltCheckTime = millis(); 
          playInitSound();
        } else if (currentMillis - lastTiltCheckTime >= calibrationDuration) {
          relativeAngle = -calculateRelativeAngle(mpu.getAccelerationX(), mpu.getAccelerationY(), mpu.getAccelerationZ());
          Serial.print("Relative Angle: ");
          Serial.println(relativeAngle);

          forwardDirectionDefined = true;
          playSetupCompleteSound();
          Serial.println("Forward direction defined!");
          Serial.println("Setup Complete!");
        }
        if (maxTiltAngle < calculateTilt(pitch, roll) && calculateTilt(pitch, roll) <= maxTiltTolerance) {
          maxTiltAngle = calculateTilt(pitch, roll);
        }
      } else {
        if (lastTiltCheckTime != 0) { 
          playErrorSound();
        }
        lastTiltCheckTime = 0; 
      }
      blinkLED();
      return;
    }

    if (maxTiltAngle < calculateTilt(pitch, roll) && calculateTilt(pitch, roll) <= maxTiltTolerance) {
      maxTiltAngle = calculateTilt(pitch, roll);
    } 

    float cosAngle = -cos(relativeAngle);
    float sinAngle = -sin(relativeAngle);

    float newRoll = pitch * sinAngle + roll * cosAngle;
    float newPitch = pitch * cosAngle - roll * sinAngle;

    newRoll = constrain(newRoll, -maxTiltAngle, maxTiltAngle);
    newPitch = constrain(newPitch, -maxTiltAngle, maxTiltAngle);

    joystickX = map(newRoll, -maxTiltAngle, maxTiltAngle+1, 0, 32767);
    joystickY = map(newPitch, -maxTiltAngle, maxTiltAngle+1, 0, 32767);
    joystickZ = map(accelZ, -32767, 32767, 0, 32767);
    joystickRx = map(newRoll, -90, 90, 0, 32767);
    joystickRy = map(newPitch, -90, 90, 0, 32767);
    joystickRz = map(yaw, -180, 180, 0, 32767); 

    joystickX = applyDeadzone(joystickX, 5);
    joystickY = applyDeadzone(joystickY, 5);

    if (bleGamepad.isConnected()) {
      if (accelZ > jumpThreshold) {
        Serial.println("Jump detected!");
        bleGamepad.press(BUTTON_1);
      } else {
        bleGamepad.release(BUTTON_1);
      }
      bleGamepad.setAxes(joystickX, joystickY, joystickZ, joystickRz, joystickRx, joystickRy);
      bleGamepad.setBatteryLevel(soc);
      isConnected=true;
    } else {
      isConnected = false;
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
  setLEDColor("green");
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
  setLEDColor("none");
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

    if (isConnected) {
      // Toggle the LED states based on the control variables
      if (blinkRed) {
        digitalWrite(redPin, HIGH);
        digitalWrite(greenPin, LOW);
        digitalWrite(bluePin, LOW);
      } else if (blinkGreen) {
        digitalWrite(redPin, LOW);
        digitalWrite(greenPin, HIGH);
        digitalWrite(bluePin, LOW);
      } else if (blinkYellow) {
        digitalWrite(redPin, HIGH);
        digitalWrite(greenPin, HIGH);
        digitalWrite(bluePin, LOW);
      } else {
        // Default state Pink
        digitalWrite(redPin, HIGH);
        digitalWrite(greenPin, LOW);
        digitalWrite(bluePin, HIGH);
      }
    } else {
      if (blinkRed) {
        digitalWrite(redPin, !digitalRead(redPin));
        digitalWrite(greenPin, LOW);
        digitalWrite(bluePin, LOW);
      } else if (blinkGreen) {
        digitalWrite(redPin, LOW);
        digitalWrite(greenPin, !digitalRead(greenPin));
        digitalWrite(bluePin, LOW);
      } else if (blinkYellow) {
        digitalWrite(redPin, !digitalRead(redPin));
        digitalWrite(greenPin, !digitalRead(greenPin));
        digitalWrite(bluePin, LOW);
      } else {
        // Default state Pink
        digitalWrite(redPin, HIGH);
        digitalWrite(greenPin, LOW);
        digitalWrite(bluePin, HIGH);
      }
    }  
  }
}

void setLEDColor(String color) {
  // Set all blink flags to false initially
  blinkRed = false;
  blinkGreen = false;
  blinkYellow = false;

  // Statische Farben, wenn verbunden
  if (color == "red") {
    blinkRed = true;
  } else if (color == "green") {
    blinkGreen = true;
  } else if (color == "yellow") {
    blinkYellow = true;
  } else if (color == "none") {
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, HIGH);
  }
}

// Reclibration for forward direction
void recalibrate() {
  Serial.println("CLB button pressed!");
  Serial.println("Waiting for new player");
  forwardDirectionDefined = false;
  sittingCheckComplete = false;
  maxTiltAngle = 0;
  setLEDColor("green");
  playInitSound();
}
