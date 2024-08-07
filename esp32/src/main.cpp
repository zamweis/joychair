/*
 * This example turns the ESP32 into a Bluetooth LE gamepad that presses buttons and moves axis
 *
 * At the moment we are using the default settings, but they can be changed using a BleGamepadConfig instance as a parameter for the begin function.
 *
 * Possible buttons are:
 * BUTTON_1 through BUTTON_16
 * (16 buttons by default. The library can be configured to use up to 128)
 *
 * Possible DPAD/HAT switch position values are:
 * DPAD_CENTERED, DPAD_UP, DPAD_UP_RIGHT, DPAD_RIGHT, DPAD_DOWN_RIGHT, DPAD_DOWN, DPAD_DOWN_LEFT, DPAD_LEFT, DPAD_UP_LEFT
 * (or HAT_CENTERED, HAT_UP, etc.)
 *
 * bleGamepad.setAxes sets all axes at once. There are a few:
 * (x axis, y axis, z axis, rx axis, ry axis, rz axis, slider 1, slider 2)
 *
 * The library can also be configured to support up to 5 simulation controls
 * (rudder, throttle, accelerator, brake, steering), but they are not enabled by default.
 *
 * The library can also be configured to support different function buttons
 * (start, select, menu, home, back, volume increase, volume decrease, volume mute)
 * start and select are enabled by default
 */

#include <BleGamepad.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Constants for hardware configuration
constexpr int RED_PIN = 26;
constexpr int GREEN_PIN = 25;
constexpr int BLUE_PIN = 27;
constexpr int BUZZER_PIN = 13;
constexpr int BOOT_BUTTON_PIN = 0;  // Boot button typically connected to GPIO 0
constexpr int BATTERY_PIN = 36; // ADC1 channel 0 is GPIO36

// Battery configuration
constexpr float MIN_BATTERY_VOLTAGE = 3.0; // Minimum expected battery voltage (discharged LiPo)
constexpr float MAX_BATTERY_VOLTAGE = 4.2; // Maximum expected battery voltage (fully charged LiPo)

// Constants for timings and thresholds
constexpr long BLINK_INTERVAL = 500; // Interval at which to blink (milliseconds)
constexpr int JUMP_THRESHOLD = 3000;
constexpr unsigned long JUMP_COOLDOWN = 500; // 500ms cooldown between jumps
constexpr unsigned long CALIBRATION_DURATION = 5000; // Calibration duration in milliseconds
constexpr float TILT_MINIMUM = 5; // Tolerance for tilt check in radians
constexpr float SITTING_ACCEL_Z_THRESHOLD = 4000; // Threshold for detecting sitting
constexpr float MAX_TILT_TOLERANCE = 45;

// Function declarations
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
void setLEDColor(const String& color);
int applyDeadzone(int value, int deadzone);
float calculateTilt(float pitch, float roll);
float calculateRelativeAngle(float accelX, float accelY, float accelZ);

// Global variables for state management
long lastBlink = 0;  // Will store last time LED was updated
bool blinkRed = false;
bool blinkBlue = false;
bool blinkGreen = false;
bool blinkGreenStatic = false;

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

BleGamepad bleGamepad("Joychair", "iXperienceLab", 100);

int joystickX;
int joystickY;
int joystickZ;
int joystickRx;
int joystickRy;
int joystickRz;

bool forwardDirectionDefined = false;
bool sittingCheckComplete = false;
bool mpuInitialized = false;
unsigned long lastTiltCheckTime = 0;
unsigned long lastJumpTime = 0; // Track the last jump time

void setup() {
  playInitSound();

  // Initialize pins for RGB LED and buzzer
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Set initial LED state
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, HIGH);

  // Initialize battery pin and boot button
  pinMode(BATTERY_PIN, INPUT);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP); // Use internal pull-up for button

  Wire.begin(4, 17);  
  Serial.begin(115200);

  Serial.println("Starting BLE work!");
  bleGamepad.begin();

  // Attempt to initialize MPU6050
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

  static unsigned long lastButtonPressTime = 0; // Track the last button press time
  static bool lastButtonState = HIGH; // Track the previous state of the button
  bool currentButtonState = digitalRead(BOOT_BUTTON_PIN); // Read the current state of the button

  // Check if the boot button is pressed (LOW state because of internal pull-up)
  if (currentButtonState == LOW && lastButtonState == HIGH && currentMillis - lastButtonPressTime >= 1000) {
    recalibrate();  // Call the recalibration for new forward direction
    lastButtonPressTime = currentMillis; // Update the last button press time
  }
  lastButtonState = currentButtonState; // Update the last button state

  // Check the battery level
  //int batteryLevel = getBatteryLevel();

  // Check MPU6050 connection
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

    // Wait for player to sit
    if (!sittingCheckComplete) {
      if (accelZ > SITTING_ACCEL_Z_THRESHOLD) {
        playSuccessSound();
        sittingCheckComplete = true;
        Serial.println("Player detected.");
        Serial.println("Tilt forward and hold for a few seconds...");
        setLEDColor("green");
      }
      
      blinkLED();
      return;
    }

    // Check for continuous tilt of 5 seconds
    if (!forwardDirectionDefined) {
      if (calculateTilt(pitch, roll) >= TILT_MINIMUM) {
        if (lastTiltCheckTime == 0) {
          lastTiltCheckTime = millis(); // Start timing when tilt is sufficient
          playInitSound();
        } else if (currentMillis - lastTiltCheckTime >= CALIBRATION_DURATION) {
          // Calculate relative angle
          relativeAngle = -calculateRelativeAngle(mpu.getAccelerationX(), mpu.getAccelerationY(), mpu.getAccelerationZ());
          Serial.print("Relative Angle: ");
          Serial.println(relativeAngle);

          forwardDirectionDefined = true;
          playSetupCompleteSound();
          Serial.println("Forward direction defined!");
          Serial.println("Setup Complete!");
        }
        if (maxTiltAngle < calculateTilt(pitch, roll) && calculateTilt(pitch, roll) <= MAX_TILT_TOLERANCE) {
          maxTiltAngle = calculateTilt(pitch, roll);
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

    // Always update maxTilt to better fit the player's weight
    if (maxTiltAngle < calculateTilt(pitch, roll) && calculateTilt(pitch, roll) <= MAX_TILT_TOLERANCE) {
      maxTiltAngle = calculateTilt(pitch, roll);
    } 

    float cosAngle = -cos(relativeAngle);
    float sinAngle = -sin(relativeAngle);

    // Rotate Roll and Pitch values according to the relative angle
    float newRoll = pitch * sinAngle + roll * cosAngle;
    float newPitch = pitch * cosAngle - roll * sinAngle;

    // Map new Roll and Pitch values to joystick axes
    joystickX = map(newRoll, -maxTiltAngle, maxTiltAngle, 0, 32767);
    joystickY = map(newPitch, -maxTiltAngle, maxTiltAngle, 0, 32767);
    joystickZ = map(accelZ, -32767, 32767, 0, 32767);
    joystickRx = map(newRoll, -90, 90, 0, 32767);
    joystickRy = map(newPitch, -90, 90, 0, 32767);
    joystickRz = map(yaw, -180, 180, 0, 32767); // Use Yaw for Rz

    // Apply deadzones to X and Y axes
    joystickX = applyDeadzone(joystickX, 5);
    joystickY = applyDeadzone(joystickY, 5);

    if (bleGamepad.isConnected()) {
      if (accelZ > JUMP_THRESHOLD && currentMillis - lastJumpTime >= JUMP_COOLDOWN) {
        Serial.println("Jump detected!");
        bleGamepad.press(BUTTON_5);
        lastJumpTime = currentMillis; // Update the last jump time
      } else {
        bleGamepad.release(BUTTON_5);
      }
      bleGamepad.setAxes(joystickX, joystickY, joystickZ, joystickRx, joystickRy, joystickRz, 16383, 16383);
      //bleGamepad.setBatteryLevel(batteryLevel);
      if (blinkBlue) {
        setLEDColor("none"); 
      }
      
    } else {
      setLEDColor("blue");
    }
  }
  blinkLED();
}

bool initializeMPU6050() {
  Serial.println("Initializing MPU6050...");

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
  constexpr int CENTER = 511; // Center for range [0, 1023]
  int distance = value - CENTER;

  // If the distance from the center is less than the deadzone, set output to center
  if (abs(distance) < deadzone) {
    return CENTER;
  } else {
    // If outside the deadzone, adjust value linearly from the edge of the deadzone
    return (distance > 0) ? map(distance, deadzone, CENTER, CENTER + deadzone, 1023) :
                            map(distance, -CENTER, -deadzone, 0, CENTER - deadzone);
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
  tone(BUZZER_PIN, 1000);  // Play tone at 1000 Hz
  delay(150); // Continue for 150 ms
  noTone(BUZZER_PIN); // Stop tone
  delay(150);
  tone(BUZZER_PIN, 1000);
  delay(150);
  noTone(BUZZER_PIN);
}

void playErrorSound() {
  setLEDColor("red");
  // The buzzer emits a distinct error sound by using a lower tone
  tone(BUZZER_PIN, 500); // Play tone at 500 Hz
  delay(250); // Continue for 250 ms
  noTone(BUZZER_PIN); // Stop tone
  delay(100); // Short pause
  tone(BUZZER_PIN, 500); // Repeat tone at 500 Hz
  delay(250); // Continue for 250 ms
  noTone(BUZZER_PIN); // Stop tone
}

void playSuccessSound() {
  setLEDColor("green-static");
  tone(BUZZER_PIN, 1000); // Play tone at 1000 Hz
  delay(150); // Continue for 150 ms
  tone(BUZZER_PIN, 1200); // Increase tone to 1200 Hz
  delay(150);
  tone(BUZZER_PIN, 1400);
  delay(150);
  noTone(BUZZER_PIN);
}

void playSetupCompleteSound() {
  setLEDColor("none");
  // Play a simple ascending tone to indicate completion
  tone(BUZZER_PIN, 800); // Play tone at 800 Hz
  delay(150); // Continue for 150 ms
  tone(BUZZER_PIN, 1000); // Increase tone to 1000 Hz
  delay(150); // Continue for 150 ms
  tone(BUZZER_PIN, 1200); // Further increase tone to 1200 Hz
  delay(150); // Continue for 150 ms
  noTone(BUZZER_PIN); // Stop tone

  // Add a short pause
  delay(150);

  // Play final high tone to signal readiness
  tone(BUZZER_PIN, 1500); // Play higher tone at 1500 Hz
  delay(300); // Continue for 300 ms
  noTone(BUZZER_PIN); // Stop tone
}

void blinkLED() {
  unsigned long currentMillis = millis();

  // Non-blocking LED blink code
  if (currentMillis - lastBlink >= BLINK_INTERVAL) {
    // Save the last time you blinked the LED
    lastBlink = currentMillis;

    // Toggle the LED states based on the control variables
    if (blinkRed) {
      digitalWrite(RED_PIN, !digitalRead(RED_PIN));
      digitalWrite(GREEN_PIN, LOW);
      digitalWrite(BLUE_PIN, LOW);
    } else if (blinkGreen) {
      digitalWrite(RED_PIN, LOW);
      digitalWrite(GREEN_PIN, !digitalRead(GREEN_PIN));
      digitalWrite(BLUE_PIN, LOW);
    } else if (blinkBlue) {
      digitalWrite(RED_PIN, LOW);
      digitalWrite(GREEN_PIN, LOW);
      digitalWrite(BLUE_PIN, !digitalRead(BLUE_PIN));
    } else if (blinkGreenStatic) {
      digitalWrite(RED_PIN, LOW);
      digitalWrite(GREEN_PIN, HIGH);
      digitalWrite(BLUE_PIN, LOW);
    } else {
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(GREEN_PIN, LOW);
      digitalWrite(BLUE_PIN, HIGH);
    }
  }
}

void setLEDColor(const String& color) {
  // Set all blink flags to false initially
  blinkRed = false;
  blinkGreen = false;
  blinkBlue = false;
  blinkGreenStatic = false;

  // Check the input string and set the appropriate flag to true
  if (color == "red") {
    blinkRed = true;
  } else if (color == "green") {
    blinkGreen = true;
  } else if (color == "blue") {
    blinkBlue = true;
  } else if (color == "green_static") {
    blinkGreenStatic = true;
  }
}

int getBatteryLevel() {
  // Read the raw analog value from the battery pin
  int rawValue = analogRead(BATTERY_PIN);

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
  int batteryPercentage = (int)((batteryVoltage - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE) * 100);

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

// Define the pseudo-function
void recalibrate() {
  Serial.println("CLB button pressed!");
  Serial.println("Waiting for new player");
  forwardDirectionDefined = false;
  sittingCheckComplete = false;
  maxTiltAngle = 0;
  setLEDColor("green");
  playInitSound();
}
