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

#include <Arduino.h>
#include <BleGamepad.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

int16_t applyDeadzone(int16_t value, int16_t deadzone);
void playInitSound();
void playSuccessSound();
void playSetupCompleteSound();
void playErrorSound();

uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];  // yaw, pitch, roll
// Define pin numbers for the RGB LED
int redPin = 11;
int greenPin = 10;
int bluePin = 9;
long previousMillis = 0;  // will store last time LED was updated
long interval = 250;  // interval at which to blink (milliseconds)
int16_t jumpThreshold = 800;

int16_t joystickX=16383;
int16_t joystickY=16383;
int16_t joystickZ=16383;
int16_t joystickRx=16383;
int16_t joystickRy=16383;
int16_t joystickRz=16383;

MPU6050 mpu;
BleGamepad bleGamepad("Joychair", "iXperienceLab", 100);

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting BLE work!");
    bleGamepad.begin();
    /*
    // The default bleGamepad.begin() above enables 16 buttons, all axes, one hat, and no simulation controls or special buttons
     // Set the RGB LED pins as output
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
    pinMode(8, OUTPUT);       // Set digital pin 8 as an OUTPUT for buzzer
    playInitSound();
    delay(1000);
    */
    Wire.begin();
    Serial.println("Initializing...");

    // Initialize MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
      Serial.println("MPU6050 connection failed!");
      //playErrorSound();
     // while (1);
    }
    
    // Initialize DMP
    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
      mpu.setDMPEnabled(true);
      packetSize = mpu.dmpGetFIFOPacketSize();
      Serial.println("DMP ready!");
    } else {
      Serial.print("DMP Initialization failed (code ");
      //playErrorSound();
      Serial.print(devStatus);
      Serial.println(")");
    }

    mpu.CalibrateAccel();
    mpu.CalibrateGyro();
    
    //playSetupCompleteSound();
    
}

void loop()
{
     // Check if there's new data from the DMP
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
      // Get the quaternion data
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      // Get the gravity vector
      mpu.dmpGetGravity(&gravity, &q);
      // Get yaw, pitch, and roll
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      float accelZ = mpu.getAccelerationZ() - 16383; 

      // Convert to degrees
      float yaw = ypr[0] * 180/M_PI;
      float pitch = ypr[1] * 180/M_PI;
      float roll = ypr[2] * 180/M_PI;
      // Linearly map angles and accelerations to joystick ranges
      joystickX = map(roll, -100, 100, 0, 32767);
      joystickY = map(pitch, -100, 100, 0, 32767);
      joystickZ = map(accelZ, -32767, 32767, 0, 32767);
      joystickRx = map(pitch, -180, 180, 0, 32767);
      joystickRy = map(yaw, -180, 180, 0, 32767)+11; // Sensor got error of 14
      joystickRz = map(roll, -180, 180, 0, 32767);

      // Deadzones for X-and Y-Axis
      joystickX = applyDeadzone(joystickX, 11);
      joystickY = applyDeadzone(joystickY, 11);
    }
    if (bleGamepad.isConnected())
    {
        if (joystickZ > jumpThreshold)
        {
          bleGamepad.press(BUTTON_5);
        } else {
          bleGamepad.release(BUTTON_5);
        }

        bleGamepad.setAxes(joystickX, joystickY, joystickZ, joystickRx, joystickRy, joystickRz, 16383, 16383);
        // All axes, sliders, hats etc can also be set independently. See the IndividualAxes.ino example
    }
}

int16_t applyDeadzone(int16_t value, int16_t deadzone) {
    // Calculate the center point of the output range
    const int16_t center = 16383;  // Center for the range [0, 1023]
    int16_t distance = value - center;
    
    // If the distance from center is less than the deadzone, set output to center
    if (abs(distance) < deadzone) {
        return center;
    } else {
        // If outside the deadzone, adjust the value linearly from the edge of the deadzone
        return (distance > 0) ? map(distance, deadzone, center, center + deadzone, 32767) : 
                                map(distance, -center, -deadzone, 0, center - deadzone);
    }
}

void playInitSound() {
  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin, LOW);
  // Buzzer
  tone(6, 1000);  // Play tone at 1000 Hz
  delay(150);     // Continue for 1 second
  noTone(6);      // Stop the tone
  delay(150);
  tone(6, 1000);
  delay(150);
  noTone(6);
}

void playErrorSound() {
  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);
    // Buzzer emits a distinct error sound, using a lower tone
    tone(6, 500);   // Play tone at 500 Hz
    delay(250);     // Continue for 0.25 seconds
    noTone(6);      // Stop the tone
    delay(100);     // Short pause
    tone(6, 500);   // Repeat tone at 500 Hz
    delay(250);     // Continue for 0.25 seconds
    noTone(6);      // Stop the tone
}

void playSuccessSound() {
  tone(6, 1000);  // Play tone at 1000 Hz
  delay(150);     // Continue for 0.75 seconds
  tone(6, 1200);  // Play tone at 1000 Hz
  delay(150);
  tone(6, 1400);
  delay(150);
  noTone(6);
}

void playSetupCompleteSound() {
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin, LOW);
  // Play a simple ascending tone sequence to indicate completion
  tone(6, 800);   // Play tone at 800 Hz
  delay(150);     // Continue for 0.2 seconds
  tone(6, 1000);  // Increase tone to 1000 Hz
  delay(150);     // Continue for 0.2 seconds
  tone(6, 1200);  // Increase tone further to 1200 Hz
  delay(150);     // Continue for 0.2 seconds
  noTone(6);      // Stop the tone

  // Add a short pause
  delay(150);

  // Play a final high tone to definitively signal readiness
  tone(6, 1500);  // Play a higher tone at 1500 Hz
  delay(300);     // Continue for 0.3 seconds
  noTone(6);      // Stop the tone
}