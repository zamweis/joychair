/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*
  This example shows how to send HID (keyboard/mouse/etc) data via BLE
  Note that not all devices support BLE keyboard! BLE Keyboard != Bluetooth Keyboard
*/

#include <Arduino.h>
#include <SPI.h>
#if not defined(_VARIANT_ARDUINO_DUE_X_) && not defined(ARDUINO_ARCH_SAMD)
#include <SoftwareSerial.h>
#endif

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE 1
/*=========================================================================*/


// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// LED 
int redPin = 11;
int greenPin = 10;
int bluePin = 9;
long previousMillis = 0;  // will store last time LED was updated
long interval = 250;      // interval at which to blink (milliseconds)

int16_t joystickX;
int16_t joystickY;
int16_t joystickZ;


// A small helper
void error(const __FlashStringHelper* err) {
  Serial.println(err);
  while (1)
    ;
}

void setup() {
  playInitSound();
  Serial.begin(115200);
  Wire.begin();
  Serial.println(F("Adafruit Bluefruit HID Gamepad"));
  Serial.println(F("---------------------------------------"));
  pinMode(buzzerPin, OUTPUT); // Set digital pin as output for the buzzer
  Joystick.begin(false);

   // Set the RGB LED pins as output
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(8, OUTPUT);  // Set digital pin 8 as an OUTPUT for buzzer
  playInitSound();
  delay(1000);

  if (!initializeMPU6050()) {
    Serial.println("Failed to initialize MPU6050. Check connections and restart.");
    while (true) {
      playErrorSound();
      delay(1000);
    }
  }

  if (!ble.begin(VERBOSE_MODE)) {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println(F("OK!"));

  if (FACTORYRESET_ENABLE) {
    // Perform a factory reset to make sure everything is in a known state
    Serial.println(F("Performing a factory reset: "));
    ble.factoryReset();
  }

  // Disable command echo from Bluefruit
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  // Print Bluefruit information
  ble.info();

  // Enable HID Service if not enabled
  int32_t hid_en = 0;
  ble.sendCommandWithIntReply(F("AT+BLEHIDEN?"), &hid_en);
  if (!hid_en) {
    Serial.println(F("Enable HID: "));
    ble.sendCommandCheckOK(F("AT+BLEHIDEN=1"));
  }

  // Enable HID Service if not enabled
  int32_t gamepad_en = 0;
  ble.sendCommandWithIntReply(F("AT+BLEHIDGAMEPADEN?"), &gamepad_en);
  if (!gamepad_en) {
    Serial.println(F("Enable HID GamePad: "));
    ble.sendCommandCheckOK(F("AT+BLEHIDGAMEPADEN=1"));
  }

  if (!hid_en || !gamepad_en) {
    // Add or remove service requires a reset
    Serial.println(F("Performing a SW reset (service changes require a reset): "));
    !ble.reset();
  }

  Serial.println();
  Serial.println(F("Go to your phone's Bluetooth settings to pair your device"));
  Serial.println(F("then open an application that accepts gamepad input"));
  Serial.println();

  Serial.println("Waiting for player to sit down...");
  playInitSound();
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
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

    accelX = mpu.getAccelerationX();
    accelY = mpu.getAccelerationY();
    accelZ = mpu.getAccelerationZ() - 16383;

    // Convert to degrees
    yaw = ypr[0] * 180 / M_PI;
    pitch = ypr[1] * 180 / M_PI;
    roll = ypr[2] * 180 / M_PI;

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
    joystickX = map(newRoll, -90, 90, 0, 1023);
    joystickY = map(newPitch, -90, 90, 0, 1023);
    joystickZ = map(accelZ, -32767, 32767, 0, 1023);
    joystickRx = map(newRoll, -90, 90, 0, 1023);
    joystickRy = map(newPitch, -90, 90, 0, 1023);
    joystickRz = map(yaw, -180, 180, 0, 1023); // Use Yaw for Rz

    // Apply deadzones to X and Y axes
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

    Joystick.sendState(); // Send updated state to host
    
    int buttons = 0;
    if (joystickZ > jumpThreshold) {
      buttons = 1;
      Serial.println("Jump detected with height: " + String(joystickZ));
    }

    // Send axises and buttons
    String command = String("AT+BLEHIDGAMEPAD=") + String(joystickX) + "," + String(joystickY, DEC) + "," + String(buttons, HEX);
    Serial.println(command);
    ble.println(command);
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
