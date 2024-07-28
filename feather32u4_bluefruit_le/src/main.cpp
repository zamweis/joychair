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


// MPU 
MPU6050 mpu;
Quaternion q;
VectorFloat gravity;
uint16_t packetSize;
uint8_t fifoBuffer[64];
float ypr[3];  // yaw, pitch, roll

// LED 
int redPin = 11;
int greenPin = 10;
int bluePin = 9;
long previousMillis = 0;  // will store last time LED was updated
long interval = 250;      // interval at which to blink (milliseconds)
int16_t jumpThreshold = 800;

int16_t joystickX;
int16_t joystickY;
int16_t joystickZ;

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
  tone(6, 500);  // Play tone at 500 Hz
  delay(250);    // Continue for 0.25 seconds
  noTone(6);     // Stop the tone
  delay(100);    // Short pause
  tone(6, 500);  // Repeat tone at 500 Hz
  delay(250);    // Continue for 0.25 seconds
  noTone(6);     // Stop the tone
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


// A small helper
void error(const __FlashStringHelper* err) {
  Serial.println(err);
  while (1)
    ;
}


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void) {
  //while (!Serial);  // required for Flora & Micro
  
  Wire.begin();
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit HID Gamepad"));
  Serial.println(F("---------------------------------------"));

  // Set the RGB LED pins as output
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(8, OUTPUT);  // Set digital pin 8 as an OUTPUT for buzzer
  playInitSound();
  delay(1000);

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    playErrorSound();
    while (1)
      ;
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

  delay(1000);
  playSetupCompleteSound();

  // Initialise the module
  //Serial.print(F("Initialising the Bluefruit LE module: "));

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
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop() {
  // Check if there's new data from the DMP
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // Get the quaternion data
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    // Get the gravity vector
    mpu.dmpGetGravity(&gravity, &q);
    // Get yaw, pitch, and roll
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float accelZ = mpu.getAccelerationZ() - 16383;

    // Convert to degrees
    //float yaw = ypr[0] * 180 / M_PI;
    float pitch = ypr[1] * 180 / M_PI;
    float roll = ypr[2] * 180 / M_PI;
    // Linearly map angles and accelerations to joystick ranges
    
     joystickX = map(roll, -90, 90, -32768, 32767);
     joystickY = map(pitch, -90, 90, -32768, 32767);
     joystickZ = accelZ;
  }
    /*
    int16_t joystickRx = map(pitch, -90, 90, 0, 1023);
    int16_t joystickRy = map(yaw, -180, 180, 0, 1023) + 11;  // Sensor got error of 14
    int16_t joystickRz = map(roll, -90, 90, 0, 1023);
*/
    // Deadzones for X-and Y-Axis
    //joystickX = applyDeadzone(roll, 11);
   // joystickY = applyDeadzone(pitch, 11);
    int buttons = 0;
    if (joystickZ > jumpThreshold) {
      buttons = 1;
      Serial.println("Jump detected with height: " + String(joystickZ));
    }

    // Send axises and buttons
    String command = String("AT+BLEHIDGAMEPAD=") + String(joystickX) + "," + String(joystickY, DEC) + "," + String(buttons, HEX);
    Serial.println(command);
    ble.println(command);

    // scaning period is 50 ms
    delay(50);
  
}


int16_t applyDeadzone(int16_t value, int16_t deadzone) {
  // Calculate the center point of the output range
  const int16_t center = 511;  // Center for the range [0, 1023]
  int16_t distance = value - center;

  // If the distance from center is less than the deadzone, set output to center
  if (abs(distance) < deadzone) {
    return center;
  } else {
    // If outside the deadzone, adjust the value linearly from the edge of the deadzone
    return (distance > 0) ? map(distance, deadzone, 511, center + deadzone, 1023) : map(distance, -511, -deadzone, 0, center - deadzone);
  }
}
