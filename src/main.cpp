#include <Wire.h>
#include <Joystick.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                   1, 0,                   // Button Count, Hat Switch Count
                   true, true, true,       // X and Y, and Z Axis
                   true, true, true,       // Rx, Ry, and Rz
                   false, false,           // rudder and throttle
                   false, false, false);   // accelerator, brake, and steering

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

void setup() {
  // Set the RGB LED pins as output
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(8, OUTPUT);       // Set digital pin 8 as an OUTPUT for buzzer
  playInitSound();
  delay(1000);
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Initializing...");

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
  
  delay(1000);
  Joystick.begin(false);
  playSetupCompleteSound();
}


void loop() {
  long currentMillis = millis();

  // Non-blocking LED blink code
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    // if the LED is off turn it on and vice-versa:
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, !digitalRead(greenPin));
    digitalWrite(bluePin, LOW);
  }
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
    float yaw = ypr[0] * 180/M_PI;
    float pitch = ypr[1] * 180/M_PI;
    float roll = ypr[2] * 180/M_PI;
/*
    Serial.print(F("roll: ")); Serial.print(roll);
    Serial.print(F(", pitch: ")); Serial.print(pitch);
    Serial.print(F(", yaw: ")); Serial.print(yaw);  
    Serial.print(F(", accelZ: ")); Serial.print(accelZ);
*/

    // Linearly map angles and accelerations to joystick ranges
    int16_t joystickX = map(roll, -45, 45, 0, 1023);
    int16_t joystickY = map(pitch, -45, 45, 0, 1023);
    int16_t joystickZ = map(accelZ, -32767, 32767, 0, 1023);
    int16_t joystickRx = map(pitch, -90, 90, 0, 1023);
    int16_t joystickRy = map(yaw, -180, 180, 0, 1023)+11; // Sensor got error of 14
    int16_t joystickRz = map(roll, -90, 90, 0, 1023);

    Serial.print(F("joystickX: ")); Serial.print(joystickX);
    Serial.print(F(", joystickY: ")); Serial.print(joystickY);
    Serial.print(F(", joystickZ: ")); Serial.print(joystickZ);
    Serial.print(F(", joystickRx: ")); Serial.print(joystickRx);
    Serial.print(F(", joystickRy: ")); Serial.print(joystickRy);
    Serial.print(F(", joystickRz: ")); Serial.println(joystickRz);

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

int16_t applyDeadzone(int16_t value, int16_t deadzone) {
    // Calculate the center point of the output range
    const int16_t center = 511;  // Center for the range [0, 1023]
    int16_t distance = value - center;
    
    // If the distance from center is less than the deadzone, set output to center
    if (abs(distance) < deadzone) {
        return center;
    } else {
        // If outside the deadzone, adjust the value linearly from the edge of the deadzone
        return (distance > 0) ? map(distance, deadzone, 511, center + deadzone, 1023) : 
                                map(distance, -511, -deadzone, 0, center - deadzone);
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