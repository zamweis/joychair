#include <Wire.h>
#include <Joystick.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                   1, 0,                   // Button Count, Hat Switch Count
                   true, true, true,       // X and Y, and Z Axis
                   true, true, true,       // Rx, Ry, and Rz
                   false, false,           // rudder and throttle
                   false, false, false);   // accelerator, brake, and steering


  
void playInitSound();
void playSuccessSound();
void playSetupCompleteSound();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.begin();    
  pinMode(8, OUTPUT);     // Set digital pin 8 as an OUTPUT for buzzer
  mpu.calcOffsets(true, true);
  Joystick.begin(false);
  playSetupCompleteSound();
}

  int16_t joystickXMax=-5000;
  int16_t joystickYMax=-5000;
  int16_t joystickZMax=-5000;
  int16_t joystickRxMax=-5000;
  int16_t joystickRyMax=-5000;
  int16_t joystickRzMax=-5000;

  int16_t joystickXMin=5102;
  int16_t joystickYMin=5102;
  int16_t joystickZMin=5102;
  int16_t joystickRxMin=5012;
  int16_t joystickRyMin=5012;
  int16_t joystickRzMin=5012;

  int16_t jumpThreshold=1020;

void loop() {
  mpu.update();
  float angleX = mpu.getAngleX(); // Roll
  float angleY = mpu.getAngleY(); // Pitch
  float accelZ = mpu.getAccZ();   // Use Z-acceleration to simulate vertical movement
  float gyroX = mpu.getGyroX();   // Rotation rate (gyroscope X-axis)
  float gyroY = mpu.getGyroY();   // Tilt rate (gyroscope Y-axis)
  float gyroZ = mpu.getGyroZ();   // Yaw rate (gyroscope Z-axis)

  float accelZ2= accelZ *1000;


  Serial.print(F("angleX: ")); Serial.print(angleX);
  Serial.print(F(", angleY: ")); Serial.print(angleY);
  Serial.print(F(", accelZ: ")); Serial.print(accelZ);
  Serial.print(F(", accelZ2: ")); Serial.print(accelZ2);
  Serial.print(F(", gyroZ: ")); Serial.print(gyroX);
  Serial.print(F(", gyroY: ")); Serial.print(gyroY);
  Serial.print(F(", gyroZ: ")); Serial.print(gyroZ);

  // Normalize angles from [-90, 90] to [-1, 1]
  float normalizedX = angleX / 30.0;
  float normalizedY = angleY / 30.0;
  float normalizedZ = (accelZ + 2) / 4.0;  // Normalize acceleration assuming range [-2, 2] g

  int range = 5000;

  // Apply a quadratic transformation (moderate exponential effect)
  int16_t joystickX = (int)(511.5 * (pow(normalizedX, 2) * (normalizedX < 0 ? -1 : 1) + 1));
  int16_t joystickY = (int)(511.5 * (pow(normalizedY, 2) * (normalizedY < 0 ? -1 : 1) + 1));
  int16_t joystickZ;
  int16_t joystickRx;
  int16_t joystickRy;
  int16_t joystickRz;


  // Ensure values are within bounds
  joystickX = constrain(joystickX, 0, 1023);
  joystickY = constrain(joystickY, 0, 1023);
  joystickZ = map((int)(accelZ * 100), 0, 200, 0, 1023);
  joystickRx = map(gyroX, -500, 500, 0, 1023);
  joystickRy = map(gyroY, -500, 500, 0, 1023);
  joystickRz = map(gyroZ, -500, 500, 0, 1023);

  
  joystickXMax = joystickX > joystickXMax ? joystickX : joystickXMax;
  joystickYMax = joystickY > joystickYMax ? joystickY : joystickYMax;
  joystickZMax = joystickZ > joystickZMax ? joystickZ : joystickZMax;
  joystickRxMax = joystickRx > joystickRxMax ? joystickRx : joystickRxMax;
  joystickRyMax = joystickRy > joystickRyMax ? joystickRy : joystickRyMax;
  joystickRzMax = joystickRz > joystickRzMax ? joystickRz : joystickRzMax;
  
  joystickXMin = joystickX < joystickXMin ? joystickX : joystickXMin;
  joystickYMin = joystickY < joystickYMin ? joystickY : joystickYMin;
  joystickZMin = joystickZ < joystickZMin ? joystickZ : joystickZMin;
  joystickRxMin = joystickRx < joystickRxMin ? joystickRx : joystickRxMin;
  joystickRyMin = joystickRy < joystickRyMin ? joystickRy : joystickRyMin;
  joystickRzMin = joystickRz < joystickRzMin ? joystickRz : joystickRzMin;

   Serial.print(F(", joystickX: ")); Serial.print(joystickX);
  Serial.print(F(", joystickY: ")); Serial.print(joystickY);
  Serial.print(F(", joystickZ: ")); Serial.print(joystickZ);
  Serial.print(F(", joystickRx: ")); Serial.print(joystickRx);
  Serial.print(F(", joystickRy: ")); Serial.print(joystickRy);
  Serial.print(F(", joystickRz: ")); Serial.println(joystickRz);


/*

Serial.print(F(", joystickX: ")); Serial.print(joystickXMax);
  Serial.print(F(", joystickY: ")); Serial.print(joystickYMax);
  Serial.print(F(", joystickZ: ")); Serial.print(joystickZMax);
  Serial.print(F(", joystickRx: ")); Serial.print(joystickRxMax);
  Serial.print(F(", joystickRy: ")); Serial.print(joystickRyMax);
  Serial.print(F(", joystickRz: ")); Serial.print(joystickRzMax);
  Serial.print("     ");
Serial.print(F(", joystickX: ")); Serial.print(joystickXMin);
  Serial.print(F(", joystickY: ")); Serial.print(joystickYMin);
  Serial.print(F(", joystickZ: ")); Serial.print(joystickZMin);
  Serial.print(F(", joystickRx: ")); Serial.print(joystickRxMin);
  Serial.print(F(", joystickRy: ")); Serial.print(joystickRyMin);
  Serial.print(F(", joystickRz: ")); Serial.println(joystickRzMin);
*/
  Joystick.setXAxis(joystickX);
  Joystick.setYAxis(joystickY);
  Joystick.setZAxis(joystickZ);
  Joystick.setRxAxis(joystickRx);
  Joystick.setRyAxis(joystickRy);
  Joystick.setRzAxis(joystickRz);

  if (joystickZ > jumpThreshold) {
        Joystick.setButton(0, 1);
        Serial.println("Jump detected with height: " + String(joystickZ));
    } else Joystick.setButton(0, 0);

  Joystick.sendState(); // Send the updated state to the host
  delay(3);
}

void playInitSound() {
    // Buzzer
    tone(8, 1000);  // Play tone at 1000 Hz
    delay(150);     // Continue for 1 second
    noTone(8);      // Stop the tone
    delay(150);
    tone(8, 1000);
    delay(150);
    noTone(8);
}

void playSuccessSound() {
    tone(8, 1000);  // Play tone at 1000 Hz
    delay(150);     // Continue for 0.75 seconds
    tone(8, 1200);  // Play tone at 1000 Hz
    delay(150);
    tone(8, 1400);
    delay(150);
    noTone(8);
}

void playSetupCompleteSound() {
    // Play a simple ascending tone sequence to indicate completion
    tone(8, 800);   // Play tone at 800 Hz
    delay(150);     // Continue for 0.2 seconds
    tone(8, 1000);  // Increase tone to 1000 Hz
    delay(150);     // Continue for 0.2 seconds
    tone(8, 1200);  // Increase tone further to 1200 Hz
    delay(150);     // Continue for 0.2 seconds
    noTone(8);      // Stop the tone

    // Add a short pause
    delay(150);

    // Play a final high tone to definitively signal readiness
    tone(8, 1500);  // Play a higher tone at 1500 Hz
    delay(300);     // Continue for 0.3 seconds
    noTone(8);      // Stop the tone
}