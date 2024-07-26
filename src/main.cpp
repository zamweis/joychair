#include <Wire.h>
#include <Joystick.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Deklarationen der Funktionen und Variablen
void playInitSound();
void playSuccessSound();
void playSetupCompleteSound();
void playErrorSound();
void printRollPitchYawAccelZ();
void printJoystickValues();
int applyDeadzone(int value, int deadzone);
bool checkTilt(float pitch, float roll, float tolerance);
float calculateRelativeAngle(float accelX, float accelY, float accelZ);

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
float relativeAngle; // Winkel zur Drehung des Sensors

MPU6050 mpu;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                   1, 0,                   // Anzahl der Buttons, Hat Switch Count
                   true, true, true,       // X und Y, und Z Achse
                   true, true, true,       // Rx, Ry und Rz
                   false, false,           // Ruder und Gashebel
                   false, false, false);   // Beschleuniger, Bremse und Steuerung

int joystickX;
int joystickY;
int joystickZ;
int joystickRx;
int joystickRy;
int joystickRz;

float cosAngle;
float sinAngle;

int jumpThreshold = 800;
int buzzerPin = 6;

bool forwardDirectionDefined = false;
bool sittingCheckComplete = false;
unsigned long lastTiltCheckTime = 0;
const unsigned long calibrationDuration = 5000; // Kalibrierdauer in Millisekunden
const float tiltTolerance = 10.0; // Toleranz für die Neigungsprüfung in Grad
const float sittingAccelZThreshold = 4000; // Schwellenwert für das Erkennen des Sitzens

void setup() {
  Serial.begin(115200);
  Wire.begin();
  playInitSound();
  Serial.println("Initializing...");

  pinMode(buzzerPin, OUTPUT); // Setzen des digitalen Pins als Ausgang für den Summer
  Joystick.begin(false);

  // Initialisieren des MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    playErrorSound();
    while (1);
  }
  
  // Initialisieren des DMP
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
  Serial.println("Calibration complete!");
  playSuccessSound();
  delay(1000);
  Serial.println("Waiting for player to sit down...");
  playInitSound();
}

void loop() {
  long currentMillis = millis();
  // Prüfen, ob neue Daten vom DMP vorliegen
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // Holen der Quaternion-Daten
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    // Holen des Schwerkraftvektors
    mpu.dmpGetGravity(&gravity, &q);
    // Holen von Yaw, Pitch und Roll
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    accelX = mpu.getAccelerationX();
    accelY = mpu.getAccelerationY();
    accelZ = mpu.getAccelerationZ() - 16383; 

    // Umrechnung in Grad
    yaw = ypr[0] * 180/M_PI;
    pitch = ypr[1] * 180/M_PI;
    roll = ypr[2] * 180/M_PI;
    
    // Warten, bis der Spieler sitzt
    if (!sittingCheckComplete) {
      if (accelZ > sittingAccelZThreshold) {
        playSuccessSound();
        sittingCheckComplete = true;
        Serial.println("Player detected.");
        Serial.println("Tilt forward and hold for a few seconds...");
      }
      return;
    }

    // Prüfen auf ununterbrochene Neigung von 5 Sekunden
    if (!forwardDirectionDefined) {
        if (checkTilt(pitch, roll, tiltTolerance)) {
          if (lastTiltCheckTime == 0) {
            lastTiltCheckTime = millis(); // Starten der Zeitmessung, wenn die Neigung ausreichend ist
            playInitSound();
          } else if (currentMillis - lastTiltCheckTime >= calibrationDuration) {
            // Berechnung des relativen Winkels
            relativeAngle = -calculateRelativeAngle(mpu.getAccelerationX(), mpu.getAccelerationY(), mpu.getAccelerationZ());
            Serial.print("Relative Angle: ");
            Serial.println(relativeAngle);

            forwardDirectionDefined = true;
            playSetupCompleteSound();
            Serial.println("Forward direction defined!");
            Serial.println("Setup Complete!");
          }
        } else {
          if (lastTiltCheckTime != 0) { // Fehlergeräusch nur einmal abspielen
            playErrorSound();
          }
          lastTiltCheckTime = 0; // Zurücksetzen der Zeitmessung, wenn die Neigung nicht ausreichend ist
        }
        return;
    }
    
    float cosAngle = -cos(relativeAngle);
    float sinAngle = -sin(relativeAngle);

    // Drehung der Roll- und Pitch-Werte entsprechend des relativen Winkels
    float newRoll = pitch * sinAngle + roll * cosAngle;
    float newPitch = pitch * cosAngle - roll * sinAngle;

    // Korrekte Zuordnung der neuen Roll- und Pitch-Werte zu den Joystick-Achsen
    joystickX = map(newRoll, -90, 90, 0, 1023);
    joystickY = map(newPitch, -90, 90, 0, 1023);
    joystickZ = map(accelZ, -32767, 32767, 0, 1023);
    joystickRx = map(newRoll, -90, 90, 0, 1023);
    joystickRy = map(newPitch, -90, 90, 0, 1023);
    joystickRz = map(yaw, -180, 180, 0, 1023); // Verwenden von Yaw für Rz
    
    // Deadzones für X- und Y-Achse
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

    Joystick.sendState(); // Senden des aktualisierten Zustands an den Host
  }
}

int applyDeadzone(int value, int deadzone) {
    // Berechnung des Mittelpunktes des Ausgabebereichs
    const int center = 511;  // Mittelpunkt für den Bereich [0, 1023]
    int distance = value - center;
    
    // Wenn der Abstand vom Mittelpunkt kleiner als die Deadzone ist, den Ausgang auf den Mittelpunkt setzen
    if (abs(distance) < deadzone) {
        return center;
    } else {
        // Wenn außerhalb der Deadzone, den Wert linear vom Rand der Deadzone anpassen
        return (distance > 0) ? map(distance, deadzone, 511, center + deadzone, 1023) : 
                                map(distance, -511, -deadzone, 0, center - deadzone);
    }
}

float calculateRelativeAngle(float accelX, float accelY, float accelZ) {
  // Normalisieren der Beschleunigungswerte
  float norm = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
  accelX /= norm;
  accelY /= norm;
  accelZ /= norm;
  
  // Berechnung der Projektion auf die XY-Ebene (Bodenprojektion)
  float proj_x = accelX;
  float proj_y = accelY;
  
  // Berechnung des Winkels in der XY-Ebene zur Vorwärtsrichtung (positive y-Achse)
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
  tone(buzzerPin, 1000);  // Ton bei 1000 Hz abspielen
  delay(150);     // Fortsetzen für 1 Sekunde
  noTone(buzzerPin);      // Ton stoppen
  delay(150);
  tone(buzzerPin, 1000);
  delay(150);
  noTone(buzzerPin);
}

void playErrorSound() {
  // Der Summer gibt ein deutliches Fehlergeräusch von sich, indem ein tieferer Ton verwendet wird
  tone(buzzerPin, 500);   // Ton bei 500 Hz abspielen
  delay(250);     // Fortsetzen für 0,25 Sekunden
  noTone(buzzerPin);      // Ton stoppen
  delay(100);     // Kurze Pause
  tone(buzzerPin, 500);   // Ton bei 500 Hz wiederholen
  delay(250);     // Fortsetzen für 0,25 Sekunden
  noTone(buzzerPin);      // Ton stoppen
}

void playSuccessSound() {
  tone(buzzerPin, 1000);  // Ton bei 1000 Hz abspielen
  delay(150);     // Fortsetzen für 0,75 Sekunden
  tone(buzzerPin, 1200);  // Ton bei 1000 Hz abspielen
  delay(150);
  tone(buzzerPin, 1400);
  delay(150);
  noTone(buzzerPin);
}

void playSetupCompleteSound() {
  // Einfachen aufsteigenden Ton abspielen, um die Fertigstellung anzuzeigen
  tone(buzzerPin, 800);   // Ton bei 800 Hz abspielen
  delay(150);     // Fortsetzen für 0,2 Sekunden
  tone(buzzerPin, 1000);  // Ton auf 1000 Hz erhöhen
  delay(150);     // Fortsetzen für 0,2 Sekunden
  tone(buzzerPin, 1200);  // Ton weiter auf 1200 Hz erhöhen
  delay(150);     // Fortsetzen für 0,2 Sekunden
  noTone(buzzerPin);      // Ton stoppen

  // Kurze Pause einlegen
  delay(150);

  // Letzten hohen Ton abspielen, um die Bereitschaft endgültig zu signalisieren
  tone(buzzerPin, 1500);  // Höheren Ton bei 1500 Hz abspielen
  delay(300);     // Fortsetzen für 0,3 Sekunden
  noTone(buzzerPin);      // Ton stoppen
}
