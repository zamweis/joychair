#include <Wire.h>
#include <Joystick.h>
#include <MPU6050.h>

MPU6050 mpu;
int16_t ax; // Acceleration along the X-axis
int16_t ay; // Acceleration along the Y-axis
int16_t az; // Acceleration along the Z-axis
int16_t gx; // Gyroscope reading along the X-axis
int16_t gy; // Gyroscope reading along the Y-axis
int16_t gz; // Gyroscope reading along the Z-axis

// Offset for accelerometer and gyroscope
int16_t ax_offset, ay_offset, az_offset;
int16_t gx_offset, gy_offset, gz_offset;

// Joystick setup
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                   7, 0,                // 7 Buttons, no Hat Switch
                   true, true, false,   // X and Y Axes (used for left-right and forward-backward)
                   false, false, true); // No Rx, No Ry, Rz for Rotation (used for rotating left-right)

// Thresholds for movements and rotations
const int accelThreshold = 8000; // Acceleration threshold for movement detection
const int gyroThreshold = 6000;  // Gyroscope threshold for rotational detection
const int jumpThreshold = 20000; // Threshold for jump detection
const int numReadings = 1000;    // Calibration readings


void calibrateSensors();
void checkForMovement();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1); // Halt if connection failed
  }
  
  calibrateSensors(); // Perform sensor calibration
  
  Joystick.begin();
  Serial.println("Device is calibrated and ready!");
}

void loop() {
  static unsigned long lastMillis = 0;
  if (millis() - lastMillis >= 10) {
    lastMillis = millis();
    
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax -= ax_offset;
    ay -= ay_offset;
    az -= az_offset;
    gx -= gx_offset;
    gy -= gy_offset;
    gz -= gz_offset;

    checkForMovement();
  }
}

void checkForMovement() {
    // Output detected movements to the console or update game state
    /*
    Serial.print("Smoothed Ax: "); Serial.print(ax); Serial.print(", ");
    Serial.print("Smoothed Ay: "); Serial.print(ay); Serial.print(", ");
    Serial.print("Smoothed Az: "); Serial.print(az); Serial.print(", ");
    Serial.print("Smoothed Gx: "); Serial.print(gx); Serial.print(", ");
    Serial.print("Smoothed Gy: "); Serial.print(gy); Serial.print(", ");
    Serial.print("Smoothed Gz: "); Serial.println(gz);
    */

    // Map movements to joystick controls
    Joystick.setXAxis(ax);
    Joystick.setYAxis(ay);
    Joystick.setRzAxis(gz);

    // Handle specific buttons for rotational moves
    Joystick.setButton(1, gx > gyroThreshold ? 1 : 0);    // Rotate right
    Joystick.setButton(2, gx < -gyroThreshold ? 1 : 0);   // Rotate left

    // Directions based on accelerometer thresholds
    Joystick.setButton(3, ax > accelThreshold ? 1 : 0);   // Move right
    Joystick.setButton(4, ax < -accelThreshold ? 1 : 0);  // Move left
    Joystick.setButton(5, ay > accelThreshold ? 1 : 0);   // Move backward
    Joystick.setButton(6, ay < -accelThreshold ? 1 : 0);  // Move forward

    // Apply dead zones to axes to reduce sensitivity to small movements
    const int deadZone = 10000; // Define dead zone threshold

    // Output detected movements to the console
    if (ax > deadZone) {
        Serial.println("Moving right");
    } else if (ax < -deadZone) {
        Serial.println("Moving left");
    }
    if (ay > deadZone) {
        Serial.println("Moving backward");
    } else if (ay < -deadZone) {
        Serial.println("Moving forward");
    }
    if (gx > gyroThreshold) {
        Serial.println("Rotating right");
    } else if (gx < -gyroThreshold) {
        Serial.println("Rotating left");
    }
    if (gz > jumpThreshold) {
        Serial.println("Jump detected");
    }
}

void calibrateSensors() {
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;

    Serial.println("Calibrating sensors, make sure the device is stationary and flat...");

    for (int i = 0; i < numReadings; i++) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        ax_sum += ax; ay_sum += ay; az_sum += az;
        gx_sum += gx; gy_sum += gy; gz_sum += gz;
        delay(10);
    }

    ax_offset = ax_sum / numReadings;
    ay_offset = ay_sum / numReadings;
    az_offset = az_sum / numReadings;
    gx_offset = gx_sum / numReadings;
    gy_offset = gy_sum / numReadings;
    gz_offset = gz_sum / numReadings;

    // Calculate offsets for the X and Y axes
    // These offsets will center the axes
    ax_offset -= 0; // Adjust as needed based on calibration results
    ay_offset -= 0; // Adjust as needed based on calibration results

    Serial.println("Calibration complete.");
    Serial.print("Offsets: ");
    Serial.print(ax_offset); Serial.print(", ");
    Serial.print(ay_offset); Serial.print(", ");
    Serial.print(az_offset); Serial.print(", ");
    Serial.print(gx_offset); Serial.print(", ");
    Serial.print(gy_offset); Serial.print(", ");
    Serial.println(gz_offset);
}
