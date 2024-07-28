# Joychair Interface for VR Navigation - Arduino Leonardo

Welcome to the Joychair Interface project for Arduino Leonardo! This setup allows you to integrate the movements of an Aeris Swopper chair into virtual reality (VR) environments using an Arduino Leonardo board and sensors.

## Project Overview

The Joychair Interface captures the movements of the Aeris Swopper chair and integrates them into a solution for VR environments, particularly in Unity. This setup uses the Arduino Leonardo microcontroller to read data from an Inertial Measurement Unit (IMU) and transmit it wirelessly/wired.

## Hardware Setup

### Components
- **Arduino Leonardo**
- **MPU6050 IMU**
- **Buzzer**
- **Aeris Swopper Chair**

### Casing

3D-printed with a prusa MK2
[Thingyverse](https://www.thingiverse.com/thing:628929)

### Wiring
1. Connect the MPU6050 IMU to the Arduino Leonardo via I2C (SDA to A4, SCL to A5).
2. Connect the buzzer to pin 6 of the Arduino Leonardo.
3. Attach the MPU6050 IMU to the Aeris Swopper chair.

## Software Setup

### Libraries
Make sure you have the following libraries installed in your PlatformIO project:

```ini
lib_deps = 
    mheironimus/Joystick@^2.1.1
    mike-matera/ArduinoSTL@^1.3.3
    mbed-syundo0730/I2Cdev@0.0.0+sha.3aa973ebe3e5
    electroniccats/MPU6050@^1.3.1
