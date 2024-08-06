# Joychair Interface for VR Navigation - ESP32

Welcome to the Joychair Interface project for ESP32! This setup allows you to integrate the movements of an Aeris Swopper chair into virtual reality (VR) environments using an Sparkfun ESP32 Thing board. The ESP32 can transmit data over Bluetooth connection.

## Project Overview

The Joychair Interface captures the movements of the Aeris Swopper chair and integrates them directly into VR environments, particularly in Unity. This setup uses the ESP32 microcontroller to read data from an Inertial Measurement Unit (IMU) and transmit it wirelessly via Bluetooth connection.

## Hardware Setup

### Components
- **ESP32**
- **MPU6050 IMU**
- **Buzzer**
- **RGB LED**
- **Aeris Swopper Chair**

### Wiring
1. Connect the MPU6050 IMU to the ESP32 via I2C (SDA to GPIO21, SCL to GPIO22).
2. Connect the buzzer to a GPIO pin (e.g., GPIO6).
3. Connect the RGB LED to three GPIO pins (e.g., GPIO11 for red, GPIO10 for green, GPIO9 for blue).
4. Attach the MPU6050 IMU to the Aeris Swopper chair.

### Casing
## Spatial Light Transfer Files

You can find the SLT files in the [case](./case) directory.

- [Case.slt](./case/case.STL)
- [Case_Lid.slt](./case/case_lid.STL)
- [Push_Button.slt](./case/push_button.STL)
  
## Software Setup

### Libraries
Make sure you have the following libraries installed in your PlatformIO project:

```ini
lib_deps = 
    lemmingdev/ESP32-BLE-Gamepad@^0.5.5
    mbed-syundo0730/I2Cdev@0.0.0+sha.3aa973ebe3e5
    electroniccats/MPU6050@^1.3.1
```
## Description

### LED-Indicator
- ***red blinking:*** An error occurred
- ***green blinking:*** Initialization has begun
- ***blue blinking:*** Bluetooth device not connected
- ***green static:*** Initialization has ended without errors
- ***pink static:*** Code running normaly
