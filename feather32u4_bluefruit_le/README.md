# Joychair Interface for VR Navigation - Feather 32u4 Bluefruit LE

Welcome to the Joychair Interface project for Feather 32u4 Bluefruit LE! This setup allows you to integrate the movements of an Aeris Swopper chair into virtual reality (VR) environments using a Feather 32u4 Bluefruit LE board. Note: Currently, only wired transmission is supported as the Bluetooth function is not working properly.

## Project Overview

The Joychair Interface captures the movements of the Aeris Swopper chair and integrates them directly into VR environments, particularly in Unity. This setup uses the Feather 32u4 Bluefruit LE microcontroller to read data from an Inertial Measurement Unit (IMU) and transmit it via a wired connection.

## Hardware Setup

### Components
- **Feather 32u4 Bluefruit LE**
- **MPU6050 IMU**
- **Buzzer**
- **RGB LED**
- **Aeris Swopper Chair**

### Casing
3D-printed with Prusa MK2  
[Thingyverse](https://www.thingiverse.com/thing:2209964)  
[Additional Tutorial](https://www.thingiverse.com/thing:2209964) for accu/battery integration  

### Wiring
1. Connect the MPU6050 IMU to the Feather 32u4 via I2C (SDA to SDA, SCL to SCL).
2. Connect the buzzer to a GPIO pin (e.g., GPIO6).
3. Connect the RGB LED to three GPIO pins (e.g., GPIO11 for red, GPIO10 for green, GPIO9 for blue).
4. Attach the MPU6050 IMU to the Aeris Swopper chair.

## Software Setup

### Libraries
Make sure you have the following libraries installed in your PlatformIO project:

```ini
lib_deps = 
    mbed-syundo0730/I2Cdev@0.0.0+sha.3aa973ebe3e5
    electroniccats/MPU6050@^1.3.1
    adafruit/Adafruit BluefruitLE nRF51@^1.0.0
