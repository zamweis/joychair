
# Joychair Interface for VR Navigation - ESP32

Welcome to the Joychair Interface project for ESP32! This setup enables you to integrate the dynamic movements of an Aeris Swopper chair into virtual reality (VR) environments using a SparkFun ESP32 Thing board. The ESP32 communicates data wirelessly via Bluetooth, providing seamless interaction within VR applications.

## Project Overview

The Joychair Interface captures and translates the movements of the Aeris Swopper chair into VR environments, particularly in Unity. The setup uses an ESP32 microcontroller to read data from an Inertial Measurement Unit (IMU) and transmit it wirelessly. Additionally, the system includes a battery management solution, featuring a power switch and a fuel gauge for real-time battery monitoring.

## Hardware Setup

### Components
- **ESP32 (SparkFun ESP32 Thing)**
- **MPU6050 IMU (Inertial Measurement Unit)**
- **Buzzer**
- **RGB LED**
- **Battery with Power Switch**
- **MAX1704x Fuel Gauge**
- **Aeris Swopper Chair**

### Wiring
1. **MPU6050 IMU to ESP32:**
   - Connect via I2C (SDA to GPIO4, SCL to GPIO17).
   - Attach the IMU securely to the Aeris Swopper chair.
2. **Buzzer to ESP32:**
   - Connect the buzzer to a GPIO pin (e.g., GPIO6).
3. **RGB LED to ESP32:**
   - Connect the RGB LED to three GPIO pins (e.g., GPIO11 for red, GPIO10 for green, GPIO9 for blue).
4. **Battery Setup:**
   - Connect the battery to the power switch, and then connect the power switch to the MAX1704x fuel gauge.
   - Connect the fuel gauge to the ESP32 to monitor the battery level.
5. **Power and Ground Connections:**
   - Ensure all components share a common ground and are powered appropriately by the battery.

### Casing
The components should be housed securely to protect them from damage during use. You can 3D print the casing using the provided STL files.

### Spatial Light Transfer Files
You can find the STL files for the casing in the [case](./case) directory:
- [Case.stl](./case/case.STL)
- [Case_Lid.stl](./case/case_lid.STL)
- [Push_Button.stl](./case/push_button.STL)

## Software Setup

### Libraries
Ensure the following libraries are installed in your PlatformIO project:

```ini
lib_deps = 
    lemmingdev/ESP32-BLE-Gamepad@^0.5.5
    mbed-syundo0730/I2Cdev@0.0.0+sha.3aa973ebe3e5
    electroniccats/MPU6050@^1.3.1
    https://github.com/sparkfun/SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.git
```

## Functionality

### Battery Management
The setup includes a MAX1704x fuel gauge that monitors the battery level, providing real-time updates to the ESP32. This ensures the system can alert the user if the battery is running low, and manage power efficiently.

### LED Indicators
The RGB LED provides visual feedback on the system’s status:
- **Red Blinking:** An error has occurred.
- **Green Blinking:** Initialization is in progress.
- **Blue Blinking:** Bluetooth device is not connected.
- **Green Static:** Initialization completed successfully.
- **Pink Static:** System is operating normally.
- **Red/Yellow/Green (Static/Blinking):** Indicates battery level, where red indicates low battery, yellow indicates medium, and green indicates high.

### Code Overview
The provided code turns the ESP32 into a Bluetooth LE gamepad, which reads IMU data and transmits it over Bluetooth. It also manages power based on battery levels and provides visual and auditory feedback via the LED and buzzer.

## Usage Instructions
1. **Setup the hardware** according to the wiring instructions.
2. **Flash the code** to the ESP32 using PlatformIO.
3. **Power on the system** using the power switch connected to the battery.
4. **Monitor the LED indicators** to ensure everything is functioning correctly.
5. **Use the Joychair** to navigate in your VR environment, with movements of the Aeris Swopper chair translated directly into VR motion.

## Troubleshooting
- If the LED blinks red, check the connections and ensure all components are properly wired and powered.
- If the battery level is low, charge or replace the battery to ensure continuous operation.
- If the MPU6050 fails to initialize, verify the I2C connections and ensure the IMU is correctly attached to the ESP32.

## Conclusion
The Joychair Interface brings a new dimension to VR navigation, making interactions more immersive and intuitive. With proper setup and usage, this system can greatly enhance your VR experiences.
