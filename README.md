# ChairIO Interface for VR Navigation

Welcome to the ChairIO Interface project! This project aims to integrate the movements of an Aeris Swopper chair into virtual reality (VR) environments, allowing users to navigate virtual worlds by shifting their weight on the chair. The idea behind this project is inspired by the concept of ChairIO, as described in the paper ["ChairIO - the Chair-Based Interface"](https://www.researchgate.net/publication/233819716_ChairIO--the_Chair-Based_Interface).

## Project Overview

The ChairIO Interface project is being developed at the iXperience Lab of the Faculty of Computer Science and Business Information Systems (Fakultät IWI). The goal is to capture the movements of the Aeris Swopper chair and integrate them into a middleware solution that can be used within VR environments, particularly in Unity.

## Project Components

### Hardware Setup

The project utilizes an Arduino Leonardo microcontroller board along with sensors, such as an Inertial Measurement Unit (IMU), attached to the chair. The IMU captures the chair's movements, which are then transmitted wirelessly to the middleware.

### Middleware

The middleware serves as the bridge between the hardware (Arduino) and the VR environment (Unity). It receives the data from the sensors and translates it into usable inputs for VR navigation.

### Unity Integration

In Unity, the middleware's output is used to control the user's movement within the VR environment. By shifting their weight on the chair, users can navigate through virtual worlds seamlessly.

## Setup Instructions

To set up the ChairIO Interface project, follow these steps:

1. **Hardware Setup**: Attach the IMU sensor to the Aeris Swopper chair as per the instructions provided.
2. **Arduino Configuration**: Program the Arduino Leonardo board to read data from the IMU and transmit it wirelessly to the middleware.
3. **Middleware Setup**: Set up the middleware software (NodeJS-based) to receive data from the Arduino and process it for Unity integration.
4. **Unity Integration**: Import the middleware's output into Unity and configure the VR environment to respond to the chair's movements for navigation.

## Additional Resources

- [Aeris Swopper Product Page](https://en.aeris.de/products/aeris-swopper-wollmischung-capture-gruen): Learn more about the Aeris Swopper chair.
- [Arduino Joystick Tutorial](https://www.instructables.com/Create-a-Joystick-Using-the-Arduino-Joystick-Libra/): Useful guide for setting up the Arduino Leonardo as a joystick.

## Contributors

- Sam Weiler (@zamweis): Project Lead

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

Feel free to reach out to the project team for any questions or feedback. Happy navigating in VR with ChairIO Interface! 🎮🪑🌐
