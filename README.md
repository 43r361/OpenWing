# OpenWing: DIY Quadcopter Project

![OpenWing Logo](logo.png)

OpenWing is a custom-built quadcopter designed and developed by Alexander Shestakov, Alexander Malinov, and Lora Apostolova as a VMKS assignment in TUES and TUES Fest 2025 project. This project showcases a quadcopter with a custom flight controller based on the ESP32 microcontroller, integrating various sensors and control systems for stable flight and precise maneuverability.

## Features

- **Custom Flight Controller**: Built using the ESP32, the flight controller handles sensor data processing, stabilization, and control.
- **Sensor Integration**:
  - MPU6050 for gyroscope and accelerometer data.
  - BMP085 for barometric pressure and altitude measurement.
  - HMC5883L for compass functionality.
- **Joystick Control**: A joystick interface for manual control of the quadcopter.
- **Kalman Filter**: Implements a Kalman filter for precise angle estimation and stabilization.
- **Testing Visualization**: A Python-based OpenGL application used for testing the quadcopter's orientation.

## Components

### Hardware

- **ESP32**: The core microcontroller for the flight controller.
- **MPU6050**: Gyroscope and accelerometer for motion sensing.
- **BMP085**: Barometric pressure sensor for altitude measurement.
- **HMC5883L**: Magnetometer for compass functionality.
- **Joysticks**: For manual control of the quadcopter.

### Software

- **Arduino Code**: Experimental snippets for sensor data processing and joystick control.
- **Python Testing Tool**: A Python script using OpenGL and Pygame for testing orientation data (code taken from [here](https://github.com/mattzzw/Arduino-mpu6050)).

## Acknowledgments

This project was made possible through the collaboration of Alexander Shestakov, Alexander Malinov, and Lora Apostolova. Special thanks to our school and mentors for their guidance and support.
