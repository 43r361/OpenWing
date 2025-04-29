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

## License

<p xmlns:cc="http://creativecommons.org/ns#" xmlns:dct="http://purl.org/dc/terms/"><a property="dct:title" rel="cc:attributionURL" href="https://github.com/43r361/OpenWing">OpenWing</a> by <a rel="cc:attributionURL dct:creator" property="cc:attributionName" href="https://github.com/43r361">Alexander Shestakov, Alexander Malinov and Lora Apostolova</a> is licensed under <a href="https://creativecommons.org/licenses/by-nc-sa/4.0/?ref=chooser-v1" target="_blank" rel="license noopener noreferrer" style="display:inline-block;">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International<img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/cc.svg?ref=chooser-v1" alt=""><img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/by.svg?ref=chooser-v1" alt=""><img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/nc.svg?ref=chooser-v1" alt=""><img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/sa.svg?ref=chooser-v1" alt=""></a></p>
