# Self_Balancing_Bike_using_Omniwheel
A two-wheeled self-balancing robotic system designed to maintain stability while enabling omnidirectional movement using an omni-wheel mechanism. The system uses real-time feedback from an MPU6050 IMU sensor and a PID-based closed-loop control algorithm to dynamically adjust motor response and maintain balance.

## Features

* Real-time tilt angle measurement using MPU6050 accelerometer and gyroscope data
* PID-based feedback control for automatic balance correction
* Arduino C/C++ firmware for sensor processing and motor control
* PWM-based motor actuation for smooth speed and direction control
* Omni-wheel mechanism for improved maneuverability

## Technologies Used

* Arduino Nano
* MPU6050 IMU
* PID Control Algorithm
* Embedded C/C++ (Arduino)
* Motor Driver
* DC Motors

  ## Working Flow
  MPU6050 IMU > Angle Calculation > PID Controller > PWM Motor Control > Balance Correction
