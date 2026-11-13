# Self-Balancing-Robot
Arduino-based two-wheel self-balancing robot using PID control and MPU6050 sensor fusion
This project implements a closed-loop feedback control system to stabilize a two-wheeled robotic platform based on the inverted pendulum principle. It was developed as part of the Mechanical Measurements and Mechatronics Laboratory (ME22201) at the Indian Institute of Technology Kharagpur.
The system utilizes continuous orientation tracking and a tuned PID (Proportional-Integral-Derivative) algorithm to dynamically adjust motor outputs and maintain vertical equilibrium.
# Hardware Architecture
The electrical system integrates real-time sensing with mechanical actuation to counteract chassis inclination.
Microcontroller: Arduino Uno serves as the central processing unit.
IMU Sensor: MPU6050 (6-axis Accelerometer and Gyroscope) communicates real-time spatial data via the I2C interface.   
Motor Driver: L298N Motor Driver Shield regulates the speed and direction of the actuation system.
Actuators: Dual Geared DC Motors operating independently on opposite sides of the chassis for speed and directional control.
Power Supply: Rechargeable dual Lithium-ion battery pack (3.7V).
# Control System & PID Implementation
The control architecture continuously calculates the error between the measured instantaneous tilt angle and a predefined vertical reference (nominally $0^{\circ}$). 
The error is processed through a PID controller with the following tuned parameters:
Proportional Gain ($K_p$) = 15: Provides a corrective response proportional to the immediate tilt error.
Derivative Gain ($K_d$) = 3.5: Anticipates the rate of change of the error, acting as a shock absorber to reduce oscillation and improve system stability.
Integral Gain ($K_i$) = 0: Disabled for this iteration to prevent integral windup.   
# Safety Mechanisms
Tilt Kill-Switch: An automated software fail-safe shuts down motor operation entirely if the chassis inclination exceeds a non-recoverable 30-degree threshold.
Minimum Absolute Speed: Motor movement requests are bounded by a minimum speed threshold (MIN_ABS_SPEED = 15) to allow for gentle micro-corrections near the center reference point.
# Dependencies
This project requires the following Arduino libraries to compile:
1. PID_v1.h
2. LMotorController.h
3. I2Cdev.h
4. MPU6050_6Axis_MotionApps20.h
5. Wire.h
