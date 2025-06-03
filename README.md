# Autonomous Car

## Description:
A 1:18 scale autonomous car developed to navigate two distinct routes: a straight 10-meter drive and an obstacle-avoidance path requiring lane switching. The project progressed from simulation to hardware integration, applying principles of robotics, control systems, and embedded development.

In the simulation phase, we used ROS with Gazebo on Ubuntu to test and validate communication protocols and control logic. This allowed us to fine-tune the system before hardware deployment.

In the hardware phase, steering was controlled via a servo motor, with motion commands sent from a Raspberry Pi to an RP2040 microcontroller over serial communication. Sensor data from an Arduino (IR sensor and gyroscope) was used for feedback. The system underwent PID tuning to handle real-world disturbances not present in the simulation environment.

## Key Aspects:
- Developed obstacle-avoidance logic using ROS and Python in a simulated self-driving car environment.
- Integrated a Hall effect sensor to measure real-time speed, enhancing control accuracy.
- Implemented PID control and behavior planning for smooth and reliable autonomous navigation.
