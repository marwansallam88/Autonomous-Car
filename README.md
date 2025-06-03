# Autonomous Car

## Description:
This project develops a complete autonomous ground vehicle system using ROS (Robot Operating System) for both simulation and hardware implementation. The system involves modifying a scaled RC car with Ackermann steering to autonomously navigate a racing track at constant speed while avoiding obstacles through lane-changing maneuvers. The project integrates four key autonomous system modules: localization, planning, control, and communication, implemented across simulation environments (Gazebo) and real hardware platforms (Raspberry Pi and Arduino).

## Key Aspects:
- Implemented autonomous vehicle system using ROS Noetic/Melodic with dual simulation (Ubuntu 20.04) and hardware (Raspberry Pi) deployment.
- Developed integrated control architecture with speed control and lateral control algorithms for lane-changing capabilities.
- Created planning module for velocity profiling and obstacle-free path planning with cooperative vehicle behavior.
- Implemented Kalman Filter-based localization using IMU and encoder sensors for accurate state estimation.
- Designed inter-processor communication protocol (mainly serial communication) between Arduino (actuator control) and Raspberry Pi (high-level processing).
- Built complete hardware system including PCB design, sensor integration, and mechanical modifications to 1:14-1:18 scale RC cars.
- Tested system performance across multiple map environments including racing tracks and obstacle avoidance scenarios.
