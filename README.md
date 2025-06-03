# Autonomous Car

## Description  
This project implements a complete autonomous driving system for a 1:18 scale RC car using ROS (Robot Operating System), relying solely on hall-effect sensors for all perception tasks. The system features Ackermann steering modification and integrates localization, planning, and control modules to enable basic autonomous navigation on predefined tracks without additional sensors.

## Key Aspects  
- **Minimalist Sensor Approach**: Uses only hall-effect sensors (no IMU, cameras, or lidar) for speed measurement and odometry.  
- **Hardware Modifications**:  
  - Custom Ackermann steering system  
  - Hall-effect sensor mounted on front wheel for speed feedback  
  - Raspberry Pi (ROS master) + Arduino (actuator control) architecture  
- **Core Autonomy Functions**:  
  - Speed control via PWM motor output  
  - Basic path tracking using pre-mapped waypoints  
  - Emergency stop functionality via wheel speed monitoring  

## System Architecture  
1. **Perception**  
   - Hall-effect sensor for:  
     - Instantaneous speed calculation (pulse counting)  
     - Crude odometry (distance traveled)  

2. **Control**  
   - PID speed controller maintains constant velocity  
   - Open-loop steering control for predefined paths  

3. **Hardware Interface**  
   - Arduino: Reads hall sensor pulses → calculates RPM → streams to Pi  
   - Raspberry Pi: Runs ROS nodes for decision making  

## Performance Characteristics  
- **Speed Accuracy**: ±0.1 m/s at 1.5m/s max speed  
- **Odometry Drift**: ~5% error per 10m traveled  
- **Compute Latency**: <50ms sensor-to-actuation delay  

## Limitations & Workarounds  
- **No Obstacle Detection**: Requires pre-mapped, obstacle-free tracks  
- **No Absolute Positioning**: Relies on wheel odometry only (drift accumulates)  
- **Simplified Steering**: Pre-programmed turns without dynamic correction  

## Development Tools  
- **Software**: ROS Melodic, Python (RPi), Arduino C++  
- **Hardware**:  
  - A3144 Hall-effect sensors  
  - Neodymium magnets for wheel pulse generation  
  - 3D-printed sensor mounts  

*Demonstrates minimalist approach to autonomous systems using only essential sensors.* 
*Developed by Ibrahim Elsahhar, Hana Ahmed, Marwan Sallam, Farida Moubarak, and Amr Abd El-Latif at the German University in Cairo (GUC).* 
