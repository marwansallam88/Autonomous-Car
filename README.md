Desciption:
The 1:18 scale car has to travel two predetermined routes as part of the project's goal. The first calls for a 10-meter straight-line movement of the vehicle which was an unexpectedly challenging task. On the alternate road, two obstacles had to be avoided by lane switching.

Before moving on to the hardware phase, we employed a simulation phase to refine our system. We carefully verified our nodes and communication protocols using the Gazebo platform on Ubuntu. This simulation phase allowed us to adjust our system, spot potential problems, and optimize performance.

Moving on to the hardware phase, we implemented our steering mechanism using a servo motor to help control the car better. Then, we successfully created a connection between the RP2040 microcontroller and a Raspberry Pi. By sending control inputs from the Raspberry Pi to the RP2040 via serial communication, we successfully controlled the motors. This made it possible to control the vehicle's motions precisely. Sensory data (speed and orientation) was collected from the Arduino via an Infrared sensor and an integrated gyroscope and sent to the Pi while the Pi sent motor control inputs (steering and speed) to the Arduino. When dealing with the hardware, multiple testing had to be done to tune the PID and control parameters of the system and this was due to the disturbances which were not present in the simulation phase.

Key aspects:
Developed obstacle-avoidance logic using ROS and Python in a simulated self-driving car.
Integrated a Hall effect sensor on a toy car to measure real-time speed, providing feedback for precise speed control and improved navigation accuracy.
Implemented PID control and behavior planning for reliable and smooth autonomous navigation.
