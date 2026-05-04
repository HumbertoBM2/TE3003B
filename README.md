# Autonomous Forklift System – Team 7

## Overview
This project presents the design and implementation of an autonomous forklift system built on a Puzzlebot platform. The system integrates embedded systems, robotics, computer vision, and control theory to enable fully autonomous pallet detection, transportation, and unloading within a mapped environment.

The platform is capable of autonomous navigation, object localization, pallet manipulation, and human interaction, forming a complete end-to-end robotic solution.

---

## Team Members – Team 7

- Humberto Barrera – (GitHub: [https://github.com/HumbertoBM2])
- Erick Campos – (GitHub: [https://github.com/Erick-CamposA01247257])
- María José Pardo – (GitHub: [https://github.com/mariajosepardoc18])
- Mauricio Zavala – (GitHub: [https://github.com/mzzzavalas])

---

## System Description

The autonomous forklift operates by combining SLAM-based navigation, probabilistic localization, and computer vision techniques to locate, transport, and deliver pallets within a structured environment.

### Core Capabilities
- Autonomous mapping and navigation using SLAM
- Pallet detection using ArUco markers and vision systems
- Probabilistic localization using Monte Carlo methods
- Sensor fusion via Kalman filtering
- Closed-loop motion control using PID controllers
- Autonomous pallet lifting using FPGA-based control
- Voice command interface using vector quantization
- Remote development and monitoring via onboard hotspot

---

## Hardware Architecture

The system is composed of the following hardware components:

- Puzzlebot mobile base
- Jetson Nano 2GB (main processing unit)
- Hackerboard (low-level control and interfacing)
- Tang Nano 20K FPGA (forklift actuation control)
- DC motors with encoders (locomotion and feedback)
- LiDAR sensor (environment perception and mapping)
- Raspberry Pi Camera (ArUco marker detection)
- WiFi antenna / hotspot module (remote connectivity)
- 20,000 mAh battery (power supply)

---

## Software Architecture

The system integrates multiple algorithms and frameworks:

### Navigation and Localization
- Simultaneous Localization and Mapping (SLAM)
- Monte Carlo Localization (Particle Filter)
- Kalman Filtering for sensor fusion

### Control Systems
- PID-based motion control for trajectory tracking
- Encoder feedback for velocity and position estimation

### Perception
- ArUco marker detection for pallet identification
- LiDAR-based obstacle detection and mapping

### Embedded and FPGA Control
- FPGA-based control logic for pallet lifting mechanism
- Real-time coordination between processing units

### Human Interaction
- Voice recognition system based on vector quantization
- Command-based interaction for task execution

---

## System Workflow

1. The robot initializes and builds or loads a map using SLAM  
2. The system performs localization using Monte Carlo methods  
3. Pallets are detected using ArUco markers via the camera  
4. The robot navigates toward the target using PID-controlled motion  
5. The FPGA subsystem activates the forklift mechanism to lift the pallet  
6. The robot plans a path toward the unloading zone (e.g., truck)  
7. The pallet is delivered and unloaded autonomously  
8. Optional voice commands can trigger or modify behaviors  

---

## Connectivity and Development

- Onboard hotspot enables remote access and development  
- Supports real-time monitoring and debugging  
- Modular architecture allows integration of additional subsystems  

---

## Project Scope

This project demonstrates the integration of:
- Robotics and autonomous navigation  
- Embedded systems and real-time control  
- FPGA-based hardware acceleration  
- Computer vision and perception systems  
- Human-machine interaction  

---

## Future Work

- Advanced path planning (e.g., dynamic obstacle avoidance)
- Improved voice recognition models
- Multi-robot coordination
- Enhanced precision in pallet manipulation

---

## Conclusion

The Autonomous Forklift System showcases a robust and scalable approach to industrial automation using modern robotics technologies. By combining perception, control, and hardware acceleration, the system achieves reliable autonomous operation in complex environments.
