# 🚀 2025 NURA Avionics - TEAM ASEC
![Language](https://img.shields.io/badge/Language-C++-blue.svg)
![Framework](https://img.shields.io/badge/Framework-Arduino-00979D)
![MCU](https://img.shields.io/badge/MCU-ESP32-E7352C)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-orange)

## Introduction

<div align="center">

<img src="images/Launch.gif" width="350" />

</div>

This repository contains the Avionics software developed by **ASEC Rocket Team, Konkuk University** for the **2025 NURA Competition**.

The objective of this year’s mission is **roll control using four canards**, demonstrating control performance by rotating the rocket **90° after launch and maintaining the roll angle stably**.

### System Goals
- Perform standard launch and recovery sequence  
- PD Based Canard Roll Control 

### Key Requirements
1. Maintain stable operation for **at least 30 minutes** after power-up  
2. Prevent parachute deployment during ground preparation via a **Safety Pin**  
3. Log all sensor data to SD card and transmit key telemetry to Ground Station  
4. Perform roll-axis control based on real-time attitude estimation  
5. Trigger parachute deployment when predefined conditions are met  
6. Ensure a minimum communication range of **400 m** with Ground Station  

### Features

* **Real-time roll control**: Reads IMU yaw angle and computes canard commands through a PD controller to counter roll motion.
* **FreeRTOS-based multitasking**: Handles sensing, control, logging, and communication concurrently with separate tasks of different priorities.
* **Multi-sensor processing**: Tracks flight state using IMU, barometer, and GPS data with a checksum-verified binary protocol.
* **Multi-condition deployment system**: Ensures parachute deployment based on altitude drop, attitude instability, or timeout conditions.
* **Robust data management**: Logs all flight data to the onboard SD card while transmitting telemetry via 447 MHz RF.
* **Efficient communication protocol**: Compresses and scales `float` values into compact binary packets to maximize transmission efficiency.

## Hardware Configuration

### Bill of Materials (BOM)

<div align="center">

| Main Category         | Sub Category         | Component / Model                  |
|-----------------------|----------------------|------------------------------------|
| **MCU & Core**        | MCU                  | Arduino Nano ESP32                 |
| **Sensors**           | IMU                  | EBIMU-9DOFV5-R3                    |
|                       | Barometric           | DFRobot Fermion BMP390L            |
|                       | GPS Module           | SparkFun GPS Breakout - NEO-M9N, SMA |
|                       | GPS Antenna          | AKA150                             |
| **RF & Communication**| RF Module            | NMT-UM434R1-C SEVB (G2)            |
|                       | RF Antenna           | NMT-SA434G2                        |
|                       | GCS Antenna          | YG-447                             |
| **Storage & Power**   | SD Socket            | SZH-EKBZ-005                       |
|                       | 3.3V LDO             | LM1117T-3.3/NOPB                   |
|                       | 5V LDO               | L4940V5                            |
| **Actuators**         | Canard Servo         | AFRC-D3519HB-S                     |
|                       | Parachute Servo      | S2300M                             |

</div>

## PCB Design

<div align="center">

  <img src="images/pcb1.jpg" width="250" />
  <img src="images/pcb2.jpg" width="250" />
  <img src="images/avionics_module.jpg" width="250" />

</div>

The PCB was designed using **KiCad 9.0**, with the entire avionics system split into **four boards** and assembled in a **stacked architecture** to maximize internal space efficiency.

Each PCB layer is interconnected via Molex connectors, using **Molex 5268 (90° bent)** on the PCB side and **Molex 5264** for inter-board wiring.

📂 PCB source files:  
[Google Drive - PCB Files](https://drive.google.com/file/d/1G7LwpJqrYb3B2x5KDOTYG1Fmn34BIEfY/view?usp=drive_link)

## Code Description

### 1. Code Structure


```
.
├── 2025nura.ino         # Main application and FreeRTOS task definitions 
├── EBIMU_AHRS.h/.cpp    # IMU driver  
├── BMP390L.h/.cpp       # Barometer driver (Wrapper)
├── bmp3.h/.c            # Barometer driver (Low-Level)
├── bmp3_defs.h          # Barometer definitions
├── ubx_gps.h/.cpp       # GNSS driver
├── ubx_config.h         # GNSS configuration
├── ejection.h/.cpp      # Parachute deployment logic
├── SDLogger.h/.cpp      # SD card logger
├── packet_2025.h/.cpp   # RF communication packet builder
├── NMT.h/.cpp           # RF module handler
└── baro_I2C.h/.cpp      # I2C communication helper
```

### 2. Software Architecture

The system adopts **FreeRTOS** for real-time multitasking and a **modular, object-oriented design** for maintainability and scalability.

* **Real-time multitasking**  
  - `FlightControl` (Priority 3): Roll control task operating above 100 Hz  
  - `ATTALT` (Priority 2): IMU + barometer sensing task at 100 Hz  
  - `Parachute` (Priority 2): Evaluates deployment conditions  
  - `SRG` (Priority 1): SD logging, RF transmission, and GPS reception at 20 Hz  

* **Modular class design**  
  Each hardware module and logic unit is abstracted as a C++ class for clarity and reusability.

* **Data flow**  
  Inter-task communication is handled via FreeRTOS **Queues** to prevent collisions and ensure deterministic transfer.

### 3. Core Components

**2025nura.ino**  
- Initializes hardware drivers and logic modules  
- Defines FreeRTOS tasks and launches the scheduler  

**Sensor Processing**  
- **`EBIMU_AHRS` (IMU)**: Reads real-time attitude via a checksum-verified binary UART protocol  
- **`BMP390L` (Barometric)**: Reads pressure/temperature via I2C and computes altitude  
- **`UbxGPS` (GPS)**: Receives position/velocity via I2C using **UBX-NAV-PVT**  

**Parachute Deployment & Servo Control**  
- **`ejection`**: Determines deployment based on altitude drop, attitude instability, or timeout  
- **`FlightControl`**: Computes canard angles through PD control based on yaw error  

**Data Logging & RF Communication**  
- **`SDFatLogger`**: Logs all flight data in CSV format  
- **`Packet`**: Encodes scaled binary packets with checksum  
- **`NMT`**: Configures and transmits via the 447 MHz RF module  

## Anticipated Improvements

The avionics software successfully fulfilled mission objectives in the 2025 competition, but several limitations were identified during testing. The following summarizes the issues and planned improvements.

### 1. RTOS Task Structure & Control Cycle

- Issues:  
  - Some tasks operated below the target frequency of **100 Hz**  
  - Combining IMU and barometer into a single task caused **barometer I/O delay to propagate into the IMU path**  
  - Handling SD logging and RF transmission in a single task reduced **RF transmission rate** and Ground Station update rate  

- Improvements:  
  - Separate and restructure the RTOS task graph  
  - Split sensor data paths and optimize data flow  

### 2. Quaternion-Based Parachute Deployment & Roll Control

- Issues:  
  - Deployment logic and roll control were implemented using **Euler angles**  
  - Near-singularity effects caused deployment to trigger **below the intended 70° threshold**, with cases observed near **50°**  
  - Reduced reliability of attitude-based decision-making and roll stabilization  

- Improvements:  
  - Transition tilt-angle estimation for deployment logic to **Quaternion-based computation**  
  - Migrate roll control to **Quaternion-based control** to eliminate Euler singularities  

## Contributors

### Team Members

**JunHyeong Kim — Smart Vehicle Engineering**  
Avionics Lead. IMU/GPS processing, RTOS, PID control, RF communication, SD logging, PCB design & hardware  

**RangHyeon Kim — Aerospace Engineering**  
Barometric sensing, parachute deployment, RF communication, SD logging  

**YongJin Kim — Electrical & Electronic Engineering**  
Ground Station visualization, RF packet optimization  
[Konkuk Univ. NURA GCS GitHub](https://github.com/kywls405/NURA2025_GCS)

**SeungMin Shin — Electrical & Electronic Engineering**  
RF communication, PCB design & hardware fabrication  

### Honorable Mention

**HyunBin Joo — Electrical & Electronic Engineering**  
RF hardware verification  
