# 🤖 ESP32 Self-Balancing Robot

A two-wheeled inverted pendulum robot based on **ESP32**, utilizing **PID control** and **Sensor Fusion** for real-time stability.
Designed and built as a personal engineering project.

![Robot Demo](my_robot.gif)

## ⚡ Key Features
* **Real-time Control:** Implements a PID algorithm running at ~200Hz loop time.
* **Sensor Fusion:** Uses a **Complementary Filter** to fuse data from the Accelerometer and Gyroscope (MPU6050).
* **Safety Mechanisms:** Auto-shutoff when falling (>35° tilt) and battery voltage monitoring.
* **Custom Hardware Design:** Designed schematic and component libraries using **OrCAD Capture CIS**.

## 🛠️ Hardware Specifications
| Component | Description |
| :--- | :--- |
| **Microcontroller** | ESP32 DevKit V1 (Dual Core, 240MHz) |
| **IMU Sensor** | MPU6050 (6-DOF Gyro + Accel) |
| **Motor Driver** | L298N Dual H-Bridge |
| **Motors** | 12V DC Gear Motors with Encoders |
| **Power** | 2x 18650 Li-Ion Batteries (7.4V - 8.4V) |

## 🔋 Battery Monitoring
Designed a voltage divider to monitor the battery via ESP32 ADC:
* **Divider:** 33kΩ (Top) / 10kΩ (Bottom).
* **Logic:** LED indication and Buzzer warnings when voltage drops below 5.6V.

## 🚀 Technical Challenges
**Issue:** "Integral Windup" caused instability when recovering from a tilt.
**Solution:** Tuned the PID controller by clamping the Integral (Ki) term and implementing a conditional integration zone (only integrating when error is small).

## ⚙️ Mechanical Design
The robot's chassis was designed from scratch to ensure structural stability and modularity.
* **Design Software:** FreeCAD (`.FCStd` source files included).
* **Manufacturing:** 3D printed using **PETG** on a **Prusa i3 MK3**.
* **Files:** All STL files are available in the `3D_models` folder for replication.
---
*Created by sharon yerukhimovich - Electrical Engineering Student*
