# Automotive-Smart-Safety-System

This project presents an embedded automotive safety system that integrates two critical functionalities: **Intelligent Car Door Locking** and **Rear Parking Assistance**. The system is implemented on the **TM4C123GH6PM** microcontroller and utilizes **FreeRTOS** to manage concurrent tasks in real time.

## Overview

The project simulates key automotive safety features within an embedded environment, emphasizing modular design, real-time responsiveness, and efficient resource management. It demonstrates how FreeRTOS can be employed to coordinate tasks such as sensor data acquisition, actuator control, and user feedback across multiple hardware interfaces.

## Features

### 1. Intelligent Door Locking System
- **Speed-Based Locking:** A potentiometer is used to simulate vehicle motion and estimate speed. When the estimated speed exceeds a defined threshold (e.g., 10 km/h), the system automatically locks all doors.
- **Ignition Control:** When the simulated ignition switch is turned off, the system automatically unlocks the doors.
- **Manual Control:** Two dedicated pushbuttons allow the user to manually lock or unlock the doors at any time, overriding automatic behavior.
- **Safety Alert:** If the driver door is open while the vehicle is in motion, an audible warning is activated and a message is displayed on the LCD.

### 2. Rear Parking Assistance System
- **Obstacle Detection:** An HC-SR04 ultrasonic sensor continuously measures the distance to objects behind the vehicle when in reverse gear.
- **Proximity Alerts:**
  - An audible buzzer increases its beeping frequency as the vehicle approaches an obstacle.
  - An RGB LED provides visual feedback based on proximity:
    - **Green:** Safe zone (distance > 100 cm)
    - **Yellow:** Caution zone (30–100 cm)
    - **Red:** Danger zone (< 30 cm)
- **LCD Feedback:** The measured distance is displayed on the LCD in real time.

## System Architecture

- **Microcontroller:** TM4C123GH6PM (Tiva C Series)
- **RTOS:** FreeRTOS for deterministic task scheduling
- **Sensor Interfaces:** Potentiometer (analog), HC-SR04 ultrasonic (digital)
- **User Interfaces:** LCD display, RGB LED, buzzer, pushbuttons, and toggle switches

## Software Design

The system is structured around FreeRTOS tasks that manage the following functions:
- Vehicle motion detection and speed estimation
- Door locking/unlocking logic
- Ultrasonic distance measurement
- User interface updates (LCD, LED, buzzer)
- Event handling based on gear state and door status

### Inter-task Communication
- **Queues:** Used for passing sensor data between producer and consumer tasks.
- **Semaphores/Mutexes:** Employed to ensure safe access to shared peripherals (e.g., LCD, buzzer).


![Alt Text](D:\ahmed\RTOS\rtos image.jpg)
