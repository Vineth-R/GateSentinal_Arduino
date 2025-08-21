Gate Lock System – Arduino & IoT Project
Overview

This project is a smart gate lock system that combines ESP32 microcontrollers, biometric authentication, camera surveillance, and audio communication for enhanced security. The system is designed to provide reliable, real-time access control while being powered by a backup UPS battery for uninterrupted operation.

Features

Fingerprint Authentication – Secure access via R503 fingerprint sensor

Visual Display – Status and menu displayed on an OLED screen and a touchscreen Raspberry Pi interface

Motion Detection – PIR sensor triggers alerts and camera capture

Surveillance – ESP32-CAM module for real-time image/video monitoring

Audio Communication – Two-way audio with INMP441 microphone, PCM5102 DAC, and TPA3110 amplifier driving external speakers

Locking Mechanism – Solenoid lock controlled via a relay module

Safety & Feedback – Limit switch for gate position detection

Power Management – 12V UPS battery and regulated supply for continuous operation

Hardware Components

ESP32 Development Board

OLED Display

R503 Fingerprint Sensor

PIR Motion Sensor

ESP32-CAM Module

INMP441 Microphone

PCM5102 DAC Module

TPA3110 Audio Amplifier

Speakers

Touchscreen Raspberry Pi (for user interface)

Limit Switch

Solenoid Lock

Relay Module

UPS with 12V Battery and Power Supply

Software

Arduino sketches written in C/C++ for ESP32 boards

Control logic for authentication, camera streaming, audio processing, and lock actuation

Integration with Raspberry Pi interface for touchscreen control

How It Works

Authentication – The user places a finger on the R503 sensor. If matched, the system unlocks the gate.

Monitoring – PIR sensor detects movement, triggering the ESP32-CAM to capture footage.

Communication – Users can speak and listen through the integrated microphone, DAC, and amplifier.

Control – Relay activates the solenoid lock to open or close the gate.

Status Display – OLED and Raspberry Pi touchscreen show system status and control options.

Backup Power – UPS ensures uninterrupted operation during power outages.
