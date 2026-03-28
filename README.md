# Drone Landing Assist System

Sensor fusion prototype for safe autonomous drone landings. Combines ultrasonic altitude measurement with 24GHz radar human presence detection on an ESP32.

Built as a hardware subsystem for my [autonomous face-tracking drone](https://github.com/kryptonus/Drone-Projekt) (ROS2, OpenCV, Gazebo).

![System Overview](docs/images/full_setup.jpg)

## What it does

Two sensors answer two questions before a drone lands:

**HC-SR04 ultrasonic** — "How high am I?" Measures ground distance in the 2–400cm range using time-of-flight of a 40kHz sound pulse.

**LD2410C 24GHz FMCW radar** — "Is there a person below?" Detects human presence by sensing micro-movements (breathing, heartbeat) that static objects do not produce. Range up to 6 meters.

The ESP32 fuses both inputs into a landing decision and displays telemetry on an OLED screen and a WiFi web dashboard.

## Hardware

| Component | Model | Role |
|-----------|-------|------|
| MCU | ESP32 NodeMCU | Controller, WiFi host |
| Altitude | HC-SR04 | Ultrasonic distance |
| Presence | LD2410C | 24GHz radar |
| Display | SSD1315 128x64 | OLED telemetry |

### Circuit notes

The HC-SR04 echo pin outputs 5V. ESP32 GPIOs handle 3.3V max. A 1kΩ/2kΩ resistive voltage divider steps the signal down to 3.33V.

The LD2410C requires 5V minimum. It does not operate on 3.3V despite having a 3.3V logic-level output.

## Web Dashboard

The ESP32 creates a WiFi access point and serves a real-time dashboard at `192.168.4.1`. The browser fetches JSON sensor data every 200ms and renders live altitude graphs, radar status, and landing decisions.

![Dashboard](docs/images/dashboard.png)

## OLED Display

The onboard display shows altitude, a scrolling altitude graph, radar status, and the current landing decision. No laptop required.

![OLED](docs/images/oled.jpg)

## Landing Logic

```
altitude < 30cm AND no human  →  SAFE TO LAND
human detected                →  ABORT LANDING
altitude > 50cm               →  CRUISING
no echo                       →  NO SIGNAL
```

## Pin Mapping

```
ESP32 GPIO5   →  HC-SR04 TRIG
ESP32 GPIO18  →  HC-SR04 ECHO (via voltage divider)
ESP32 GPIO4   →  LD2410C OUT
ESP32 GPIO16  →  LD2410C TX
ESP32 GPIO21  →  SSD1315 SDA (I2C)
ESP32 GPIO22  →  SSD1315 SCL (I2C)
ESP32 U5      →  HC-SR04 VCC, LD2410C VCC (5V)
ESP32 3U3     →  SSD1315 VCC (3.3V via power rail)
```

## Project Structure

```
drone-landing-assist/
├── README.md
├── src/
│   └── landing_assist.ino
├── docs/
│   ├── technical_documentation.md
│   ├── coursework_connections.md
│   └── images/
│       ├── full_setup.jpg
│       ├── dashboard.png
│       ├── oled.jpg
│       ├── voltage_divider.jpg
│       └── wiring_closeup.jpg
└── schematics/
    └── wiring_diagram.md
```

## Related

This is the hardware component of a larger autonomous drone project:

**[Drone-Projekt](https://github.com/kryptonus/Drone-Projekt)** — Face-tracking drone in C++ with ROS2, OpenCV, and Gazebo. Three independent PID controllers (yaw, altitude, distance) with OOP architecture and smart pointer ownership semantics.

## Built with

ESP32 (Arduino framework), HC-SR04, LD2410C, SSD1315, HTML/CSS/JS for dashboard.

## Author

**Vaishnav Vinod** — B.Sc. Informationstechnik/Elektronik, TH Mannheim

[GitHub](https://github.com/kryptonus) · [LinkedIn](https://linkedin.com/in/vaishnavvinodde)
