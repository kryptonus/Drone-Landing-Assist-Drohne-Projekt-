# 🛬 Drone Landing Assist System

![ESP32](https://img.shields.io/badge/ESP32-NodeMCU-blue?style=flat-square&logo=espressif)
![HC-SR04](https://img.shields.io/badge/Ultrasonic-HC--SR04-green?style=flat-square)
![LD2410C](https://img.shields.io/badge/Radar-24GHz%20LD2410C-red?style=flat-square)
![OLED](https://img.shields.io/badge/Display-SSD1315%20OLED-yellow?style=flat-square)
![C++](https://img.shields.io/badge/C++-Arduino-00599C?style=flat-square&logo=cplusplus)
![WiFi](https://img.shields.io/badge/WiFi-Dashboard-purple?style=flat-square)

> Sensor fusion prototype for safe autonomous drone landings. Answers two questions before touchdown: **"How high am I?"** and **"Is there a person below?"**

Built as the hardware subsystem for my [autonomous face-tracking drone](https://github.com/kryptonus/Drone-Projekt) (C++, ROS2, OpenCV, Gazebo).

---

## 🎯 What It Does

| Feature | Sensor | How It Works |
|---------|--------|-------------|
| **Altitude Measurement** | HC-SR04 Ultrasonic | Sends 40kHz sound pulse, measures echo return time. Range: 2–400cm |
| **Human Presence Detection** | LD2410C 24GHz FMCW Radar | Detects micro-movements (breathing, heartbeat) to distinguish humans from objects. Range: up to 6m |
| **Onboard Telemetry** | SSD1315 OLED 128x64 | Real-time altitude, scrolling graph, radar status, landing decision |
| **Remote Dashboard** | ESP32 WiFi AP | Browser-based live dashboard with altitude charts and sensor fusion display |

## ⚡ Landing Decision Logic

```
altitude < 30cm  +  no human   →  ✅ SAFE TO LAND
human detected                  →  🚫 ABORT LANDING
altitude > 50cm                 →  🔵 CRUISING
no echo signal                  →  ⚠️  NO SIGNAL
```

## 🔧 Hardware Setup

### Components

| Component | Model | Interface | Voltage |
|-----------|-------|-----------|---------|
| Microcontroller | ESP32 NodeMCU (BerryBase) | USB | 5V via USB |
| Altitude Sensor | HC-SR04 | GPIO (trigger/echo) | 5V |
| Presence Sensor | LD2410C | GPIO + UART | 5V |
| Display | SSD1315 128x64 OLED | I2C | 3.3V |

### Pin Mapping

```
ESP32 GPIO5   →  HC-SR04 TRIG
ESP32 GPIO18  →  HC-SR04 ECHO (via 1kΩ/2kΩ voltage divider)
ESP32 GPIO4   →  LD2410C OUT
ESP32 GPIO16  →  LD2410C TX
ESP32 GPIO21  →  SSD1315 SDA (I2C)
ESP32 GPIO22  →  SSD1315 SCL (I2C)
ESP32 U5      →  HC-SR04 VCC, LD2410C VCC (5V)
ESP32 3U3     →  SSD1315 VCC (3.3V via power rail)
```

### ⚠️ Voltage Divider (Level Shifting)

The HC-SR04 echo pin outputs **5V**. ESP32 GPIOs handle **3.3V max**. A resistive voltage divider protects the microcontroller:

```
ECHO (5V) ──── 1kΩ ──── Junction ──── 2kΩ ──── GND
                            │
                        ESP32 GPIO18
                        (3.33V safe)
```

`V_out = 5V × 2000/(1000+2000) = 3.33V`

## 🌐 Web Dashboard

The ESP32 creates a WiFi access point (`DroneAssist`) and serves a real-time dashboard at `192.168.4.1`. The browser fetches JSON sensor data every 200ms and renders live altitude graphs, radar status, and landing decisions.

**Features:**
- Live altitude graph with history
- Min / Avg / Max statistics
- Pulsing radar detection indicator
- Color-coded landing decision status
- Responsive design (works on phone and laptop)

## 📂 Project Structure

```
drone-landing-assist/
├── README.md
├── src/
│   └── landing_assist.ino          # Complete ESP32 firmware
├── docs/
│   ├── technical_documentation.md  # Full project writeup
│   ├── coursework_connections.md   # Links to SS & EEL theory
│   └── images/
│       ├── full_setup.jpg
│       ├── dashboard.png
│       ├── oled.jpg
│       └── voltage_divider.jpg
└── schematics/
    └── wiring_diagram.md
```

## 🚀 Quick Start

1. Wire the components as per the pin mapping above
2. Open `src/landing_assist.ino` in Arduino IDE
3. Select **ESP32 Dev Module** and your port
4. Upload (hold BOOT button during "Connecting...")
5. Connect to WiFi: `DroneAssist` / Password: `12345678`
6. Open `http://192.168.4.1` in your browser

**Dependencies:** Install via Arduino Library Manager:
- `Adafruit SSD1306`
- `Adafruit GFX Library`

## 🔗 Related Projects

| Project | Description |
|---------|-------------|
| [Drone-Projekt](https://github.com/kryptonus/Drone-Projekt) | Autonomous face-tracking drone in C++ with ROS2, OpenCV, and Gazebo. Three PID controllers with OOP architecture and smart pointer ownership semantics |
| [CircuitSense](https://github.com/kryptonus/CircuitSense) | AI electronics lab assistant using Gemini Live API for real-time voice/vision circuit analysis |
| [PID-Controller-Cpp](https://github.com/kryptonus/PID-Controller-Cpp) | Standalone PID controller library in C++ |

## 📚 Documentation

- [Technical Documentation](docs/technical_documentation.md) — Full project writeup with circuit design, sensor theory, software architecture
- [Coursework Connections](docs/coursework_connections.md) — How this project maps to Signale und Systeme (SS) and EEL coursework

## 👤 Author

**Vaishnav Vinod** — B.Sc. Informationstechnik / Elektronik, TH Mannheim

[![GitHub](https://img.shields.io/badge/GitHub-kryptonus-181717?style=flat-square&logo=github)](https://github.com/kryptonus)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-vaishnavvinodde-0A66C2?style=flat-square&logo=linkedin)](https://linkedin.com/in/vaishnavvinodde)
