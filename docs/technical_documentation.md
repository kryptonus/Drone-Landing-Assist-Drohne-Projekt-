&nbsp;

&nbsp;

&nbsp;

# Drone Landing Assist System

### A Sensor Fusion Approach to Safe Autonomous Drone Landing

&nbsp;

&nbsp;

**Vaishnav Vinod**

B.Sc. Informationstechnik / Elektronik

Technische Hochschule Mannheim

Fakultät für Informationstechnik

&nbsp;

March 2026

&nbsp;

&nbsp;

---

&nbsp;

## 1. Abstract

This document describes the design, implementation, and testing of a Drone Landing Assist System — a sensor fusion prototype that combines ultrasonic distance measurement with 24GHz radar human presence detection to enable safe autonomous drone landings. The system runs on an ESP32 microcontroller and provides real-time telemetry through both an onboard OLED display and a WiFi-hosted web dashboard.

The project is a hardware subsystem prototype for a larger autonomous face-tracking drone (built in ROS2/Gazebo with C++ and OpenCV). It demonstrates embedded systems engineering, analog circuit design (voltage divider for level shifting), multi-sensor integration, and real-time web-based data visualization.

---

## 2. Problem Statement

An autonomous drone must answer two critical questions before landing:

1. **"How high am I?"** — The drone needs precise altitude measurement during the final approach. GPS is too imprecise (±2m accuracy), and barometric altimeters drift. Ultrasonic sensors provide centimeter-level accuracy in the 2–400cm range, which is exactly the landing approach window.

2. **"Is there a person below me?"** — A spinning quadcopter descending onto a person is a safety hazard. Simple distance sensors cannot distinguish between a person and a landing pad. A 24GHz FMCW radar can detect human presence by sensing micro-movements (breathing, heartbeat) that static objects do not produce.

Neither sensor alone provides a complete landing solution. Sensor fusion — combining altitude data with presence detection — enables a reliable automated landing decision.

---

## 3. System Architecture

### 3.1 Hardware Components

| Component | Model | Function | Interface | Voltage |
|-----------|-------|----------|-----------|---------|
| Microcontroller | ESP32 NodeMCU (BerryBase) | Central controller, WiFi host | USB for programming | 5V via USB |
| Ultrasonic Sensor | HC-SR04 | Distance measurement (2–400cm) | GPIO (trigger/echo) | 5V |
| Radar Sensor | LD2410C | Human presence detection (0–6m) | GPIO (OUT pin) + UART | 5V |
| OLED Display | SSD1315 (128x64) | Onboard telemetry display | I2C (SDA/SCL) | 3.3V |

### 3.2 Pin Mapping

| ESP32 Pin | Connected To | Function |
|-----------|-------------|----------|
| U5 (5V) | HC-SR04 VCC, LD2410C VCC | 5V power supply |
| 3U3 (3.3V) | Power rail → OLED VCC | 3.3V power supply |
| GND | All component GNDs | Common ground |
| GPIO 5 | HC-SR04 TRIG | Trigger pulse output |
| GPIO 18 | Voltage divider junction | Echo signal input (3.3V safe) |
| GPIO 4 | LD2410C OUT | Presence detection input |
| GPIO 16 | LD2410C TX | UART data (reserved for future use) |
| GPIO 21 | SSD1315 SDA | I2C data line |
| GPIO 22 | SSD1315 SCL | I2C clock line |

### 3.3 System Block Diagram

```
                    ┌─────────────────────────────────────────────┐
                    │              ESP32 NodeMCU                   │
                    │                                             │
 HC-SR04           │  GPIO5 ──→ TRIG (output pulse)              │
 Ultrasonic  ←────→│  GPIO18 ←── ECHO (via voltage divider)      │
 (5V)              │                                             │
                    │  GPIO4 ←── OUT (presence flag)              │
 LD2410C           │  GPIO16 ←── TX (UART data)                  │
 24GHz Radar ←────→│                                             │
 (5V)              │  GPIO21 ──→ SDA (I2C data)                  │
                    │  GPIO22 ──→ SCL (I2C clock)                 │
 SSD1315    ←─────│                                             │
 OLED (3.3V)       │  WiFi AP ──→ Web Dashboard                  │
                    │              (192.168.4.1)                   │
                    └─────────────────────────────────────────────┘
```

---

## 4. Circuit Design

### 4.1 Voltage Divider for HC-SR04 Echo Signal

The HC-SR04 operates at 5V and outputs a 5V echo signal. The ESP32 GPIO pins are rated for a maximum of 3.3V. Connecting the echo pin directly to the ESP32 would damage the microcontroller.

A resistive voltage divider reduces the 5V echo signal to approximately 3.3V.

**Circuit:**

```
HC-SR04 ECHO (5V) ──── 1kΩ ──── Junction ──── 2kΩ ──── GND
                                    │
                                    └──── ESP32 GPIO18
```

**Calculation:**

V_out = V_in × R2 / (R1 + R2)
V_out = 5V × 2000Ω / (1000Ω + 2000Ω)
V_out = 5V × 0.667
V_out = 3.33V

This is within the ESP32's safe operating range (max 3.3V), providing adequate signal level for reliable digital HIGH detection (threshold approximately 2.5V).

**Component values:** R1 = 1kΩ (brown-black-red), R2 = 2kΩ (red-black-red)

### 4.2 Shared Power Distribution

Multiple devices share the 3.3V supply through a breadboard power rail:

```
ESP32 3U3 ──→ Breadboard + Rail ──→ OLED VCC (3.3V)

ESP32 U5  ──→ HC-SR04 VCC (5V)
          ──→ LD2410C VCC (5V)

ESP32 GND ──→ Breadboard − Rail ──→ All component GNDs
```

**Design note:** The LD2410C datasheet specifies 5–12V power input. Initial testing at 3.3V produced no response from the OUT pin. Moving VCC to the 5V rail (U5) resolved the issue.

---

## 5. Sensor Theory

### 5.1 HC-SR04 Ultrasonic Sensor

**Operating principle:** The HC-SR04 measures distance using the time-of-flight of a 40kHz ultrasonic pulse.

**Measurement sequence:**
1. The ESP32 sends a 10μs HIGH pulse on the TRIG pin
2. The HC-SR04 emits 8 cycles of 40kHz ultrasound
3. Sound travels to the target, reflects, and returns
4. The ECHO pin goes HIGH for a duration proportional to the round-trip time
5. Distance = (duration × speed of sound) / 2

**Speed of sound:** 343 m/s at 20°C = 0.0343 cm/μs = 0.034 cm/μs (approximation used in code)

**Division by 2:** The sound travels to the target AND back. The measured duration is the round trip, so we divide by 2 to get the one-way distance.

**Specifications:**
- Range: 2cm to 400cm
- Accuracy: ±3mm
- Measuring angle: 15°
- Operating frequency: 40kHz

**Relevance to Signale und Systeme (SS):** The ultrasonic sensor is a time-domain measurement system. The trigger pulse is the input signal, the echo is the output signal. The transfer function of the air channel introduces a pure time delay proportional to distance. Sampling at 10Hz (100ms intervals) introduces Nyquist considerations — altitude changes faster than 5Hz will be aliased (Lernpaket 14: Abtastung und Rekonstruktion).

### 5.2 LD2410C 24GHz FMCW Radar

**Operating principle:** The LD2410C transmits a 24GHz frequency-modulated continuous wave (FMCW) radar signal. Unlike ultrasonic, it does not measure distance by echo timing — it analyzes the frequency shift of the returned signal.

**How it detects humans vs objects:** Static objects (walls, floors, furniture) reflect the radar signal at the same frequency. Humans produce micro-Doppler signatures — the chest moves during breathing (~0.5Hz), the heart beats (~1Hz), and limbs make subtle involuntary movements. The LD2410C's internal DSP processes these frequency shifts to distinguish between:
- **No target** (state 0): No significant reflections
- **Moving target** (state 1): Active movement detected (walking, waving)
- **Stationary target** (state 2): Micro-movements detected (breathing, sitting)
- **Both** (state 3): Moving and stationary targets simultaneously

**Output modes:**
- GPIO (OUT pin): HIGH when human detected, LOW when clear — used in this project
- UART (TX pin): Detailed data frames with distance, energy levels, target state — available for future integration

**Specifications:**
- Detection range: up to 6 meters
- Frequency: 24GHz ISM band
- Power: 5–12V input
- Detection types: moving and stationary human presence

**Relevance to EEL (Elektronische Schaltungen):** The LD2410C's internal front-end uses a mixer (multiplying the transmitted and received signals) to produce an intermediate frequency proportional to target distance. This is a direct application of signal multiplication and frequency analysis — concepts from both EEL and SS coursework.

---

## 6. Software Design

### 6.1 ESP32 Firmware (C++ / Arduino Framework)

The firmware performs four tasks in the main loop, executing approximately 10 times per second:

**Task 1: Read ultrasonic distance**
```cpp
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);    // Send trigger pulse
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // Measure echo time
  return duration * 0.034 / 2.0;                    // Convert to cm
}
```

The `pulseIn` function blocks until the ECHO pin goes HIGH, then measures how long it stays HIGH (in microseconds). The timeout of 30000μs prevents infinite blocking if no echo returns.

**Task 2: Read radar presence**
```cpp
bool humanDetected = digitalRead(RADAR_OUT) == HIGH;
```

A single digital read. The LD2410C handles all signal processing internally and outputs a binary decision.

**Task 3: Update OLED display**
The Adafruit SSD1306 library communicates with the OLED over I2C. Each frame, the display buffer is cleared, redrawn with current data, and pushed to the screen. The display shows: title bar, distance value, altitude bar graph, scrolling altitude history, radar status, and landing decision.

**Task 4: Serve web dashboard**
The ESP32 creates a WiFi access point and runs an HTTP server on port 80. Two endpoints:
- `GET /` — returns the full HTML/CSS/JS dashboard page
- `GET /data` — returns current sensor readings as JSON: `{"distance":45.3,"human":true}`

The browser-side JavaScript fetches `/data` every 200ms and updates the chart and UI elements.

### 6.2 Landing Decision Logic

```
IF distance > 0 AND distance < 30cm AND NOT humanDetected:
    → SAFE TO LAND

IF humanDetected:
    → ABORT LANDING

IF distance == 0:
    → NO SIGNAL

IF distance >= 50cm:
    → CRUISING
```

This is a simplified state machine. A production system would add hysteresis (to prevent flickering between states), minimum time thresholds (require "clear" for N consecutive readings), and configurable altitude limits.

### 6.3 Web Dashboard Architecture

```
┌──────────────┐         WiFi          ┌──────────────────┐
│   ESP32      │ ◄─────────────────── │   Browser         │
│              │                       │                   │
│  /data       │ ──── JSON ──────────→ │  JavaScript       │
│  endpoint    │    every 200ms        │  fetch() loop     │
│              │                       │                   │
│  Sensors     │                       │  Canvas graph     │
│  read loop   │                       │  Status cards     │
└──────────────┘                       └──────────────────┘
```

The dashboard uses a Canvas element for the altitude history graph, CSS Grid for layout, and CSS animations for the radar detection pulse indicator.

---

## 7. Problems Encountered and Solutions

### 7.1 HC-SR04 Echo Pin at 5V

**Problem:** The HC-SR04 echo output is 5V. ESP32 GPIO is rated for 3.3V maximum. Direct connection risks permanent damage to the microcontroller.

**Solution:** Implemented a voltage divider using 1kΩ and 2kΩ resistors to step down the echo signal from 5V to 3.33V. This is a standard level-shifting technique for interfacing 5V peripherals with 3.3V microcontrollers.

### 7.2 LD2410C Not Responding at 3.3V

**Problem:** The LD2410C OUT pin remained at 0 regardless of human presence when powered from the 3.3V rail.

**Solution:** Consulted the LD2410C datasheet, which specifies 5–12V input. Moved VCC connection from 3U3 (3.3V) to U5 (5V). Sensor began responding immediately.

### 7.3 Upload Failures

**Problem:** ESP32 did not enter flash mode during upload, causing "chip stopped responding" errors.

**Solution:** Hold the BOOT button on the ESP32 during the "Connecting..." phase of the upload process. This forces the chip into bootloader mode. Release after upload begins.

---

## 8. Future Work

1. **Digital filtering:** Implement a moving average or low-pass FIR filter on the ultrasonic readings to reduce noise (directly applies SS Lernpaket 23: Filterentwurf)
2. **FreeRTOS dual-core:** Run sensor reading on Core 0 and web server on Core 1 for true parallel processing
3. **Kalman filter:** Fuse ultrasonic and radar distance estimates for improved accuracy
4. **Integration with drone:** Mount the sensor module on the face-tracking drone (Drone-Projekt) as a real landing subsystem
5. **UART radar parsing:** Use the LD2410C TX data for detailed moving/stationary target distance and energy levels
6. **nRF24L01 RF telemetry:** Replace WiFi with long-range RF for drone-to-ground-station communication

---

## 9. Repository

All source code is available at: **github.com/kryptonus**

Related project: **github.com/kryptonus/Drone-Projekt** — Autonomous face-tracking drone in ROS2/Gazebo with C++ OOP architecture.

---

## Appendix A: Full Source Code

The complete firmware (ESP32 Arduino sketch) integrates all three sensors with the OLED display and WiFi web dashboard in a single file. The code is structured around four functions:

- `getDistance()` — ultrasonic measurement
- `handleRoot()` — serves the HTML dashboard
- `handleData()` — serves JSON sensor data
- `loop()` — main execution loop (sensor reading, display update, server handling)

Total code size: approximately 160 lines of C++ and 120 lines of embedded HTML/CSS/JavaScript.
