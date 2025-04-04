# IoT System with Adaptive Sampling using ESP32 and FreeRTOS

## Overview
This repository contains the source code and documentation for an **IoT system** designed to collect, process, and transmit sensor data efficiently using **adaptive sampling**. The system is built on **ESP32 with FreeRTOS**, utilizing **FFT** for signal analysis and dynamically adjusting the sampling frequency to save energy and reduce communication overhead.

---

## Features
- **Dynamic Sampling Frequency Adjustment**: Adjusts sampling rate based on signal frequency components.
- **FFT-based Analysis**: Uses Fast Fourier Transform to detect dominant frequencies.
- **Data Aggregation**: Computes average values over a moving time window.
- **Dual Communication**:
  - **WiFi (MQTT)**: Sends data to a local edge server.
  - **LoRaWAN + TTN**: Transmits data to the cloud.
- **Performance Metrics**: Includes energy savings, transmission volume, and latency measurement.

---

## System Architecture

### Hardware Components
- **ESP32 Development Board**
- **Heltec WiFi LoRa V3 Board** (with antenna)
- **Connections**:
  - `GPIO25 (ESP32)` → `GPIO07 (Heltec)`
  - `GND ↔ GND`
  - Use breadboard and jumper wires (or FF jumpers for direct connection)

### Software Components
- **RTOS**: [FreeRTOS](https://www.freertos.org/)
- **Signal Processing**: [ArduinoFFT](https://github.com/kosme/arduinoFFT)
- **Communication Protocols**:
  - MQTT over WiFi
  - LoRaWAN via [TTN (The Things Network)](https://www.thethingsnetwork.org/)

### Prerequisites
- ESP32 board with FreeRTOS support
- Configured MQTT broker
- LoRaWAN gateway with a registered TTN application/account

---

## Setup Instructions
1. **Clone this repository**
   ```bash
   git clone https://github.com/yourusername/iot-adaptive-sampling.git
   ```
2. **Open the project in Arduino IDE or PlatformIO**
3. **Install required libraries**
   - ArduinoFFT
   - FreeRTOS (if using Arduino IDE, select ESP32 board with FreeRTOS)
4. **Configure MQTT and LoRaWAN settings** in the source files
5. **Upload firmware** to the ESP32 board
6. **Monitor the Serial output** for live logs and debug info

---

## Implementation Details

### Input Signal
Generated by the ESP32 DAC on `GPIO25`:
```math
2 \cdot \sin(2\pi \cdot 3t) + 4 \cdot \sin(2\pi \cdot 5t)
```
### Sampling Frequency
- **Max Sampling Rate**: 34.10 kHz (ADC on `GPIO07` of Heltec)
- **Optimal Sampling**: FFT detects a major peak at 6 Hz → Sampling adjusted to **12 Hz** using sampling theorem

### Aggregate Function Computation
- computing the average over a sliding window implemented with a **circular buffer**
- Defined in `window_code`

### MQTT Communication
- Aggregated data (avg) sent to local edge server via MQTT
- "window_code" exploited to compute this task
- adafruit topic is volpeffervescente/feed/avg 
- MQTT broker setup required (e.g., Mosquitto)

### LoRaWAN + TTN Communication
[NOTE: Incomplete]
- Aggregated data sent to cloud using LoRa
- LoRaWAN configuration from **Heltec LoRaWAN connectivity repository**

---

## Performance Evaluation

### Energy Savings
Compares adaptive sampling vs. fixed over-sampling  
[NOTE: Not final – definitely needs review]

**Results**: 
Adaptive Sampling is expected to significantly reduce energy usage. Since data is sampled and transmitted only when necessary, the microcontroller and communication modules can stay in low-power modes longer. This is especially beneficial in low-activity scenarios.
Fixed Oversampling results in higher and constant energy consumption, as the system continuously samples and transmits data, regardless of its relevance.

### Data Transmission Volume
Evaluates reduction in data size using adaptive sampling  
**Results**: 
Adaptive Sampling reduces the amount of data sent over the network by avoiding redundant transmissions. This lowers network load and can improve overall system scalability.
Fixed Oversampling generates a high volume of data, including many unnecessary or repeated values, leading to increased bandwidth usage and storage requirements.

### End-to-End Latency
Measures latency from data generation to reception at edge server  
**Results**: 
Adaptive Sampling may result in slightly lower average latency, as the reduced number of transmissions lessens network congestion and processing delays.
Fixed Oversampling can increase latency under load, especially in systems with multiple nodes or limited bandwidth, due to constant data flow.

---

## Bonus: Multiple Input Signals
[NOTE: to be done]
_

---
