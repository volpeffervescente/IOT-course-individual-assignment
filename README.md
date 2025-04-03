# IoT System with Adaptive Sampling using ESP32 and FreeRTOS

## Introduction
This project implements an IoT system that collects data from a sensor, processes it locally, and transmits an aggregated value to a nearby server and the cloud. The system adapts its sampling frequency dynamically to optimize energy consumption and reduce communication overhead. The firmware is developed using FreeRTOS on an ESP32 board.

## Features
The system dynamically adjusts the sampling frequency based on the frequency components of the input signal. It computes the frequency spectrum of the signal using FFT to determine the optimal sampling frequency. The sampled data is aggregated locally by computing the average over a defined time window. The aggregated data is then transmitted to a nearby server via MQTT over WiFi and to the cloud using LoRaWAN + TTN. Performance evaluation includes measuring energy savings, data transmission volume, and system latency.

## System Architecture
### Hardware Components
The project uses an ESP32 prototype board, a Heltec WiFi LoRa V3 board with its antenna, linked thorough a breadboard and two jumpers, one for the connection to the gnd, and one to connect the two boards, respectively the connection is between GPIO25 of the ESP32 and the GPIO07 of the Heltec WiFi LoRa (you can also link the two boards directly with two FF jumpers). 

### Software Components
The firmware is based on FreeRTOS, which efficiently manages tasks. MQTT is used for communication with the edge server over WiFi, while LoRaWAN + TTN enables data transmission to the cloud. FFT analysis helps compute the signal frequency spectrum, and an adaptive sampling algorithm dynamically adjusts the sampling rate.
Thus, the libraries you need are: ArduinoFFT, FreeRTOS, ------

#### How to Set Up and Run
Prerequisites: an ESP32 board with FreeRTOS support, an MQTT broker, and a LoRaWAN gateway with a TTN account are required.
Configure the MQTT broker and LoRaWAN settings, start the ESP32 firmware, and observe sensor data processing and transmission.

## Implementation Details

### Input
The input signal is given (generated) by the DAC (digital to analog converter) enbedded in the ESP32 (GPIO25). 
In particular, the input signal is a wave that has the form 2sin(2π3t) + 4sin(2π5t).

### Maximum Sampling Frequency
The ADC (Analog to digital converter) enbedded in the Heltec WiFI LoRa V3 (GPIO07) samples the wave signal it receives. 
The maximum sampling frequency is 34.10 kHz. 

### Optimal Sampling Frequency
In the "optimal_samp_frequency" code FFT is computed on the sampled signal to determine its frequency spectrum. The sampling frequency is then adapted to 12Hz to match the highest frequency component, that is then displayed on the serial. The peak is obtained through the majorPeak() function.The major peak 6 Hz. As per the Nyquist sampling theorem, the final sampling frequency was set to 12 Hz.

### Aggregate Function Computation
In the "window_code" code there is a task that computes the average avg() of the sampled signal over a specified time window. This one is defined as a ciruclar buffer.

### Communication with Edge Server
The computed average is transmitted to a nearby server using MQTT over WiFi. This is done by exploiting the window_code and integrating MQTT code. The MQTT broker is a simple ----------

### Communication with Cloud
The computed average is transmitted to the cloud using LoRaWAN + TTN using the LoRaWAN connectivity repository provied by Heltec. 

-----------------------------------------------------------------------------------------------------------

## Performance Evaluation
### Energy Savings
Energy consumption is compared between adaptive sampling and fixed over-sampling.---------

### Data Transmission Volume
The amount of data transmitted using adaptive sampling is measured against over-sampling.---------

### End-to-End Latency
Latency is measured from the point of data generation to reception at the edge server.---------

## Bonus: Multiple Input Signals
---
