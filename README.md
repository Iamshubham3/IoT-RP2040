# RP2040 Ethernet MQTT Testbench Firmware

This repository contains embedded firmware developed for hardware validation and system-level testing of an IoT-based control unit.

The firmware runs on an RP2040 platform using the Arduino framework and integrates Ethernet communication, MQTT messaging, temperature sensing, proximity detection, and lock control.

---

## Overview

The system performs:

- Ethernet-based MQTT communication
- Dual DS18B20 temperature sensing
- Proximity detection via digital input
- Control of dual electromagnetic locks
- JSON-based message parsing and publishing
- Periodic telemetry reporting

The firmware was used for testbench validation and functional verification of a prototype IoT device.

---

## Hardware Components

- Microcontroller: RP2040
- Ethernet interface (SPI-based)
- 2x DS18B20 temperature sensors
- Proximity sensor (digital input)
- 2x electromagnetic lock outputs
- MQTT server (local network)

---

## Software Stack

- Arduino Framework
- Ethernet library
- PubSubClient (MQTT)
- ArduinoJson
- microDS18B20 library

---

## Functional Description

### 1. MQTT Communication

- Connects to a local MQTT broker
- Subscribes to: `/bob/status`
- Publishes to: `BOB`
- Handles remote lock commands via JSON payload
- Serial debug output for monitoring state transitions

---

### 2. Temperature Monitoring

- Reads two DS18B20 sensors
- Temperature acquisition every 10 minutes
- Publishes temperature values as JSON over MQTT
- Includes error handling for sensor read failures

---

### 3. Proximity Monitoring

- Polls proximity input every 500 ms
- Detects state transitions
- Automatically:
  - Unlocks when no detection
  - Locks when detection occurs
- Publishes proximity and lock state via MQTT

---

### 4. Lock Control Logic

Two digital outputs control electromagnetic locks.
