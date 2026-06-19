<!--
  GitHub project README for Project Herbinator.
  Drop this in the repo root as README.md.
-->

<div align="center">

# Project Herbinator 🌱

### Landon Wardle, Robert Cromer, Jasmine Sellers, Juan Ramirez, Zach Worcester, Hunter Mass
### May, 2026

A low-cost smart watering device that keeps small plants alive on its own.
An ESP32-S3 runs a finite-state machine that samples soil moisture, factors in
temperature and humidity, and pumps water only when the plant actually needs it.
A companion Python app monitors the device over Wi-Fi.

<!-- Badge row — these render as little pills on GitHub -->
![Firmware](https://img.shields.io/badge/firmware-Arduino%20C++-00599C?style=for-the-badge&logo=arduino&logoColor=white)

![App](https://img.shields.io/badge/app-Python%203-3776AB?style=for-the-badge&logo=python&logoColor=white)

![MCU](https://img.shields.io/badge/MCU-ESP32--S3-E7352C?style=for-the-badge&logo=espressif&logoColor=white)

![Focus](https://img.shields.io/badge/focus-embedded%20systems-2b6cb0?style=for-the-badge)

</div>

---

## Table of Contents

1. [Overview](#overview)
2. [How It Works](#how-it-works)
3. [Hardware](#hardware)
4. [Pinout](#pinout)
5. [Firmware](#firmware)
6. [Calibration](#calibration)
7. [HTTP API](#http-api)
8. [Python Application](#python-application)
9. [Configuration](#configuration)
10. [State and LED Reference](#state-and-led-reference)
11. [Data Logging](#data-logging)
12. [Known Limitations](#known-limitations)

---

# Overview

Herbinator is an automated irrigation device for small-scale plants such as indoor gardens or
windowsill collections. It continuously monitors soil moisture and waters the plant only when the
soil dries below a calibrated threshold, then stops once the soil is satisfied. The goal is to cut
out manual watering while using water efficiently.

The brain is an **ESP32-S3** running firmware structured as a finite-state machine (FSM). The
board reads a capacitive moisture sensor and a DHT11 temperature/humidity sensor, drives a small
submersible pump, and protects itself with a float switch that prevents the pump from running dry.
All readings and pump events are logged to an SD card, and the device hosts a small HTTP server so
a Python app can monitor it remotely.

> [!NOTE]
> To see the Herbinator Roadmap and Project Specification Template
> see the Documentation folder in the repository.

# How It Works

At a high level, Herbinator spends most of its life idling and sampling the moisture sensor
occasionally. When the soil gets too dry it starts the pump, and once the soil is wet enough (or a
safety limit trips) it stops and goes back to idling.

| Step | What happens |
|:-----|:-------------|
| Idle / monitor | The FSM sits in `WaterLoop`, sampling the moisture sensor at a slow interval to save power. |
| Decide | If moisture drops to or below the *on* threshold, it enters `PumpOn`. The threshold is nudged by the current temperature and humidity (see VPD below). |
| Water | The pump runs until soil moisture rises past the *off* threshold, the reservoir runs dry, or a maximum run-time is reached. |
| Recover | After watering it returns to monitoring. If the pump ran for the full safety limit, it first cools down in `PumpCD`. |
| Protect | If the float switch reports no water, or a sensor stops responding, the FSM jumps to the matching error state and parks the pump. |

> [!NOTE]
> The decision threshold is adjusted using **Vapor Pressure Deficit (VPD)**, which is a measure of how fast
> a plant loses water to the air. Hotter, drier air raises the threshold so the plant is watered a
> little sooner; cool, humid air lowers it so watering is delayed. The base *on* threshold is 25%
> and the *off* threshold is 75%, with VPD shifting the *on* threshold by up to +-10%.

# Hardware

Core components for the v1.0 custom PCB:

| Component | Part | Purpose |
|:----------|:-----|:--------|
| Microcontroller | ESP32-S3-WROOM-1 | Runs the FSM firmware, all I/O, and Wi-Fi for Python App |
| Moisture Sensor | Capacitive soil moisture sensor | Estimates soil moisture |
| Temp/Humidity Sensor | DHT11 (U5) | Measures temperature (°C) and humidity |
| Water Pump | 5 V DC mini submersible pump | Delivers water to the plant |
| Float Switch | Magnetic float switch (J5) | Measures if water is available to pump |
| SD Card Reader | SPI SD card module (J4) | Logs readings and pump events to a `.csv` file |
| State LEDs | Green (D2), Blue (D3), Red (D4) | Visual feedback |
| Calibration Button | SW3 tactile push-button | Triggers the moisture calibration routine |
| 3.3 V regulator | AP2112K-3.3V | Powers the ESP32-S3 and sensor logic |
| 5 V regulator + jack | AMS1117-5.0 + 5.5 mm barrel jack | Powers the pump from a wall adapter |
| USB-C | USB-C receptacle (J1, FUSB302) | Alternate power input and serial programming |

> [!NOTE]
> There is no circuitry to switch from the USB-C Port to the
> barrel jack because the PCB was created to have two
> distinct power rails. Both must be plugged in for
> correct results.

> [!NOTE]
> The PCB is still being assembled. More documentation
> and photos will be available once it is done.

# Pinout

GPIO assignments as defined at the top of `ESPMain.ino`:

| Function | Pin | Function | Pin |
|:---------|:---:|:---------|:---:|
| Pump (servo/ESC signal) | 7 | Green LED | 13 |
| Float switch drive (high) | 0 | Blue LED | 12 |
| Float switch read | 10 | Red LED | 11 |
| Moisture sensor (analog) | 4 | Calibration button | 6 |
| DHT11 data | 5 | SD `SCK` | 3 |
| SD `CS` | 18 | SD `MISO` | 46 |
| SD `MOSI` | 8 | | |

> [!NOTE]
> The pump is driven through the `ESP32Servo` library as an ESC-style PWM signal, **not** a raw
> on/off pin. A write of `0` runs the pump at full speed and `90` stops it completely.

# Firmware

The firmware lives in `main/ESPMain/ESPMain.ino` and is built and flashed with the **Arduino IDE** targeting an **ESP32-S3**.

### Required libraries

Install these through the Arduino Library Manager:

`DHT11` · `ESP32Servo` (provides `ESP32PWM`) · `ArduinoJson`

Remaining libraries are available on the **ESP32-S3** by default.

### Build & flash

1. Install the **ESP32 board package** in the Arduino IDE.
2. Open `main/ESPMain/ESPMain.ino`.
3. Select an **ESP32-S3** board and the correct serial port.
4. Set your Wi-Fi credentials (see [Configuration](#configuration)).
5. Upload, then open the Serial Monitor at **9600 baud** (or change the baud rate specified in the code) to watch state transitions.

> [!WARNING]
> On boot the firmware **blocks until the SD card initializes**. If no card is inserted, the red
> LED stays on and the device prints an SD warning in a loop instead of starting the FSM.

# Calibration

Capacitive moisture sensors read different raw values per unit and per soil type, so Herbinator
calibrates against your specific sensor and soil. Calibration values are saved to a simulated
**EEPROM** on the **ESP32-S3** so they are non-volatile.

To calibrate, **hold the button (SW3) for 2 seconds** from the idle state. Herbinator then uses
the LEDs to guide you:

| Step | LED cue | What to do |
|:-----|:--------|:-----------|
| Start | Red flashes 3s | Calibration mode has begun |
| Dry point | Green flashes | Place the sensor in dry soil / air, then press and hold the button |
| Wet point | Blue flashes | For best results water your plant, then place the sensor in the expected wet soil, then press and hold the button. Partially submerging the sensor in a cup of water is also acceptable. |
| Done | All LEDs blink | Done! Wait for Herbinator to resume the water loop. |

> [!NOTE]
> If the stored calibration is missing or invalid on startup, the device automatically drops into
> the calibration routine rather than watering with bad thresholds.

# HTTP API

After connecting to Wi-Fi the device registers an mDNS name and serves a small HTTP API on **port
80**. With mDNS working it is reachable at `http://herbnet.local/`, or directly by its IP address.

| Method | Endpoint | Returns |
|:------:|:---------|:--------|
| `GET` | `/` | JSON snapshot: `temperature`, `humidity`, `moisture`, `state`, `time` |
| `GET` | `/time` | Current time string (synced over NTP) |
| `GET` | `/temperature` | Last temperature reading |
| `GET` | `/humidity` | Last humidity reading |
| `GET` | `/moisture` | Last moisture percentage |
| `GET` | `/state` | Current FSM state name |
| `POST` | `/water` | Acknowledges a manual-water request |

<details>
<summary><b>Example <code>/</code> response</b></summary>

<br>

```json
{
  "temperature": 67,
  "humidity": 21,
  "moisture": 69,
  "state": "PumpOn",
  "time": "2026-05-28 13:45:27"
}
```

</details>

> [!NOTE]
> The `/water` endpoint currently returns a `watering` status but does not yet trigger the pump —
> the actual watering hook is still a TODO. The `/pause` and `/resume` endpoints are scaffolded in
> the firmware but commented out, so the app's pause/resume buttons have nothing to talk to yet.

# Python Application

The desktop app in `python/herbinator.py` is a **Tkinter** GUI that polls the device and displays
live data. It records each sensor's value plus the device's current state, and pulls the timestamp
from the device (synced over NTP).

### Running it

```bash
pip install requests
python herbinator.py
```

### What it shows

The window displays **Temperature**, **Humidity**, **Moisture**, **State**, and **Last Update**,
plus a scrolling log of every request and response. It refreshes automatically every **30 seconds**.

### Buttons

| Button | Action |
|:-------|:-------|
| Water Plant | Sends `POST /water` |
| Pause | Sends `POST /pause` |
| Pause 1 Minute | Sends `POST /pause` with a 60-second duration |
| Resume | Sends `POST /resume` |

> [!NOTE]
> Set `BASE_URL` at the top of `herbinator.py` to your device's address — either `http://herbnet.local`
> (if mDNS resolves on your network) or the device's IP shown in the Serial Monitor on boot. If the
> device is unreachable, every field shows `Disconnected` and the error appears in the log box.

# Configuration

Most tunable values are constants near the top of `ESPMain.ino`:

| Setting | Constant | Default |
|:--------|:---------|:--------|
| Wi-Fi SSID | `ROUTER_NAME` | `HerbyIOT` |
| Wi-Fi password | `ROUTER_PASSWORD` | `herbinator` |
| mDNS name | `mdnsName` | `herbnet` → `herbnet.local` |
| Timezone offset | `gmtOffset_sec` | `-28800` (PST) |
| Moisture *on* threshold | `ON_THRESHOLD_BASE` | `0.25` |
| Moisture *off* threshold | `OFF_THRESHOLD_BASE` | `0.75` |
| Max pump run-time | `MAX_PUMP_ON_MS` | 45s |
| Long-run pump cooldown | `PUMP_CD_MILLIS` | 5 min |
| Idle sample interval | `WATER_LOOP_SAMPLE_MS` | 5s |
| Calibration hold time | `CALIBRATION_HOLD_MS` | 2s |

> [!WARNING]
> Wi-Fi credentials are stored in plain text in the source. Don't commit real network credentials
> to a public repository.

# State and LED Reference

The FSM has the following states. The three status LEDs signal what the device is doing at a glance:

| State | Meaning | LEDs |
|:------|:--------|:-----|
| `MainLoop` | Validates calibration, checks for water, then enters monitoring | Green |
| `WaterLoop` | Idle monitoring; sampling soil moisture | Green |
| `PumpOn` | Pump running | Blue |
| `PumpOff` | Pump shutting off | Off |
| `PumpCD` | Cooldown after a long pump run | Red (flashing) |
| `NoWater` | Reservoir empty (float switch) — pump parked | Red + Blue |
| `RedFlash` | Entry into calibration | Red (flashing) |
| `PromptDry` | Waiting for dry calibration point | Green (flashing) |
| `PromptWet` | Waiting for wet calibration point | Blue (flashing) |
| `ButtonPressed` / `CalibrationButtonYield` | Hold-detection during calibration | — |
| `CalibrationDone` | Calibration saved | All (blinking) |
| `MoistureSensorFail` | Moisture sensor out of range / unresponsive | Red (fast flash) + Blue |
| `TempSensorFail` | DHT11 unresponsive | Red (fast flash) + Green |

# Data Logging

When an SD card is present, the firmware appends every sensor cycle and pump event to
`/herbinator_logs.txt` as CSV, flushing to the card every 30 seconds. Columns:

```text
timestamp_ms, state, rawMoist, moist_pct, on_thresh, temp_c,
humidity_pct, vpd_kpa, dht_ok, pump_on, water_raw, water_present
```

This makes it easy to pull the card afterward and analyze how the soil dried and recovered around
each watering event. It is recommended to do this in MATLAB or similar software that can crunch 
large .csv files quickly.

# Known Limitations

- **Manual watering isn't wired up.** `/water` returns a status but doesn't actually run the pump yet.
- **Pause/resume are stubs.** Those firmware endpoints are commented out, so the app's Pause / Resume / Pause 1 Minute buttons have no effect on the device.
- **CLI is a placeholder.** `python/interface.py` is an early scaffold and not yet a working interface.
- **Credentials are hardcoded.** Wi-Fi SSID/password live in the firmware source.

<!--
  Suggested next steps for the team:
  - Implement the watering hook behind /water and reinstate /pause + /resume to match the app.
  - Move Wi-Fi credentials out of source (e.g. a config the user enters, or a secrets header that's gitignored).
  - Finish interface.py or remove it if the Tkinter app is the path forward.
  - Consider documenting the BOM footprint lessons from the spec doc here too.
-->
