

---

# Tugbot Project 🚢

**Autonomous + RC Model Tugboat Platform**

Tugbot is a long-running experimental project focused on building a **robust, modular, and extensible control system** for a scale model tugboat.
It combines **traditional RC control**, **autonomous navigation**, and **rich telemetry**, with an emphasis on reliability, observability, and real-world hardware constraints.

This repository represents the **canonical source** for Tugbot firmware, configuration, and system documentation.

---

## Project Goals

* Hybrid **manual RC + autonomous operation**
* Reliable **long-range radio communication**
* Modular, testable firmware architecture
* Clear separation of **TX (transmitter)** and **RX (vessel)** code
* Rich telemetry with fault detection and logging
* Hardware-aware software (timeouts, non-blocking, failure tolerant)

Old man wisdom: *A boat that fails silently is worse than one that fails loudly.*

---

## System Overview

### Tugbot Vessel (RX)

* Arduino Mega–class controller (historical + canonical pin map)
* Single propulsion motor with high-frequency PWM drive
* Steering servo
* GPS + compass for navigation
* Multiple sensors:

  * Current
  * Voltage (multiple batteries)
  * Temperature (motor & speed controller)
  * Water ingress / bilge
* Onboard effects:

  * Engine sound
  * Horn / SFX
  * Smoke generator
  * Pumps
  * Lighting

### Transmitter (TX)

* ESP32-based controller
* NRF24L01 radio link to Tugbot
* Physical controls (switches, encoders, buttons)
* Local display (OLED / LED matrix)
* Optional BLE link to iPhone for telemetry & configuration

---

## Communication Architecture

* **Primary link:** NRF24L01 (2.4 GHz)
* **Protocol:** Custom packed binary frames
* **Features:**

  * Acknowledged packets
  * Link health monitoring
  * Fail counters and timeouts
  * Optional AckPayload usage

Canon radio note:

> Always allow ~2 seconds after `Serial.begin()` before initializing or printing NRF status.

---

## Repository Structure

```
Tugbot-Project/
├── Arduino/
│   ├── TugbotRX/          # Vessel firmware
│   ├── TugbotTX/          # Transmitter firmware
│   ├── libraries/        # Custom Tugbot libraries
│   └── tests/            # Hardware & integration test sketches
│
├── docs/
│   ├── architecture.md
│   ├── protocol.md
│   ├── pinout.md
│   └── testing.md
│
├── tools/
│   └── scripts and utilities
│
├── .gitignore
└── README.md
```

> All `.ino` and library files must be clearly marked as **TX** or **RX** in their headers.

---

## Canonical Design Rules

These rules are **non-negotiable** unless explicitly revised:

* **No blocking waits** on:

  * Serial
  * I2C
  * SPI
  * UART
  * Sensors or peripherals
* All failures must be:

  * Detected
  * Logged
  * Reported (when possible)
* Pin assignments are centralized in:

  * `pins_tugbot.h`
* Configuration values live in:

  * `tugbot_config_*` headers
* Energy-management code is **frozen** unless explicitly reopened
* All new code must tolerate missing or non-responsive devices

---

## Testing Philosophy

Testing is first-class, not an afterthought.

* Dedicated sketches exist for:

  * Radio detection
  * Motor ramping
  * Servo motion
  * Sensor validation
  * Integration testing
* Integration tests emit structured **STATE** and **EVT** telemetry
* Hardware bring-up happens **before** feature development

If you can’t test it alone on the bench, it doesn’t belong on the lake.

---

## Development Status

* Multiple beta revisions completed
* Core architecture stable
* Ongoing work focuses on:

  * Hardening
  * Observability
  * Transmitter UX
  * Autonomous navigation refinement

This is **not** a beginner Arduino project. It is intentionally engineered to behave like a real embedded system.

---

## License

Private / experimental project.
Reuse or redistribution only with explicit permission.

---

## Final Note

Tugbot exists to answer one question:

> *What happens when you stop treating a model boat like a toy and start treating it like a vessel?*


