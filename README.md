# ATmega8A PID Temperature Control with Modbus RTU and MQTT Monitoring

[![MCU](https://img.shields.io/badge/MCU-ATmega8A-red.svg)]()
[![Protocol](https://img.shields.io/badge/Protocol-Modbus_RTU-blue.svg)]()
[![Cloud](https://img.shields.io/badge/IoT-MQTT_HiveMQ-orange.svg)]()
[![Language](https://img.shields.io/badge/Language-C_&_Python-green.svg)]()

## 📌 Project Overview
Bare-metal C firmware for an **ATmega8A** that reads a **J-type thermocouple**, built in two stages:

1. **PID temperature control** (`src/pid_control/`): a PID loop drives a 12 V heater with hardware PWM. The setpoint and PID gains are written from the PC over a custom **Modbus RTU** slave.
2. **IoT monitoring** (`src/monitoring/`): the ATmega8A serves the temperature over Modbus RTU, and a Python application acts as the Modbus master and publishes the value to a **HiveMQ MQTT** broker.

Both stages share the same sensor chain and Modbus RTU implementation.

## ⚙️ Shared Building Blocks

### Sensor chain
*   The thermocouple's millivolt signal is amplified by an LM358 op-amp (gain 100) and sampled by the 10-bit ADC (PC2).
*   An 8-sample moving average reduces noise.
*   The voltage is converted to °C with the NIST J-type inverse polynomial (coefficients up to 8th order, valid for 0–42.919 mV). Out-of-range readings return an error value (999).

### Modbus RTU slave (written from scratch)
*   USART at 19200 bps; bytes are received in the `USART_RXC_vect` interrupt, so reception does not block the main loop.
*   End of frame is detected by 4 ms of line silence, counted by a 1 ms Timer1 compare interrupt.
*   Frames are checked with a software CRC16 (polynomial 0xA001). Frames with the wrong slave ID (2) or a bad CRC are dropped.

## 🔥 Stage 1: PID Control (`src/pid_control/main.c`)
*   **Timing:** Timer1 (1 ms) also counts a 100 ms tick; the PID update runs in the main loop on each tick (dt = 0.1 s).
*   **Controller:** PID with integral clamping (anti-windup) and output limited to 0–255.
*   **Actuator:** Timer2 fast PWM on OC2 (PB3) drives the heater.
*   **Fail-safe:** If the sensor reading is out of range, the heater is switched off immediately.
*   **Register map (holding registers):**

| Address | Content |
|---|---|
| 0 | Setpoint (°C) |
| 1 | Kp |
| 2 | Ki |
| 3 | Kd |
| 4 | Measured temperature (°C, read-only in practice) |

*   **Function codes:** `0x03` Read Holding Registers, `0x06` Write Single Register, `0x10` Write Multiple Registers.

## 🌐 Stage 2: IoT Monitoring (`src/monitoring/`)
*   **Firmware (`main.c`):** Stores the temperature in holding register 100 and answers `0x03` read requests.
*   **Gateway (`mqtt_gateway.py`):** Tkinter application that polls the register every 2 s over the serial port, shows the value and publishes it to the topic `iyte/ee446/berke/sicaklik` on `broker.hivemq.com`.
*   `broker.hivemq.com` is a public test broker without authentication: anyone who knows the topic can subscribe. This is fine for a demo, not for a real deployment.

## ⚠️ Known Limitations
*   **Cold junction:** The cold-junction temperature is a fixed 20 °C constant; there is no cold-junction sensor.
*   **Integer gains:** PID gains are stored in 16-bit integer registers, so only whole-number gains can be set. A fixed-point scale (for example ×100) would allow fractional gains.
*   **Derivative kick:** The derivative term acts on the error, so a setpoint change causes a one-step derivative spike. Using the derivative of the measurement would avoid this.
*   **Resolution:** Temperature is sent as an integer °C value.

## ▶️ Running the Gateway
```bash
pip install pyserial paho-mqtt
python src/monitoring/mqtt_gateway.py   # set port_name (e.g. COM11) in the script first
```

## 📂 Repository Structure
```text
├── src/
│   ├── pid_control/
│   │   └── main.c            # Stage 1: PID heater control + Modbus RTU slave (0x03/0x06/0x10)
│   └── monitoring/
│       ├── main.c            # Stage 2: temperature monitoring Modbus RTU slave
│       └── mqtt_gateway.py   # Python Modbus master + MQTT publisher (Tkinter GUI)
├── schematics/               # Proteus circuit project
├── docs/                     # Technical report (IoT monitoring stage)
└── README.md
```