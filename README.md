# Motorcycle Cornering Lights V2

ESP32-C6 based motorcycle cornering light system with 10 individually controlled LEDs, lean angle detection via distance sensors and IMU fusion, thermal management, and current protection.

---

## Hardware

### Microcontroller
- **Seeed XIAO ESP32-C6**

### Sensors
- 2x DFRobot Laser Distance Sensor SEN0590 (I2C address 0x74, via TCA9548A channels 0 and 1)
- ICM-42688-PC 6-axis IMU (I2C address 0x68, via TCA9548A channel 2)
- DS18B20 waterproof temperature probe (1-Wire)
- INA219 current sensor (I2C address 0x40)
- Speed sensor (interrupt-based pulse counting on GPIO18)
- 12V voltage divider (ADC)

### LED Driver
- RECOM RCD-24-1.20 constant current driver (1.2A output, PWM dimmable)
- 10x Nichia 519A LEDs in series
- 10x DMN2041L N-channel MOSFETs (bypass switching)
- MCP23008 I2C GPIO expander (LED 1-8 gates, QFN-20, address 0x20)
- TCA9548A I2C multiplexer (TSSOP-24, address 0x70)

### Cooling
- 5V PWM fan (25-30mm, ball bearing recommended)
- Single aluminum heatsink (50×38×16mm)
- 2mm copper spreading plate (100×30mm)

---

## Pin Assignments

| Pin | GPIO | Function |
|-----|------|----------|
| D4  | 22   | I2C SDA (TCA9548A, MCP23008, INA219) |
| D5  | 23   | I2C SCL |
| D6  | 16   | RCD-24 PWM-DIM (100Ω series, 10kΩ pulldown on driver side) |
| D7  | 17   | LED9 MOSFET gate (direct) |
| D8  | 19   | LED10 MOSFET gate (direct) |
| D3  | 21   | DS18B20 1-Wire data (4.7kΩ pullup to 3.3V) |
| D9  | 20   | Fan PWM |
| D10 | 18   | Speed sensor (interrupt, 10kΩ/3.3kΩ voltage divider from 12V signal) |
| D0  | 0    | 12V voltage monitor (10kΩ/3.3kΩ divider) |

---

## I2C Addresses

| Device | Address | TCA Channel |
|--------|---------|-------------|
| TCA9548A | 0x70 | — |
| Left distance sensor | 0x74 | 0 |
| Right distance sensor | 0x74 | 1 |
| ICM-42688-PC IMU | 0x68 | 2 |
| MCP23008 | 0x20 | — (direct) |
| INA219 | 0x40 | — (direct) |

---

## LED Assignment

| LED | Side  | Activation Order |
|-----|-------|-----------------|
| 1   | Left  | 5th (extreme left) |
| 2   | Left  | 4th |
| 3   | Left  | 3rd |
| 4   | Left  | 2nd |
| 5   | Left  | 1st (closest to center) |
| 6   | Right | 1st (closest to center) |
| 7   | Right | 2nd |
| 8   | Right | 3rd |
| 9   | Right | 4th |
| 10  | Right | 5th (extreme right) |

Gates controlled by MCP23008 GP0-GP7 = LED1-LED8. LED9/LED10 via direct GPIO (D7/D8).

**Gate logic:** GPIO HIGH = MOSFET ON = LED bypassed (OFF). GPIO LOW = MOSFET OFF = LED illuminated (ON).

**Failsafe hardware:** Gates 3, 4, 7, 8 have 10kΩ pulldown resistors (default ON in hardware failsafe). Gates 1, 2, 5, 6, 9, 10 have 100kΩ pullup resistors (default OFF on ESP32 crash). Fan is also triggered by failsafe SPDT lines via BAT54C OR-gate and NPN transistor level shifter.

---

## Lean Angle Detection

The system uses two methods which are fused together:

**Distance sensors (primary at low speed)**
Two downward-facing laser sensors measure clearance to the road surface. Simple and effective at any speed including stationary.
```
lean = atan((rightDist - leftDist) / sensorSpacing)
```

**IMU + speed (primary at high speed)**
ICM-42688-PC gyro yaw rate combined with wheel speed gives accurate lean angle at speed. Self-contained — no external sensors beyond the headlight unit.
```
lean = atan(speed_m/s × yawRate_rad/s / 9.81)
```

**Fusion logic:**
- Speed < 2 m/s: distance sensors only
- Speed ≥ 2 m/s (both available): 30% distance + 70% IMU
- Speed ≥ 2 m/s (distance invalid): IMU only
- Speed < 2 m/s (distance invalid): 0° (safe default)

**IMU configuration:**
- Range: ±250 dps, ODR: 1kHz
- Gyro bias calibrated at startup (200 samples, bike must be stationary)
- Yaw axis configurable (default Z — verify after bench test)

---

## LED Activation Logic

Maximum 3 LEDs on at any time. The active window slides outward as lean angle increases:

**Left turn example (thresholds: 5°, 10°, 20°, 30°, 40°):**
- >5°:  LED5 on
- >10°: LED5, LED4 on
- >20°: LED5, LED4, LED3 on  ← 3 LED max reached
- >30°: LED4, LED3, LED2 on  (LED5 off)
- >40°: LED3, LED2, LED1 on  (LED4 off)

Mirror sequence for right turn (LED6→7→8→9→10).

Each threshold has independent hysteresis — LEDs turn off only when lean drops below (threshold − hysteresis), preventing flickering at boundary angles.

---

## Thermal Management

| Stage | Temp Range | Action |
|-------|-----------|--------|
| Fan off | < Fan On Temp | Fan 0% |
| Fan ramp | Fan On → Fan Full Temp | Fan 20%→100% linear |
| LDD ramp down | Dim Start → Max Temp | LDD 100%→0% linear |
| Sensor failure | Any | Fan 100%, LDD unchanged |

**Thermal analysis (single heatsink, fan failed):**
- Heatsink: 50×38×16mm aluminum, natural convection ~5.6°C/W
- 2 LEDs active (failsafe mode): ~6.2W, steady state ~75°C at 40°C ambient
- 3 LEDs active (software mode): ~9.3W, steady state ~92°C at 40°C ambient
- Time to reach 65°C from cold: ~3 minutes — DS18B20 monitoring provides ample warning

Fan failsafe: both left and right SPDT relay lines OR'd via BAT54C diodes through an NPN transistor (MMBT3904) to drive the fan MOSFET gate independently of the ESP32.

---

## Speed Sensor

Reads the existing motorcycle speedometer signal via a 10kΩ/3.3kΩ voltage divider (12V → 3.3V safe for GPIO18). Interrupt-driven pulse counting updated every 100ms.

| Parameter | Default | Notes |
|-----------|---------|-------|
| Pulses per revolution | 4 | Verify for your bike |
| Wheel circumference | 1.95m | Measure or look up for your tire |

---

## Current Protection

INA219 monitors LED string current via 100mΩ shunt resistor.
If current exceeds the configured maximum, RCD-24 PWM is reduced proportionally.

Default max current: **1.32A** (1.2A rated + 10% margin)

---

## Voltage Monitor

10kΩ / 3.3kΩ divider on 12V rail → D0 ADC pin. Displayed on dashboard for diagnostics only.

---

## Wiring Notes

- I2C pullups: 4.7kΩ SDA and SCL to 3.3V (on PCB only)
- RCD-24 PWM-DIM: 100Ω series resistor + 10kΩ pulldown on driver side (holds driver off during ESP32 boot)
- DS18B20: 4.7kΩ pullup between 3.3V and data line
- Fan: flyback diode across fan terminals, MOSFET gate 10kΩ pulldown
- Speed sensor: 10kΩ/3.3kΩ divider, no separate ground wire needed (PCB ground = motorcycle ground)
- ICM-42688-PC: AP_CS pin tied to 3.3V to enable I2C mode

---

## Web Interface

**AP Mode (default):** Connect to `Cornering-Lights-V2` WiFi, navigate to `http://192.168.5.1`
**Station Mode:** Configure SSID/password, then access via `http://cornering-lights-v2.local`

| Page | URL | Description |
|------|-----|-------------|
| Dashboard | `/` | Live sensor data, LED states, system status |
| Calibration | `/calibrate` | Lean thresholds, hysteresis, sensor offsets |
| Thermal | `/thermal` | Fan and LDD dimming temperature setpoints |
| Configuration | `/config` | WiFi settings, device name |
| OTA Update | `/update` | Web-based firmware update |

---

## OTA Firmware Update

**Web method:** Go to `/update`, select `.bin` file (Arduino IDE: Sketch → Export Compiled Binary)

**Arduino IDE method:** Tools → Port → Network Ports → select device hostname

---

## Required Libraries

Install via Arduino Library Manager:
- `OneWire`
- `DallasTemperature`

Built-in (ESP32 Arduino core):
- `Wire`, `WiFi`, `WebServer`, `ESPmDNS`, `ArduinoOTA`, `Update`, `Preferences`

---

## Calibration Procedure

1. Mount sensors with known spacing, measure center-to-center distance in mm
2. Place bike on center stand (upright) — gyro bias calibration runs automatically at startup (keep bike still for ~1 second after power on)
3. Verify lean angle reads ~0° upright on dashboard
4. Lean bike slightly left, verify positive angle reading
5. Adjust sensor offsets if left/right readings are unequal at same distance
6. Verify speed sensor pulse count and wheel circumference match your bike
7. Set LED thresholds to match desired activation angles
8. Set hysteresis to 2-5° to prevent flickering at threshold boundaries
9. Road test: verify LED activation sequence matches lean angle
10. After bench test, verify correct IMU yaw axis (X/Y/Z) and adjust `imuYawAxis` in config if needed

---

## Default Settings

| Parameter | Default |
|-----------|---------|
| Sensor spacing | 800 mm |
| LED thresholds | 5°, 10°, 20°, 30°, 40° |
| Hysteresis | 3° |
| Fan on temp | 45°C |
| Fan full temp | 65°C |
| Dim start temp | 70°C |
| LDD off temp | 85°C |
| Max current | 1.32A |
| AP password | cornering123 |
| AP IP | 192.168.5.1 |
| Pulses per rev | 4 |
| Wheel circumference | 1.95m |
| IMU yaw axis | Z (index 2) |
| Sensor fusion threshold | 2.0 m/s |
