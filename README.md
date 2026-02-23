# Motorcycle Cornering Lights V2

ESP32-C6 based motorcycle cornering light system with 10 individually controlled LEDs, lean angle detection via distance sensors, thermal management, and current protection.

---

## Hardware

### Microcontroller
- **Xiao ESP32-C6**

### Sensors
- 2x DFRobot Laser Distance Sensor SEN0590 (I2C address 0x74, via TCA9548A)
- DS18B20 waterproof temperature probe (1-Wire)
- INA219 current sensor (I2C address 0x40)
- 12V voltage divider (ADC)

### LED Driver
- Meanwell LDD-1000H constant current driver (1A output)
- 10x Samsung LH351D LEDs in series
- 10x DMN2041L N-channel MOSFETs (bypass switching)
- MCP23008 I2C GPIO expander (LED 1-8 gates, address 0x20)
- TCA9548A I2C multiplexer (address 0x70, for distance sensors)

### Cooling
- 5V PWM fan (25-30mm, ball bearing recommended)

---

## Pin Assignments

| Pin | GPIO | Function |
|-----|------|----------|
| D4  | 22   | I2C SDA (TCA9548A, MCP23008, INA219) |
| D5  | 23   | I2C SCL |
| D6  | 16   | LDD PWM-DIM (100Ω series, 10kΩ pulldown on LDD side) |
| D7  | 17   | LED9 MOSFET gate (direct) |
| D8  | 19   | LED10 MOSFET gate (direct) |
| D3  | 21   | DS18B20 1-Wire data (4.7kΩ pullup to 3.3V) |
| D10 | 18   | Fan PWM |
| D0  | 0    | 12V voltage monitor |

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

Gates controlled by MCP23008 GP0-GP7 = LED1-LED8. LED9/LED10 via direct GPIO.

**Gate logic:** GPIO HIGH = MOSFET ON = LED bypassed (OFF). GPIO LOW = MOSFET OFF = LED illuminated (ON).

**Failsafe hardware:** Gates 3, 4, 7, 8 have 10kΩ pulldown resistors (default ON in hardware failsafe). Gates 1, 2, 5, 6, 9, 10 have 100kΩ pullup resistors (default OFF on ESP32 crash).

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

Hysteresis applies to the entry threshold only - LEDs turn off at (threshold[0] - hysteresis).

---

## Thermal Management

| Stage | Temp Range | Action |
|-------|-----------|--------|
| Fan off | < Fan On Temp | Fan 0% |
| Fan ramp | Fan On → Fan Full Temp | Fan 20%→100% linear |
| LDD ramp down | Dim Start → Max Temp | LDD 100%→0% linear |

If temperature sensor fails, fan runs at 100% as a precaution.

---

## Current Protection

INA219 monitors LED string current via 0.1Ω shunt resistor (LVK24R100CER).  
If current exceeds the configured maximum, LDD PWM is reduced proportionally.

---

## Voltage Monitor

10kΩ / 3.3kΩ divider on 12V rail → A0 ADC pin. Displayed on dashboard for diagnostics only.

---

## Wiring Notes

- I2C pullups: 4.7kΩ SDA and SCL to 3.3V (on controller PCB only)
- LDD PWM-DIM: 100Ω series resistor + 10kΩ pulldown on LDD side (holds LDD off during ESP32 boot)
- DS18B20: 4.7kΩ pullup between 3.3V and data line
- Fan: Flyback diode (1N5819) across fan terminals, MOSFET gate pulldown 10kΩ

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
2. Place bike on center stand (upright) and note lean angle reads ~0°
3. Lean bike slightly left, verify positive angle reading
4. Adjust sensor offsets if left/right readings are unequal at same distance
5. Set LED thresholds to match desired activation angles
6. Set hysteresis to 2-5° to prevent flickering at threshold boundaries
7. Road test: verify LED activation sequence matches lean angle

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
| Max current | 1.1A |
| AP password | cornering123 |
| AP IP | 192.168.5.1 |
