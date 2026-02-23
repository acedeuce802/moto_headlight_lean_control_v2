/*
 * Motorcycle Cornering Lights System V2
 * Xiao ESP32-C6
 *
 * Hardware:
 * - Xiao ESP32-C6 microcontroller
 * - TCA9548A I2C multiplexer
 * - 2x DFRobot Laser Distance Sensors (SEN0590, address 0x74)
 * - MCP23008 I2C GPIO expander (LED 1-8 MOSFET gates)
 * - INA219 current sensor
 * - DS18B20 temperature sensor (1-Wire)
 * - 10x Samsung LH351D LEDs in series with DMN2041L MOSFET bypasses
 * - LDD-1000H constant current driver (PWM dim control)
 * - Cooling fan (PWM controlled)
 * - 12V voltage monitor (ADC)
 *
 * Pin Assignments (ESP32-C6):
 * - D4  (GPIO22): I2C SDA -> TCA9548A, MCP23008, INA219
 * - D5  (GPIO23): I2C SCL -> TCA9548A, MCP23008, INA219
 * - D6  (GPIO?):  LDD PWM-DIM (100R series, 10k pulldown on LDD side)
 * - D7  (GPIO?):  LED9 MOSFET gate (direct)
 * - D8  (GPIO?):  LED10 MOSFET gate (direct)
 * - D3  (GPIO21): DS18B20 1-Wire data (4.7k pullup to 3.3V)
 * - D10 (GPIO10): Fan PWM
 * - A0  (GPIO2):  12V voltage monitor (10k/3.3k divider)
 *
 * I2C Addresses:
 * - TCA9548A:  0x70
 * - MCP23008:  0x20
 * - INA219:    0x40
 * - Sensors:   0x74 (via TCA9548A channels 0 and 1)
 *
 * LED Assignment:
 * - LED 1-5:  Left side  (LED1 = extreme left, LED5 = center-left)
 * - LED 6-10: Right side (LED6 = center-right, LED10 = extreme right)
 * - MCP23008 GP0-GP7 = LED1-LED8 gates
 * - D7 = LED9 gate, D8 = LED10 gate
 * - Logic: GPIO HIGH = MOSFET ON = LED bypassed (OFF)
 *          GPIO LOW  = MOSFET OFF = LED illuminated (ON)
 *
 * LED Activation Order:
 * - Left turn:  LED5 -> LED4 -> LED3 -> (LED5 off) LED2 -> (LED4 off) LED1
 * - Right turn: LED6 -> LED7 -> LED8 -> (LED6 off) LED9 -> (LED7 off) LED10
 * - Max 3 LEDs on at any time
 *
 * Thermal Management:
 * - DS18B20 monitors heatsink temperature
 * - Fan ramps from 0% to 100% between configurable temps
 * - LDD PWM dims from 100% to 0% between configurable temps
 *
 * Web Server (AP mode, IP: 192.168.5.1):
 * - Dashboard: live diagnostics
 * - Calibration: lean thresholds, hysteresis, sensor offsets
 * - Thermal: fan and dimming temperature setpoints
 * - Configuration: WiFi, device name
 * - OTA: firmware update
 */

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define I2C_SDA             22    // D4
#define I2C_SCL             23    // D5
#define LDD_PWM_PIN         16    // D6
#define LED9_PIN            17    // D7
#define LED10_PIN           19    // D8
#define DS18B20_PIN         21    // D3
#define FAN_PWM_PIN         18    // D10
#define VOLTAGE_ADC_PIN     0     // D0

// ============================================================================
// I2C ADDRESSES
// ============================================================================
#define TCA9548A_ADDR       0x70
#define MCP23008_ADDR       0x20
#define INA219_ADDR         0x40
#define SENSOR_ADDR         0x74

#define LEFT_SENSOR_CH      0
#define RIGHT_SENSOR_CH     1

// ============================================================================
// DFROBOT SENSOR REGISTERS
// ============================================================================
#define SENSOR_START_REG    0x10
#define SENSOR_DATA_REG     0x02
#define SENSOR_START_CMD    0xB0

// ============================================================================
// MCP23008 REGISTERS
// ============================================================================
#define MCP_IODIR           0x00
#define MCP_GPIO            0x09
#define MCP_OLAT            0x0A

// ============================================================================
// INA219 REGISTERS
// ============================================================================
#define INA219_CONFIG       0x00
#define INA219_SHUNT_V      0x01
#define INA219_BUS_V        0x02
#define INA219_POWER        0x03
#define INA219_CURRENT      0x04
#define INA219_CALIB        0x05

// ============================================================================
// PWM CHANNELS
// ============================================================================
#define LDD_PWM_CHANNEL     0
#define FAN_PWM_CHANNEL     1
#define PWM_FREQ            1000
#define PWM_RESOLUTION      8     // 8-bit: 0-255

// ============================================================================
// VOLTAGE DIVIDER
// ============================================================================
#define VDIV_R1             10000.0   // 10k ohm
#define VDIV_R2             3300.0    // 3.3k ohm
#define ADC_REF             3.3
#define ADC_RESOLUTION      4095.0

// ============================================================================
// DEFAULTS
// ============================================================================
#define DEFAULT_LED_THRESHOLDS  {5.0, 10.0, 20.0, 30.0, 40.0}
#define DEFAULT_HYSTERESIS      3.0
#define DEFAULT_SENSOR_SPACING  800.0

#define DEFAULT_FAN_ON_TEMP     45.0   // deg C: fan starts at 20%
#define DEFAULT_FAN_FULL_TEMP   65.0   // deg C: fan at 100%
#define DEFAULT_DIM_START_TEMP  70.0   // deg C: begin dimming LDD
#define DEFAULT_DIM_MAX_TEMP    85.0   // deg C: LDD at 0%

#define DEFAULT_MAX_CURRENT     1.1    // Amps: INA219 overcurrent threshold

// ============================================================================
// CONFIG STRUCT
// ============================================================================
struct Config {
  char deviceName[32];
  char wifiSSID[64];
  char wifiPassword[64];
  bool useAPMode;
  char apPassword[64];

  float sensorSpacing;
  float ledThresholds[5];       // Degrees for LED 1-5 activation
  float hysteresis;

  uint16_t minDistance;
  uint16_t maxDistance;
  uint16_t sampleInterval;

  float leftSensorOffset;
  float rightSensorOffset;

  float fanOnTemp;
  float fanFullTemp;
  float dimStartTemp;
  float dimMaxTemp;

  float maxCurrent;
};

Config config = {
  "Cornering-Lights-V2",
  "",
  "",
  true,
  "cornering123",

  DEFAULT_SENSOR_SPACING,
  DEFAULT_LED_THRESHOLDS,
  DEFAULT_HYSTERESIS,

  50,
  2000,
  100,

  0.0,
  0.0,

  DEFAULT_FAN_ON_TEMP,
  DEFAULT_FAN_FULL_TEMP,
  DEFAULT_DIM_START_TEMP,
  DEFAULT_DIM_MAX_TEMP,

  DEFAULT_MAX_CURRENT
};

// ============================================================================
// SYSTEM STATE
// ============================================================================
struct SystemState {
  // Sensors
  uint16_t leftDistance;
  uint16_t rightDistance;
  bool leftValid;
  bool rightValid;
  float leanAngle;

  // LEDs (true = illuminated)
  bool ledState[10];
  uint8_t activeLedCount;

  // Thermal
  float heatsinkTemp;
  bool tempSensorValid;
  uint8_t fanPwm;           // 0-255
  uint8_t lddPwm;           // 0-255

  // Power
  float current;            // Amps
  float voltage12v;         // Volts

  // Diagnostics
  uint32_t errorCount;
  uint32_t sensorErrorCount;
  unsigned long lastUpdate;
  unsigned long uptime;
  bool systemInitialized;
  bool mcpInitialized;
  bool ina219Initialized;
};

SystemState state = {};

// ============================================================================
// OBJECTS
// ============================================================================
WebServer server(80);
Preferences preferences;
OneWire oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);

unsigned long lastSampleTime = 0;
unsigned long lastTempTime = 0;
unsigned long lastCurrentTime = 0;

// ============================================================================
// TCA9548A MULTIPLEXER
// ============================================================================

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void tcaDisableAll() {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

// ============================================================================
// MCP23008 GPIO EXPANDER
// ============================================================================

bool mcpWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MCP23008_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool initMCP23008() {
  // Set all GP0-GP7 as outputs
  if (!mcpWriteReg(MCP_IODIR, 0x00)) {
    Serial.println("ERROR: MCP23008 init failed");
    return false;
  }
  // Default all outputs HIGH (all LEDs bypassed/off) for safe startup
  if (!mcpWriteReg(MCP_OLAT, 0xFF)) {
    Serial.println("ERROR: MCP23008 output init failed");
    return false;
  }
  Serial.println("MCP23008 initialized");
  return true;
}

// Sets MCP GP0-GP7 (LED1-LED8) gate states
// ledMask bit 0 = LED1, bit 7 = LED8
// 1 = LED ON (MOSFET off), 0 = LED OFF (MOSFET on/bypassed)
void mcpSetLEDs(uint8_t ledOnMask) {
  // Invert: MCP output HIGH = MOSFET on = LED off
  uint8_t mcpVal = ~ledOnMask;
  mcpWriteReg(MCP_OLAT, mcpVal);
}

// ============================================================================
// DFROBOT DISTANCE SENSOR
// ============================================================================

bool sensorWriteReg(uint8_t reg, const void* pBuf, size_t size) {
  if (pBuf == NULL) return false;
  uint8_t* _pBuf = (uint8_t*)pBuf;
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(&reg, 1);
  for (uint16_t i = 0; i < size; i++) Wire.write(_pBuf[i]);
  return (Wire.endTransmission() == 0);
}

uint8_t sensorReadReg(uint8_t reg, void* pBuf, size_t size) {
  if (pBuf == NULL) return 0;
  uint8_t* _pBuf = (uint8_t*)pBuf;
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(&reg, 1);
  if (Wire.endTransmission() != 0) return 0;
  delay(20);
  Wire.requestFrom(SENSOR_ADDR, (uint8_t)size);
  for (uint16_t i = 0; i < size; i++) _pBuf[i] = Wire.read();
  return size;
}

uint16_t readDistance() {
  uint8_t cmd = SENSOR_START_CMD;
  uint8_t buf[2] = {0};
  if (!sensorWriteReg(SENSOR_START_REG, &cmd, 1)) return 0;
  delay(50);
  if (sensorReadReg(SENSOR_DATA_REG, buf, 2) != 2) return 0;
  return buf[0] * 256 + buf[1] + 10;
}

bool initializeSensors() {
  tcaSelect(LEFT_SENSOR_CH);
  delay(10);
  uint16_t testDist = readDistance();
  if (testDist == 0 || testDist > 5000) {
    Serial.println("ERROR: Left sensor init failed");
    return false;
  }
  Serial.printf("Left sensor OK (%d mm)\n", testDist);

  tcaSelect(RIGHT_SENSOR_CH);
  delay(10);
  testDist = readDistance();
  if (testDist == 0 || testDist > 5000) {
    Serial.println("ERROR: Right sensor init failed");
    return false;
  }
  Serial.printf("Right sensor OK (%d mm)\n", testDist);

  tcaDisableAll();
  return true;
}

// ============================================================================
// INA219 CURRENT SENSOR
// ============================================================================

void ina219WriteReg(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

uint16_t ina219ReadReg(uint8_t reg) {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(INA219_ADDR, (uint8_t)2);
  return (Wire.read() << 8) | Wire.read();
}

bool initINA219() {
  // Config: 32V range, +/-320mV shunt, 12-bit, continuous
  ina219WriteReg(INA219_CONFIG, 0x399F);
  // Calibration for 0.1 ohm shunt, max 1.6A
  // Cal = 0.04096 / (0.1 * 0.001) -- current_lsb = 1mA
  ina219WriteReg(INA219_CALIB, 4096);

  // Verify by reading config back
  uint16_t cfg = ina219ReadReg(INA219_CONFIG);
  if (cfg == 0 || cfg == 0xFFFF) {
    Serial.println("ERROR: INA219 init failed");
    return false;
  }
  Serial.println("INA219 initialized");
  return true;
}

float readCurrent() {
  int16_t raw = (int16_t)ina219ReadReg(INA219_CURRENT);
  return raw * 0.001; // 1mA per LSB
}

// ============================================================================
// DS18B20 TEMPERATURE SENSOR
// ============================================================================

void initTempSensor() {
  tempSensor.begin();
  int count = tempSensor.getDeviceCount();
  Serial.printf("DS18B20: %d device(s) found\n", count);
  if (count > 0) {
    tempSensor.setResolution(12);
    state.tempSensorValid = true;
  }
}

float readTemperature() {
  tempSensor.requestTemperatures();
  float t = tempSensor.getTempCByIndex(0);
  if (t == DEVICE_DISCONNECTED_C || t < -50 || t > 150) {
    state.tempSensorValid = false;
    return -99.0;
  }
  state.tempSensorValid = true;
  return t;
}

// ============================================================================
// VOLTAGE MONITOR
// ============================================================================

float readVoltage12v() {
  int raw = analogRead(VOLTAGE_ADC_PIN);
  float vADC = (raw / ADC_RESOLUTION) * ADC_REF;
  return vADC * ((VDIV_R1 + VDIV_R2) / VDIV_R2);
}

// ============================================================================
// LEAN ANGLE CALCULATION
// ============================================================================

float calculateLeanAngle(uint16_t leftDist, uint16_t rightDist) {
  float leftCorrected  = leftDist  + config.leftSensorOffset;
  float rightCorrected = rightDist + config.rightSensorOffset;
  float distDiff = rightCorrected - leftCorrected;
  // Positive = leaning left, negative = leaning right
  return atan2(distDiff, config.sensorSpacing) * (180.0 / PI);
}

// ============================================================================
// LED CONTROL
// ============================================================================

// Left LEDs:  1,2,3,4,5  (index 0-4)  LED5 activates first at smallest angle
// Right LEDs: 6,7,8,9,10 (index 5-9)  LED6 activates first at smallest angle
// thresholds[0..4] map to angles for 1st through 5th LED activation per side
//
// Activation sequence (left turn example):
//   >thresh[0]: LED5 on                           -> 1 LED
//   >thresh[1]: LED4 on                           -> 2 LEDs
//   >thresh[2]: LED3 on                           -> 3 LEDs (max)
//   >thresh[3]: LED5 off, LED2 on                 -> 3 LEDs
//   >thresh[4]: LED4 off, LED1 on                 -> 3 LEDs
//
// Mirror for right: LED6,7,8,9,10

void computeLEDStates(float leanAngle) {
  bool newState[10] = {false};

  auto activateSide = [&](bool isLeft) {
    // LED indices: left = 4,3,2,1,0 (LED5..LED1), right = 5,6,7,8,9 (LED6..LED10)
    int ledOrder[5];
    if (isLeft) {
      ledOrder[0] = 4; ledOrder[1] = 3; ledOrder[2] = 2; ledOrder[3] = 1; ledOrder[4] = 0;
    } else {
      ledOrder[0] = 5; ledOrder[1] = 6; ledOrder[2] = 7; ledOrder[3] = 8; ledOrder[4] = 9;
    }

    float absAngle = fabsf(leanAngle);

    // Determine how many thresholds are exceeded
    int exceeded = 0;
    for (int i = 0; i < 5; i++) {
      if (absAngle >= config.ledThresholds[i]) exceeded = i + 1;
      else break;
    }

    // Max 3 LEDs on; once exceeded >= 4, oldest start turning off
    // Active window of 3 slides along ledOrder as exceeded increases
    if (exceeded == 0) return;

    int startIdx = (exceeded > 3) ? (exceeded - 3) : 0;
    int endIdx   = min(exceeded, 5) - 1;

    for (int i = startIdx; i <= endIdx; i++) {
      newState[ledOrder[i]] = true;
    }
  };

  float onAngle  = config.ledThresholds[0];
  float offAngle = onAngle - config.hysteresis;

  // Determine active direction with hysteresis
  static bool wasLeft  = false;
  static bool wasRight = false;

  if (leanAngle >= onAngle) {
    wasLeft  = true;
    wasRight = false;
  } else if (leanAngle <= -onAngle) {
    wasRight = true;
    wasLeft  = false;
  } else if (leanAngle > -offAngle && leanAngle < offAngle) {
    // Within deadband - turn everything off
    wasLeft  = false;
    wasRight = false;
  }
  // Between offAngle and onAngle: maintain previous state (hysteresis)

  if (wasLeft)  activateSide(true);
  if (wasRight) activateSide(false);

  // Apply new states
  for (int i = 0; i < 10; i++) state.ledState[i] = newState[i];
}

void applyLEDStates() {
  // Build MCP mask for LED1-LED8 (indices 0-7)
  uint8_t mcpMask = 0;
  for (int i = 0; i < 8; i++) {
    if (state.ledState[i]) mcpMask |= (1 << i);
  }
  mcpSetLEDs(mcpMask);

  // LED9 (index 8) and LED10 (index 9) via direct GPIO
  // HIGH = MOSFET on = LED OFF, LOW = MOSFET off = LED ON
  digitalWrite(LED9_PIN,  state.ledState[8] ? LOW : HIGH);
  digitalWrite(LED10_PIN, state.ledState[9] ? LOW : HIGH);

  // Count active LEDs
  state.activeLedCount = 0;
  for (int i = 0; i < 10; i++) if (state.ledState[i]) state.activeLedCount++;
}

void allLEDsOff() {
  for (int i = 0; i < 10; i++) state.ledState[i] = false;
  applyLEDStates();
}

// ============================================================================
// THERMAL MANAGEMENT
// ============================================================================

uint8_t calcFanPwm(float temp) {
  if (temp <= config.fanOnTemp)   return 0;
  if (temp >= config.fanFullTemp) return 255;
  float ratio = (temp - config.fanOnTemp) / (config.fanFullTemp - config.fanOnTemp);
  // Start at 20% (51/255) minimum when fan turns on
  return (uint8_t)(51 + ratio * (255 - 51));
}

uint8_t calcLDDPwm(float temp) {
  if (temp <= config.dimStartTemp) return 255;  // Full brightness
  if (temp >= config.dimMaxTemp)   return 0;    // Off
  float ratio = (temp - config.dimStartTemp) / (config.dimMaxTemp - config.dimStartTemp);
  return (uint8_t)(255 * (1.0 - ratio));
}

void updateThermal() {
  float temp = readTemperature();
  if (state.tempSensorValid) {
    state.heatsinkTemp = temp;
    state.fanPwm = calcFanPwm(temp);
    state.lddPwm = calcLDDPwm(temp);
  } else {
    // Sensor failure: run fan full, don't dim (can't assess risk)
    state.fanPwm = 255;
    state.lddPwm = 255;
  }
  ledcWrite(FAN_PWM_CHANNEL, state.fanPwm);
  ledcWrite(LDD_PWM_CHANNEL, state.lddPwm);
}

// ============================================================================
// CURRENT PROTECTION
// ============================================================================

void updateCurrent() {
  state.current = readCurrent();
  // If overcurrent, dim LDD further (take minimum of thermal and current dim)
  if (state.current > config.maxCurrent) {
    float overcurrentRatio = config.maxCurrent / state.current;
    uint8_t currentPwm = (uint8_t)(state.lddPwm * overcurrentRatio);
    ledcWrite(LDD_PWM_CHANNEL, currentPwm);
  }
}

// ============================================================================
// SENSOR READING (main loop task)
// ============================================================================

void readSensors() {
  tcaSelect(LEFT_SENSOR_CH);
  state.leftDistance = readDistance();
  state.leftValid = (state.leftDistance >= config.minDistance &&
                     state.leftDistance <= config.maxDistance);

  tcaSelect(RIGHT_SENSOR_CH);
  state.rightDistance = readDistance();
  state.rightValid = (state.rightDistance >= config.minDistance &&
                      state.rightDistance <= config.maxDistance);

  tcaDisableAll();

  if (state.leftValid && state.rightValid) {
    state.leanAngle = calculateLeanAngle(state.leftDistance, state.rightDistance);
    computeLEDStates(state.leanAngle);
    applyLEDStates();
  } else {
    allLEDsOff();
    state.sensorErrorCount++;
    state.errorCount++;
  }

  state.lastUpdate = millis();
  state.voltage12v = readVoltage12v();
}

// ============================================================================
// CONFIG: LOAD / SAVE / RESET
// ============================================================================

void loadConfig() {
  preferences.begin("cl_v2", false);
  preferences.getString("deviceName",  config.deviceName,  sizeof(config.deviceName));
  preferences.getString("wifiSSID",    config.wifiSSID,    sizeof(config.wifiSSID));
  preferences.getString("wifiPass",    config.wifiPassword,sizeof(config.wifiPassword));
  config.useAPMode = preferences.getBool("useAPMode", true);
  preferences.getString("apPass",      config.apPassword,  sizeof(config.apPassword));

  config.sensorSpacing    = preferences.getFloat("spacing",   DEFAULT_SENSOR_SPACING);
  config.hysteresis       = preferences.getFloat("hysteresis",DEFAULT_HYSTERESIS);

  float defThresh[5] = DEFAULT_LED_THRESHOLDS;
  char key[16];
  for (int i = 0; i < 5; i++) {
    snprintf(key, sizeof(key), "thresh%d", i);
    config.ledThresholds[i] = preferences.getFloat(key, defThresh[i]);
  }

  config.minDistance    = preferences.getUShort("minDist",    50);
  config.maxDistance    = preferences.getUShort("maxDist",    2000);
  config.sampleInterval = preferences.getUShort("sampleInt",  100);
  config.leftSensorOffset  = preferences.getFloat("leftOff",  0.0);
  config.rightSensorOffset = preferences.getFloat("rightOff", 0.0);

  config.fanOnTemp    = preferences.getFloat("fanOn",    DEFAULT_FAN_ON_TEMP);
  config.fanFullTemp  = preferences.getFloat("fanFull",  DEFAULT_FAN_FULL_TEMP);
  config.dimStartTemp = preferences.getFloat("dimStart", DEFAULT_DIM_START_TEMP);
  config.dimMaxTemp   = preferences.getFloat("dimMax",   DEFAULT_DIM_MAX_TEMP);
  config.maxCurrent   = preferences.getFloat("maxCurr",  DEFAULT_MAX_CURRENT);

  preferences.end();
  Serial.println("Config loaded");
}

void saveConfig() {
  preferences.begin("cl_v2", false);
  preferences.putString("deviceName",  config.deviceName);
  preferences.putString("wifiSSID",    config.wifiSSID);
  preferences.putString("wifiPass",    config.wifiPassword);
  preferences.putBool("useAPMode",     config.useAPMode);
  preferences.putString("apPass",      config.apPassword);

  preferences.putFloat("spacing",      config.sensorSpacing);
  preferences.putFloat("hysteresis",   config.hysteresis);

  char key[16];
  for (int i = 0; i < 5; i++) {
    snprintf(key, sizeof(key), "thresh%d", i);
    preferences.putFloat(key, config.ledThresholds[i]);
  }

  preferences.putUShort("minDist",     config.minDistance);
  preferences.putUShort("maxDist",     config.maxDistance);
  preferences.putUShort("sampleInt",   config.sampleInterval);
  preferences.putFloat("leftOff",      config.leftSensorOffset);
  preferences.putFloat("rightOff",     config.rightSensorOffset);

  preferences.putFloat("fanOn",        config.fanOnTemp);
  preferences.putFloat("fanFull",      config.fanFullTemp);
  preferences.putFloat("dimStart",     config.dimStartTemp);
  preferences.putFloat("dimMax",       config.dimMaxTemp);
  preferences.putFloat("maxCurr",      config.maxCurrent);

  preferences.end();
  Serial.println("Config saved");
}

void resetConfig() {
  preferences.begin("cl_v2", false);
  preferences.clear();
  preferences.end();
  ESP.restart();
}

// ============================================================================
// WIFI
// ============================================================================

void setupWiFi() {
  if (config.useAPMode || strlen(config.wifiSSID) == 0) {
    WiFi.mode(WIFI_AP);
    IPAddress local_IP(192, 168, 5, 1);
    IPAddress gateway(192, 168, 5, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(config.deviceName, config.apPassword);
    Serial.printf("AP started: %s  IP: %s\n", config.deviceName, WiFi.softAPIP().toString().c_str());
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.begin(config.wifiSSID, config.wifiPassword);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500); Serial.print("."); attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("\nWiFi connected: %s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("\nWiFi failed, falling back to AP");
      config.useAPMode = true;
      WiFi.mode(WIFI_AP);
      IPAddress local_IP(192, 168, 5, 1);
      IPAddress gateway(192, 168, 5, 1);
      IPAddress subnet(255, 255, 255, 0);
      WiFi.softAPConfig(local_IP, gateway, subnet);
      WiFi.softAP(config.deviceName, config.apPassword);
    }
  }
  if (MDNS.begin(config.deviceName)) {
    Serial.printf("mDNS: http://%s.local\n", config.deviceName);
  }
}

// ============================================================================
// SHARED HTML HELPERS
// ============================================================================

String htmlHeader(const String& title) {
  String h = "<!DOCTYPE html><html><head>";
  h += "<meta charset='UTF-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>" + title + " - " + String(config.deviceName) + "</title>";
  h += "<style>";
  h += "body{font-family:Arial,sans-serif;margin:0;background:#1a1a2e;color:#eee;}";
  h += ".wrap{max-width:900px;margin:0 auto;padding:20px;}";
  h += "h1{color:#e94560;margin-bottom:5px;}";
  h += "h2{color:#0f3460;background:#e94560;padding:8px 12px;border-radius:5px;font-size:15px;margin-top:25px;}";
  h += "nav{margin:15px 0;}nav a{color:#e94560;text-decoration:none;margin-right:18px;font-weight:bold;}";
  h += "nav a:hover{text-decoration:underline;}";
  h += ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:12px;margin:15px 0;}";
  h += ".card{background:#16213e;padding:14px;border-radius:8px;border-left:4px solid #0f3460;}";
  h += ".card.on{border-left-color:#4CAF50;background:#1a2e1a;}";
  h += ".card.warn{border-left-color:#FFC107;background:#2e2a1a;}";
  h += ".card.err{border-left-color:#f44336;background:#2e1a1a;}";
  h += ".card h3{margin:0 0 8px;font-size:12px;color:#aaa;text-transform:uppercase;}";
  h += ".card .val{font-size:26px;font-weight:bold;color:#fff;}";
  h += ".card .unit{font-size:12px;color:#888;}";
  h += ".led-row{display:flex;gap:6px;margin:12px 0;flex-wrap:wrap;}";
  h += ".led{width:40px;height:40px;border-radius:50%;background:#333;display:flex;align-items:center;justify-content:center;font-size:11px;font-weight:bold;color:#aaa;border:2px solid #444;}";
  h += ".led.on{background:#ffcc00;color:#333;border-color:#ffaa00;box-shadow:0 0 10px #ffcc0088;}";
  h += "label{display:block;margin:12px 0 4px;font-size:13px;color:#ccc;}";
  h += "input[type=number],input[type=text],input[type=password]{width:100%;padding:8px;background:#16213e;border:1px solid #0f3460;border-radius:4px;color:#fff;box-sizing:border-box;}";
  h += "button{background:#e94560;color:#fff;border:none;padding:10px 22px;border-radius:5px;cursor:pointer;margin:5px 5px 0 0;font-size:14px;}";
  h += "button:hover{background:#c73652;}";
  h += "button.sec{background:#0f3460;}button.sec:hover{background:#1a4a8a;}";
  h += "button.danger{background:#7a1a1a;}button.danger:hover{background:#a02020;}";
  h += ".info{background:#0f3460;padding:12px;border-radius:5px;margin:12px 0;font-size:13px;}";
  h += ".row2{display:grid;grid-template-columns:1fr 1fr;gap:12px;}";
  h += "@media(max-width:500px){.row2{grid-template-columns:1fr;}}";
  h += "</style></head><body><div class='wrap'>";
  h += "<h1>🏍️ " + String(config.deviceName) + "</h1>";
  h += "<nav>";
  h += "<a href='/'>Dashboard</a>";
  h += "<a href='/calibrate'>Calibration</a>";
  h += "<a href='/thermal'>Thermal</a>";
  h += "<a href='/config'>Config</a>";
  h += "<a href='/update'>OTA</a>";
  h += "</nav>";
  return h;
}

String htmlFooter() {
  return "</div></body></html>";
}

// ============================================================================
// DASHBOARD
// ============================================================================

void handleRoot() {
  String html = htmlHeader("Dashboard");

  // Live update script
  html += "<script>";
  html += "setInterval(function(){";
  html += "fetch('/api/status').then(r=>r.json()).then(d=>{";
  html += "document.getElementById('lDist').innerText=d.leftDistance;";
  html += "document.getElementById('rDist').innerText=d.rightDistance;";
  html += "document.getElementById('lean').innerText=d.leanAngle.toFixed(1);";
  html += "document.getElementById('temp').innerText=d.temp.toFixed(1);";
  html += "document.getElementById('curr').innerText=d.current.toFixed(2);";
  html += "document.getElementById('volt').innerText=d.voltage.toFixed(1);";
  html += "document.getElementById('fan').innerText=Math.round(d.fanPwm/2.55)+'%';";
  html += "document.getElementById('lddpwm').innerText=Math.round(d.lddPwm/2.55)+'%';";
  html += "document.getElementById('errs').innerText=d.errors;";
  html += "document.getElementById('uptime').innerText=Math.floor(d.uptime/1000)+'s';";
  html += "for(var i=1;i<=10;i++){";
  html += "var el=document.getElementById('led'+i);";
  html += "el.className=d.leds[i-1]?'led on':'led';";
  html += "}";
  html += "});";
  html += "},400);";
  html += "</script>";

  html += "<h2>Sensors</h2><div class='grid'>";
  html += "<div class='card'><h3>Left Distance</h3><div class='val' id='lDist'>" + String(state.leftDistance) + "</div><div class='unit'>mm</div></div>";
  html += "<div class='card'><h3>Right Distance</h3><div class='val' id='rDist'>" + String(state.rightDistance) + "</div><div class='unit'>mm</div></div>";
  html += "<div class='card'><h3>Lean Angle</h3><div class='val' id='lean'>" + String(state.leanAngle, 1) + "</div><div class='unit'>degrees (+left/-right)</div></div>";
  html += "</div>";

  html += "<h2>LEDs</h2>";
  html += "<div class='led-row'>";
  for (int i = 1; i <= 10; i++) {
    html += "<div class='led" + String(state.ledState[i-1] ? " on" : "") + "' id='led" + String(i) + "'>" + String(i) + "</div>";
  }
  html += "</div>";
  html += "<div style='font-size:12px;color:#888;'>1-5: Left &nbsp;|&nbsp; 6-10: Right</div>";

  html += "<h2>System</h2><div class='grid'>";
  html += "<div class='card'><h3>Heatsink Temp</h3><div class='val' id='temp'>" + String(state.heatsinkTemp, 1) + "</div><div class='unit'>°C</div></div>";
  html += "<div class='card'><h3>Fan Speed</h3><div class='val' id='fan'>" + String(state.fanPwm * 100 / 255) + "%</div><div class='unit'>PWM</div></div>";
  html += "<div class='card'><h3>LDD Brightness</h3><div class='val' id='lddpwm'>" + String(state.lddPwm * 100 / 255) + "%</div><div class='unit'>PWM</div></div>";
  html += "<div class='card'><h3>LED Current</h3><div class='val' id='curr'>" + String(state.current, 2) + "</div><div class='unit'>A</div></div>";
  html += "<div class='card'><h3>12V Rail</h3><div class='val' id='volt'>" + String(state.voltage12v, 1) + "</div><div class='unit'>V</div></div>";
  html += "<div class='card'><h3>Errors</h3><div class='val' id='errs'>" + String(state.errorCount) + "</div></div>";
  html += "<div class='card'><h3>Uptime</h3><div class='val' id='uptime'>" + String(millis()/1000) + "s</div></div>";
  html += "</div>";

  html += htmlFooter();
  server.send(200, "text/html", html);
}

// ============================================================================
// API STATUS
// ============================================================================

void handleAPIStatus() {
  String json = "{";
  json += "\"leftDistance\":" + String(state.leftDistance) + ",";
  json += "\"rightDistance\":" + String(state.rightDistance) + ",";
  json += "\"leftValid\":" + String(state.leftValid?"true":"false") + ",";
  json += "\"rightValid\":" + String(state.rightValid?"true":"false") + ",";
  json += "\"leanAngle\":" + String(state.leanAngle, 2) + ",";
  json += "\"temp\":" + String(state.heatsinkTemp, 1) + ",";
  json += "\"tempValid\":" + String(state.tempSensorValid?"true":"false") + ",";
  json += "\"fanPwm\":" + String(state.fanPwm) + ",";
  json += "\"lddPwm\":" + String(state.lddPwm) + ",";
  json += "\"current\":" + String(state.current, 3) + ",";
  json += "\"voltage\":" + String(state.voltage12v, 2) + ",";
  json += "\"errors\":" + String(state.errorCount) + ",";
  json += "\"uptime\":" + String(millis()) + ",";
  json += "\"leds\":[";
  for (int i = 0; i < 10; i++) {
    json += String(state.ledState[i]?"true":"false");
    if (i < 9) json += ",";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

// ============================================================================
// CALIBRATION PAGE
// ============================================================================

void handleCalibrate() {
  String html = htmlHeader("Calibration");
  html += "<form action='/api/calibrate' method='POST'>";
  html += "<h2>Lean Angle Thresholds</h2>";
  html += "<div class='info'>Each threshold is the lean angle (degrees) at which the next LED activates. Max 3 LEDs on at once - older LEDs turn off as newer ones activate.</div>";
  html += "<div class='row2'>";
  const char* ledLabels[] = {"LED 5/6 (1st)", "LED 4/7 (2nd)", "LED 3/8 (3rd)", "LED 2/9 (4th)", "LED 1/10 (5th)"};
  for (int i = 0; i < 5; i++) {
    html += "<div><label>" + String(ledLabels[i]) + " threshold (°)</label>";
    html += "<input type='number' name='thresh" + String(i) + "' step='0.5' value='" + String(config.ledThresholds[i], 1) + "'></div>";
  }
  html += "</div>";
  html += "<label>Hysteresis (°)</label>";
  html += "<input type='number' name='hysteresis' step='0.5' value='" + String(config.hysteresis, 1) + "'>";
  html += "<div class='info'>LEDs turn off at (threshold - hysteresis) to prevent flickering at the boundary.</div>";

  html += "<h2>Sensor Settings</h2>";
  html += "<div class='row2'>";
  html += "<div><label>Sensor Spacing (mm)</label><input type='number' name='spacing' step='1' value='" + String(config.sensorSpacing, 0) + "'></div>";
  html += "<div><label>Sample Interval (ms)</label><input type='number' name='sampleInt' step='10' value='" + String(config.sampleInterval) + "'></div>";
  html += "<div><label>Left Sensor Offset (mm)</label><input type='number' name='leftOff' step='0.1' value='" + String(config.leftSensorOffset, 1) + "'></div>";
  html += "<div><label>Right Sensor Offset (mm)</label><input type='number' name='rightOff' step='0.1' value='" + String(config.rightSensorOffset, 1) + "'></div>";
  html += "<div><label>Min Valid Distance (mm)</label><input type='number' name='minDist' step='1' value='" + String(config.minDistance) + "'></div>";
  html += "<div><label>Max Valid Distance (mm)</label><input type='number' name='maxDist' step='1' value='" + String(config.maxDistance) + "'></div>";
  html += "</div>";

  html += "<button type='submit'>Save Calibration</button>";
  html += "</form>";
  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleAPICalibrate() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  for (int i = 0; i < 5; i++) {
    char key[10]; snprintf(key, sizeof(key), "thresh%d", i);
    if (server.hasArg(key)) config.ledThresholds[i] = server.arg(key).toFloat();
  }
  if (server.hasArg("hysteresis")) config.hysteresis       = server.arg("hysteresis").toFloat();
  if (server.hasArg("spacing"))    config.sensorSpacing    = server.arg("spacing").toFloat();
  if (server.hasArg("sampleInt"))  config.sampleInterval   = server.arg("sampleInt").toInt();
  if (server.hasArg("leftOff"))    config.leftSensorOffset  = server.arg("leftOff").toFloat();
  if (server.hasArg("rightOff"))   config.rightSensorOffset = server.arg("rightOff").toFloat();
  if (server.hasArg("minDist"))    config.minDistance       = server.arg("minDist").toInt();
  if (server.hasArg("maxDist"))    config.maxDistance       = server.arg("maxDist").toInt();
  saveConfig();
  server.sendHeader("Location", "/calibrate");
  server.send(303);
}

// ============================================================================
// THERMAL PAGE
// ============================================================================

void handleThermal() {
  String html = htmlHeader("Thermal");
  html += "<form action='/api/thermal' method='POST'>";
  html += "<h2>Fan Control</h2>";
  html += "<div class='info'>Fan ramps linearly from 20% at Fan On Temp to 100% at Fan Full Temp.</div>";
  html += "<div class='row2'>";
  html += "<div><label>Fan On Temp (°C)</label><input type='number' name='fanOn' step='1' value='" + String(config.fanOnTemp, 0) + "'></div>";
  html += "<div><label>Fan Full Speed Temp (°C)</label><input type='number' name='fanFull' step='1' value='" + String(config.fanFullTemp, 0) + "'></div>";
  html += "</div>";

  html += "<h2>LDD Dimming</h2>";
  html += "<div class='info'>LDD PWM ramps linearly from 100% at Dim Start Temp down to 0% at Max Temp.</div>";
  html += "<div class='row2'>";
  html += "<div><label>Dim Start Temp (°C)</label><input type='number' name='dimStart' step='1' value='" + String(config.dimStartTemp, 0) + "'></div>";
  html += "<div><label>Max Temp / LDD Off (°C)</label><input type='number' name='dimMax' step='1' value='" + String(config.dimMaxTemp, 0) + "'></div>";
  html += "</div>";

  html += "<h2>Current Protection</h2>";
  html += "<div class='info'>If INA219 reads above Max Current, LDD is dimmed proportionally.</div>";
  html += "<label>Max Current (A)</label>";
  html += "<input type='number' name='maxCurr' step='0.05' value='" + String(config.maxCurrent, 2) + "'>";

  html += "<br><button type='submit'>Save Thermal Settings</button>";
  html += "</form>";
  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleAPIThermal() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  if (server.hasArg("fanOn"))    config.fanOnTemp    = server.arg("fanOn").toFloat();
  if (server.hasArg("fanFull"))  config.fanFullTemp  = server.arg("fanFull").toFloat();
  if (server.hasArg("dimStart")) config.dimStartTemp = server.arg("dimStart").toFloat();
  if (server.hasArg("dimMax"))   config.dimMaxTemp   = server.arg("dimMax").toFloat();
  if (server.hasArg("maxCurr"))  config.maxCurrent   = server.arg("maxCurr").toFloat();
  saveConfig();
  server.sendHeader("Location", "/thermal");
  server.send(303);
}

// ============================================================================
// CONFIG PAGE
// ============================================================================

void handleConfig() {
  String html = htmlHeader("Configuration");
  html += "<form action='/api/config' method='POST'>";
  html += "<h2>Device</h2>";
  html += "<label>Device Name (AP SSID / mDNS)</label>";
  html += "<input type='text' name='deviceName' value='" + String(config.deviceName) + "' maxlength='31'>";
  html += "<h2>WiFi</h2>";
  html += "<label>WiFi SSID (leave empty for AP mode)</label>";
  html += "<input type='text' name='wifiSSID' value='" + String(config.wifiSSID) + "'>";
  html += "<label>WiFi Password</label>";
  html += "<input type='password' name='wifiPassword' value='" + String(config.wifiPassword) + "'>";
  html += "<label>AP Mode Password</label>";
  html += "<input type='password' name='apPassword' value='" + String(config.apPassword) + "'>";
  html += "<br><button type='submit'>Save & Restart</button>";
  html += "<button type='button' class='danger' onclick=\"if(confirm('Reset ALL settings to defaults?'))window.location='/api/reset';\">Factory Reset</button>";
  html += "</form>";
  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleAPIConfig() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  if (server.hasArg("deviceName"))   strncpy(config.deviceName,   server.arg("deviceName").c_str(),   sizeof(config.deviceName)-1);
  if (server.hasArg("wifiSSID"))     strncpy(config.wifiSSID,     server.arg("wifiSSID").c_str(),     sizeof(config.wifiSSID)-1);
  if (server.hasArg("wifiPassword")) strncpy(config.wifiPassword, server.arg("wifiPassword").c_str(), sizeof(config.wifiPassword)-1);
  if (server.hasArg("apPassword"))   strncpy(config.apPassword,   server.arg("apPassword").c_str(),   sizeof(config.apPassword)-1);
  config.useAPMode = (strlen(config.wifiSSID) == 0);
  saveConfig();
  server.send(200, "text/html", "<html><body style='background:#1a1a2e;color:#eee;font-family:Arial'><h2>Saved! Restarting...</h2><script>setTimeout(()=>window.location='/',6000)</script></body></html>");
  delay(1000);
  ESP.restart();
}

void handleAPIReset() {
  resetConfig(); // restarts inside
}

// ============================================================================
// OTA UPDATE PAGE
// ============================================================================

void handleUpdate() {
  String html = htmlHeader("OTA Update");
  html += "<h2>Web Upload</h2>";
  html += "<p>Export .bin from Arduino IDE: Sketch → Export Compiled Binary</p>";
  html += "<form id='upForm' enctype='multipart/form-data'>";
  html += "<input type='file' id='file' name='update' accept='.bin' style='width:100%;padding:10px;background:#16213e;color:#eee;border:1px solid #0f3460;border-radius:4px;margin-bottom:10px;'>";
  html += "<button type='submit'>Upload Firmware</button>";
  html += "</form>";
  html += "<div id='prog' style='display:none;margin:15px 0;'><div style='background:#0f3460;border-radius:5px;overflow:hidden;'><div id='bar' style='height:28px;background:#e94560;width:0%;transition:width 0.3s;text-align:center;line-height:28px;color:#fff;'>0%</div></div></div>";
  html += "<div id='msg' style='display:none;padding:12px;border-radius:5px;margin:15px 0;'></div>";
  html += "<script>";
  html += "document.getElementById('upForm').addEventListener('submit',function(e){";
  html += "e.preventDefault();var f=document.getElementById('file').files[0];";
  html += "if(!f){alert('Select a file');return;}";
  html += "var fd=new FormData();fd.append('update',f);";
  html += "var x=new XMLHttpRequest();";
  html += "x.upload.onprogress=function(e){if(e.lengthComputable){var p=Math.round(e.loaded/e.total*100);";
  html += "document.getElementById('prog').style.display='block';";
  html += "document.getElementById('bar').style.width=p+'%';document.getElementById('bar').innerText=p+'%';}};";
  html += "x.onload=function(){var m=document.getElementById('msg');m.style.display='block';";
  html += "if(x.status===200){m.style.background='#1a2e1a';m.innerHTML='<b>✓ Update successful!</b> Restarting...';setTimeout(()=>window.location='/',12000);}";
  html += "else{m.style.background='#2e1a1a';m.innerHTML='<b>✗ Update failed:</b> '+x.responseText;}};";
  html += "x.open('POST','/updateUpload',true);x.send(fd);});";
  html += "</script>";
  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleUpdateUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("[OTA] Start: %s\n", upload.filename.c_str());
    allLEDsOff();
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) Serial.printf("[OTA] Done: %u bytes\n", upload.totalSize);
    else Update.printError(Serial);
  }
}

void handleUpdateComplete() {
  if (Update.hasError()) {
    server.send(500, "text/plain", Update.errorString());
  } else {
    server.send(200, "text/plain", "OK");
    delay(1000);
    ESP.restart();
  }
}

// ============================================================================
// WEB SERVER SETUP
// ============================================================================

void setupWebServer() {
  server.on("/",               handleRoot);
  server.on("/calibrate",      handleCalibrate);
  server.on("/thermal",        handleThermal);
  server.on("/config",         handleConfig);
  server.on("/update",  HTTP_GET,  handleUpdate);
  server.on("/updateUpload", HTTP_POST, handleUpdateComplete, handleUpdateUpload);
  server.on("/api/status",     handleAPIStatus);
  server.on("/api/calibrate",  handleAPICalibrate);
  server.on("/api/thermal",    handleAPIThermal);
  server.on("/api/config",     handleAPIConfig);
  server.on("/api/reset",      handleAPIReset);
  server.begin();
  Serial.println("Web server started");
}

// ============================================================================
// OTA (Arduino IDE)
// ============================================================================

void setupOTA() {
  ArduinoOTA.setHostname(config.deviceName);
  ArduinoOTA.onStart([]() { allLEDsOff(); Serial.println("[OTA] Start"); });
  ArduinoOTA.onEnd([]()   { Serial.println("[OTA] Done"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    Serial.printf("[OTA] %u%%\r", p / (t / 100));
  });
  ArduinoOTA.onError([](ota_error_t e) {
    Serial.printf("[OTA] Error %u\n", e);
  });
  ArduinoOTA.begin();
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================");
  Serial.println("Motorcycle Cornering Lights V2");
  Serial.println("========================================");

  // Output pins
  pinMode(LED9_PIN,  OUTPUT); digitalWrite(LED9_PIN,  HIGH); // High = LED off
  pinMode(LED10_PIN, OUTPUT); digitalWrite(LED10_PIN, HIGH);
  pinMode(FAN_PWM_PIN, OUTPUT);

  // PWM channels
  ledcSetup(LDD_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LDD_PWM_PIN, LDD_PWM_CHANNEL);
  ledcWrite(LDD_PWM_CHANNEL, 255);  // Full brightness initially

  ledcSetup(FAN_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(FAN_PWM_PIN, FAN_PWM_CHANNEL);
  ledcWrite(FAN_PWM_CHANNEL, 0);    // Fan off initially

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);   // 0-3.3V range

  // Load config
  loadConfig();

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // Init peripherals
  state.mcpInitialized = initMCP23008();
  state.ina219Initialized = initINA219();
  initTempSensor();
  state.systemInitialized = initializeSensors();

  if (!state.systemInitialized) {
    Serial.println("WARNING: Distance sensors failed - LEDs will not activate");
  }

  // WiFi and services
  setupWiFi();
  setupWebServer();
  setupOTA();

  state.uptime = millis();
  Serial.println("System ready!");
  Serial.printf("Access: http://%s.local  or  http://192.168.5.1\n", config.deviceName);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  ArduinoOTA.handle();
  server.handleClient();

  unsigned long now = millis();

  // Sensor read + LED update
  if (state.systemInitialized && (now - lastSampleTime >= config.sampleInterval)) {
    lastSampleTime = now;
    readSensors();
  }

  // Temperature + fan + LDD dim (every 2 seconds)
  if (now - lastTempTime >= 2000) {
    lastTempTime = now;
    updateThermal();
  }

  // Current monitor (every 500ms)
  if (state.ina219Initialized && (now - lastCurrentTime >= 500)) {
    lastCurrentTime = now;
    updateCurrent();
  }

  state.uptime = now;
  delay(1);
}
