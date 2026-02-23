// ========================================================================================
// MegaHc160Storm32.ino  - RC Lawn Mower By TN MOWER
// ควบคุมรถบังคับตัดหญ้า Mega2560 + FlySky IBUS + HC160AS2 + Storm32BGC
// ========================================================================================

#define DEBUG_SERIAL 1  // 1=เปิด 0 =ปิด : DEBUG_SERIAL
#if DEBUG_SERIAL
#define DBG(x) Serial.print(x)
#define DBGL(x) Serial.println(x)
#else
#define DBG(x)
#define DBGL(x)
#endif
#define TELEMETRY_CSV 1         // 1=เปิด 0=ปิด : CSV telemetry
#define TELEMETRY_PERIOD_MS 50  // 15 Hz (เหมาะกับ plot โหลดหนัก)
#define TEST_MODE 0             // 1=ทดสอบ ไม่มีเซ็นเซฮร์ , 0=สนามจริง มีเซ็นเซอร์

#include <IBusBM.h>
#include <Servo.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include "Storm32Controller.h"
#include "SafetyManager.h"
#include <Adafruit_ADS1X15.h>

// ----- MAX31865 (PT100) -----
constexpr uint8_t MAX_CS_L = 49;
constexpr uint8_t MAX_CS_R = 48;

Adafruit_MAX31865 maxL(MAX_CS_L);
Adafruit_MAX31865 maxR(MAX_CS_R);

// PT100 constants
constexpr float RTD_RNOMINAL = 100.0f;
constexpr float RTD_RREF = 430.0f;

// ============================================================================
// HARDWARE PIN MAP
// ============================================================================
constexpr uint8_t PIN_HW_WD_HB = 34;    // HEARTBEAT PIN สำหรับ External Watchdog
constexpr uint8_t PIN_DRV_ENABLE = 33;  // NEW: driver enable (active HIGH)

// ----- Motor / Drive -----
constexpr uint8_t PWM_L = 5;
constexpr uint8_t PWM_R = 6;
constexpr uint8_t DIR_L1 = 22;
constexpr uint8_t DIR_L2 = 23;
constexpr uint8_t DIR_R1 = 24;
constexpr uint8_t DIR_R2 = 25;

// ----- Cooling Fan -----
constexpr uint8_t FAN_L = 44;
constexpr uint8_t FAN_R = 45;

// ----- Servo -----
constexpr uint8_t SERVO_ENGINE_PIN = 9;

// ----- Analog Sensors -----
constexpr uint8_t PIN_CUR_L1 = A0;
constexpr uint8_t PIN_CUR_L2 = A1;
constexpr uint8_t PIN_CUR_R1 = A2;
constexpr uint8_t PIN_CUR_R2 = A3;

// ----- Digital I/O -----
constexpr uint8_t RELAY_IGNITION = 28;
constexpr uint8_t RELAY_STARTER = 29;
constexpr uint8_t PIN_BUZZER = 30;
constexpr uint8_t RELAY_WARN = 31;
constexpr uint8_t PIN_CUR_TRIP = 32;  // HW overcurrent comparator (active LOW)

// ============================================================================
// IBUS CHANNEL MAP
// ============================================================================
constexpr uint8_t CH_STEER = 1;
constexpr uint8_t CH_THROTTLE = 2;
constexpr uint8_t CH_ENGINE = 8;
constexpr uint8_t CH_IGNITION = 6;
constexpr uint8_t CH_RESET = 7;
constexpr uint8_t CH_STARTER = 10;

// ===== Time Budget (ms) =====
#define BUDGET_SENSORS_MS 5
#define BUDGET_COMMS_MS 3
#define BUDGET_DRIVE_MS 2
#define BUDGET_BLADE_MS 2
#define BUDGET_LOOP_MS 20

// ============================================================================
// FUNCTION PROTOTYPES (FINAL / MATCH ARDUINO ABI)
// ============================================================================
bool readDriverTempsPT100(int &tL, int &tR);
bool readEngineVoltageADS1115(float &vOut);
float readCurrentADC(uint8_t adsChannel, uint8_t idx);
void updateSensors(void);

void updateDriverFans(void);
void logicWatchdog(unsigned long now);

// ============================================================================
// SYSTEM LIMITS / THRESHOLDS
// ============================================================================

// ----- Current -----
const int16_t CUR_WARN_A = 55;
const int16_t CUR_LIMP_A = 75;
constexpr float CUR_SPIKE_A = 120.0f;

constexpr int16_t CUR_MIN_PLAUSIBLE = -10;
constexpr int16_t CUR_MAX_PLAUSIBLE = 150;
constexpr float CUR_LPF_ALPHA = 0.12f;  // tuned for 15kHz PWM

// ----- Temperature -----
constexpr int16_t TEMP_WARN_C = 70;
constexpr int16_t TEMP_LIMP_C = 85;
constexpr int16_t TEMP_TRIP_C = 95;

// ----- Timing -----
constexpr uint32_t IBUS_TIMEOUT_MS = 300;
constexpr uint32_t LOGIC_WDT_MS = 500;
constexpr uint32_t LIMP_RECOVER_MS = 1000;
constexpr uint32_t DRIVE_SOFT_STOP_TIMEOUT_MS = 1500;
constexpr uint32_t BLADE_SOFT_STOP_TIMEOUT_MS = 3000;
constexpr uint32_t ENGINE_RESTART_GUARD_MS = 3000;

constexpr int16_t CUR_TRIP_A_CH[4] = {
  100,  // L1
  100,  // L2
  90,   // R1
  90    // R2
};

// ============================================================================
// ENGINE VOLTAGE DETECTION
// ============================================================================
constexpr float ENGINE_RUNNING_VOLT = 27.2f;
constexpr float ENGINE_STOP_VOLT = 25.5f;
constexpr uint32_t ENGINE_CONFIRM_MS = 900;

constexpr uint8_t MAX_AUTO_REVERSE = 2;
uint8_t autoReverseCount = 0;

enum class FaultCode : uint8_t {
  NONE,
  // ===== Communication =====
  IBUS_LOST,       // รีโมทขาดจริง
  COMMS_TIMEOUT,   // loop / comms เกินเวลา
  LOGIC_WATCHDOG,  // watchdog logic fail

  // ===== Sensor =====
  CUR_SENSOR_FAULT,  // ค่า current เพี้ยน / หลุดช่วง
  VOLT_SENSOR_FAULT,
  TEMP_SENSOR_FAULT,  // PT100 fault

  // ===== Power / Thermal =====
  OVER_CURRENT,  // กระแสเกิน threshold
  OVER_TEMP,     // อุณหภูมิเกิน

  // ===== Actuation timing =====
  DRIVE_TIMEOUT,  // drive state ใช้เวลานานเกิน
  BLADE_TIMEOUT,  // blade state ใช้เวลานานเกิน

  // ===== System =====
  LOOP_OVERRUN  // loop ใช้เวลานานเกิน budget
};

// ============================================================================
// FAULT CONTROL (NO SENSOR STALL)
// ============================================================================
FaultCode activeFault = FaultCode::NONE;
bool faultLatched = false;

// ============================================================================
// SENSOR / CALIBRATION
// ============================================================================
constexpr uint32_t SENSOR_WARMUP_MS = 2000;

// Current sensor (ACS + MCU ADC)
float g_acsOffsetV[4] = { 2.5f, 2.5f, 2.5f, 2.5f };
constexpr float ACS_SENS_V_PER_A = 0.04f;

// ============================================================================
// COMMUNICATION
// ============================================================================
IBusBM ibus;
uint32_t lastIbusByte_ms = 0;
bool ibusCommLost = false;
bool requireIbusConfirm = false;

// ============================================================================
// ENGINE / STARTER STATE
// ============================================================================
float engineVolt = 0.0f;
bool engineRunning = false;
bool starterActive = false;
uint32_t engineStopped_ms = 0;
uint32_t starterStart_ms = 0;
constexpr uint32_t STARTER_MAX_MS = 3000;

// ============================================================================
// ACTUATORS
// ============================================================================
Servo bladeServo;

// ============================================================================
// SYSTEM STATE MACHINES
// ============================================================================
SystemState systemState = SystemState::INIT;
DriveState driveState = DriveState::IDLE;
BladeState bladeState = BladeState::IDLE;
SafetyState driveSafety = SafetyState::SAFE;

// ============================================================================
// DRIVE CONTROL
// ============================================================================
int16_t targetL = 0;
int16_t targetR = 0;
int16_t curL = 0;
int16_t curR = 0;

uint32_t driveSoftStopStart_ms = 0;
uint32_t bladeSoftStopStart_ms = 0;

// ============================================================================
// SENSOR RUNTIME VALUES
// ============================================================================
float curA[4] = { 0 };
int16_t tempDriverL = 0;
int16_t tempDriverR = 0;

volatile DriveEvent lastDriveEvent = DriveEvent::NONE;
Storm32Controller gimbal(Serial2, ibus);

bool autoReverseActive = false;
uint32_t autoReverseStart_ms = 0;

constexpr uint16_t AUTO_REV_PWM = 300;
constexpr uint32_t AUTO_REV_MS = 350;

Adafruit_ADS1115 adsCur;
Adafruit_ADS1115 adsVolt;

// LSB ต่อโวลต์ (GAIN = ±4.096V)
constexpr float ADS1115_LSB_V = 4.096f / 32768.0f;  // ≈ 0.000125 V

// ============================================================================
// ADS1115 CURRENT CHANNEL MAP (HARDWARE → LOGIC)
// logic index:
// 0 = L1, 1 = L2, 2 = R1, 3 = R2
// ============================================================================
constexpr uint8_t ADS_CUR_CH_MAP[4] = {
  0,  // L1 → ADS1115 A0
  1,  // L2 → ADS1115 A1
  2,  // R1 → ADS1115 A2
  3   // R2 → ADS1115 A3
};

// ============================================================================
// FILTER / DEBOUNCE COUNTERS
// ============================================================================
uint8_t overCurCnt[4] = { 0 };

// ============================================================================
// WATCHDOG FLAGS (LOGIC-LEVEL)
// ============================================================================
volatile bool wdCommsOK = false;
volatile bool wdSensorOK = false;
volatile bool wdDriveOK = false;
volatile bool wdBladeOK = false;

// ============================================================================
// PWM CONFIG
// ============================================================================
#define PWM_TOP 1067  // 15 kHz

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
uint8_t calcFanPwm(int temp, int tStart, int tFull);

// ============================================================================
// FAN CONTROL (DRIVER COOLING)
// ============================================================================

// LEFT DRIVER
constexpr int16_t FAN_L_START_C = 55;  // เริ่มหมุน
constexpr int16_t FAN_L_FULL_C = 85;   // เต็มรอบ

// RIGHT DRIVER
constexpr int16_t FAN_R_START_C = 60;
constexpr int16_t FAN_R_FULL_C = 88;

// PWM behavior
constexpr uint8_t FAN_MIN_PWM = 80;   // ต่ำกว่านี้พัดลมไม่หมุน
constexpr uint8_t FAN_PWM_HYST = 8;   // hysteresis กันแกว่ง
constexpr uint8_t FAN_IDLE_PWM = 50;  // ~20% idle spin

// ============================================================================
// DRIVE DIRECTION DEAD-TIME
// ============================================================================

constexpr uint16_t REVERSE_DEADTIME_MS = 150;  // หน่วงก่อนกลับทิศ (ms)
uint32_t revBlockUntilL = 0;
uint32_t revBlockUntilR = 0;

// ============================================================================
// VOLTAGE WARNING / BUZZER MUTE
// ============================================================================

constexpr float V_WARN_LOW = 24.0f;
constexpr float V_WARN_CRITICAL = 23.0f;

bool voltageWarnMutedByReset = false;

// ============================================================================
// UTIL
// ============================================================================
inline bool neutral(uint16_t v) {
  return (v > 1450 && v < 1550);
}
int16_t ramp(int16_t c, int16_t t, int16_t s) {
  if (c < t) {
    c += s;
    if (c > t) c = t;
  } else if (c > t) {
    c -= s;
    if (c < t) c = t;
  }
  return c;
}

uint8_t calcFanPwm(int temp, int tStart, int tFull) {
  if (temp < tStart) return 0;
  if (temp >= tFull) return 255;
  int mid = (tStart + tFull) / 2;
  if (temp <= mid) {
    return map(temp, tStart, mid,
               FAN_MIN_PWM, 140);
  } else {
    return map(temp, mid, tFull,
               140, 255);
  }
}

// ============================================================================
// READ DRIVER TEMPERATURE (PT100 via MAX31865)
// ============================================================================
bool readDriverTempsPT100(int &tL, int &tR) {

  uint8_t faultL = maxL.readFault();
  uint8_t faultR = maxR.readFault();

  if (faultL || faultR) {
    maxL.clearFault();
    maxR.clearFault();
    return false;
  }

  float TL = maxL.temperature(RTD_RNOMINAL, RTD_RREF);
  float TR = maxR.temperature(RTD_RNOMINAL, RTD_RREF);

  // plausibility guard
  if (TL < -40.0f || TL > 200.0f) return false;
  if (TR < -40.0f || TR > 200.0f) return false;

  tL = (int)TL;
  tR = (int)TR;
  return true;
}

// ============================================================================
// UPDATE DRIVER COOLING FANS
// ============================================================================
void updateDriverFans(void) {

  static uint8_t lastPwmL = 0;
  static uint8_t lastPwmR = 0;

  uint8_t pwmL = calcFanPwm(tempDriverL, FAN_L_START_C, FAN_L_FULL_C);
  uint8_t pwmR = calcFanPwm(tempDriverR, FAN_R_START_C, FAN_R_FULL_C);

  // hysteresis กันสั่น
  if (abs((int)pwmL - (int)lastPwmL) < FAN_PWM_HYST) pwmL = lastPwmL;
  if (abs((int)pwmR - (int)lastPwmR) < FAN_PWM_HYST) pwmR = lastPwmR;

  if (pwmL == 0 && tempDriverL >= FAN_L_START_C) pwmL = FAN_IDLE_PWM;
  if (pwmR == 0 && tempDriverR >= FAN_R_START_C) pwmR = FAN_IDLE_PWM;

  setFanPWM_L(pwmL);
  setFanPWM_R(pwmR);

  lastPwmL = pwmL;
  lastPwmR = pwmR;
}

// ============================================================================
// LOGIC WATCHDOG (SOFT)
// ============================================================================
void logicWatchdog(unsigned long now) {

  static unsigned long lastOk_ms = 0;

  if (wdCommsOK && wdSensorOK && wdDriveOK && wdBladeOK) {
    lastOk_ms = now;
    return;
  }

  if (now - lastOk_ms > LOGIC_WDT_MS) {
    latchFault(FaultCode::LOGIC_WATCHDOG);
  }
}

// ============================================================================
// READ ENGINE VOLTAGE VIA ADS1115 (ISOLATED, FILTERED)
// ใช้ ADS1115 ตัวแรงดันเท่านั้น
// ============================================================================
bool readEngineVoltageADS1115(float &vOut) {

  constexpr uint8_t ADS_CH = 0;  // ช่อง ADS1115 ที่ต่อ divider
  constexpr uint8_t SAMPLE_N = 5;

  // Divider ratio (150k / 33k)
  constexpr float DIV_RATIO = (150.0f + 33.0f) / 33.0f;

  // ADS1115 LSB @ GAIN_ONE
  constexpr float ADS_LSB_V = 4.096f / 32768.0f;

  constexpr float MAX_DV_DT = 1.2f;  // V ต่อรอบ

  static float buf[SAMPLE_N] = { 0 };
  static uint8_t idx = 0;
  static bool filled = false;
  static float lastV = 0;

  int32_t sum = 0;

  // -------- SAMPLE FROM ADS1115 (VOLTAGE ADC) --------
  for (uint8_t i = 0; i < SAMPLE_N; i++) {
    sum += adsVolt.readADC_SingleEnded(ADS_CH);
  }

  int32_t raw = sum / SAMPLE_N;

  float v_adc = raw * ADS_LSB_V;
  float vRaw = v_adc * DIV_RATIO;

  if (vRaw < 0.0f || vRaw > 40.0f) return false;

  if (filled && fabs(vRaw - lastV) > MAX_DV_DT) return false;
  lastV = vRaw;

  buf[idx++] = vRaw;
  if (idx >= SAMPLE_N) {
    idx = 0;
    filled = true;
  }

  if (!filled) return false;

  float tmp[SAMPLE_N];
  memcpy(tmp, buf, sizeof(tmp));
  for (uint8_t i = 0; i < SAMPLE_N - 1; i++) {
    for (uint8_t j = i + 1; j < SAMPLE_N; j++) {
      if (tmp[j] < tmp[i]) {
        float t = tmp[i];
        tmp[i] = tmp[j];
        tmp[j] = t;
      }
    }
  }

  vOut = tmp[SAMPLE_N / 2];
  return true;
}

// ============================================================================
// READ CURRENT VIA MCU ADC (ACS SENSOR)
// ============================================================================
// ============================================================================
// READ CURRENT VIA ADS1115 (ACS SENSOR, ISOLATED ADC)
// แทน readCurrentADC() เดิม 100%
// ============================================================================
float readCurrentADC(uint8_t adsChannel, uint8_t idx) {

  constexpr uint8_t SAMPLE_N = 4;
  int32_t sum = 0;

  // ------------------------------
  // SAMPLE FROM ADS1115
  // ------------------------------
  for (uint8_t i = 0; i < SAMPLE_N; i++) {
    sum += adsCur.readADC_SingleEnded(adsChannel);
  }

  int32_t raw = sum / SAMPLE_N;  // signed 16-bit

  // ------------------------------
  // RAW → VOLT
  // ADS1115 @ GAIN_ONE = ±4.096V
  // LSB = 4.096 / 32768 ≈ 0.000125 V
  // ------------------------------
  constexpr float ADS_LSB_V = 4.096f / 32768.0f;
  float v = raw * ADS_LSB_V;

  // ------------------------------
  // OFFSET (0A reference, normally ~2.5V)
  // ------------------------------
  float offsetV = g_acsOffsetV[idx];

  // ------------------------------
  // VOLT → CURRENT
  // ACS sensitivity = 40 mV/A
  // ------------------------------
  float a = (v - offsetV) / ACS_SENS_V_PER_A;

  // ------------------------------
  // CLAMP (PLAUSIBILITY)
  // ------------------------------
  if (a < CUR_MIN_PLAUSIBLE) a = CUR_MIN_PLAUSIBLE;
  if (a > CUR_MAX_PLAUSIBLE) a = CUR_MAX_PLAUSIBLE;

  return a;  // Ampere
}

// ============================================================================
// CURRENT AGGREGATION HELPERS
// ============================================================================
inline float curLeft() {
  return curA[0] + curA[1];  // L1 + L2
}

inline float curRight() {
  return curA[2] + curA[3];  // R1 + R2
}

// ============================================================================
// AUTO REVERSE (FIELD-SAFE VERSION)
// ============================================================================
void startAutoReverse(uint32_t now) {

  // --------------------------------------------------
  // HARD GUARD
  // --------------------------------------------------
  if (autoReverseActive) return;                                                // กำลัง reverse อยู่
  if (autoReverseCount >= MAX_AUTO_REVERSE) return;                             // เกินโควตา
  if (driveState != DriveState::RUN && driveState != DriveState::LIMP) return;  // ต้องอยู่ใน state ที่ขับจริง
  if (driveSafety == SafetyState::EMERGENCY) return;
  if (systemState != SystemState::ACTIVE) return;

  // --------------------------------------------------
  // COOLDOWN (CRITICAL FOR FIELD)
  // --------------------------------------------------
  static uint32_t lastAutoRev_ms = 0;
  constexpr uint32_t AUTO_REV_COOLDOWN_MS = 1500;  // กัน reverse ถี่เกิน

  if (lastAutoRev_ms != 0 && now - lastAutoRev_ms < AUTO_REV_COOLDOWN_MS) {
    return;
  }

  // --------------------------------------------------
  // VALIDATE DRIVE COMMAND (MUST BE MOVING)
  // --------------------------------------------------
  if (abs(curL) < 200 && abs(curR) < 200) {
    return;  // รถแทบไม่ขยับ ไม่ต้อง reverse
  }

  // --------------------------------------------------
  // ENGAGE AUTO REVERSE
  // --------------------------------------------------
  autoReverseActive = true;
  autoReverseStart_ms = now;
  autoReverseCount++;
  lastAutoRev_ms = now;
  lastDriveEvent = DriveEvent::AUTO_REVERSE;

#if DEBUG_SERIAL
  Serial.print(F("[AUTO REV] START #"));
  Serial.println(autoReverseCount);
#endif
}

// ============================================================================
// FORCE DRIVE SOFT STOP (SINGLE ENTRY)
// ============================================================================
void forceDriveSoftStop(uint32_t now) {
  if (driveState != DriveState::SOFT_STOP) {
    driveState = DriveState::SOFT_STOP;
    driveSoftStopStart_ms = now;
  }
}

void detectSideImbalanceAndSteer() {

  if (abs(targetL) < 200 && abs(targetR) < 200) return;
  if (abs(targetL - targetR) > 200) return;  // กำลังเลี้ยวอยู่

  float cL = curLeft();
  float cR = curRight();

  constexpr float CUR_IMBALANCE_A = 25.0f;
  constexpr int16_t STEER_COMP = 140;

  if (abs(cL - cR) < CUR_IMBALANCE_A) return;

  lastDriveEvent = DriveEvent::IMBALANCE;

  // ซ้ายหนัก → เบี่ยงขวานิด
  if (cL > cR) {
    targetL -= STEER_COMP;
    targetR += STEER_COMP;
  }
  // ขวาหนัก → เบี่ยงซ้ายนิด
  else {
    targetL += STEER_COMP;
    targetR -= STEER_COMP;
  }

  driveSafety = SafetyState::WARN;
}

void detectWheelStuck(uint32_t now) {

  if (abs(targetL) < 200 && abs(targetR) < 200) return;

  float cL = curLeft();
  float cR = curRight();

  if (cL > CUR_LIMP_A && cR < CUR_WARN_A) {
    lastDriveEvent = DriveEvent::STUCK_LEFT;
    startAutoReverse(now);
    driveSafety = SafetyState::LIMP;
  }

  if (cR > CUR_LIMP_A && cL < CUR_WARN_A) {
    lastDriveEvent = DriveEvent::STUCK_RIGHT;
    startAutoReverse(now);
    driveSafety = SafetyState::LIMP;
  }
}

void detectWheelLock() {
  static uint8_t lockCnt = 0;
  if (abs(targetL) < 200 && abs(targetR) < 200) {
    lockCnt = 0;
    return;
  }
  if (curLeft() > CUR_LIMP_A && curRight() > CUR_LIMP_A) {
    if (++lockCnt >= 3) {
      lastDriveEvent = DriveEvent::WHEEL_LOCK;
      latchFault(FaultCode::OVER_CURRENT);
    }
  } else {
    lockCnt = 0;
  }
}

void telemetryCSV(uint32_t now) {
#if TELEMETRY_CSV
  static uint32_t lastTx = 0;
  if (now - lastTx < TELEMETRY_PERIOD_MS) return;
  lastTx = now;
  float curMax = max(max(curA[0], curA[1]), max(curA[2], curA[3]));
  float v24 = engineVolt;
  // ===== CSV FORMAT =====
  // time,curL1,curL2,curR1,curR2,curMax,tempDL,tempDR,volt,pwmL,pwmR,driveState,safety
  Serial.print(now);
  Serial.print(',');
  Serial.print(curA[0], 2);
  Serial.print(',');
  Serial.print(curA[1], 2);
  Serial.print(',');
  Serial.print(curA[2], 2);
  Serial.print(',');
  Serial.print(curA[3], 2);
  Serial.print(',');
  Serial.print(curMax, 2);
  Serial.print(',');
  Serial.print(tempDriverL);
  Serial.print(',');
  Serial.print(tempDriverR);
  Serial.print(',');
  Serial.print(v24, 2);
  Serial.print(',');
  Serial.print(curL);
  Serial.print(',');
  Serial.print(curR);
  Serial.print(',');
  Serial.print((uint8_t)driveState);
  Serial.print(',');
  Serial.print((uint8_t)driveSafety);
  Serial.print(',');
  Serial.println((uint8_t)lastDriveEvent);

#endif
}

void updateDriveTarget() {
  // =========================
  // HARD GUARD : NO DRIVE
  // =========================
  if (ibusCommLost || driveSafety == SafetyState::EMERGENCY || systemState != SystemState::ACTIVE) {
    targetL = 0;
    targetR = 0;
    return;
  }
  // =========================
  // AXIS MAPPING (REAL DEADBAND AWARE)
  // =========================
  auto mapAxis = [](int16_t v) -> int16_t {
    constexpr int16_t IN_MIN = 1000;
    constexpr int16_t IN_MAX = 2000;
    constexpr int16_t DB_MIN = 1450;  // deadband lower
    constexpr int16_t DB_MAX = 1550;  // deadband upper
    constexpr int16_t OUT_MAX = PWM_TOP;
    // ----- DEAD BAND -----
    if (v >= DB_MIN && v <= DB_MAX) {
      return 0;
    }
    // ----- BELOW DEAD BAND -----
    if (v < DB_MIN) {
      long m = map(v, IN_MIN, DB_MIN, -OUT_MAX, 0);
      return (int16_t)constrain(m, -OUT_MAX, 0);
    }
    // ----- ABOVE DEAD BAND -----
    long m = map(v, DB_MAX, IN_MAX, 0, OUT_MAX);
    return (int16_t)constrain(m, 0, OUT_MAX);
  };
  // =========================
  // READ AXES
  // =========================
  int16_t thr = mapAxis(ibus.readChannel(CH_THROTTLE));
  int16_t str = mapAxis(ibus.readChannel(CH_STEER));
  // =========================
  // BLEND FACTOR (HYBRID)
  // 0.0 = arcade
  // 1.0 = differential
  // =========================
  float absThr = abs(thr);
  float blend;
  if (absThr <= 150.0f) {
    blend = 0.0f;  // ช้ามาก / หมุนในที่แคบ
  } else if (absThr >= 450.0f) {
    blend = 1.0f;  // ไหลจริง
  } else {
    blend = (absThr - 150.0f) / (450.0f - 150.0f);
  }
  // =========================
  // ARCADE PART
  // =========================
  float arcL = (float)thr + (float)str;
  float arcR = (float)thr - (float)str;
  // =========================
  // DIFFERENTIAL PART
  // =========================
  float k = abs(str) / (float)PWM_TOP;
  if (str < 0) k = -k;  // ทิศเลี้ยว
  float diffL = thr * (1.0f + k);
  float diffR = thr * (1.0f - k);
  // =========================
  // HYBRID BLEND
  // =========================
  float outL = arcL * (1.0f - blend) + diffL * blend;
  float outR = arcR * (1.0f - blend) + diffR * blend;
  // =========================
  // NORMALIZE (ANTI-SATURATION)
  // =========================
  float maxMag = max(abs(outL), abs(outR));
  if (maxMag > PWM_TOP) {
    float scale = (float)PWM_TOP / maxMag;
    outL *= scale;
    outR *= scale;
  }
  // =========================
  // OUTPUT
  // =========================
  targetL = (int16_t)outL;
  targetR = (int16_t)outR;
}


// ============================================================================
// ENGINE THROTTLE SERVO (CH3 ANALOG CONTROL)
// ============================================================================
void updateEngineThrottle() {

  // ---------- HARD SAFETY ----------
  if (systemState == SystemState::FAULT || driveSafety == SafetyState::EMERGENCY || bladeState != BladeState::RUN) {

    bladeServo.writeMicroseconds(1000);
    return;
  }
  uint16_t ch3 = ibus.readChannel(CH_ENGINE);

  // ---------- INPUT GUARD ----------
  if (ch3 < 1000) ch3 = 1000;
  if (ch3 > 2000) ch3 = 2000;

  // ---------- DEADZONE กันสั่น ----------
  if (ch3 < 1050) ch3 = 1000;

  bladeServo.writeMicroseconds(ch3);
}

void updateEngineState(uint32_t now) {
  static uint32_t runConfirm_ms = 0;
  static uint32_t stopConfirm_ms = 0;
  float v = engineVolt;

  // ===== IGNORE DURING STARTER =====
  if (starterActive) return;

  // =================================================
  // LOCK DURING STARTER
  // =================================================
  if (starterActive) {
    // ระหว่างสตาร์ต ห้ามเปลี่ยนสถานะ engineRunning
    runConfirm_ms = 0;
    stopConfirm_ms = 0;
    return;
  }

  if (engineVolt < 10.0f || engineVolt > 40.0f) {
    return;  // ค่าเพี้ยน ไม่เอาไปตัดสิน state
  }

  // =================================================
  // ENGINE OFF → CHECK RUN
  // =================================================
  if (!engineRunning) {
    stopConfirm_ms = 0;
    if (v >= ENGINE_RUNNING_VOLT) {
      if (runConfirm_ms == 0) {
        runConfirm_ms = now;
      } else if (now - runConfirm_ms >= ENGINE_CONFIRM_MS) {
        engineRunning = true;
        runConfirm_ms = 0;
#if DEBUG_SERIAL
        Serial.println(F("[ENGINE] RUN CONFIRMED"));
#endif
      }
    } else {
      runConfirm_ms = 0;
    }
  }
  // =================================================
  // ENGINE ON → CHECK STOP
  // =================================================
  else {
    runConfirm_ms = 0;
    if (v <= ENGINE_STOP_VOLT) {
      if (stopConfirm_ms == 0) {
        stopConfirm_ms = now;
      } else if (now - stopConfirm_ms >= ENGINE_CONFIRM_MS) {
        engineRunning = false;
        stopConfirm_ms = 0;
        engineStopped_ms = now;
#if DEBUG_SERIAL
        Serial.println(F("[ENGINE] STOP CONFIRMED"));
#endif
      }
    } else {
      stopConfirm_ms = 0;
    }
  }
}

// ============================================================================
// PWM
// ============================================================================
void setupPWM15K() {
  TCCR3A = _BV(COM3A1) | _BV(WGM31);
  TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS30);
  ICR3 = 1067;
  OCR3A = 0;
  TCCR4A = _BV(COM4A1) | _BV(WGM41);
  TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS40);
  ICR4 = 1067;
  OCR4A = 0;
}
inline void setPWM_L(uint16_t v) {
  OCR3A = constrain(v, 0, ICR3);
}
inline void setPWM_R(uint16_t v) {
  OCR4A = constrain(v, 0, ICR4);
}
// ===== FAN PWM HELPERS (TIMER5) =====
inline void setFanPWM_L(uint8_t pwm) {
  OCR5C = map(pwm, 0, 255, 0, ICR5);
}

inline void setFanPWM_R(uint8_t pwm) {
  OCR5B = map(pwm, 0, 255, 0, ICR5);
}

// ============================================================================
// FAN PWM (TIMER5 @ 15kHz)
// FAN_L = pin 44 (OC5C)
// FAN_R = pin 45 (OC5B)
// ============================================================================
void setupFanPWM15K() {
  // Fast PWM, TOP = ICR5
  TCCR5A = _BV(COM5B1) | _BV(COM5C1) | _BV(WGM51);
  TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS50);  // prescaler = 1

  ICR5 = 1067;  // ~15 kHz @ 16 MHz

  OCR5B = 0;  // FAN_R (pin 45)
  OCR5C = 0;  // FAN_L (pin 44)
}

void driveSafe() {
  // 1) ตัด PWM ก่อน (สำคัญสุด)
  setPWM_L(0);
  setPWM_R(0);

  // 2) หน่วงสั้นมาก กัน shoot-through (ไม่ block loop)
  delayMicroseconds(5);

  // 3) ปลดทิศทางทั้งหมด
  digitalWrite(DIR_L1, LOW);
  digitalWrite(DIR_L2, LOW);
  digitalWrite(DIR_R1, LOW);
  digitalWrite(DIR_R2, LOW);
}

// ============================================================================
// latchFault  (WITH EEPROM FAULT HISTORY)
// ============================================================================
void latchFault(FaultCode code) {

  if (faultLatched) return;

  activeFault = code;
  faultLatched = true;

  // ==================================================
  // STORE LAST FAULT TO EEPROM (WRITE ONLY IF CHANGED)
  // ==================================================
  FaultCode lastStored;
  EEPROM.get(100, lastStored);

  if (lastStored != code) {
    EEPROM.put(100, code);
  }

#if DEBUG_SERIAL
  Serial.println(F("========== FAULT SNAPSHOT =========="));
  Serial.print(F("FaultCode="));
  Serial.println((uint8_t)code);
  Serial.print(F("SystemState="));
  Serial.println((uint8_t)systemState);
  Serial.print(F("DriveState="));
  Serial.println((uint8_t)driveState);
  Serial.print(F("BladeState="));
  Serial.println((uint8_t)bladeState);
  Serial.print(F("Safety="));
  Serial.println((uint8_t)driveSafety);

  Serial.print(F("CurA: "));
  for (int i = 0; i < 4; i++) {
    Serial.print(curA[i]);
    Serial.print(F(" "));
  }
  Serial.println();

  Serial.print(F("TempDriverL="));
  Serial.print(tempDriverL);
  Serial.print(F(" TempDriverR="));
  Serial.println(tempDriverR);

  Serial.print(F("PWM L/R="));
  Serial.print(curL);
  Serial.print(F("/"));
  Serial.println(curR);
  Serial.println(F("===================================="));
#endif
}

void updateComms(uint32_t now) {
  // =========================
  // IBUS BYTE PARSING
  // =========================
  bool gotByte = false;
  while (Serial1.available()) {
    ibus.loop();
    lastIbusByte_ms = now;
    gotByte = true;
  }
  // =========================
  // RECOVERY: BYTE COMES BACK
  // =========================
  if (gotByte && ibusCommLost) {
    ibusCommLost = false;
    // ❗ ห้าม clear driveSafety ที่นี่
  }

  // =========================
  // SOFT TIMEOUT → COMM LOST
  // =========================
  if (now - lastIbusByte_ms > 100) {
    if (!ibusCommLost) {
      ibusCommLost = true;
      requireIbusConfirm = true;  // <<< สำคัญ
    }
    driveSafety = SafetyState::EMERGENCY;
  }

  // =========================
  // HARD TIMEOUT → FAULT
  // =========================
  if (now - lastIbusByte_ms > IBUS_TIMEOUT_MS) {
#if DEBUG_SERIAL
    Serial.println(F("[IBUS] HARD LOST -> FAULT"));
#endif
    latchFault(FaultCode::IBUS_LOST);
    return;
  }
  // =========================
  // COMMS OK
  // =========================
  wdCommsOK = true;
}



void updateSensors() {
  uint32_t now = millis();

#if TEST_MODE
  curA[0] = curA[1] = curA[2] = curA[3] = 5.0f;
  tempDriverL = 45;
  tempDriverR = 47;
  engineVolt = 26.0f;
  wdSensorOK = true;
  return;
#endif

  // ==================================================
  // HARDWARE OVERCURRENT TRIP (µs-level)
  // ==================================================
  if (digitalRead(PIN_CUR_TRIP) == LOW) {
    latchFault(FaultCode::OVER_CURRENT);
    return;
  }

  // ==================================================
  // CURRENT (ADS1115) — EVERY LOOP
  // ==================================================
  static float lastCurA[4] = { 0 };
  static uint8_t stuckCnt[4] = { 0 };

  for (uint8_t idx = 0; idx < 4; idx++) {

    uint8_t adsCh = ADS_CUR_CH_MAP[idx];
    float a = readCurrentADC(adsCh, idx);

    // plausibility
    if (a < CUR_MIN_PLAUSIBLE || a > CUR_MAX_PLAUSIBLE) {
      latchFault(FaultCode::CUR_SENSOR_FAULT);
      return;
    }

    // IIR LPF
    curA[idx] += CUR_LPF_ALPHA * (a - curA[idx]);

    // stuck detect (PWM active)
    if (abs(curL) > 200 || abs(curR) > 200) {
      if (fabs(curA[idx] - lastCurA[idx]) < 0.03f) {
        if (++stuckCnt[idx] > 40) {
          latchFault(FaultCode::CUR_SENSOR_FAULT);
          return;
        }
      } else {
        stuckCnt[idx] = 0;
      }
    }
    lastCurA[idx] = curA[idx];

    // spike protect
    if (a > CUR_SPIKE_A) {
      forceDriveSoftStop(now);
      bladeState = BladeState::SOFT_STOP;
    }

    // overcurrent latch
    if (a > CUR_TRIP_A_CH[idx]) {
      if (++overCurCnt[idx] >= 2) {
        latchFault(FaultCode::OVER_CURRENT);
        return;
      }
    } else {
      overCurCnt[idx] = 0;
    }
  }

  // ==================================================
  // TEMPERATURE (PT100) — 100 ms RATE
  // ==================================================
  static uint32_t lastTemp_ms = 0;
  static uint32_t lastTempOk_ms = 0;

  if (now - lastTemp_ms >= 100) {
    lastTemp_ms = now;

    int16_t tL, tR;
    if (!readDriverTempsPT100(tL, tR)) {
      latchFault(FaultCode::TEMP_SENSOR_FAULT);
      return;
    }

    tempDriverL = tL;
    tempDriverR = tR;
    lastTempOk_ms = now;

    if (tL > TEMP_TRIP_C || tR > TEMP_TRIP_C) {
      latchFault(FaultCode::OVER_TEMP);
      return;
    }
  }

  if (now - lastTempOk_ms > 500) {
    latchFault(FaultCode::TEMP_SENSOR_FAULT);
    return;
  }

  // ==================================================
  // ENGINE VOLTAGE (ADS1115) — 50 ms RATE
  // ==================================================
  static uint32_t lastVolt_ms = 0;
  static uint32_t lastVoltOk_ms = 0;

  if (now - lastVolt_ms >= 50) {
    lastVolt_ms = now;

    float v;
    if (readEngineVoltageADS1115(v)) {
      engineVolt += 0.15f * (v - engineVolt);
      lastVoltOk_ms = now;
    }
  }

  if (now - lastVoltOk_ms > 500) {
    latchFault(FaultCode::VOLT_SENSOR_FAULT);
    return;
  }

  // ==================================================
  // SENSOR PATH OK
  // ==================================================
  wdSensorOK = true;
}

// ============================================================================
// DRIVE STATE MACHINE (FIXED)
// ============================================================================
void runDrive(uint32_t now) {
  static DriveState lastDriveState = DriveState::IDLE;
  static uint32_t limpSafeStart_ms = 0;
  // =========================
  // STATE MACHINE
  // =========================
  switch (driveState) {
    // --------------------------------------------------
    case DriveState::IDLE:
      targetL = 0;
      targetR = 0;
      curL = 0;
      curR = 0;
      limpSafeStart_ms = 0;
      driveSoftStopStart_ms = 0;  // reset soft stop อย่างชัดเจน
      if (systemState == SystemState::ACTIVE) {
        driveState = DriveState::RUN;
      }
      break;
    // --------------------------------------------------
    case DriveState::RUN:
      updateDriveTarget();
      if (driveSafety == SafetyState::EMERGENCY) {
        driveState = DriveState::SOFT_STOP;
      } else if (driveSafety == SafetyState::LIMP) {
        driveState = DriveState::LIMP;
        limpSafeStart_ms = 0;
      }
      break;

    case DriveState::LIMP:
      updateDriveTarget();
      targetL /= 2;
      targetR /= 2;
      if (driveSafety == SafetyState::EMERGENCY) {
        driveState = DriveState::SOFT_STOP;
        break;
      }

      if (driveSafety == SafetyState::SAFE) {
        if (limpSafeStart_ms == 0) {
          limpSafeStart_ms = now;
        } else if (now - limpSafeStart_ms >= LIMP_RECOVER_MS) {
#if DEBUG_SERIAL
          Serial.println(F("[DRIVE] LIMP RECOVER -> RUN"));
#endif
          driveState = DriveState::RUN;
          limpSafeStart_ms = 0;
        }
      } else {
        limpSafeStart_ms = 0;
      }
      break;
    // --------------------------------------------------
    case DriveState::SOFT_STOP:
      targetL = 0;
      targetR = 0;
      // รอให้ ramp ลงจริง หรือ timeout
      if ((curL == 0 && curR == 0) || (now - driveSoftStopStart_ms >= DRIVE_SOFT_STOP_TIMEOUT_MS)) {
        driveSafe();  // ตัดกำลังจริง
        driveState = DriveState::LOCKED;
      }
      break;
    // --------------------------------------------------
    case DriveState::LOCKED:
      driveSafe();  // safety latch
      break;
  }
  // =========================
  // DRIVE STATE TRANSITION DEBUG + INIT
  // =========================
  if (driveState != lastDriveState) {
#if DEBUG_SERIAL
    Serial.print(F("[DRIVE STATE] "));
    Serial.print((uint8_t)lastDriveState);
    Serial.print(F(" -> "));
    Serial.println((uint8_t)driveState);
#endif
    // ตั้งเวลา soft stop เฉพาะตอน "เข้า" ครั้งแรก
    if (driveState == DriveState::SOFT_STOP) {
      driveSoftStopStart_ms = now;
    }
    lastDriveState = driveState;
  }
  // =========================
  // DRIVE LOGIC OK
  // =========================
  wdDriveOK = true;
}

// ============================================================================
// runBlade
// ============================================================================
void runBlade(uint32_t now) {

  switch (bladeState) {

    case BladeState::IDLE:
      // คันเร่ง idle อยู่แล้วจาก updateEngineThrottle()
      if (systemState == SystemState::ACTIVE && !faultLatched) {
        bladeState = BladeState::RUN;
      }
      break;

    case BladeState::RUN:
      // ใบมีดหมุนตามรอบเครื่องยนต์ (ทางกล)
      if (faultLatched || driveSafety == SafetyState::EMERGENCY) {
#if DEBUG_SERIAL
        Serial.println(F("[ENGINE] THROTTLE KILL"));
#endif
        bladeState = BladeState::SOFT_STOP;
        bladeSoftStopStart_ms = now;
      }
      break;

    case BladeState::SOFT_STOP:
      // ลดคันเร่งนุ่ม
      bladeServo.writeMicroseconds(1000);
      if (now - bladeSoftStopStart_ms > BLADE_SOFT_STOP_TIMEOUT_MS) {
        bladeState = BladeState::LOCKED;
      }
      break;

    case BladeState::LOCKED:
      bladeServo.writeMicroseconds(1000);
      break;
  }

  wdBladeOK = true;
}

// ============================================================================
// APPLY DRIVE OUTPUT (FIXED / FIELD-SAFE / AUTO-REVERSE AWARE)
// ============================================================================
void applyDrive() {
  uint32_t now = millis();

  // ==================================================
  // HARD STOP : SYSTEM FAULT
  // ==================================================
  if (systemState == SystemState::FAULT || driveState == DriveState::LOCKED) {
    driveSafe();
    curL = 0;
    curR = 0;
    targetL = 0;
    targetR = 0;
    revBlockUntilL = 0;
    revBlockUntilR = 0;
    return;
  }

  // ==================================================
  // AUTO REVERSE OVERRIDE (SHORT PULSE ONLY)
  // ==================================================
  if (autoReverseActive) {
    if (now - autoReverseStart_ms < AUTO_REV_MS) {
      // ถอยตรง
      digitalWrite(DIR_L1, LOW);
      digitalWrite(DIR_L2, HIGH);
      digitalWrite(DIR_R1, LOW);
      digitalWrite(DIR_R2, HIGH);
      setPWM_L(AUTO_REV_PWM);
      setPWM_R(AUTO_REV_PWM);
      return;  // ❗ override logic ปกติ
    } else {
      autoReverseActive = false;
#if DEBUG_SERIAL
      Serial.println(F("[AUTO REV] END"));
#endif
    }
  }

  // ==================================================
  // SAFETY EMERGENCY (NO DRIVE)
  // ==================================================
  if (driveSafety == SafetyState::EMERGENCY) {
    targetL = 0;
    targetR = 0;
  }

  // ==================================================
  // DIRECTION CHANGE DEAD-TIME
  // ==================================================
  bool wantRevL = (curL > 0 && targetL < 0) || (curL < 0 && targetL > 0);
  bool wantRevR = (curR > 0 && targetR < 0) || (curR < 0 && targetR > 0);

  if (wantRevL && revBlockUntilL == 0) {
    revBlockUntilL = now + REVERSE_DEADTIME_MS;
  }
  if (wantRevR && revBlockUntilR == 0) {
    revBlockUntilR = now + REVERSE_DEADTIME_MS;
  }

  if (revBlockUntilL && now < revBlockUntilL) {
    targetL = 0;
  } else if (revBlockUntilL && now >= revBlockUntilL) {
    revBlockUntilL = 0;
  }

  if (revBlockUntilR && now < revBlockUntilR) {
    targetR = 0;
  } else if (revBlockUntilR && now >= revBlockUntilR) {
    revBlockUntilR = 0;
  }

  // ==================================================
  // RAMP RATE CONTROL
  // ==================================================
  int16_t step;
  if (targetL == 0 && targetR == 0) {
    step = 2;
  } else if (driveState == DriveState::SOFT_STOP) {
    step = 2;
  } else if (driveState == DriveState::LIMP) {
    step = 4;
  } else {
    step = 5;
  }

  curL = ramp(curL, targetL, step);
  curR = ramp(curR, targetR, step);

  // ==================================================
  // OUTPUT DIRECTION
  // ==================================================
  digitalWrite(DIR_L1, curL > 0);
  digitalWrite(DIR_L2, curL < 0);
  digitalWrite(DIR_R1, curR > 0);
  digitalWrite(DIR_R2, curR < 0);

  // ==================================================
  // OUTPUT PWM
  // ==================================================
  setPWM_L(abs(curL));
  setPWM_R(abs(curR));
}

// ============================================================================
// FAULT HANDLER (SINGLE POINT OF CUT)
// ============================================================================
void handleFaultImmediateCut() {
#if DEBUG_SERIAL
  static bool printed = false;
  if (!printed) {
    Serial.println(F("[FAULT] IMMEDIATE CUT"));
    printed = true;
  }
#endif
  // ตัด drive
  driveSafe();
  curL = 0;
  curR = 0;
  // ตัด blade
  bladeServo.writeMicroseconds(1000);
  // กัน watchdog trip ซ้ำ
  wdCommsOK = true;
  wdSensorOK = true;
  wdDriveOK = true;
  wdBladeOK = true;
}

void updateIgnition() {
  uint16_t ch6 = ibus.readChannel(CH_IGNITION);
  bool ignitionOn = (ch6 > 1600);
  // ❗ ถ้า FAULT → ดับเครื่องเสมอ
  if (systemState == SystemState::FAULT) {
    ignitionOn = false;
  }
  digitalWrite(RELAY_IGNITION, ignitionOn ? HIGH : LOW);
}

void updateStarter(uint32_t now) {

  uint16_t ch10 = ibus.readChannel(CH_STARTER);
  bool requestStart = (ch10 > 1600);

  bool ignitionOn = digitalRead(RELAY_IGNITION);

  // =====================================================
  // 1️⃣ HARD SAFETY GATE (BLOCK EVERYTHING)
  // =====================================================
  if (systemState == SystemState::FAULT ||
      driveState != DriveState::IDLE ||
      ibus.readChannel(CH_ENGINE) > 1100 ||
      !neutral(ibus.readChannel(CH_THROTTLE)) ||
      !ignitionOn ||
      engineRunning ||
      (engineStopped_ms != 0 && now - engineStopped_ms < ENGINE_RESTART_GUARD_MS)) {

    starterActive = false;
    digitalWrite(RELAY_STARTER, LOW);
    return;
  }

  // =====================================================
  // 2️⃣ IF STARTER IS ACTIVE → HANDLE TIMEOUT ONLY
  // =====================================================
  if (starterActive) {

    if (!requestStart || (now - starterStart_ms > STARTER_MAX_MS)) {

      starterActive = false;
      digitalWrite(RELAY_STARTER, LOW);

      // ถ้าเครื่องยังไม่ติด → ถือว่าสตาร์ทล้มเหลว
      if (!engineRunning) {
        engineStopped_ms = now;
      }
    }

    return;
  }

  // =====================================================
  // 3️⃣ NEW START REQUEST
  // =====================================================
  if (requestStart) {

    starterActive = true;
    starterStart_ms = now;
    digitalWrite(RELAY_STARTER, HIGH);
  }
}

// ============================================================================
// VOLTAGE WARNING (BUZZER + RELAY, MUTE AFTER RESET UNTIL VOLTAGE NORMAL)
// ============================================================================
void updateVoltageWarning(uint32_t now) {
  static uint32_t lastToggle_ms = 0;
  static bool buzzerOn = false;
  float v24 = engineVolt;

  // ===== SENSOR INVALID → DO NOTHING =====
  if (!wdSensorOK) {
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(RELAY_WARN, LOW);
    return;
  }

  // --------------------------------------------------
  // UNMUTE CONDITION
  // แรงดันกลับสู่ปกติ → ปลด mute
  // --------------------------------------------------
  if (voltageWarnMutedByReset && v24 >= V_WARN_LOW) {
#if DEBUG_SERIAL
    Serial.println(F("[VOLT WARN] MUTE CLEARED (VOLT NORMAL)"));
#endif
    voltageWarnMutedByReset = false;
  }
  // --------------------------------------------------
  // NORMAL VOLTAGE
  // --------------------------------------------------
  if (v24 >= V_WARN_LOW) {
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(RELAY_WARN, LOW);
    buzzerOn = false;
    return;
  }
  // --------------------------------------------------
  // MUTED STATE (AFTER RESET)
  // --------------------------------------------------
  if (voltageWarnMutedByReset) {
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(RELAY_WARN, HIGH);
    return;
  }
  // --------------------------------------------------
  // LEVEL 1 : LOW VOLTAGE
  // --------------------------------------------------
  if (v24 < V_WARN_LOW && v24 >= V_WARN_CRITICAL) {
    digitalWrite(RELAY_WARN, HIGH);
    if (now - lastToggle_ms >= 500) {
      lastToggle_ms = now;
      buzzerOn = !buzzerOn;
      digitalWrite(PIN_BUZZER, buzzerOn ? HIGH : LOW);
    }
    return;
  }
  // --------------------------------------------------
  // LEVEL 2 : CRITICAL VOLTAGE
  // --------------------------------------------------
  if (v24 < V_WARN_CRITICAL) {
    digitalWrite(PIN_BUZZER, HIGH);
    digitalWrite(RELAY_WARN, HIGH);
  }
}

void debugTestMode(uint32_t now) {
#if TEST_MODE && DEBUG_SERIAL
  static uint32_t lastPrint = 0;
  if (now - lastPrint < 500) return;  // 2 Hz
  lastPrint = now;
  Serial.print(F("[TEST MODE] "));
  Serial.print(F("Drive="));
  Serial.print((uint8_t)driveState);
  Serial.print(F(" Blade="));
  Serial.print((uint8_t)bladeState);
  Serial.print(F(" | PWM L/R="));
  Serial.print(curL);
  Serial.print(F("/"));
  Serial.print(curR);
  Serial.print(F(" | Cur(A)="));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(curA[i], 1);
    Serial.print(F(" "));
  }
  Serial.print(tempDriverL);
  Serial.print(F("/"));
  Serial.print(tempDriverR);
  Serial.println();
#endif
}

void debugTelemetry(uint32_t now) {
#if DEBUG_SERIAL
  static uint32_t lastPrint = 0;
  if (now - lastPrint < 200) return;  // 5 Hz
  lastPrint = now;
  Serial.print(F("[TEL] "));
  Serial.print(F("S="));
  Serial.print((uint8_t)systemState);
  Serial.print(F(" D="));
  Serial.print((uint8_t)driveState);
  Serial.print(F(" B="));
  Serial.print((uint8_t)bladeState);
  Serial.print(F(" | IbusAge="));
  Serial.print(now - lastIbusByte_ms);
  Serial.print(F(" | Cur="));
  for (int i = 0; i < 4; i++) {
    Serial.print(curA[i]);
    Serial.print(F(" "));
  }
  Serial.print(F(" | TdL="));
  Serial.print(tempDriverL);
  Serial.print(F(" TdR="));
  Serial.print(tempDriverR);
  Serial.print(F(" | PWM="));
  Serial.print(curL);
  Serial.print(F("/"));
  Serial.println(curR);
  Serial.print(F(" | V24="));
  Serial.print(engineVolt, 1);
  Serial.print(F(" ER="));
  Serial.print(engineRunning ? 1 : 0);
#endif
}

void debugIBus(uint32_t now) {
#if DEBUG_SERIAL
  static uint32_t lastPrint = 0;
  if (now - lastPrint < 500) return;
  lastPrint = now;
  Serial.print(F("[IBUS] "));
  Serial.print(F("THR="));
  Serial.print(ibus.readChannel(CH_THROTTLE));
  Serial.print(F(" STR="));
  Serial.print(ibus.readChannel(CH_STEER));
  Serial.print(F(" ENG="));
  Serial.print(ibus.readChannel(CH_ENGINE));
  Serial.print(F(" RST="));
  Serial.println(ibus.readChannel(CH_RESET));
#endif
}

// ============================================================================
// SETUP  (FIXED / FIELD-SAFE)
// ============================================================================
void setup() {

  // --------------------------------------------------
  // SERIAL (DEBUG)
  // --------------------------------------------------
  Serial.begin(115200);

  // --------------------------------------------------
  // SERIAL COMMUNICATION
  // --------------------------------------------------
  Serial1.begin(115200);  // iBUS
  Serial2.begin(115200);  // Storm32

  ibus.begin(Serial1);  // iBUS = INPUT ONLY
  gimbal.begin();       // Storm32 uses Serial2

  gimbal.forceOff();  // <<< HARD LOCK gimbal at boot

  // --------------------------------------------------
  // I2C
  // --------------------------------------------------
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(6000, true);

  adsCur.begin(0x48);  // ADS1115 กระแส
  adsCur.setGain(GAIN_ONE);

  adsVolt.begin(0x49);  // ADS1115 แรงดัน
  adsVolt.setGain(GAIN_ONE);

  // --------------------------------------------------
  // READ LAST FAULT FROM EEPROM (FAULT HISTORY)
  // --------------------------------------------------
  FaultCode lastFault;
  EEPROM.get(100, lastFault);

#if DEBUG_SERIAL
  Serial.print(F("[BOOT] LAST FAULT = "));
  Serial.println((uint8_t)lastFault);
#endif

#if TEST_MODE && DEBUG_SERIAL
  Serial.println();
  Serial.println(F("==================================="));
  Serial.println(F("!!! TEST MODE ACTIVE !!!"));
  Serial.println(F("NO REAL SENSORS / BENCH TEST ONLY"));
  Serial.println(F("DO NOT USE IN FIELD OPERATION"));
  Serial.println(F("==================================="));
#endif

  // --------------------------------------------------
  // GPIO OUTPUT
  // --------------------------------------------------
  pinMode(PIN_HW_WD_HB, OUTPUT);
  digitalWrite(PIN_HW_WD_HB, LOW);  // ค่าเริ่มต้น = silent

  pinMode(PIN_DRV_ENABLE, OUTPUT);
  digitalWrite(PIN_DRV_ENABLE, LOW);  // ❗ default = disable driver
  pinMode(DIR_L1, OUTPUT);
  pinMode(DIR_L2, OUTPUT);
  pinMode(DIR_R1, OUTPUT);
  pinMode(DIR_R2, OUTPUT);

  pinMode(MAX_CS_L, OUTPUT);
  pinMode(MAX_CS_R, OUTPUT);
  digitalWrite(MAX_CS_L, HIGH);
  digitalWrite(MAX_CS_R, HIGH);
  pinMode(PIN_CUR_TRIP, INPUT_PULLUP);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(RELAY_WARN, OUTPUT);
  digitalWrite(RELAY_WARN, LOW);
  pinMode(RELAY_STARTER, OUTPUT);
  digitalWrite(RELAY_STARTER, LOW);
  pinMode(RELAY_IGNITION, OUTPUT);
  digitalWrite(RELAY_IGNITION, LOW);
  pinMode(FAN_L, OUTPUT);
  digitalWrite(FAN_L, LOW);
  pinMode(FAN_R, OUTPUT);
  digitalWrite(FAN_R, LOW);

  driveSafe();  // <<< HARD motor cut at boot

  // --------------------------------------------------
  // SPI / TEMPERATURE
  // --------------------------------------------------
  SPI.begin();
  maxL.begin(MAX31865_3WIRE);
  maxR.begin(MAX31865_3WIRE);

  // --------------------------------------------------
  // PWM / SERVO
  // --------------------------------------------------
  setupPWM15K();
  setupFanPWM15K();

  bladeServo.attach(SERVO_ENGINE_PIN);
  bladeServo.writeMicroseconds(1000);

  // --------------------------------------------------
  // INIT SENSOR / STATE
  // --------------------------------------------------
  for (uint8_t i = 0; i < 4; i++) {
    curA[i] = 0.0f;
    overCurCnt[i] = 0;
  }

  tempDriverL = 0;
  tempDriverR = 0;
  engineVolt = 0.0f;

  systemState = SystemState::INIT;
  driveState = DriveState::IDLE;
  bladeState = BladeState::IDLE;
  driveSafety = SafetyState::SAFE;

  faultLatched = false;
  activeFault = FaultCode::NONE;

  curL = curR = 0;
  targetL = targetR = 0;

  revBlockUntilL = 0;
  revBlockUntilR = 0;
  driveSoftStopStart_ms = 0;
  bladeSoftStopStart_ms = 0;

  // --------------------------------------------------
  // MISSING RUNTIME RESET (CRITICAL)
  // --------------------------------------------------
  engineStopped_ms = 0;
  starterActive = false;

  lastIbusByte_ms = millis();
  ibusCommLost = true;

  voltageWarnMutedByReset = false;

  // --------------------------------------------------
  // WATCHDOG FLAGS
  // --------------------------------------------------
  wdCommsOK = false;
  wdSensorOK = false;
  wdDriveOK = false;
  wdBladeOK = false;

  // --------------------------------------------------
  // HW WATCHDOG (LAST)
  // --------------------------------------------------
  wdt_reset();
  wdt_enable(WDTO_1S);
  MCUSR &= ~(1 << WDRF);
  wdt_enable(WDTO_1S);
}

void loop() {
  // ==================================================
  // HW WATCHDOG (MCU)
  // ==================================================
  wdt_reset();

  uint32_t now = millis();
  uint32_t loopStart_us = micros();

  // ==================================================
  // DEBUG (NON-BLOCKING)
  // ==================================================
  debugTestMode(now);
  debugTelemetry(now);
  debugIBus(now);

  // ==================================================
  // PHASE 1 : COMMS
  // ==================================================
  uint32_t tComms_us = micros();
  updateComms(now);
  uint32_t dtComms_us = micros() - tComms_us;

  if (!ibusCommLost) wdCommsOK = true;

  if (dtComms_us > BUDGET_COMMS_MS * 1000UL) {
    latchFault(FaultCode::COMMS_TIMEOUT);
    goto LOOP_FAULT_EXIT;
  }

  // ==================================================
  // PHASE 1 : SENSORS
  // ==================================================
  uint32_t tSensors_us = micros();
  updateSensors();
  uint32_t dtSensors_us = micros() - tSensors_us;

  if (dtSensors_us > BUDGET_SENSORS_MS * 1000UL) {
    latchFault(FaultCode::LOGIC_WATCHDOG);
    goto LOOP_FAULT_EXIT;
  }

  updateEngineState(now);

  // ==================================================
  // DRIVE ANOMALY DETECTION
  // ==================================================
  detectSideImbalanceAndSteer();
  detectWheelStuck(now);
  detectWheelLock();

  SafetyInput sin;
memcpy(sin.curA, curA, sizeof(curA));
sin.tempDriverL = tempDriverL;
sin.tempDriverR = tempDriverR;
sin.faultLatched = faultLatched;

SafetyThresholds sth = {
  CUR_WARN_A,
  CUR_LIMP_A,
  TEMP_WARN_C,
  TEMP_LIMP_C
};

SafetyState rawSafety = evaluateSafetyRaw(sin, sth);

updateSafetyStability(
  rawSafety,
  now,
  autoReverseCount,
  autoReverseActive,
  lastDriveEvent
);


  // ==================================================
  // SYSTEM STATE UPDATE
  // ==================================================
  if (faultLatched) {
    systemState = SystemState::FAULT;
  } else {
    switch (systemState) {
      case SystemState::INIT:
        {
          static uint32_t warmupStart_ms = 0;
          driveSafe();

          if (warmupStart_ms == 0) {
            warmupStart_ms = now;
            break;
          }
          if (now - warmupStart_ms < SENSOR_WARMUP_MS) break;

          if (!readDriverTempsPT100(tempDriverL, tempDriverR)) {
            latchFault(FaultCode::TEMP_SENSOR_FAULT);
            break;
          }
          if (ibus.readChannel(CH_THROTTLE) < 900 || ibus.readChannel(CH_THROTTLE) > 2100) {
            latchFault(FaultCode::IBUS_LOST);
            break;
          }
          for (uint8_t i = 0; i < 4; i++) g_acsOffsetV[i] = 2.5f;

          systemState = SystemState::WAIT_NEUTRAL;
          warmupStart_ms = 0;
          break;
        }

      case SystemState::WAIT_NEUTRAL:
        {
          static uint32_t powerStable_ms = 0;

          if (engineVolt < 24.5f) {
            powerStable_ms = 0;
            break;
          }
          if (powerStable_ms == 0) {
            powerStable_ms = now;
            break;
          }
          if (now - powerStable_ms < 2000) break;

          if (!ibusCommLost && neutral(ibus.readChannel(CH_THROTTLE))) {
            systemState = SystemState::ACTIVE;
            powerStable_ms = 0;
          }
          break;
        }

      case SystemState::WAIT_BLADE_ARM:
        if (ibus.readChannel(CH_ENGINE) < 1200)
          systemState = SystemState::ACTIVE;
        break;

      default:
        break;
    }
  }

  // ==================================================
  // FAULT HANDLING (ABSOLUTE CUT)
  // ==================================================
  if (systemState == SystemState::FAULT) {
    handleFaultImmediateCut();

    // ❗ NO HEARTBEAT HERE
    wdCommsOK = wdSensorOK = wdDriveOK = wdBladeOK = true;
    return;
  }

  // ==================================================
  // EARLY EXIT (NOT ACTIVE)
  // ==================================================
  if (systemState != SystemState::ACTIVE) {
    driveSafe();
    wdDriveOK = wdBladeOK = true;
    goto LOOP_WATCHDOG;
  }

  // ==================================================
  // PHASE 2 : DRIVE / BLADE
  // ==================================================
  uint32_t tDrive_us = micros();
  runDrive(now);
  if (micros() - tDrive_us > BUDGET_DRIVE_MS * 1000UL) {
    latchFault(FaultCode::DRIVE_TIMEOUT);
    goto LOOP_FAULT_EXIT;
  }

  uint32_t tBlade_us = micros();
  runBlade(now);
  if (micros() - tBlade_us > BUDGET_BLADE_MS * 1000UL) {
    latchFault(FaultCode::BLADE_TIMEOUT);
    goto LOOP_FAULT_EXIT;
  }

  applyDrive();

  // ==================================================
  // PHASE 3 : AUX
  // ==================================================
  updateVoltageWarning(now);
  updateDriverFans();
  updateEngineThrottle();
  updateIgnition();
  updateStarter(now);

  // ==================================================
  // STORM32 (STRICT SLAVE)
  // ==================================================
  gimbal.setSystemEnabled(
    systemState == SystemState::ACTIVE && driveSafety != SafetyState::EMERGENCY && !requireIbusConfirm);

  gimbal.update(now);
  telemetryCSV(now);

  if (micros() - loopStart_us > BUDGET_LOOP_MS * 1000UL) {
    latchFault(FaultCode::LOOP_OVERRUN);
    goto LOOP_FAULT_EXIT;
  }

  digitalWrite(PIN_DRV_ENABLE,
               (systemState == SystemState::ACTIVE && !faultLatched && driveSafety != SafetyState::EMERGENCY));

  // ==================================================
  // ❤️ HEARTBEAT (ONLY IF LOOP COMPLETED CLEANLY)
  // ==================================================
  digitalWrite(PIN_HW_WD_HB, !digitalRead(PIN_HW_WD_HB));

LOOP_WATCHDOG:
  logicWatchdog(now);
  return;

LOOP_FAULT_EXIT:
  // ❗ FAULT PATH → NO HEARTBEAT
  wdCommsOK = wdSensorOK = wdDriveOK = wdBladeOK = true;
  return;
}

