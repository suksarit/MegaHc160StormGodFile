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
float readCurrentADC(uint8_t adsChannel, uint8_t idx);
void updateSensors(void);
void latchFault(FaultCode code);
void updateDriverFans(void);
void setFanPWM_L(uint8_t pwm);
void setFanPWM_R(uint8_t pwm);
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


// ============================================================================
// FAULT CONTROL (NO SENSOR STALL)
// ============================================================================
FaultCode activeFault = FaultCode::NONE;
bool faultLatched = false;

// ============================================================================
// EEPROM FAULT WRITE CONTROL (ANTI-WEAR PROTECTION)
// ============================================================================
constexpr uint8_t MAX_FAULT_WRITES_PER_BOOT = 8;     // จำกัดสูงสุดต่อ boot
constexpr uint32_t FAULT_EEPROM_COOLDOWN_MS = 5000;  // เขียนได้ทุก 5 วินาที

static uint8_t faultWriteCount = 0;
static uint32_t lastFaultWriteMs = 0;

// ============================================================================
// FAULT BACKGROUND WRITE QUEUE
// ============================================================================
volatile bool faultWritePending = false;
FaultCode faultToStore = FaultCode::NONE;

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

DriveEvent lastDriveEvent = DriveEvent::NONE;
Storm32Controller gimbal(Serial2, ibus);

bool autoReverseActive = false;
uint32_t autoReverseStart_ms = 0;
constexpr uint16_t AUTO_REV_PWM = 300;
constexpr uint32_t AUTO_REV_MS = 350;
constexpr uint32_t AUTO_REV_RESET_WINDOW_MS = 5000;

Adafruit_ADS1115 adsCur;
Adafruit_ADS1115 adsVolt;
// ============================================================================
// NON-BLOCKING CURRENT AUTO ZERO CALIBRATION
// ============================================================================
enum class ACSCalState : uint8_t {
  IDLE,
  START_CH,
  WAIT_CONV,
  NEXT_SAMPLE,
  NEXT_CH,
  DONE,
  FAIL
};

ACSCalState acsCalState = ACSCalState::IDLE;

bool currentOffsetCalibrated = false;

constexpr uint8_t ACS_CAL_SAMPLE_N = 32;
constexpr uint16_t ACS_CAL_TIMEOUT_MS = 400;

uint8_t acsCalCh = 0;
uint8_t acsCalSampleCnt = 0;
int32_t acsCalSumRaw = 0;
uint32_t acsCalStart_ms = 0;
uint32_t acsCalConvStart_ms = 0;

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
// WATCHDOG DOMAINS (DUAL-LAYER ARCH)
// ============================================================================
constexpr uint16_t WD_SENSOR_TIMEOUT_MS = 120;
constexpr uint8_t TEMP_TIMEOUT_MULTIPLIER = 3;  // ≥2 เท่า WD
constexpr uint16_t TEMP_SENSOR_TIMEOUT_MS =
  WD_SENSOR_TIMEOUT_MS * TEMP_TIMEOUT_MULTIPLIER;

struct WatchdogDomain {
  uint32_t lastUpdate_ms;
  uint32_t timeout_ms;
  bool faulted;
};

WatchdogDomain wdSensor = { 0, WD_SENSOR_TIMEOUT_MS, false };
WatchdogDomain wdComms = { 0, 150, false };
WatchdogDomain wdDrive = { 0, 100, false };
WatchdogDomain wdBlade = { 0, 100, false };

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
// IGNITION LOGICAL STATE (DEBOUNCED / SYSTEM SOURCE OF TRUTH)
// ============================================================================
bool ignitionActive = false;

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

constexpr uint32_t VOLT_SENSOR_TIMEOUT_MS = 1500;  // เดิม 500 → 1.5 วินาที
constexpr uint8_t VOLT_SENSOR_FAIL_COUNT = 3;      // ต้อง fail 3 รอบติดก่อน latch

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
  if (getDriveSafety() == SafetyState::EMERGENCY) return;
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
}

void detectWheelStuck(uint32_t now) {

  if (abs(targetL) < 200 && abs(targetR) < 200) return;

  float cL = curLeft();
  float cR = curRight();

  if (cL > CUR_LIMP_A && cR < CUR_WARN_A) {
    lastDriveEvent = DriveEvent::STUCK_LEFT;
    startAutoReverse(now);
  }

  if (cR > CUR_LIMP_A && cL < CUR_WARN_A) {
    lastDriveEvent = DriveEvent::STUCK_RIGHT;
    startAutoReverse(now);
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
  Serial.print((uint8_t)getDriveSafety());
  Serial.print(',');
  Serial.println((uint8_t)lastDriveEvent);

#endif
}

void updateDriveTarget() {

  if (ibusCommLost || getDriveSafety() == SafetyState::EMERGENCY || systemState != SystemState::ACTIVE) {
    targetL = 0;
    targetR = 0;
    return;
  }

  auto mapAxis = [](int16_t v) -> int16_t {
    constexpr int16_t IN_MIN = 1000;
    constexpr int16_t IN_MAX = 2000;
    constexpr int16_t DB_MIN = 1450;
    constexpr int16_t DB_MAX = 1550;
    constexpr int16_t OUT_MAX = PWM_TOP;

    if (v >= DB_MIN && v <= DB_MAX) return 0;

    if (v < DB_MIN) {
      long m = map(v, IN_MIN, DB_MIN, -OUT_MAX, 0);
      return (int16_t)constrain(m, -OUT_MAX, 0);
    }

    long m = map(v, DB_MAX, IN_MAX, 0, OUT_MAX);
    return (int16_t)constrain(m, 0, OUT_MAX);
  };

  int16_t thr = mapAxis(ibus.readChannel(CH_THROTTLE));
  int16_t str = mapAxis(ibus.readChannel(CH_STEER));

  float absThr = abs(thr);
  float blend;

  if (absThr <= 150.0f) {
    blend = 0.0f;
  } else if (absThr >= 450.0f) {
    blend = 1.0f;
  } else {
    blend = (absThr - 150.0f) / (450.0f - 150.0f);
  }

  float arcL = (float)thr + (float)str;
  float arcR = (float)thr - (float)str;

  float k = abs(str) / (float)PWM_TOP;
  if (str < 0) k = -k;

  float diffL = thr * (1.0f + k);
  float diffR = thr * (1.0f - k);

  float outL = arcL * (1.0f - blend) + diffL * blend;
  float outR = arcR * (1.0f - blend) + diffR * blend;

  float maxMag = max(abs(outL), abs(outR));
  if (maxMag > PWM_TOP) {
    float scale = (float)PWM_TOP / maxMag;
    outL *= scale;
    outR *= scale;
  }

  targetL = (int16_t)outL;
  targetR = (int16_t)outR;
}

void updateEngineThrottle() {

  // ---------- HARD SAFETY ----------
  if (systemState == SystemState::FAULT || getDriveSafety() == SafetyState::EMERGENCY || bladeState != BladeState::RUN) {

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
// latchFault  (NON-BLOCKING / EEPROM MOVED TO BACKGROUND)
// ============================================================================
void latchFault(FaultCode code) {

  if (faultLatched)
    return;

  activeFault = code;
  faultLatched = true;

  // --------------------------------------------------
  // Queue EEPROM write (background task will handle)
  // --------------------------------------------------
  faultToStore = code;
  faultWritePending = true;

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
  Serial.println((uint8_t)getDriveSafety());

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

void backgroundFaultEEPROMTask(uint32_t now) {

  // --------------------------------------------------
  // NOTHING TO DO
  // --------------------------------------------------
  if (!faultWritePending)
    return;

  // --------------------------------------------------
  // ANTI-WEAR LIMITS
  // --------------------------------------------------
  if (faultWriteCount >= MAX_FAULT_WRITES_PER_BOOT)
    return;

  if (now - lastFaultWriteMs < FAULT_EEPROM_COOLDOWN_MS)
    return;

  // --------------------------------------------------
  // READ LAST STORED VALUE (SAFE RAW READ)
  // --------------------------------------------------
  uint8_t raw;
  EEPROM.get(100, raw);

  FaultCode lastStored;

  if (!isValidEnum<FaultCode>(raw)) {
    // EEPROM corruption → treat as NONE
    lastStored = FaultCode::NONE;
  } else {
    lastStored = static_cast<FaultCode>(raw);
  }

  // --------------------------------------------------
  // VALIDATE CURRENT FAULT BEFORE WRITE
  // --------------------------------------------------
  uint8_t newRaw = static_cast<uint8_t>(faultToStore);

  if (!isValidEnum<FaultCode>(newRaw)) {
    // Corrupted RAM value → do NOT write garbage
    faultWritePending = false;
    return;
  }

  // --------------------------------------------------
  // WRITE ONLY IF CHANGED
  // --------------------------------------------------
  if (lastStored != faultToStore) {

    EEPROM.put(100, newRaw);

    faultWriteCount++;
    lastFaultWriteMs = now;

#if DEBUG_SERIAL
    Serial.print(F("[EEPROM] FAULT STORED: "));
    Serial.println(newRaw);
#endif
  }

  faultWritePending = false;
}

void updateComms(uint32_t now) {
  // ==================================================
  // IBUS BYTE PARSING
  // ==================================================
  bool gotByte = false;

  while (Serial1.available()) {
    ibus.loop();
    lastIbusByte_ms = now;
    gotByte = true;
  }

  // ==================================================
  // RECOVERY: BYTE RETURNS
  // ==================================================
  if (gotByte) {
    if (ibusCommLost) {
#if DEBUG_SERIAL
      Serial.println(F("[IBUS] RECOVERED"));
#endif
    }

    ibusCommLost = false;

    // ✔ Dual-layer freshness update
    wdComms.lastUpdate_ms = now;
  }

  // ==================================================
  // SOFT TIMEOUT (COMM LOST → EMERGENCY)
  // ==================================================
  if (now - lastIbusByte_ms > 100) {
    if (!ibusCommLost) {
#if DEBUG_SERIAL
      Serial.println(F("[IBUS] SOFT LOST"));
#endif
      ibusCommLost = true;
      requireIbusConfirm = true;
    }
  }

  // ==================================================
  // HARD TIMEOUT (FAULT)
  // ==================================================
  if (now - lastIbusByte_ms > IBUS_TIMEOUT_MS) {
#if DEBUG_SERIAL
    Serial.println(F("[IBUS] HARD LOST -> FAULT"));
#endif
    latchFault(FaultCode::IBUS_LOST);
    return;
  }

  // ==================================================
  // COMMS HEALTHY → update freshness
  // (กรณีไม่มี byte ใหม่ แต่ยังไม่ timeout)
  // ==================================================
  if (!ibusCommLost) {
    wdComms.lastUpdate_ms = now;
  }
}

// ============================================================================
// NON-BLOCKING AUTO ZERO CURRENT CALIBRATION
// RUN ONLY IN SystemState::INIT
// ============================================================================
bool calibrateCurrentOffsetNonBlocking(uint32_t now) {
  if (currentOffsetCalibrated)
    return true;

  // ต้องอยู่ INIT และ drive ต้อง IDLE เท่านั้น
  if (systemState != SystemState::INIT)
    return false;

  if (driveState != DriveState::IDLE)
    return false;

  // ตัด driver เพื่อความปลอดภัย
  digitalWrite(PIN_DRV_ENABLE, LOW);
  driveSafe();

  switch (acsCalState) {
    // --------------------------------------------------
    case ACSCalState::IDLE:
      acsCalCh = 0;
      acsCalSampleCnt = 0;
      acsCalSumRaw = 0;
      acsCalStart_ms = now;
      acsCalState = ACSCalState::START_CH;
#if DEBUG_SERIAL
      Serial.println(F("[ACS] CAL START"));
#endif
      break;

    // --------------------------------------------------
    case ACSCalState::START_CH:
      {
        if (now - acsCalStart_ms > ACS_CAL_TIMEOUT_MS) {
          acsCalState = ACSCalState::FAIL;
          break;
        }

        uint16_t mux =
          ADS1X15_REG_CONFIG_MUX_SINGLE_0 + (ADS_CUR_CH_MAP[acsCalCh] << 12);

        adsCur.startADCReading(mux, false);

        acsCalConvStart_ms = now;
        acsCalState = ACSCalState::WAIT_CONV;
        break;
      }

    // --------------------------------------------------
    case ACSCalState::WAIT_CONV:
      {
        if (adsCur.conversionComplete()) {
          int16_t raw = adsCur.getLastConversionResults();
          acsCalSumRaw += raw;
          acsCalSampleCnt++;
          acsCalState = ACSCalState::NEXT_SAMPLE;
        } else if (now - acsCalConvStart_ms > 30) {
          // conversion timeout
          acsCalState = ACSCalState::FAIL;
        }
        break;
      }

    // --------------------------------------------------
    case ACSCalState::NEXT_SAMPLE:
      {
        if (acsCalSampleCnt >= ACS_CAL_SAMPLE_N) {
          int16_t avgRaw = acsCalSumRaw / ACS_CAL_SAMPLE_N;
          float v = avgRaw * ADS1115_LSB_V;

          // plausibility guard
          if (v < 2.0f || v > 3.0f) {
            acsCalState = ACSCalState::FAIL;
            break;
          }

          g_acsOffsetV[acsCalCh] = v;

#if DEBUG_SERIAL
          Serial.print(F("[ACS] CH"));
          Serial.print(acsCalCh);
          Serial.print(F(" OFFSET="));
          Serial.println(v, 4);
#endif

          acsCalState = ACSCalState::NEXT_CH;
        } else {
          acsCalState = ACSCalState::START_CH;
        }
        break;
      }

    // --------------------------------------------------
    case ACSCalState::NEXT_CH:
      {
        acsCalCh++;
        acsCalSampleCnt = 0;
        acsCalSumRaw = 0;

        if (acsCalCh >= 4) {
          acsCalState = ACSCalState::DONE;
        } else {
          acsCalState = ACSCalState::START_CH;
        }
        break;
      }

    // --------------------------------------------------
    case ACSCalState::DONE:
      currentOffsetCalibrated = true;
#if DEBUG_SERIAL
      Serial.println(F("[ACS] CAL DONE"));
#endif
      return true;

    // --------------------------------------------------
    case ACSCalState::FAIL:
#if DEBUG_SERIAL
      Serial.println(F("[ACS] CAL FAIL"));
#endif
      latchFault(FaultCode::CUR_SENSOR_FAULT);
      return false;
  }

  return false;
}

void updateSensors() {

  uint32_t now = millis();

  // ==================================================
  // SENSOR TASK ALIVE (ANTI-FALSE WATCHDOG)
  // ==================================================
  wdSensor.lastUpdate_ms = now;

#if TEST_MODE
  curA[0] = curA[1] = curA[2] = curA[3] = 5.0f;
  tempDriverL = 45;
  tempDriverR = 47;
  engineVolt = 26.0f;
  return;
#endif

  // ==================================================
  // ---------- I2C RECOVERY STATE MACHINE ----------
  // ==================================================
  enum class I2CRecoverState : uint8_t {
    IDLE,
    END_BUS,
    BEGIN_BUS,
    REINIT_ADS,
    DONE
  };

  static I2CRecoverState i2cState = I2CRecoverState::IDLE;
  static uint32_t i2cStateStart_ms = 0;
  static uint8_t i2cResetCount = 0;

  if (Wire.getWireTimeoutFlag() && i2cState == I2CRecoverState::IDLE) {
    Wire.clearWireTimeoutFlag();
    i2cState = I2CRecoverState::END_BUS;
    i2cStateStart_ms = now;
  }

  switch (i2cState) {

    case I2CRecoverState::END_BUS:
      Wire.end();
      i2cState = I2CRecoverState::BEGIN_BUS;
      break;

    case I2CRecoverState::BEGIN_BUS:
      if (now - i2cStateStart_ms >= 1) {
        Wire.begin();
        Wire.setClock(100000);
        Wire.setWireTimeout(6000, true);
        i2cState = I2CRecoverState::REINIT_ADS;
      }
      break;

    case I2CRecoverState::REINIT_ADS:
      adsCur.begin(0x48);
      adsCur.setGain(GAIN_ONE);
      adsCur.setDataRate(RATE_ADS1115_860SPS);

      adsVolt.begin(0x49);
      adsVolt.setGain(GAIN_ONE);
      adsVolt.setDataRate(RATE_ADS1115_860SPS);

      i2cResetCount++;
      i2cState = I2CRecoverState::DONE;
      break;

    case I2CRecoverState::DONE:
      if (i2cResetCount >= 3) {
        latchFault(FaultCode::VOLT_SENSOR_FAULT);
        return;
      }
      i2cState = I2CRecoverState::IDLE;
      break;

    default:
      break;
  }

  // ==================================================
  // HARDWARE OVERCURRENT
  // ==================================================
  if (digitalRead(PIN_CUR_TRIP) == LOW) {
    latchFault(FaultCode::OVER_CURRENT);
    return;
  }

  // ==================================================
  // ----------- ASYNC CURRENT (WITH STALL GUARD) -----
  // ==================================================
  static uint8_t curIdx = 0;
  static bool curConvRunning = false;
  static uint32_t curConvStart_ms = 0;

  constexpr uint16_t CUR_CONV_TIMEOUT_MS = 25;

  if (!curConvRunning) {

    uint16_t mux =
      ADS1X15_REG_CONFIG_MUX_SINGLE_0 + (ADS_CUR_CH_MAP[curIdx] << 12);

    adsCur.startADCReading(mux, false);
    curConvRunning = true;
    curConvStart_ms = now;

  } else {

    if (adsCur.conversionComplete()) {

      int16_t raw = adsCur.getLastConversionResults();
      float v = raw * ADS1115_LSB_V;
      float a = (v - g_acsOffsetV[curIdx]) / ACS_SENS_V_PER_A;

      if (a < CUR_MIN_PLAUSIBLE || a > CUR_MAX_PLAUSIBLE) {
        latchFault(FaultCode::CUR_SENSOR_FAULT);
        return;
      }

      curA[curIdx] += CUR_LPF_ALPHA * (a - curA[curIdx]);

      if (a > CUR_SPIKE_A) {
        forceDriveSoftStop(now);
        bladeState = BladeState::SOFT_STOP;
      }

      if (a > CUR_TRIP_A_CH[curIdx]) {
        if (++overCurCnt[curIdx] >= 2) {
          latchFault(FaultCode::OVER_CURRENT);
          return;
        }
      } else {
        overCurCnt[curIdx] = 0;
      }

      curIdx++;
      if (curIdx >= 4) curIdx = 0;

      curConvRunning = false;
    } else if (now - curConvStart_ms > CUR_CONV_TIMEOUT_MS) {

#if DEBUG_SERIAL
      Serial.println(F("[ADS] CURRENT CONVERSION TIMEOUT"));
#endif
      curConvRunning = false;
    }
  }

  // ==================================================
  // ----------- ASYNC VOLTAGE (WITH STALL GUARD) -----
  // ==================================================
  static bool voltConvRunning = false;
  static uint32_t voltConvStart_ms = 0;
  static uint32_t lastVoltOk_ms = 0;
  static uint8_t voltFailCnt = 0;

  constexpr uint16_t VOLT_CONV_TIMEOUT_MS = 25;

  if (!voltConvRunning) {
    adsVolt.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false);
    voltConvRunning = true;
    voltConvStart_ms = now;
  } else {

    if (adsVolt.conversionComplete()) {

      int16_t raw = adsVolt.getLastConversionResults();
      float v_adc = raw * ADS1115_LSB_V;
      float vRaw = v_adc * ((150.0f + 33.0f) / 33.0f);

      if (vRaw > 0.0f && vRaw < 40.0f) {
        engineVolt += 0.15f * (vRaw - engineVolt);
        lastVoltOk_ms = now;
      }

      voltConvRunning = false;
    } else if (now - voltConvStart_ms > VOLT_CONV_TIMEOUT_MS) {

#if DEBUG_SERIAL
      Serial.println(F("[ADS] VOLT CONVERSION TIMEOUT"));
#endif
      voltConvRunning = false;
    }
  }

  // -------- Voltage timeout logic --------
  if (now - lastVoltOk_ms > VOLT_SENSOR_TIMEOUT_MS) {

    if (++voltFailCnt >= VOLT_SENSOR_FAIL_COUNT) {
      latchFault(FaultCode::VOLT_SENSOR_FAULT);
      return;
    }

  } else {
    voltFailCnt = 0;
  }

  // ==================================================
  // ----------- TEMPERATURE --------------------------
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

  if (now - lastTempOk_ms > TEMP_SENSOR_TIMEOUT_MS) {
#if DEBUG_SERIAL
    Serial.println(F("[TEMP] SENSOR TIMEOUT"));
#endif
    latchFault(FaultCode::TEMP_SENSOR_FAULT);
    return;
  }
}

void runDrive(uint32_t now) {

  static DriveState lastDriveState = DriveState::IDLE;
  static uint32_t limpSafeStart_ms = 0;

  switch (driveState) {

    // --------------------------------------------------
    case DriveState::IDLE:
      targetL = 0;
      targetR = 0;
      curL = 0;
      curR = 0;
      limpSafeStart_ms = 0;
      driveSoftStopStart_ms = 0;

      if (systemState == SystemState::ACTIVE) {
        driveState = DriveState::RUN;
      }
      break;

    // --------------------------------------------------
    case DriveState::RUN:
      updateDriveTarget();

      if (getDriveSafety() == SafetyState::EMERGENCY) {
        driveState = DriveState::SOFT_STOP;
      } else if (getDriveSafety() == SafetyState::LIMP) {
        driveState = DriveState::LIMP;
        limpSafeStart_ms = 0;
      }
      break;

    // --------------------------------------------------
    case DriveState::LIMP:
      updateDriveTarget();
      targetL /= 2;
      targetR /= 2;

      if (getDriveSafety() == SafetyState::EMERGENCY) {
        driveState = DriveState::SOFT_STOP;
        break;
      }

      if (getDriveSafety() == SafetyState::SAFE) {
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

      if ((curL == 0 && curR == 0) || (now - driveSoftStopStart_ms >= DRIVE_SOFT_STOP_TIMEOUT_MS)) {

        driveSafe();
        driveState = DriveState::LOCKED;
      }
      break;

    // --------------------------------------------------
    case DriveState::LOCKED:
      driveSafe();
      break;

    // --------------------------------------------------
    default:
      // ENUM CORRUPTION PROTECTION
      latchFault(FaultCode::LOGIC_WATCHDOG);

      driveSafe();
      targetL = 0;
      targetR = 0;
      curL = 0;
      curR = 0;

      driveState = DriveState::LOCKED;
      limpSafeStart_ms = 0;
      driveSoftStopStart_ms = 0;
      break;
  }

  // ===================================================
  // TRANSITION DEBUG + SOFT STOP INIT
  // ===================================================
  if (driveState != lastDriveState) {

#if DEBUG_SERIAL
    Serial.print(F("[DRIVE STATE] "));
    Serial.print((uint8_t)lastDriveState);
    Serial.print(F(" -> "));
    Serial.println((uint8_t)driveState);
#endif

    if (driveState == DriveState::SOFT_STOP) {
      driveSoftStopStart_ms = now;
    }

    lastDriveState = driveState;
  }

  // ===================================================
  // DRIVE WATCHDOG FEED
  // ===================================================
  wdDrive.lastUpdate_ms = now;
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
      if (faultLatched || getDriveSafety() == SafetyState::EMERGENCY) {
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

  wdBlade.lastUpdate_ms = now;
}

void monitorSubsystemWatchdogs(uint32_t now) {
  if (now - wdSensor.lastUpdate_ms > wdSensor.timeout_ms) {
    latchFault(FaultCode::LOGIC_WATCHDOG);
    wdSensor.faulted = true;
  }

  if (now - wdComms.lastUpdate_ms > wdComms.timeout_ms) {
    latchFault(FaultCode::IBUS_LOST);
    wdComms.faulted = true;
  }

  if (now - wdDrive.lastUpdate_ms > wdDrive.timeout_ms) {
    latchFault(FaultCode::DRIVE_TIMEOUT);
    wdDrive.faulted = true;
  }

  if (now - wdBlade.lastUpdate_ms > wdBlade.timeout_ms) {
    latchFault(FaultCode::BLADE_TIMEOUT);
    wdBlade.faulted = true;
  }
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
  // AUTO REVERSE OVERRIDE (DIRECTION-AWARE)
  // ==================================================
  if (autoReverseActive) {

    if (now - autoReverseStart_ms < AUTO_REV_MS) {

      // -----------------------------------------------
      // Determine reverse direction per wheel
      // Reverse opposite of current movement
      // -----------------------------------------------

      int8_t dirL = (curL > 0) ? -1 : (curL < 0 ? 1 : 0);
      int8_t dirR = (curR > 0) ? -1 : (curR < 0 ? 1 : 0);

      // ถ้าล้อใดหยุดนิ่ง ให้ถือว่า reverse ตรง
      if (dirL == 0) dirL = -1;
      if (dirR == 0) dirR = -1;

      // LEFT
      digitalWrite(DIR_L1, dirL > 0);
      digitalWrite(DIR_L2, dirL < 0);

      // RIGHT
      digitalWrite(DIR_R1, dirR > 0);
      digitalWrite(DIR_R2, dirR < 0);

      setPWM_L(AUTO_REV_PWM);
      setPWM_R(AUTO_REV_PWM);

      return;  // override normal drive
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
  if (getDriveSafety() == SafetyState::EMERGENCY) {
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
// FAULT HANDLER (SINGLE POINT OF CUT / HARD SAFE)
// ============================================================================
void handleFaultImmediateCut() {
#if DEBUG_SERIAL
  static bool printed = false;
  if (!printed) {
    Serial.println(F("[FAULT] IMMEDIATE CUT"));
    printed = true;
  }
#endif

  // --------------------------------------------------
  // 1) DISABLE DRIVER POWER
  // --------------------------------------------------
  digitalWrite(PIN_DRV_ENABLE, LOW);

  // --------------------------------------------------
  // 2) CUT DRIVE OUTPUT
  // --------------------------------------------------
  driveSafe();
  curL = 0;
  curR = 0;
  targetL = 0;
  targetR = 0;

  // --------------------------------------------------
  // 3) CUT BLADE THROTTLE
  // --------------------------------------------------
  bladeServo.writeMicroseconds(1000);

  // --------------------------------------------------
  // 4) CUT STARTER
  // --------------------------------------------------
  starterActive = false;
  digitalWrite(RELAY_STARTER, LOW);
}

void updateIgnition() {

  static bool ignitionLatched = false;  // logical debounced state
  static bool lastRawRequest = false;
  static uint32_t edgeStart_ms = 0;

  constexpr uint32_t IGNITION_DEBOUNCE_MS = 50;

  uint32_t now = millis();

  uint16_t ch6 = ibus.readChannel(CH_IGNITION);
  bool rawRequest = (ch6 > 1600);

  // --------------------------------------------------
  // HARD SAFETY
  // --------------------------------------------------
  if (faultLatched || ibusCommLost) {

    ignitionLatched = false;
    ignitionActive = false;  // <<< UPDATE LOGICAL STATE

    digitalWrite(RELAY_IGNITION, LOW);

    lastRawRequest = rawRequest;
    edgeStart_ms = 0;
    return;
  }

  // --------------------------------------------------
  // EDGE DETECTION
  // --------------------------------------------------
  if (rawRequest != lastRawRequest) {
    edgeStart_ms = now;
    lastRawRequest = rawRequest;
  }

  // --------------------------------------------------
  // DEBOUNCE
  // --------------------------------------------------
  if (edgeStart_ms != 0 && (now - edgeStart_ms >= IGNITION_DEBOUNCE_MS)) {

    ignitionLatched = rawRequest;
    edgeStart_ms = 0;
  }

  // --------------------------------------------------
  // UPDATE GLOBAL LOGICAL STATE
  // --------------------------------------------------
  ignitionActive = ignitionLatched;

  // --------------------------------------------------
  // APPLY OUTPUT
  // --------------------------------------------------
  digitalWrite(RELAY_IGNITION,
               ignitionLatched ? HIGH : LOW);
}

void updateStarter(uint32_t now) {

  uint16_t ch10 = ibus.readChannel(CH_STARTER);
  bool requestStart = (ch10 > 1600);

  bool ignitionOn = ignitionActive;

  // =====================================================
  // 1️⃣ HARD SAFETY GATE (BLOCK EVERYTHING)
  // =====================================================
  if (systemState == SystemState::FAULT || driveState != DriveState::IDLE || ibus.readChannel(CH_ENGINE) > 1100 || !neutral(ibus.readChannel(CH_THROTTLE)) || !ignitionOn || engineRunning || (engineStopped_ms != 0 && now - engineStopped_ms < ENGINE_RESTART_GUARD_MS)) {

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
// VOLTAGE WARNING (AUTO LOGIC ONLY – NO CH_RESET)
// ============================================================================
void updateVoltageWarning(uint32_t now) {
  static uint32_t lastToggle_ms = 0;
  static bool buzzerOn = false;
  float v24 = engineVolt;

  if (now - wdSensor.lastUpdate_ms > wdSensor.timeout_ms) {
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(RELAY_WARN, LOW);
    buzzerOn = false;
    return;
  }

  if (v24 >= V_WARN_LOW) {
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(RELAY_WARN, LOW);
    buzzerOn = false;
    return;
  }

  // LEVEL 1
  if (v24 < V_WARN_LOW && v24 >= V_WARN_CRITICAL) {
    digitalWrite(RELAY_WARN, HIGH);

    if (now - lastToggle_ms >= 500) {
      lastToggle_ms = now;
      buzzerOn = !buzzerOn;
      digitalWrite(PIN_BUZZER, buzzerOn ? HIGH : LOW);
    }
    return;
  }

  // LEVEL 2
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
#endif
}

// ============================================================================
// SYSTEM STATE MACHINE (IGNITION = MASTER SAFETY KEY)
// ============================================================================
void updateSystemState(uint32_t now) {
  static uint32_t ibusStableStart_ms = 0;

  bool ignitionOn = ignitionActive;

  // ------------------------------------------------
  // HARD FAULT PRIORITY
  // ------------------------------------------------
  if (faultLatched) {
    systemState = SystemState::FAULT;
    return;
  }

  // ------------------------------------------------
  // IGNITION OFF = DISARM SYSTEM
  // ------------------------------------------------
  if (!ignitionOn) {
    systemState = SystemState::INIT;
    ibusStableStart_ms = 0;
    return;
  }

  switch (systemState) {
    // --------------------------------------------
    case SystemState::INIT:
      {
        if (ibusCommLost) {
          ibusStableStart_ms = 0;
          return;
        }

        if (ibusStableStart_ms == 0) {
          ibusStableStart_ms = now;
          return;
        }

        if (now - ibusStableStart_ms < 1000)
          return;

        if (!neutral(ibus.readChannel(CH_THROTTLE)))
          return;

        if (!neutral(ibus.readChannel(CH_STEER)))
          return;

        if (!calibrateCurrentOffsetNonBlocking(now))
          return;

        systemState = SystemState::ACTIVE;
        break;
      }

    // --------------------------------------------
    case SystemState::ACTIVE:
      {
        if (ibusCommLost) {
          systemState = SystemState::INIT;
          ibusStableStart_ms = 0;
        }
        break;
      }

    // --------------------------------------------
    case SystemState::FAULT:
      break;

    default:
      latchFault(FaultCode::LOGIC_WATCHDOG);
      systemState = SystemState::FAULT;
      break;
  }
}

// ============================================================================
// SAFE FAULT RESET VIA IGNITION TOGGLE (SINGLE SAFETY KEY)
// ต้องดับ ignition ≥3 วินาที แล้วเปิดใหม่ พร้อมเงื่อนไข safe ครบ
// ============================================================================
void processFaultReset(uint32_t now) {
  static bool ignitionWasOn = false;
  static uint32_t ignitionOffStart_ms = 0;
  static bool ignitionOffQualified = false;

  if (!faultLatched)
    return;

  bool ignitionNow = ignitionActive;

  // --------------------------------------------------
  // DETECT IGNITION OFF PERIOD
  // --------------------------------------------------
  if (!ignitionNow) {
    if (ignitionWasOn)
      ignitionOffStart_ms = now;

    if (ignitionOffStart_ms != 0 && (now - ignitionOffStart_ms >= 3000)) {
      ignitionOffQualified = true;
    }
  }

  // --------------------------------------------------
  // SAFE CONDITIONS
  // --------------------------------------------------
  bool throttleNeutral = neutral(ibus.readChannel(CH_THROTTLE));
  bool steerNeutral = neutral(ibus.readChannel(CH_STEER));
  bool engineIdle = (ibus.readChannel(CH_ENGINE) < 1100);

  bool driveIdle =
    (driveState == DriveState::IDLE || driveState == DriveState::LOCKED);

  bool bladeSafe =
    (bladeState == BladeState::IDLE || bladeState == BladeState::LOCKED);

  bool currentSafe = true;
  for (uint8_t i = 0; i < 4; i++)
    if (curA[i] > CUR_WARN_A)
      currentSafe = false;

  bool tempSafe =
    (tempDriverL < TEMP_WARN_C && tempDriverR < TEMP_WARN_C);

  bool voltSafe = (engineVolt > V_WARN_LOW);

  bool safetyClear =
    (getDriveSafety() != SafetyState::EMERGENCY);

  bool allSafe =
    throttleNeutral && steerNeutral && engineIdle && driveIdle && bladeSafe && currentSafe && tempSafe && voltSafe && safetyClear;

  // --------------------------------------------------
  // EXECUTE RESET ON RISING EDGE OF IGNITION
  // --------------------------------------------------
  if (!ignitionWasOn && ignitionNow) {
    if (ignitionOffQualified && allSafe) {
#if DEBUG_SERIAL
      Serial.println(F("[FAULT] RESET VIA IGNITION TOGGLE"));
#endif

      faultLatched = false;
      activeFault = FaultCode::NONE;

      systemState = SystemState::INIT;
      driveState = DriveState::IDLE;
      bladeState = BladeState::IDLE;
      forceSafetyState(SafetyState::SAFE);
    }

    ignitionOffQualified = false;
    ignitionOffStart_ms = 0;
  }

  ignitionWasOn = ignitionNow;
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
  adsCur.setDataRate(RATE_ADS1115_860SPS);
  adsVolt.begin(0x49);  // ADS1115 แรงดัน
  adsVolt.setGain(GAIN_ONE);
  adsVolt.setDataRate(RATE_ADS1115_860SPS);
  // --------------------------------------------------
  // READ LAST FAULT FROM EEPROM (FAULT HISTORY)
  // --------------------------------------------------
  uint8_t raw;
  EEPROM.get(100, raw);

  if (!isValidEnum<FaultCode>(raw)) {
    raw = 0;  // หรือ FaultCode::NONE
  }

  FaultCode lastFault = static_cast<FaultCode>(raw);

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
  if (!isValidEnum<SystemState>(static_cast<uint8_t>(systemState))) {
    systemState = SystemState::FAULT;
  }
  driveState = DriveState::IDLE;
  bladeState = BladeState::IDLE;

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

  // --------------------------------------------------
  // INIT WATCHDOG DOMAINS (CRITICAL: PREVENT BOOT FALSE TRIP)
  // --------------------------------------------------
  {
    uint32_t now = millis();

    wdSensor.lastUpdate_ms = now;
    wdComms.lastUpdate_ms = now;
    wdDrive.lastUpdate_ms = now;
    wdBlade.lastUpdate_ms = now;

    wdSensor.faulted = false;
    wdComms.faulted = false;
    wdDrive.faulted = false;
    wdBlade.faulted = false;
  }
  // --------------------------------------------------
  // HW WATCHDOG (LAST)
  // --------------------------------------------------
  wdt_reset();
  wdt_enable(WDTO_1S);
  MCUSR &= ~(1 << WDRF);
  wdt_enable(WDTO_1S);
}

void loop() {

  wdt_reset();

  uint32_t now = millis();
  uint32_t loopStart_us = micros();

  // ==================================================
  // DEBUG / TELEMETRY (MUTUALLY EXCLUSIVE)
  // ==================================================
#if DEBUG_SERIAL && !TELEMETRY_CSV
  debugTestMode(now);
  debugTelemetry(now);
  debugIBus(now);
#endif

#if TELEMETRY_CSV
  telemetryCSV(now);
#endif

  // ==================================================
  // PHASE 1 : COMMS
  // ==================================================
  uint32_t tComms_us = micros();

  updateComms(now);
  updateIgnition();
  updateSystemState(now);

  if (micros() - tComms_us > BUDGET_COMMS_MS * 1000UL) {
    latchFault(FaultCode::COMMS_TIMEOUT);
  }

  // ==================================================
  // PHASE 2 : SENSORS
  // ==================================================
  updateSensors();

  if (now - wdSensor.lastUpdate_ms > wdSensor.timeout_ms) {
    latchFault(FaultCode::LOGIC_WATCHDOG);
  }

  updateEngineState(now);

  detectSideImbalanceAndSteer();
  detectWheelStuck(now);
  detectWheelLock();

  // ==================================================
  // SAFETY MANAGER
  // ==================================================
  SafetyInput sin;
  memcpy(sin.curA, curA, sizeof(curA));
  sin.tempDriverL = tempDriverL;
  sin.tempDriverR = tempDriverR;
  sin.faultLatched = faultLatched;
  sin.driveEvent = lastDriveEvent;

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
    lastDriveEvent);

  // ==================================================
  // SYSTEM STATE UPDATE
  // ==================================================
  if (faultLatched) {
    systemState = SystemState::FAULT;
  }

  processFaultReset(now);

  bool emergencyActive =
    (systemState == SystemState::FAULT) || (getDriveSafety() == SafetyState::EMERGENCY);

  // ==================================================
  // SYSTEM GATE (EMERGENCY BOUNDARY)
  // ==================================================
  if (systemState != SystemState::ACTIVE || emergencyActive) {

    handleFaultImmediateCut();
    digitalWrite(PIN_DRV_ENABLE, LOW);

    // ----- Background tasks MUST run even in fault -----
    backgroundFaultEEPROMTask(now);

    // ----- Watchdog domains still monitored -----
    monitorSubsystemWatchdogs(now);

    // ----- Loop overrun guard still active -----
    if (micros() - loopStart_us > BUDGET_LOOP_MS * 1000UL) {
      latchFault(FaultCode::LOOP_OVERRUN);
    }

    // ----- Heartbeat -----
    digitalWrite(PIN_HW_WD_HB,
                 !digitalRead(PIN_HW_WD_HB));

    return;
  }

  // ==================================================
  // PHASE 3 : DRIVE
  // ==================================================
  uint32_t tDrive_us = micros();
  runDrive(now);

  if (micros() - tDrive_us > BUDGET_DRIVE_MS * 1000UL) {
    latchFault(FaultCode::DRIVE_TIMEOUT);
  }

  uint32_t tBlade_us = micros();
  runBlade(now);

  if (micros() - tBlade_us > BUDGET_BLADE_MS * 1000UL) {
    latchFault(FaultCode::BLADE_TIMEOUT);
  }

  applyDrive();

  // ==================================================
  // PHASE 4 : AUX
  // ==================================================
  updateVoltageWarning(now);
  updateDriverFans();
  updateEngineThrottle();
  updateStarter(now);

  gimbal.setSystemEnabled(
    systemState == SystemState::ACTIVE && !emergencyActive && !requireIbusConfirm);

  gimbal.update(now);

  // ==================================================
  // BACKGROUND TASKS
  // ==================================================
  monitorSubsystemWatchdogs(now);
  backgroundFaultEEPROMTask(now);

  // ==================================================
  // LOOP OVERRUN GUARD
  // ==================================================
  if (micros() - loopStart_us > BUDGET_LOOP_MS * 1000UL) {
    latchFault(FaultCode::LOOP_OVERRUN);
  }

  // ==================================================
  // DRIVER ENABLE CONTROL
  // ==================================================
  static uint32_t driveEnableArmStart_ms = 0;

  bool runAllowed =
    (driveState == DriveState::RUN || driveState == DriveState::LIMP);

  bool hardCut =
    emergencyActive || (driveState == DriveState::SOFT_STOP) || (driveState == DriveState::LOCKED);

  if (hardCut) {

    driveEnableArmStart_ms = 0;
    digitalWrite(PIN_DRV_ENABLE, LOW);

  } else if (runAllowed) {

    if (driveEnableArmStart_ms == 0)
      driveEnableArmStart_ms = now;

    if (now - driveEnableArmStart_ms >= 50)
      digitalWrite(PIN_DRV_ENABLE, HIGH);
    else
      digitalWrite(PIN_DRV_ENABLE, LOW);

  } else {

    driveEnableArmStart_ms = 0;
    digitalWrite(PIN_DRV_ENABLE, LOW);
  }

  // ==================================================
  // HEARTBEAT
  // ==================================================
  digitalWrite(PIN_HW_WD_HB,
               !digitalRead(PIN_HW_WD_HB));
}

