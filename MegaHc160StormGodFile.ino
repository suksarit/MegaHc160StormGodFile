// ========================================================================================
// MegaHc160Storm32.ino  - By TN MOWER
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
#define TELEMETRY_CSV 1  // 1=เปิด 0=ปิด : CSV telemetry
#define TELEMETRY_PERIOD_MS 200
#define TEST_MODE 0  // 1=ทดสอบ ไม่มีเซ็นเซฮร์ , 0=สนามจริง มีเซ็นเซอร์

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
bool updateSensors(void);
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

// --------------------------------------------------
// AUTO REVERSE RECOVERY GUARD
// --------------------------------------------------

bool reverseRecoveryActive = false;
uint32_t reverseRecoveryStart_ms = 0;

constexpr uint32_t REVERSE_RECOVERY_MS = 800;

// ============================================================================
// FAULT CONTROL (NO SENSOR STALL)
// ============================================================================
FaultCode activeFault = FaultCode::NONE;
bool faultLatched = false;
bool rcNeutralConfirmed = false;
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
uint32_t ibusRecoverStart_ms = 0;
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

// ==================================================
// DRIVER REARM FLAG (GLOBAL)
// ==================================================
bool driverRearmRequired = true;

// ============================================================================
// MOTOR STALL ENERGY PROTECTION
// ============================================================================

float stallEnergy = 0.0f;
uint32_t stallEnergyLast_ms = 0;

constexpr float STALL_CURRENT_A = 65.0f;   // current threshold
constexpr float STALL_POWER_LIMIT = 1.0f;  // energy limit
constexpr float STALL_DECAY = 0.92f;       // energy decay per loop

// ==================================================
// IMBALANCE CORRECTION LAYER (NO TARGET DRIFT)
// ==================================================
int16_t imbalanceCorrL = 0;
int16_t imbalanceCorrR = 0;

uint32_t driveSoftStopStart_ms = 0;
uint32_t bladeSoftStopStart_ms = 0;

// ============================================================================
// SENSOR RUNTIME VALUES
// ============================================================================
float curA[4] = { 0 };
float curA_snapshot[4] = { 0 };
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
bool adsCurPresent = false;
bool adsVoltPresent = false;

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
// INDUSTRIAL LOOP TIMING SUPERVISOR
// ============================================================================
static int32_t loopOverrunAccum_us = 0;

// ============================================================================
// PHASE BUDGET CONFIRM (GLOBAL)
// ============================================================================
uint8_t commsBudgetCnt = 0;
uint8_t driveBudgetCnt = 0;
uint8_t bladeBudgetCnt = 0;

constexpr uint8_t PHASE_BUDGET_CONFIRM = 3;

constexpr int32_t LOOP_OVERRUN_FAULT_US = 40000;   // sustained overload
constexpr int32_t LOOP_OVERRUN_RECOVER_US = 2000;  // decay per healthy loop
constexpr uint32_t LOOP_HARD_LIMIT_US =
  BUDGET_LOOP_MS * 2000UL;  // 2x budget immediate kill

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

// ======================================================
// FREE RAM (SRAM MONITOR)
// AVR SRAM calculation
//
// วิธีคำนวณ
// free RAM = stack_top - heap_top
//
// heap_top = __brkval
// ถ้า heap ยังไม่ถูก allocate (__brkval == 0)
// ให้ใช้ __heap_start แทน
//
// NOTE:
// __heap_start และ __brkval เป็น symbol ภายในของ AVR libc
// บาง toolchain อาจไม่ได้ประกาศใน header
// จึงต้อง declare extern เอง
// ======================================================

#if defined(__AVR__)

extern unsigned int __heap_start;
extern void *__brkval;

int freeRam()
{
  int stackTop;

  void* heapTop = (__brkval == 0)
                    ? (void*)&__heap_start
                    : __brkval;

  return (int)&stackTop - (int)heapTop;
}

#else

int freeRam()
{
  return -1; // not supported on non-AVR platforms
}

#endif

// ------------------------------------------------------

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
// MOTOR SHORT BRAKE
// ใช้สำหรับหยุด inertia ก่อน reverse
// ============================================================================
void motorShortBrake() {

  // ตัด PWM ก่อน
  setPWM_L(0);
  setPWM_R(0);

  // short brake
  digitalWrite(DIR_L1, HIGH);
  digitalWrite(DIR_L2, HIGH);

  digitalWrite(DIR_R1, HIGH);
  digitalWrite(DIR_R2, HIGH);
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
  return curA_snapshot[0] + curA_snapshot[1];
}

inline float curRight() {
  return curA_snapshot[2] + curA_snapshot[3];
}

// ============================================================================
// MOTOR STALL ENERGY PROTECTION
// จำกัดพลังงานเมื่อมอเตอร์ติดหรือล้อไม่หมุน
// ============================================================================

float computeStallScale(uint32_t now, float curL_A, float curR_A) {

  float curMax = max(curL_A, curR_A);

  uint32_t dt = now - stallEnergyLast_ms;
  stallEnergyLast_ms = now;

  if (dt > 200) dt = 200;

  float dtSec = dt * 0.001f;

  // --------------------------------------------------
  // ENERGY INTEGRATION
  // --------------------------------------------------

  if (curMax > STALL_CURRENT_A) {

    float excess = curMax - STALL_CURRENT_A;

    stallEnergy += excess * dtSec * 0.02f;

  } else {

    stallEnergy *= STALL_DECAY;
  }

  if (stallEnergy < 0)
    stallEnergy = 0;

  // --------------------------------------------------
  // SCALE COMPUTATION
  // --------------------------------------------------

  if (stallEnergy < STALL_POWER_LIMIT)
    return 1.0f;

  float scale = STALL_POWER_LIMIT / stallEnergy;

  if (scale < 0.25f)
    scale = 0.25f;

  return scale;
}

// ============================================================================
// AUTO REVERSE (IMPROVED FIELD VERSION)
// ป้องกัน reverse oscillation ในหญ้าหนา
// ============================================================================
void startAutoReverse(uint32_t now) {

  // --------------------------------------------------
  // SAFETY GUARD
  // --------------------------------------------------
  if (getDriveSafety() != SafetyState::SAFE)
    return;

  // --------------------------------------------------
  // RECOVERY WINDOW
  // ป้องกัน reverse ซ้ำทันทีหลังจาก reverse จบ
  // --------------------------------------------------
  if (reverseRecoveryActive) {

    if (now - reverseRecoveryStart_ms < REVERSE_RECOVERY_MS)
      return;

    reverseRecoveryActive = false;
  }

  // --------------------------------------------------
  // AUTO REVERSE WINDOW RESET
  // --------------------------------------------------
  static uint32_t autoRevWindowStart_ms = 0;

  if (autoRevWindowStart_ms == 0)
    autoRevWindowStart_ms = now;

  if (now - autoRevWindowStart_ms > AUTO_REV_RESET_WINDOW_MS) {

    autoReverseCount = 0;
    autoRevWindowStart_ms = now;
  }

  // --------------------------------------------------
  // HARD GUARD
  // --------------------------------------------------
  if (autoReverseActive)
    return;

  if (autoReverseCount >= MAX_AUTO_REVERSE) {

#if DEBUG_SERIAL
    Serial.println(F("[AUTO REV] ESCALATE -> STUCK FAULT"));
#endif

    latchFault(FaultCode::OVER_CURRENT);
    return;
  }

  // --------------------------------------------------
  // COOLDOWN (CRITICAL FOR FIELD)
  // --------------------------------------------------
  static uint32_t lastAutoRev_ms = 0;
  constexpr uint32_t AUTO_REV_COOLDOWN_MS = 1500;

  if (lastAutoRev_ms != 0 && now - lastAutoRev_ms < AUTO_REV_COOLDOWN_MS) {
    return;
  }

  // --------------------------------------------------
  // VALIDATE DRIVE COMMAND
  // ต้องมีการสั่งเดินหน้าจริงจาก operator
  // --------------------------------------------------
  if (abs(targetL) < 200 && abs(targetR) < 200) {
    return;
  }

  // ต้องมีแรงขับจริง
  if (abs(curL) < 200 && abs(curR) < 200) {
    return;
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

// ============================================================================
// SIDE IMBALANCE DETECTION + STEERING CORRECTION
// ตรวจการกินกระแสไม่สมดุลของล้อซ้าย/ขวา แล้วช่วยปรับทิศ
// มี guard หลายชั้นเพื่อไม่ให้ correction สร้าง motion เอง
// ============================================================================
void detectSideImbalanceAndSteer() {

  // ==================================================
  // RESET CORRECTION EVERY LOOP (TRANSIENT LAYER)
  // ==================================================
  imbalanceCorrL = 0;
  imbalanceCorrR = 0;

  // ==================================================
  // STOP COMMAND GUARD
  // ถ้า operator ไม่สั่งขับ ห้าม correction สร้าง motion
  // ==================================================
  if (abs(targetL) < 50 && abs(targetR) < 50) {
    return;
  }

  // ==================================================
  // DO NOT APPLY WHEN VEHICLE ACTUALLY NOT MOVING
  // ใช้ curL/curR แทน target เพื่อสัมพันธ์แรงจริง
  // ==================================================
  if (abs(curL) < 200 && abs(curR) < 200) {
    return;
  }

  // ==================================================
  // SKIP WHEN INTENTIONALLY TURNING
  // ถ้าผู้ควบคุมกำลังเลี้ยว ไม่ต้อง correction
  // ==================================================
  if (abs(targetL - targetR) > 200) {
    return;
  }

  // ==================================================
  // READ CURRENT SENSOR
  // ==================================================
  float cL = curLeft();
  float cR = curRight();

  constexpr float CUR_IMBALANCE_A = 25.0f;
  constexpr int16_t STEER_COMP = 140;

  // ==================================================
  // SENSOR PLAUSIBILITY GUARD
  // ป้องกัน sensor error ทำให้ correction ผิด
  // ==================================================
  if (cL < 0 || cR < 0 || cL > 200 || cR > 200) {
    return;
  }

  // ==================================================
  // DEBOUNCE (ANTI GRASS SPIKE)
  // ต้อง imbalance ต่อเนื่อง 2 รอบขึ้นไป
  // ==================================================
  static uint8_t imbCnt = 0;

  if (abs(cL - cR) > CUR_IMBALANCE_A) {

    if (++imbCnt < 2)
      return;

  } else {

    imbCnt = 0;
    return;
  }

  // ==================================================
  // APPLY CORRECTION
  // ==================================================
  lastDriveEvent = DriveEvent::IMBALANCE;

  // ซ้ายกินกระแสมาก → เบี่ยงขวา
  if (cL > cR) {

    imbalanceCorrL = -STEER_COMP;
    imbalanceCorrR = +STEER_COMP;

  }

  // ขวากินกระแสมาก → เบี่ยงซ้าย
  else {

    imbalanceCorrL = +STEER_COMP;
    imbalanceCorrR = -STEER_COMP;
  }
}

// ============================================================================
// WHEEL STUCK DETECTION (FIELD STABLE VERSION)
// ตรวจจับล้อติด / ล้อจม โดยใช้ current imbalance + adaptive threshold
// เพิ่ม guards เพื่อลด false trigger ในสนามจริง
// ============================================================================

void detectWheelStuck(uint32_t now) {

  // --------------------------------------------------
  // GUARD : ห้ามตรวจ stuck ระหว่าง auto reverse
  // --------------------------------------------------
  if (autoReverseActive)
    return;

  // --------------------------------------------------
  // GLOBAL COOLDOWN กัน oscillation reverse
  // --------------------------------------------------
  static uint32_t lastStuckTrigger_ms = 0;

  constexpr uint32_t STUCK_COOLDOWN_MS = 1500;

  if (now - lastStuckTrigger_ms < STUCK_COOLDOWN_MS)
    return;

  // --------------------------------------------------
  // CONFIG
  // --------------------------------------------------
  constexpr int16_t MIN_PWM_FOR_STUCK = 300;
  constexpr int16_t TURNING_DIFF_MAX = 250;

  constexpr float CURRENT_HYST = 3.0f;  // เพิ่ม hysteresis กัน sensor jitter

  // --------------------------------------------------
  // ต้องมีแรงขับจริง
  // --------------------------------------------------
  int16_t pwmMag = max(abs(curL), abs(curR));

  if (pwmMag < MIN_PWM_FOR_STUCK)
    return;

  // --------------------------------------------------
  // GUARD : ต้องไม่อยู่ในช่วง turning
  // --------------------------------------------------
  if (abs(curL - curR) > TURNING_DIFF_MAX)
    return;

  // --------------------------------------------------
  // CURRENT READ
  // --------------------------------------------------
  float cL = curLeft();
  float cR = curRight();

  float maxCur = max(cL, cR);

  // --------------------------------------------------
  // CURRENT PLAUSIBILITY
  // --------------------------------------------------
  if (!isfinite(cL) || !isfinite(cR))
    return;

  if (maxCur <= 0.01f)
    return;

  // --------------------------------------------------
  // LOAD ต่ำไม่ถือว่า stuck
  // --------------------------------------------------
  if (maxCur < CUR_WARN_A)
    return;

  // --------------------------------------------------
  // CURRENT FLOOR GUARD (กัน sensor noise)
  // --------------------------------------------------
  constexpr float MIN_VALID_CURRENT = 6.0f;

  if (maxCur < MIN_VALID_CURRENT)
    return;

  // --------------------------------------------------
  // ADAPTIVE PERCENT THRESHOLD
  // --------------------------------------------------
  float percentThreshold;

  if (pwmMag < 400)
    percentThreshold = 0.40f;
  else if (pwmMag < 700)
    percentThreshold = 0.50f;
  else
    percentThreshold = 0.60f;

  // --------------------------------------------------
  // GRASS DENSITY ESTIMATOR
  // --------------------------------------------------
  static float prevCur = 0;
  static float rippleAvg = 0;

  float curAvg = (cL + cR) * 0.5f;
  float ripple = fabs(curAvg - prevCur);

  prevCur = curAvg;

  rippleAvg = rippleAvg * 0.85f + ripple * 0.15f;

  // หญ้าหนา → ผ่อน threshold
  if (rippleAvg > 7.0f)
    percentThreshold *= 1.25f;

  // --------------------------------------------------
  // DIFFERENCE
  // --------------------------------------------------
  float diff = fabs(cL - cR);
  float percentDiff = diff / maxCur;

  // --------------------------------------------------
  // DYNAMIC STUCK TIME
  // --------------------------------------------------
  uint32_t stuckTime;

  if (pwmMag < 400)
    stuckTime = 700;
  else if (pwmMag < 700)
    stuckTime = 520;
  else
    stuckTime = 350;

  static uint32_t leftStart_ms = 0;
  static uint32_t rightStart_ms = 0;

  // --------------------------------------------------
  // LEFT STUCK
  // --------------------------------------------------
  if (cL > cR + CURRENT_HYST && percentDiff > percentThreshold && cL > CUR_LIMP_A) {

    if (leftStart_ms == 0)
      leftStart_ms = now;

    if (now - leftStart_ms >= stuckTime) {

      leftStart_ms = 0;
      rightStart_ms = 0;

      lastDriveEvent = DriveEvent::STUCK_LEFT;

      lastStuckTrigger_ms = now;

      startAutoReverse(now);

      return;
    }

  } else {

    leftStart_ms = 0;
  }

  // --------------------------------------------------
  // RIGHT STUCK
  // --------------------------------------------------
  if (cR > cL + CURRENT_HYST && percentDiff > percentThreshold && cR > CUR_LIMP_A) {

    if (rightStart_ms == 0)
      rightStart_ms = now;

    if (now - rightStart_ms >= stuckTime) {

      leftStart_ms = 0;
      rightStart_ms = 0;

      lastDriveEvent = DriveEvent::STUCK_RIGHT;

      lastStuckTrigger_ms = now;

      startAutoReverse(now);

      return;
    }

  } else {

    rightStart_ms = 0;
  }
}

bool detectMotorStall() {

  static float prevCurL = 0;
  static float prevCurR = 0;

  static uint8_t stallCnt = 0;

  float cL = curLeft();
  float cR = curRight();

  if (abs(curL) < 200 && abs(curR) < 200)
    return false;

  if (cL < 5 && cR < 5)
    return false;

  float dCurL = cL - prevCurL;
  float dCurR = cR - prevCurR;

  constexpr float STALL_CURRENT_STEP = 18.0f;
  constexpr float STALL_CURRENT_MIN = 40.0f;
  constexpr uint8_t STALL_CONFIRM_CNT = 2;

  if (cL > STALL_CURRENT_MIN || cR > STALL_CURRENT_MIN) {

    bool stallDetected = false;

    if (cL > STALL_CURRENT_MIN && dCurL > STALL_CURRENT_STEP)
      stallDetected = true;

    if (cR > STALL_CURRENT_MIN && dCurR > STALL_CURRENT_STEP)
      stallDetected = true;

    if (stallDetected) {

      if (++stallCnt >= STALL_CONFIRM_CNT) {

        stallCnt = 0;

        prevCurL = cL;
        prevCurR = cR;

        return true;
      }

    } else {

      stallCnt = 0;
    }

  } else {

    stallCnt = 0;
  }

  prevCurL = cL;
  prevCurR = cR;

  return false;
}

void detectWheelLock() {

  static uint8_t lockCnt = 0;
  static uint32_t lockStart_ms = 0;

  // ==================================================
  // CONFIG
  // ==================================================
  constexpr int16_t MIN_PWM_FOR_LOCK = 350;
  constexpr float CURRENT_BALANCE_RATIO = 0.25f;
  constexpr uint8_t LOCK_CONFIRM_CNT = 4;

  // เพิ่ม time guard
  constexpr uint16_t LOCK_CONFIRM_MS = 400;

  uint32_t now = millis();

  // ==================================================
  // ต้องมีแรงขับจริง
  // ==================================================
  int16_t pwmMag = max(abs(curL), abs(curR));

  if (pwmMag < MIN_PWM_FOR_LOCK) {
    lockCnt = 0;
    lockStart_ms = 0;
    return;
  }

  // ==================================================
  // CURRENT
  // ==================================================
  float cL = curLeft();
  float cR = curRight();

  float maxCur = max(cL, cR);
  float diffCur = fabs(cL - cR);

  // ==================================================
  // CURRENT BALANCE
  // ถ้าต่างกันมาก = หญ้าหนา ไม่ใช่ lock
  // ==================================================
  if (maxCur <= 0.1f) {
    lockCnt = 0;
    lockStart_ms = 0;
    return;
  }

  float balanceRatio = diffCur / maxCur;

  if (balanceRatio > CURRENT_BALANCE_RATIO) {
    lockCnt = 0;
    lockStart_ms = 0;
    return;
  }

  // ==================================================
  // ADAPTIVE CURRENT THRESHOLD
  // ==================================================
  float lockCurrentThreshold;

  if (pwmMag < 500)
    lockCurrentThreshold = CUR_LIMP_A * 0.85f;
  else if (pwmMag < 800)
    lockCurrentThreshold = CUR_LIMP_A;
  else
    lockCurrentThreshold = CUR_LIMP_A * 1.15f;

  // ==================================================
  // BOTH SIDES HIGH CURRENT
  // ==================================================
  if (cL > lockCurrentThreshold && cR > lockCurrentThreshold) {

    // ==================================================
    // RAMP SATURATION CHECK
    // motor output ใกล้ target แล้ว
    // ==================================================
    int16_t errL = abs(targetL - curL);
    int16_t errR = abs(targetR - curR);

    if (errL < 50 && errR < 50) {

      if (lockCnt == 0) {
        lockStart_ms = now;
      }

      lockCnt++;

      // ต้องผ่านทั้ง counter และ time guard
      if (lockCnt >= LOCK_CONFIRM_CNT && (now - lockStart_ms) >= LOCK_CONFIRM_MS) {

        lastDriveEvent = DriveEvent::WHEEL_LOCK;
        latchFault(FaultCode::OVER_CURRENT);
      }

    } else {

      lockCnt = 0;
      lockStart_ms = 0;
    }

  } else {

    lockCnt = 0;
    lockStart_ms = 0;
  }
}

void telemetryCSV(uint32_t now, uint32_t loopStart_us) {
#if TELEMETRY_CSV

  static uint32_t lastTx = 0;
  if (now - lastTx < TELEMETRY_PERIOD_MS) return;
  lastTx = now;

  // buffer สำหรับสร้าง CSV
  char line[220];
  uint8_t chk = 0;

  // ================= LOOP TIME =================
  uint32_t loopTime_us = micros() - loopStart_us;
  uint32_t loopBudget_us = BUDGET_LOOP_MS * 1000UL;

  int cpuLoad_x10 = (loopTime_us * 1000UL) / loopBudget_us;
  if (cpuLoad_x10 > 9990) cpuLoad_x10 = 9990;

  int cpuMargin_x10 = 1000 - cpuLoad_x10;

  // ================= CURRENT =================
  float curMax = max(max(curA[0], curA[1]),
                     max(curA[2], curA[3]));

  int curMax_x10 = (int)(curMax * 10.0f);
  int volt_x10 = (int)(engineVolt * 10.0f);

  // ================= WATCHDOG =================
  char wdS = wdSensor.faulted ? 'X' : 'O';
  char wdC = wdComms.faulted ? 'X' : 'O';
  char wdD = wdDrive.faulted ? 'X' : 'O';
  char wdB = wdBlade.faulted ? 'X' : 'O';

  char adsCur = adsCurPresent ? '1' : '0';
  char adsVolt = adsVoltPresent ? '1' : '0';

  char gimbalOn =
    (systemState == SystemState::ACTIVE && !faultLatched && !requireIbusConfirm) ? '1' : '0';

  // ================= SAFETY =================
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

  SafetyState rawSafety =
    evaluateSafetyRaw(sin, sth);

  // ================= CSV BUILD =================
  int n = snprintf(
    line,
    sizeof(line),
    "%lu,%d,%d,%d.%d,%d.%d,%d,%d,%d.%d,%d.%d,%lu,%c,%c,%c,%c,%u,%d,%u,%u,%c,%c,%c",
    now,
    tempDriverL,
    tempDriverR,
    volt_x10 / 10, abs(volt_x10 % 10),
    curMax_x10 / 10, abs(curMax_x10 % 10),
    curL,
    curR,
    cpuLoad_x10 / 10, abs(cpuLoad_x10 % 10),
    cpuMargin_x10 / 10, abs(cpuMargin_x10 % 10),
    now - lastIbusByte_ms,
    wdS,
    wdC,
    wdD,
    wdB,
    (uint8_t)activeFault,
    freeRam(),
    autoReverseCount,
    (uint8_t)rawSafety,
    adsCur,
    adsVolt,
    gimbalOn
  );

  // ================= CHECKSUM =================
  for (int i = 0; i < n; i++) {
    chk ^= line[i];
  }

  // ================= OUTPUT =================
  Serial.print(line);
  Serial.print(',');
  Serial.println(chk);

#endif
}

// ============================================================================
// UPDATE DRIVE TARGET (STABLE RC VERSION)
// RC filter + spike guard + smooth mixing
// ============================================================================
void updateDriveTarget() {

  // ==================================================
  // SYSTEM SAFETY GUARD
  // ==================================================
  if (ibusCommLost || getDriveSafety() == SafetyState::EMERGENCY || systemState != SystemState::ACTIVE) {

    targetL = 0;
    targetR = 0;
    return;
  }

  // ==================================================
  // READ IBUS ONCE
  // ==================================================
  uint16_t rawThr = ibus.readChannel(CH_THROTTLE);
  uint16_t rawStr = ibus.readChannel(CH_STEER);

  // ==================================================
  // IBUS PLAUSIBILITY
  // ==================================================
  if (rawThr < 900 || rawThr > 2100 || rawStr < 900 || rawStr > 2100) {

    targetL = 0;
    targetR = 0;
    return;
  }

  // ==================================================
  // IBUS RECOVERY CONFIRM
  // ==================================================
  if (requireIbusConfirm) {

    bool thrNeutral = neutral(rawThr);
    bool strNeutral = neutral(rawStr);

    if (thrNeutral && strNeutral) {

      requireIbusConfirm = false;

    } else {

      targetL = 0;
      targetR = 0;
      return;
    }
  }

  // ==================================================
  // RC SPIKE + RATE FILTER
  // ==================================================
  static uint16_t lastThr = 1500;
  static uint16_t lastStr = 1500;

  constexpr int16_t RC_SPIKE_LIMIT = 300;
  constexpr int16_t RC_RATE_LIMIT = 80;

  // spike guard
  if (abs((int)rawThr - (int)lastThr) > RC_SPIKE_LIMIT)
    rawThr = lastThr;

  if (abs((int)rawStr - (int)lastStr) > RC_SPIKE_LIMIT)
    rawStr = lastStr;

  // rate limiter
  if (rawThr > lastThr + RC_RATE_LIMIT)
    rawThr = lastThr + RC_RATE_LIMIT;

  if (rawThr < lastThr - RC_RATE_LIMIT)
    rawThr = lastThr - RC_RATE_LIMIT;

  if (rawStr > lastStr + RC_RATE_LIMIT)
    rawStr = lastStr + RC_RATE_LIMIT;

  if (rawStr < lastStr - RC_RATE_LIMIT)
    rawStr = lastStr - RC_RATE_LIMIT;

  lastThr = rawThr;
  lastStr = rawStr;

  // ==================================================
  // AXIS MAPPING
  // ==================================================
  auto mapAxis = [](int16_t v) -> int16_t {
    constexpr int16_t IN_MIN = 1000;
    constexpr int16_t IN_MAX = 2000;

    constexpr int16_t DB_MIN = 1450;
    constexpr int16_t DB_MAX = 1550;

    constexpr int16_t OUT_MAX = PWM_TOP;

    if (v >= DB_MIN && v <= DB_MAX)
      return 0;

    if (v < DB_MIN) {

      long m = map(v, IN_MIN, DB_MIN, -OUT_MAX, 0);
      return constrain((int16_t)m, -OUT_MAX, 0);
    }

    long m = map(v, DB_MAX, IN_MAX, 0, OUT_MAX);
    return constrain((int16_t)m, 0, OUT_MAX);
  };

  int16_t thr = mapAxis(rawThr);
  int16_t str = mapAxis(rawStr);

  // ==================================================
  // ZERO THROTTLE SAFETY
  // ==================================================
  constexpr int16_t THR_STOP_BAND = 50;

  if (abs(thr) < THR_STOP_BAND) {

    targetL = 0;
    targetR = 0;
    return;
  }

  // ==================================================
  // LOW SPEED STEER DAMP
  // ==================================================
  int16_t absThr = abs(thr);

  if (absThr < 200)
    str = str * 0.6;

  // ==================================================
  // FIXED POINT MIXING
  // ==================================================
  int16_t blend_x1000;

  if (absThr <= 120)
    blend_x1000 = 150;

  else if (absThr >= 380)
    blend_x1000 = 1000;

  else
    blend_x1000 = ((absThr - 120) * 1000) / 260;

  int16_t arcL = constrain(thr + str, -PWM_TOP, PWM_TOP);
  int16_t arcR = constrain(thr - str, -PWM_TOP, PWM_TOP);

  int16_t k_x1000 = (abs(str) * 1000) / PWM_TOP;

  if (str < 0)
    k_x1000 = -k_x1000;

  int32_t diffL = (int32_t)thr * (1000 + k_x1000);
  int32_t diffR = (int32_t)thr * (1000 - k_x1000);

  diffL /= 1000;
  diffR /= 1000;

  int32_t outL =
    ((int32_t)arcL * (1000 - blend_x1000) + diffL * blend_x1000) / 1000;

  int32_t outR =
    ((int32_t)arcR * (1000 - blend_x1000) + diffR * blend_x1000) / 1000;

  // ==================================================
  // NORMALIZE
  // ==================================================
  int32_t maxMag = max(abs(outL), abs(outR));

  if (maxMag > PWM_TOP) {

    outL = (outL * PWM_TOP) / maxMag;
    outR = (outR * PWM_TOP) / maxMag;
  }

  targetL = constrain(outL, -PWM_TOP, PWM_TOP);
  targetR = constrain(outR, -PWM_TOP, PWM_TOP);
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
// PWM SETUP (SAFE AFTER MCU RESET)
// ป้องกัน PWM spike หลัง watchdog / brownout reset
// ============================================================================
void setupPWM15K() {

  // --------------------------------------------------
  // FORCE TIMER RESET (CRITICAL)
  // reset register ก่อน config ใหม่
  // --------------------------------------------------
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR4A = 0;
  TCCR4B = 0;

  // ปิด PWM output ก่อน
  OCR3A = 0;
  OCR4A = 0;

  // --------------------------------------------------
  // TIMER3 → LEFT MOTOR PWM
  // Fast PWM mode 14 (ICR3 = TOP)
  // --------------------------------------------------
  TCCR3A = _BV(COM3A1) | _BV(WGM31);
  TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS30);

  ICR3 = 1067;  // ≈15kHz
  OCR3A = 0;    // duty = 0 (safe)

  // --------------------------------------------------
  // TIMER4 → RIGHT MOTOR PWM
  // Fast PWM mode 14 (ICR4 = TOP)
  // --------------------------------------------------
  TCCR4A = _BV(COM4A1) | _BV(WGM41);
  TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS40);

  ICR4 = 1067;  // ≈15kHz
  OCR4A = 0;    // duty = 0 (safe)
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

// ============================================================================
// DRIVE SAFE
// ตัดกำลังมอเตอร์ทันที และ reset state ของระบบขับเคลื่อน
// ============================================================================
void driveSafe() {

  // --------------------------------------------------
  // 1) Disable motor driver ก่อน
  // ป้องกัน PWM ยิงเข้า H-bridge ระหว่าง reset state
  // --------------------------------------------------
  digitalWrite(PIN_DRV_ENABLE, LOW);

  // --------------------------------------------------
  // 2) ตัด PWM output
  // --------------------------------------------------
  setPWM_L(0);
  setPWM_R(0);

  // --------------------------------------------------
  // 3) reset ramp state (สำคัญมาก)
  // ป้องกัน enable spike ตอน driver เปิดใหม่
  // --------------------------------------------------
  curL = 0;
  curR = 0;

  targetL = 0;
  targetR = 0;

  // --------------------------------------------------
  // 4) หน่วงสั้นมาก กัน shoot-through
  // --------------------------------------------------
  delayMicroseconds(5);

  // --------------------------------------------------
  // 5) ปลดทิศทางทั้งหมด
  // --------------------------------------------------
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

  if (faultToStore == FaultCode::NONE) {
    faultWritePending = false;
    return;
  }

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
  // IBUS LOST COUNTER (DECLARE FIRST)
  // ==================================================
  static uint8_t ibusLostCnt = 0;

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

      // สัญญาณกลับมา แต่ยังไม่อนุญาตใช้ RC
      requireIbusConfirm = true;

      // เริ่มจับเวลา recovery
      ibusRecoverStart_ms = now;
    }

    // reset lost counter
    ibusLostCnt = 0;

    ibusCommLost = false;

    // refresh watchdog
    wdComms.lastUpdate_ms = now;
  }

  // ==================================================
  // SOFT TIMEOUT (COMM LOST)
  // ==================================================
  if (now - lastIbusByte_ms > 100) {

    if (++ibusLostCnt >= 3) {

      if (!ibusCommLost) {

#if DEBUG_SERIAL
        Serial.println(F("[IBUS] SOFT LOST"));
#endif

        ibusCommLost = true;
        requireIbusConfirm = true;
      }
    }

  } else {

    ibusLostCnt = 0;
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

  // --------------------------------------------------
  // SENSOR PRESENCE GUARD (BENCH TEST ALLOW)
  // --------------------------------------------------
  if (!adsCurPresent) {
#if DEBUG_SERIAL
    Serial.println(F("[ACS] CAL SKIPPED - ADS NOT PRESENT"));
#endif
    currentOffsetCalibrated = true;  // ถือว่าคาลิเบรตแล้ว
    return true;                     // ให้ระบบไป ACTIVE ได้
  }

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

// ============================================================================
// IDLE CURRENT OFFSET AUTO RECALIBRATION (NON-BLOCKING VERSION)
// ปรับ offset ของ ACS sensor ช้า ๆ เมื่อรถหยุดนิ่ง
// ใช้ state machine อ่าน ADS1115 ทีละ channel
// ป้องกัน for-loop return ที่ทำให้ channel อื่นไม่ได้อ่าน
// ============================================================================
void idleCurrentAutoRezero(uint32_t now) {

  // --------------------------------------------------
  // RUN ทุก 3 วินาที
  // --------------------------------------------------
  static uint32_t lastRun_ms = 0;

  if (now - lastRun_ms < 3000)
    return;

  // --------------------------------------------------
  // เงื่อนไขต้อง "หยุดนิ่งจริง"
  // --------------------------------------------------
  if (targetL != 0 || targetR != 0)
    return;

  if (curL != 0 || curR != 0)
    return;

  if (digitalRead(PIN_DRV_ENABLE))
    return;

  // --------------------------------------------------
  // SENSOR ต้องมี
  // --------------------------------------------------
  if (!adsCurPresent)
    return;

  // --------------------------------------------------
  // STATE MACHINE
  // --------------------------------------------------
  enum class RezeroState : uint8_t {
    IDLE,
    WAIT_CONV
  };

  static RezeroState state = RezeroState::IDLE;

  static uint8_t ch = 0;
  static uint32_t convStart_ms = 0;

  constexpr uint16_t CONV_TIMEOUT_MS = 15;

  switch (state) {

    // --------------------------------------------------
    // START NEW RECALIBRATION CYCLE
    // --------------------------------------------------
    case RezeroState::IDLE:
      {

        lastRun_ms = now;

        ch = 0;

        uint16_t mux =
          ADS1X15_REG_CONFIG_MUX_SINGLE_0 + (ADS_CUR_CH_MAP[ch] << 12);

        adsCur.startADCReading(mux, false);

        convStart_ms = now;

        state = RezeroState::WAIT_CONV;

        return;
      }

    // --------------------------------------------------
    // WAIT FOR ADC CONVERSION
    // --------------------------------------------------
    case RezeroState::WAIT_CONV:
      {

        if (!adsCur.conversionComplete()) {

          if (now - convStart_ms > CONV_TIMEOUT_MS) {

            // timeout → abort cycle
            state = RezeroState::IDLE;
          }

          return;
        }

        int16_t raw = adsCur.getLastConversionResults();

        float v = raw * ADS1115_LSB_V;

        // --------------------------------------------------
        // Offset adjustment (slow LPF)
        // --------------------------------------------------
        constexpr float OFFSET_ALPHA = 0.02f;

        g_acsOffsetV[ch] =
          g_acsOffsetV[ch] + OFFSET_ALPHA * (v - g_acsOffsetV[ch]);

        // --------------------------------------------------
        // NEXT CHANNEL
        // --------------------------------------------------
        ch++;

        if (ch >= 4) {

#if DEBUG_SERIAL
          Serial.println(F("[ACS] IDLE REZERO"));
#endif

          state = RezeroState::IDLE;
          return;
        }

        uint16_t mux =
          ADS1X15_REG_CONFIG_MUX_SINGLE_0 + (ADS_CUR_CH_MAP[ch] << 12);

        adsCur.startADCReading(mux, false);

        convStart_ms = now;

        return;
      }
  }
}

void i2cBusClear() {

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);

  delay(5);

  if (digitalRead(SDA) == LOW) {

    pinMode(SCL, OUTPUT);

    for (int i = 0; i < 16; i++) {

      digitalWrite(SCL, HIGH);
      delayMicroseconds(5);

      digitalWrite(SCL, LOW);
      delayMicroseconds(5);
    }

    pinMode(SCL, INPUT_PULLUP);
  }
}




bool updateSensors() {

  uint32_t now = millis();

  static bool sensorCycleComplete = false;
  sensorCycleComplete = false;

  // ==================================================
  // SENSOR PRESENCE GUARD
  // ==================================================
  if (!adsCurPresent && !adsVoltPresent) {
    return false;
  }

  bool sensorHealthyThisCycle = false;

#if TEST_MODE

  curA[0] = curA[1] = curA[2] = curA[3] = 5.0f;

  tempDriverL = 45;
  tempDriverR = 47;

  engineVolt = 26.0f;

  sensorCycleComplete = true;
  return sensorCycleComplete;

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

  static uint32_t lastRecover_ms = 0;

  if (Wire.getWireTimeoutFlag() && i2cState == I2CRecoverState::IDLE && (now - lastRecover_ms > 2000)) {

    Wire.clearWireTimeoutFlag();
    i2cState = I2CRecoverState::END_BUS;
    i2cStateStart_ms = now;
  }

  switch (i2cState) {

    case I2CRecoverState::END_BUS:

      Wire.end();
      i2cState = I2CRecoverState::BEGIN_BUS;
      i2cStateStart_ms = now;
      return false;

    case I2CRecoverState::BEGIN_BUS:

      if (now - i2cStateStart_ms < 1)
        return false;

      i2cBusClear();
      Wire.begin();
      Wire.setClock(100000);
      Wire.setWireTimeout(6000, true);

      i2cState = I2CRecoverState::REINIT_ADS;
      return false;

    case I2CRecoverState::REINIT_ADS:

      adsCurPresent = adsCur.begin(0x48);
      if (adsCurPresent) {
        adsCur.setGain(GAIN_ONE);
        adsCur.setDataRate(RATE_ADS1115_250SPS);
      }

      adsVoltPresent = adsVolt.begin(0x49);
      if (adsVoltPresent) {
        adsVolt.setGain(GAIN_ONE);
        adsVolt.setDataRate(RATE_ADS1115_250SPS);
      }

      i2cResetCount++;
      i2cState = I2CRecoverState::DONE;
      return false;

    case I2CRecoverState::DONE:

      if (i2cResetCount >= 3) {
        latchFault(FaultCode::VOLT_SENSOR_FAULT);
      }

      // update cooldown timer
      lastRecover_ms = now;

      i2cState = I2CRecoverState::IDLE;
      return false;

    default:
      break;
  }

  // ==================================================
  // HARDWARE OVERCURRENT
  // ==================================================

  if (digitalRead(PIN_CUR_TRIP) == LOW) {
    latchFault(FaultCode::OVER_CURRENT);
    return false;
  }

  // ==================================================
  // ----------- ASYNC CURRENT (ADS1115) ---------------
  // ROBUST VERSION (ANTI CHANNEL STALL)
  // ==================================================

  static uint8_t curIdx = 0;
  static bool curConvRunning = false;
  static uint32_t curConvStart_ms = 0;
  static uint32_t curNextReady_ms = 0;

  constexpr uint16_t CUR_CONV_TIMEOUT_MS = 25;
  static uint8_t adsTimeoutCnt = 0;
  constexpr uint8_t ADS_TIMEOUT_LIMIT = 6;

  if (adsCurPresent) {

    // --------------------------------------------------
    // START NEW CONVERSION
    // --------------------------------------------------
    if (!curConvRunning) {

      uint16_t mux =
        ADS1X15_REG_CONFIG_MUX_SINGLE_0 + (ADS_CUR_CH_MAP[curIdx] << 12);

      adsCur.startADCReading(mux, false);

      curConvRunning = true;
      curConvStart_ms = now;

      // ADS1115 @250SPS ≈ 4ms conversion
      curNextReady_ms = now + 4;
    }

    // --------------------------------------------------
    // WAIT CONVERSION
    // --------------------------------------------------
    else {

      // ---------- CONVERSION COMPLETE ----------
      if (now >= curNextReady_ms && adsCur.conversionComplete()) {

        int16_t raw = adsCur.getLastConversionResults();
        adsTimeoutCnt = 0;

        float v = raw * ADS1115_LSB_V;
        float a = (v - g_acsOffsetV[curIdx]) / ACS_SENS_V_PER_A;

        // ---------- PLAUSIBILITY GUARD ----------
        if (a < CUR_MIN_PLAUSIBLE || a > CUR_MAX_PLAUSIBLE) {
          latchFault(FaultCode::CUR_SENSOR_FAULT);
          return false;
        }

        // ---------- LPF FILTER ----------
        curA[curIdx] += CUR_LPF_ALPHA * (a - curA[curIdx]);

        sensorHealthyThisCycle = true;

        // ---------- CURRENT SPIKE ----------
        static uint8_t spikeCnt = 0;

        if (a > CUR_SPIKE_A) {

          if (++spikeCnt >= 2) {

            forceDriveSoftStop(now);
            bladeState = BladeState::SOFT_STOP;
          }

        } else {

          spikeCnt = 0;
        }

        // ---------- HARD CURRENT LIMIT ----------
        if (a > CUR_TRIP_A_CH[curIdx]) {

          if (++overCurCnt[curIdx] >= 2) {
            latchFault(FaultCode::OVER_CURRENT);
            return false;
          }

        } else {

          overCurCnt[curIdx] = 0;
        }

        // ---------- NEXT CHANNEL ----------
        curIdx++;
        if (curIdx >= 4)
          curIdx = 0;

        curConvRunning = false;
      }

      // ---------- CONVERSION TIMEOUT ----------
      else if (now - curConvStart_ms > CUR_CONV_TIMEOUT_MS) {

#if DEBUG_SERIAL
        Serial.print(F("[ADS CUR] TIMEOUT CH="));
        Serial.println(curIdx);
#endif

        adsTimeoutCnt++;

        // ถ้า timeout หลายครั้งติด
        if (adsTimeoutCnt >= ADS_TIMEOUT_LIMIT) {

#if DEBUG_SERIAL
          Serial.println(F("[ADS CUR] RESET DEVICE"));
#endif

          adsCurPresent = adsCur.begin(0x48);

          if (adsCurPresent) {
            adsCur.setGain(GAIN_ONE);
            adsCur.setDataRate(RATE_ADS1115_250SPS);
          }

          adsTimeoutCnt = 0;
        }

        // ข้าม channel
        curIdx++;
        if (curIdx >= 4)
          curIdx = 0;

        curConvRunning = false;
      }
    }
  }

  // ==================================================
  // ----------- ASYNC VOLTAGE -------------------------
  // ==================================================

  static bool voltConvRunning = false;
  static uint32_t voltConvStart_ms = 0;
  static uint32_t voltNextReady_ms = 0;
  static uint32_t lastVoltOk_ms = 0;
  static uint8_t voltFailCnt = 0;

  constexpr uint16_t VOLT_CONV_TIMEOUT_MS = 25;

  if (adsVoltPresent) {

    if (!voltConvRunning) {

      adsVolt.startADCReading(
        ADS1X15_REG_CONFIG_MUX_SINGLE_0, false);

      voltConvRunning = true;
      voltConvStart_ms = now;
      voltNextReady_ms = now + 4;

    } else {

      if (now >= voltNextReady_ms && adsVolt.conversionComplete()) {

        int16_t raw =
          adsVolt.getLastConversionResults();

        float v_adc = raw * ADS1115_LSB_V;

        float vRaw =
          v_adc * ((150.0f + 33.0f) / 33.0f);

        // --------------------------------------------------
        // PLAUSIBILITY CHECK
        // --------------------------------------------------
        if (vRaw > 0.0f && vRaw < 40.0f) {

          float dv = vRaw - engineVolt;

          // limit step change (anti spike)
          if (dv > 8.0f) dv = 8.0f;
          if (dv < -8.0f) dv = -8.0f;

          // single LPF
          engineVolt += 0.15f * dv;

          lastVoltOk_ms = now;
          sensorHealthyThisCycle = true;
        }

        voltConvRunning = false;

      } else if (now - voltConvStart_ms > VOLT_CONV_TIMEOUT_MS) {

        voltConvRunning = false;
      }
    }
  }

  if (now - lastVoltOk_ms > VOLT_SENSOR_TIMEOUT_MS) {

    if (++voltFailCnt >= VOLT_SENSOR_FAIL_COUNT) {
      latchFault(FaultCode::VOLT_SENSOR_FAULT);
      return false;
    }

  } else {
    voltFailCnt = 0;
  }

#if !TEST_MODE

  static uint32_t lastTemp_ms = 0;
  static uint32_t lastTempOk_ms = 0;

  if (now - lastTemp_ms >= 100) {

    lastTemp_ms = now;

    int16_t tL, tR;

    if (!readDriverTempsPT100(tL, tR)) {
      latchFault(FaultCode::TEMP_SENSOR_FAULT);
      return false;
    }

    tempDriverL = tL;
    tempDriverR = tR;

    lastTempOk_ms = now;
    sensorHealthyThisCycle = true;

    if (tL > TEMP_TRIP_C || tR > TEMP_TRIP_C) {

      latchFault(FaultCode::OVER_TEMP);
      return false;
    }
  }

  if (now - lastTempOk_ms > TEMP_SENSOR_TIMEOUT_MS) {

    latchFault(FaultCode::TEMP_SENSOR_FAULT);
    return false;
  }

#endif

  if (i2cState == I2CRecoverState::IDLE) {

    if (adsCurPresent || adsVoltPresent) {

      if (curConvRunning || voltConvRunning)
        return true;

      if (sensorHealthyThisCycle)
        return true;

      return false;
    }
  }

  return false;
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
}

constexpr uint16_t WD_GRACE_MS = 40;

void monitorSubsystemWatchdogs(uint32_t now) {

  if (now - wdSensor.lastUpdate_ms > wdSensor.timeout_ms + WD_GRACE_MS) {
    latchFault(FaultCode::LOGIC_WATCHDOG);
    wdSensor.faulted = true;
  }

  if (now - wdComms.lastUpdate_ms > wdComms.timeout_ms + WD_GRACE_MS) {
    latchFault(FaultCode::IBUS_LOST);
    wdComms.faulted = true;
  }

  if (now - wdDrive.lastUpdate_ms > wdDrive.timeout_ms + WD_GRACE_MS) {
    latchFault(FaultCode::DRIVE_TIMEOUT);
    wdDrive.faulted = true;
  }

  if (now - wdBlade.lastUpdate_ms > wdBlade.timeout_ms + WD_GRACE_MS) {
    latchFault(FaultCode::BLADE_TIMEOUT);
    wdBlade.faulted = true;
  }
}

void applyDrive() {

  uint32_t now = millis();

  // ==================================================
  // HARD STOP
  // ==================================================
  if (systemState == SystemState::FAULT || driveState == DriveState::LOCKED) {

    driveSafe();

    curL = 0;
    curR = 0;
    targetL = 0;
    targetR = 0;

    return;
  }

  // ==================================================
  // DRIVER REARM
  // ==================================================

  bool thrNeutral = neutral(ibus.readChannel(CH_THROTTLE));
  bool strNeutral = neutral(ibus.readChannel(CH_STEER));

  if (driverRearmRequired) {

    curL = 0;
    curR = 0;

    if (thrNeutral && strNeutral)
      driverRearmRequired = false;

    return;
  }

  // ==================================================
  // READ SENSOR VALUES
  // ==================================================
  float curA_L = curLeft();
  float curA_R = curRight();

  float tempMax = max(tempDriverL, tempDriverR);

  // ==================================================
  // COPY TARGET
  // ==================================================
  float finalTargetL = targetL;
  float finalTargetR = targetR;

  // ==================================================
  // AUTO REVERSE
  // ==================================================
  if (autoReverseActive && !ibusCommLost && !requireIbusConfirm && systemState == SystemState::ACTIVE && (abs(targetL) > 100 || abs(targetR) > 100)) {

    if (now - autoReverseStart_ms < AUTO_REV_MS) {

      int16_t refL = (curL != 0) ? curL : targetL;
      int16_t refR = (curR != 0) ? curR : targetR;

      int8_t dirL = (refL > 0) ? -1 : (refL < 0 ? 1 : -1);
      int8_t dirR = (refR > 0) ? -1 : (refR < 0 ? 1 : -1);

      finalTargetL = dirL * AUTO_REV_PWM;
      finalTargetR = dirR * AUTO_REV_PWM;

    } else {
      autoReverseActive = false;

      reverseRecoveryActive = true;
      reverseRecoveryStart_ms = now;
    }
  }

  // ==================================================
  // SAFETY EMERGENCY
  // ==================================================
  if (getDriveSafety() == SafetyState::EMERGENCY) {

    finalTargetL = 0;
    finalTargetR = 0;
  }

  // ==================================================
  // TRACTION CONTROL
  // ==================================================
  float diff = fabs(curA_L - curA_R);

  constexpr float TRACTION_DIFF = 30.0f;

  if (diff > TRACTION_DIFF) {

    constexpr float TRACTION_REDUCE = 0.75f;
    constexpr float TRACTION_BOOST = 1.10f;

    if (curA_L > curA_R) {

      finalTargetL *= TRACTION_REDUCE;
      finalTargetR *= TRACTION_BOOST;

    } else {

      finalTargetR *= TRACTION_REDUCE;
      finalTargetL *= TRACTION_BOOST;
    }
  }

  finalTargetL = constrain(finalTargetL, -PWM_TOP, PWM_TOP);
  finalTargetR = constrain(finalTargetR, -PWM_TOP, PWM_TOP);

  // ==================================================
  // TORQUE LIMIT
  // ==================================================
  constexpr float TORQUE_SOFT = 55.0f;
  constexpr float TORQUE_HARD = 75.0f;

  if (curA_L > TORQUE_SOFT) {

    float span = TORQUE_HARD - TORQUE_SOFT;
    float excess = curA_L - TORQUE_SOFT;

    float scale = 1.0f - (excess / span);

    if (scale < 0.35f) scale = 0.35f;

    finalTargetL *= scale;
  }

  if (curA_R > TORQUE_SOFT) {

    float span = TORQUE_HARD - TORQUE_SOFT;
    float excess = curA_R - TORQUE_SOFT;

    float scale = 1.0f - (excess / span);

    if (scale < 0.35f) scale = 0.35f;

    finalTargetR *= scale;
  }

  // ==================================================
  // THERMAL DERATING
  // ==================================================
  if (tempMax > TEMP_WARN_C) {

    float span = TEMP_TRIP_C - TEMP_WARN_C;

    float thermalScale =
      1.0f - ((tempMax - TEMP_WARN_C) / span) * 0.6f;

    if (thermalScale < 0.4f)
      thermalScale = 0.4f;

    finalTargetL *= thermalScale;
    finalTargetR *= thermalScale;
  }

  // ==================================================
  // CURRENT LIMIT
  // ==================================================
  float curMax = max(curA_L, curA_R);

  if (curMax > CUR_LIMP_A) {

    float scale = CUR_LIMP_A / curMax;

    finalTargetL *= scale;
    finalTargetR *= scale;
  }

  // ==================================================
  // MOTOR STALL ENERGY PROTECTION
  // ==================================================

  float stallScale =
    computeStallScale(now, curA_L, curA_R);

  finalTargetL *= stallScale;
  finalTargetR *= stallScale;

  // ==================================================
  // TRACTION CONTROL (ANTI WHEEL SPIN)
  // ==================================================
  constexpr float TRACTION_SLIP_DIFF = 18.0f;

  float curDiff = fabs(curA_L - curA_R);

  if (curDiff > TRACTION_SLIP_DIFF) {

    constexpr float SLIP_REDUCE = 0.75f;
    constexpr float GRIP_BOOST = 1.05f;

    if (curA_L > curA_R) {

      // ล้อซ้ายติด ล้อขวาฟรี
      finalTargetR *= SLIP_REDUCE;
      finalTargetL *= GRIP_BOOST;

    } else {

      // ล้อขวาติด ล้อซ้ายฟรี
      finalTargetL *= SLIP_REDUCE;
      finalTargetR *= GRIP_BOOST;
    }
  }

  // ==================================================
  // IMBALANCE CORRECTION
  // ==================================================
  if (abs(targetL) > 80 || abs(targetR) > 80) {

    finalTargetL =
      constrain(finalTargetL + imbalanceCorrL,
                -PWM_TOP,
                PWM_TOP);

    finalTargetR =
      constrain(finalTargetR + imbalanceCorrR,
                -PWM_TOP,
                PWM_TOP);
  }

  // ==================================================
  // REVERSE SHORT BRAKE (REAL DIRECTION CHECK)
  // ==================================================
  static bool reverseBrakeActive = false;
  static uint32_t reverseBrakeStart_ms = 0;

  constexpr uint16_t REVERSE_BRAKE_MS = 40;

  // --------------------------------------------------
  // ทิศจริงของมอเตอร์
  // --------------------------------------------------
  int8_t curSignL = (curL > 0) ? 1 : (curL < 0 ? -1 : 0);
  int8_t curSignR = (curR > 0) ? 1 : (curR < 0 ? -1 : 0);

  // --------------------------------------------------
  // ทิศที่ต้องการ
  // --------------------------------------------------
  int8_t tgtSignL = (finalTargetL > 0) ? 1 : (finalTargetL < 0 ? -1 : 0);
  int8_t tgtSignR = (finalTargetR > 0) ? 1 : (finalTargetR < 0 ? -1 : 0);

  // --------------------------------------------------
  // ตรวจ reverse จริง
  // --------------------------------------------------
  bool reverseRequest =
    ((curSignL != 0 && tgtSignL != 0 && curSignL != tgtSignL) || (curSignR != 0 && tgtSignR != 0 && curSignR != tgtSignR)) && (abs(curL) > 150 || abs(curR) > 150);

  // --------------------------------------------------
  // เริ่ม short brake
  // --------------------------------------------------
  if (reverseRequest && !reverseBrakeActive) {

    reverseBrakeActive = true;
    reverseBrakeStart_ms = now;

#if DEBUG_SERIAL
    Serial.println(F("[DRIVE] SHORT BRAKE"));
#endif
  }

  // --------------------------------------------------
  // ระหว่าง short brake
  // --------------------------------------------------
  if (reverseBrakeActive) {

    if (now - reverseBrakeStart_ms < REVERSE_BRAKE_MS) {

      setPWM_L(0);
      setPWM_R(0);

      motorShortBrake();

      return;
    } else {

      reverseBrakeActive = false;

      digitalWrite(DIR_L1, LOW);
      digitalWrite(DIR_L2, LOW);
      digitalWrite(DIR_R1, LOW);
      digitalWrite(DIR_R2, LOW);
    }
  }

  // ==================================================
  // RAMP CONTROL (REVERSE SAFE)
  // ==================================================
  int16_t step;

  int16_t errMax = max(abs(finalTargetL - curL),
                       abs(finalTargetR - curR));

  // ตรวจว่ากำลัง reverse
  bool reversing =
    ((curL > 0 && finalTargetL < 0) || (curL < 0 && finalTargetL > 0) || (curR > 0 && finalTargetR < 0) || (curR < 0 && finalTargetR > 0));

  if (driveState == DriveState::LIMP)
    step = 2;

  else if (reversing)
    step = 2;  // ramp ช้าลงตอน reverse

  else if (errMax > 600)
    step = 3;

  else if (errMax > 300)
    step = 4;

  else
    step = 6;

  curL = ramp(curL, finalTargetL, step);
  curR = ramp(curR, finalTargetR, step);

  // ==================================================
  // SAFE DIR SWITCH (H-BRIDGE SHOOT-THROUGH PROTECTION)
  // ==================================================

  static int8_t lastDirL = 0;
  static int8_t lastDirR = 0;

  static bool dirSwitchPendingL = false;
  static bool dirSwitchPendingR = false;

  static uint32_t dirSwitchStartL_us = 0;
  static uint32_t dirSwitchStartR_us = 0;

  static uint32_t pwmResumeL_us = 0;
  static uint32_t pwmResumeR_us = 0;

  constexpr uint16_t DIR_DEADTIME_US = 1000;
  constexpr uint16_t PWM_RESUME_DELAY_US = 600;

  int8_t dirL = (curL > 0) ? 1 : (curL < 0 ? -1 : 0);
  int8_t dirR = (curR > 0) ? 1 : (curR < 0 ? -1 : 0);

  uint32_t now_us = micros();

  // --------------------------------------------------
  // LEFT MOTOR DIR CHANGE
  // --------------------------------------------------

  if (dirL != lastDirL && !dirSwitchPendingL) {

    setPWM_L(0);

    digitalWrite(DIR_L1, LOW);
    digitalWrite(DIR_L2, LOW);

    dirSwitchPendingL = true;
    dirSwitchStartL_us = now_us;
  }

  if (dirSwitchPendingL && now_us - dirSwitchStartL_us >= DIR_DEADTIME_US) {

    digitalWrite(DIR_L1, dirL > 0);
    digitalWrite(DIR_L2, dirL < 0);

    pwmResumeL_us = now_us;

    dirSwitchPendingL = false;
    lastDirL = dirL;
  }

  // --------------------------------------------------
  // RIGHT MOTOR DIR CHANGE
  // --------------------------------------------------

  if (dirR != lastDirR && !dirSwitchPendingR) {

    setPWM_R(0);

    digitalWrite(DIR_R1, LOW);
    digitalWrite(DIR_R2, LOW);

    dirSwitchPendingR = true;
    dirSwitchStartR_us = now_us;
  }

  if (dirSwitchPendingR && now_us - dirSwitchStartR_us >= DIR_DEADTIME_US) {

    digitalWrite(DIR_R1, dirR > 0);
    digitalWrite(DIR_R2, dirR < 0);

    pwmResumeR_us = now_us;

    dirSwitchPendingR = false;
    lastDirR = dirR;
  }

  // --------------------------------------------------
  // APPLY PWM WITH RESUME DELAY
  // --------------------------------------------------

  uint16_t pwmL = abs(curL);
  uint16_t pwmR = abs(curR);

  if (micros() - pwmResumeL_us < PWM_RESUME_DELAY_US)
    pwmL = 0;

  setPWM_L(pwmL);
  setPWM_R(pwmR);
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

  // --------------------------------------------------
  // AUTO REVERSE COUNTER RESET ON FAULT / DISARM
  // --------------------------------------------------

  if (systemState != SystemState::ACTIVE) {

    autoReverseCount = 0;
    autoReverseActive = false;
  }

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
        // --- RESET DRIVER REARM ---
        extern bool driverRearmRequired;
        driverRearmRequired = true;

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

        bool thrNeutral = neutral(ibus.readChannel(CH_THROTTLE));
        bool strNeutral = neutral(ibus.readChannel(CH_STEER));

        // ต้องผ่าน neutral ก่อน
        if (!rcNeutralConfirmed) {

          if (thrNeutral && strNeutral) {
            rcNeutralConfirmed = true;
          }

          return;
        }

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
#if DEBUG_SERIAL || TELEMETRY_CSV
  Serial.begin(115200);
#endif
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
  i2cBusClear();
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(6000, true);

  // --------------------------------------------------
  // ADS1115 DETECT (ROBUST INIT)
  // --------------------------------------------------

  adsCurPresent = adsCur.begin(0x48);
  if (adsCurPresent) {
    adsCur.setGain(GAIN_ONE);
    adsCur.setDataRate(RATE_ADS1115_250SPS);
#if DEBUG_SERIAL
    Serial.println(F("[BOOT] ADS CUR OK (0x48)"));
#endif
  } else {
#if DEBUG_SERIAL
    Serial.println(F("[BOOT] ADS CUR NOT FOUND (0x48)"));
#endif
  }

  adsVoltPresent = adsVolt.begin(0x49);
  if (adsVoltPresent) {
    adsVolt.setGain(GAIN_ONE);
    adsVolt.setDataRate(RATE_ADS1115_250SPS);
#if DEBUG_SERIAL
    Serial.println(F("[BOOT] ADS VOLT OK (0x49)"));
#endif
  } else {
#if DEBUG_SERIAL
    Serial.println(F("[BOOT] ADS VOLT NOT FOUND (0x49)"));
#endif
  }

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
  // SPI / TEMPERATURE
#if !TEST_MODE
  SPI.begin();
  maxL.begin(MAX31865_3WIRE);
  maxR.begin(MAX31865_3WIRE);
#else
#if DEBUG_SERIAL
  Serial.println(F("[BOOT] TEST MODE - PT100 DISABLED"));
#endif
#endif

  // --------------------------------------------------
  // PWM / SERVO
  // --------------------------------------------------
  driveSafe();
  setPWM_L(0);
  setPWM_R(0);
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

  uint32_t now = millis();
  uint32_t loopStart_us = micros();

#if DEBUG_SERIAL && !TELEMETRY_CSV
  debugTestMode(now);
  debugTelemetry(now);
  debugIBus(now);
#endif

  // ==================================================
  // PHASE 1 : COMMS
  // ==================================================
  uint32_t tComms_us = micros();

  updateComms(now);
  updateIgnition();
  updateSystemState(now);

  uint32_t dtComms = micros() - tComms_us;

  if (dtComms > BUDGET_COMMS_MS * 1000UL) {

    if (++commsBudgetCnt >= PHASE_BUDGET_CONFIRM)
      latchFault(FaultCode::COMMS_TIMEOUT);

  } else {

    commsBudgetCnt = 0;
  }

  // ==================================================
  // PHASE 2 : SENSORS
  // ==================================================

  uint32_t tSensor_us = micros();

  bool sensorCycleDone = updateSensors();

  uint32_t dtSensor = micros() - tSensor_us;

  static uint8_t sensorBudgetCnt = 0;

  // --------------------------------------------------
  // SENSOR BUDGET CHECK
  // --------------------------------------------------

  if (dtSensor > (BUDGET_SENSORS_MS * 1000UL)) {

    if (++sensorBudgetCnt >= PHASE_BUDGET_CONFIRM) {
      latchFault(FaultCode::SENSOR_TIMEOUT);
    }

  } else {

    sensorBudgetCnt = 0;
  }

  // --------------------------------------------------
  // SENSOR WATCHDOG FEED
  // --------------------------------------------------

  if (sensorCycleDone) {
    wdSensor.lastUpdate_ms = now;
  }

  // ==================================================
  // DRIVE EVENT LAYER
  // ==================================================
  memcpy(curA_snapshot, curA, sizeof(curA));
  lastDriveEvent = DriveEvent::NONE;

  updateEngineState(now);

  // --------------------------------------------------
  // IDLE CURRENT OFFSET AUTO RECALIBRATION
  // --------------------------------------------------
  idleCurrentAutoRezero(now);

  detectSideImbalanceAndSteer();
  detectWheelStuck(now);
  detectWheelLock();

  if (detectMotorStall())
    lastDriveEvent = DriveEvent::WHEEL_LOCK;

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

  if (faultLatched)
    systemState = SystemState::FAULT;

  processFaultReset(now);

  bool emergencyActive =
    (systemState == SystemState::FAULT) || (getDriveSafety() == SafetyState::EMERGENCY);

  // ==================================================
  // SYSTEM GATE
  // ==================================================
  if (systemState != SystemState::ACTIVE || emergencyActive) {

    handleFaultImmediateCut();

    digitalWrite(PIN_DRV_ENABLE, LOW);

    backgroundFaultEEPROMTask(now);
    monitorSubsystemWatchdogs(now);

#if TELEMETRY_CSV
    telemetryCSV(now, loopStart_us);
#endif

    digitalWrite(PIN_HW_WD_HB,
                 !digitalRead(PIN_HW_WD_HB));

    return;
  }

  // ==================================================
  // PHASE 3 : DRIVE
  // ==================================================
  uint32_t tDrive_us = micros();

  runDrive(now);

  uint32_t tApply_us = micros();
  applyDrive();
  uint32_t dtApply = micros() - tApply_us;

  // applyDrive watchdog guard
  if (dtApply > BUDGET_DRIVE_MS * 1000UL) {

    if (++driveBudgetCnt >= PHASE_BUDGET_CONFIRM)
      latchFault(FaultCode::DRIVE_TIMEOUT);

  } else {

    driveBudgetCnt = 0;
  }

  // feed watchdog AFTER applyDrive
  wdDrive.lastUpdate_ms = now;

  // ==================================================
  // PHASE 3B : BLADE
  // ==================================================
  uint32_t tBlade_us = micros();

  runBlade(now);

  uint32_t dtBlade = micros() - tBlade_us;

  if (dtBlade > BUDGET_BLADE_MS * 1000UL) {

    if (++bladeBudgetCnt >= PHASE_BUDGET_CONFIRM)
      latchFault(FaultCode::BLADE_TIMEOUT);

  } else {

    bladeBudgetCnt = 0;
    wdBlade.lastUpdate_ms = now;
  }

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
  // INDUSTRIAL LOOP TIMING SUPERVISOR
  // ==================================================
  {
    uint32_t loopTime_us = micros() - loopStart_us;
    uint32_t loopBudget_us = BUDGET_LOOP_MS * 1000UL;

    if (loopTime_us > LOOP_HARD_LIMIT_US)
      latchFault(FaultCode::LOOP_OVERRUN);

    if (loopTime_us > loopBudget_us)
      loopOverrunAccum_us += (loopTime_us - loopBudget_us);
    else {
      loopOverrunAccum_us -= LOOP_OVERRUN_RECOVER_US;
      if (loopOverrunAccum_us < 0)
        loopOverrunAccum_us = 0;
    }

    if (loopOverrunAccum_us > LOOP_OVERRUN_FAULT_US)
      latchFault(FaultCode::LOOP_OVERRUN);
  }

  // ==================================================
  // DRIVER ENABLE CONTROL (SPIKE SAFE)
  // ==================================================

  static uint32_t driveEnableArmStart_ms = 0;
  static uint32_t driverEnabled_ms = 0;

  static bool driverEnabled = false;
  static bool driverSettling = false;

  bool runAllowed =
    (driveState == DriveState::RUN || driveState == DriveState::LIMP);

  bool hardCut =
    emergencyActive || (driveState == DriveState::SOFT_STOP) || (driveState == DriveState::LOCKED);

  bool pwmSafe =
    (abs(curL) < 20) && (abs(curR) < 20);

  bool thrNeutral =
    neutral(ibus.readChannel(CH_THROTTLE));

  bool strNeutral =
    neutral(ibus.readChannel(CH_STEER));

  bool rcSafe = thrNeutral && strNeutral;

  // --------------------------------------------------
  // TARGET STABILITY GUARD
  // ต้องนิ่งก่อน enable driver
  // --------------------------------------------------

  static uint32_t targetStableStart_ms = 0;

  bool targetNowZero =
    (targetL == 0) && (targetR == 0);

  if (targetNowZero) {

    if (targetStableStart_ms == 0)
      targetStableStart_ms = now;

  } else {

    targetStableStart_ms = 0;
  }

  bool targetSafe =
    (targetStableStart_ms != 0) && (now - targetStableStart_ms >= 50);

  bool ibusConfirmed =
    !requireIbusConfirm;

  bool autoReverseInactive =
    !autoReverseActive;

  bool dirSafe =
    digitalRead(DIR_L1) == LOW && digitalRead(DIR_L2) == LOW && digitalRead(DIR_R1) == LOW && digitalRead(DIR_R2) == LOW;

  constexpr uint32_t DRIVER_ARM_MS = 80;
  constexpr uint32_t DRIVER_SETTLE_MS = 40;

  // ---------------- HARD CUT ----------------
  if (hardCut) {

    driveEnableArmStart_ms = 0;

    driverEnabled = false;
    driverSettling = false;

    digitalWrite(PIN_DRV_ENABLE, LOW);
  }

  // ---------------- ARM DRIVER ----------------
  else if (runAllowed && pwmSafe && rcSafe && targetSafe && ibusConfirmed && autoReverseInactive && dirSafe) {

    if (driveEnableArmStart_ms == 0)
      driveEnableArmStart_ms = now;

    // --------------------------------------------------
    // DRIVER ARM COMPLETE
    // --------------------------------------------------
    static uint32_t pwmZeroStart_ms = 0;

    if (!driverEnabled && (now - driveEnableArmStart_ms >= DRIVER_ARM_MS)) {

      // บังคับ PWM = 0 ก่อน
      setPWM_L(0);
      setPWM_R(0);

      curL = 0;
      curR = 0;

      targetL = 0;
      targetR = 0;

      // เริ่มจับเวลา PWM zero hold
      if (pwmZeroStart_ms == 0)
        pwmZeroStart_ms = now;

      // รอ PWM settle
      if (now - pwmZeroStart_ms >= 5) {

        digitalWrite(PIN_DRV_ENABLE, HIGH);

        driverEnabled = true;
        driverSettling = true;

        driverEnabled_ms = now;

        pwmZeroStart_ms = 0;
      }
    }
  }

  // ---------------- CONDITIONS NOT SAFE ----------------
  else {

    driveEnableArmStart_ms = 0;

    driverEnabled = false;
    driverSettling = false;

    digitalWrite(PIN_DRV_ENABLE, LOW);
  }

  // ---------------- DRIVER SETTLE WINDOW ----------------
  if (driverSettling) {

    if (now - driverEnabled_ms < DRIVER_SETTLE_MS) {

      curL = 0;
      curR = 0;

      targetL = 0;
      targetR = 0;

    } else {

      driverSettling = false;
    }
  }

#if TELEMETRY_CSV
  telemetryCSV(now, loopStart_us);
#endif

  // ==================================================
  // WATCHDOG FEED (PROGRESS SAFE + RAM GUARD)
  // ==================================================

  static uint32_t lastLoopProgress_ms = 0;

  // --------------------------------------------------
  // RAM GUARD (CRITICAL FOR MEGA2560)
  // --------------------------------------------------
  int freeMem = freeRam();

  if (freeMem < 600) {

#if DEBUG_SERIAL
    Serial.print(F("[RAM] LOW MEMORY: "));
    Serial.println(freeMem);
#endif

    latchFault(FaultCode::LOGIC_WATCHDOG);
  }

  // --------------------------------------------------
  // WATCHDOG HEALTH CHECK
  // --------------------------------------------------
  bool wdHealthy =
    !wdSensor.faulted && !wdComms.faulted && !wdDrive.faulted && !wdBlade.faulted && !faultLatched;

  // ตรวจว่า loop ยังเดินจริง
  bool loopProgress =
    (now - lastLoopProgress_ms < 200);

  if (wdHealthy && loopProgress) {

    wdt_reset();

    digitalWrite(PIN_HW_WD_HB,
                 !digitalRead(PIN_HW_WD_HB));

  } else {

    digitalWrite(PIN_HW_WD_HB, LOW);
  }

  // update loop progress marker
  lastLoopProgress_ms = now;
}
