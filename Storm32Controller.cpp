// ========================================================================================
// Storm32Controller.cpp  
// ========================================================================================
#include "Storm32Controller.h"

// =================================================
// Constructor
// =================================================
Storm32Controller::Storm32Controller(HardwareSerial& serial, IBusBM& ib)
  : stormSerial(serial), ibus(ib) {}

// =================================================
// Begin
// =================================================
void Storm32Controller::begin() {
  stormSerial.begin(115200);

  loadConfig();

  state = Storm32State::INIT;
  lastAckMs = millis();
  stateEnterMs = millis();
  lastTxMs = millis();

  targetPitch = targetYaw = 0.0f;
  currentPitch = currentYaw = 0.0f;

  hardDisabled = false;
  systemEnabled = false;  // รออนุญาตจากระบบรถ
}

// =================================================
// HARD DISABLE (override เท่านั้น ไม่ force EMERGENCY)
// =================================================
void Storm32Controller::hardDisable(bool enable) {
  hardDisabled = enable;

  if (hardDisabled) {
    sendDriveOff();
  }
}

// =================================================
// FORCE OFF (EMERGENCY LATCH แข็ง)
// =================================================
void Storm32Controller::forceOff() {
  hardDisabled = true;
  state = Storm32State::EMERGENCY;
  sendDriveOff();
}

// =================================================
// SYSTEM ENABLE (จากระบบรถ)
// =================================================
void Storm32Controller::setSystemEnabled(bool enable) {
  systemEnabled = enable;

  if (!systemEnabled) {
    sendDriveOff();
  }
}

// =================================================
// CLEAR EMERGENCY (อนุญาตเฉพาะระบบรถ)
// =================================================
void Storm32Controller::clearEmergency() {

  // ถ้า override อยู่ ห้ามปลด
  if (hardDisabled) return;

  if (state == Storm32State::EMERGENCY) {
    state = Storm32State::INIT;
    lastAckMs = millis();
    stateEnterMs = millis();

    targetPitch = targetYaw = 0.0f;
    currentPitch = currentYaw = 0.0f;
  }
}

// =================================================
// Update (เรียกจาก loop)
// =================================================
void Storm32Controller::update(uint32_t now) {

  // ------------------------------------------------
  // INIT / NOT ENABLED → HOLD ZERO
  // ------------------------------------------------
  if (state == Storm32State::INIT || !systemEnabled) {
    sendDriveOff();
    return;
  }

  // ------------------------------------------------
  // HARD DISABLE / EMERGENCY → HOLD ZERO
  // ------------------------------------------------
  if (hardDisabled || state == Storm32State::EMERGENCY) {
    sendDriveOff();
    return;
  }

  // ------------------------------------------------
  // HEARTBEAT (10 Hz)
// ------------------------------------------------
  if (now - lastTxMs >= 100) {
    int16_t p = (int16_t)(currentPitch * 100.0f);
    int16_t y = (int16_t)(currentYaw   * 100.0f);
    sendBinaryControl(p, y);
  }

  processSerial();
  updateWDT(now);

  if (state == Storm32State::EMERGENCY) {
    sendDriveOff();
    return;
  }

  updateTargetFromIBUS();
  applyMotion(now);
}

// =================================================
// Serial / ACK (Binary-safe, NO AUTO-RECOVER)
// =================================================
void Storm32Controller::processSerial() {

  if (hardDisabled || state == Storm32State::EMERGENCY) {
    while (stormSerial.available()) stormSerial.read();
    return;
  }

  if (stormSerial.available()) {
    stormSerial.read();          // alive byte only
    lastAckMs = millis();

    if (state == Storm32State::INIT || state == Storm32State::LOST) {
      enterState(Storm32State::OK);
    }
  }
}

// =================================================
// WDT State Machine
// =================================================
void Storm32Controller::updateWDT(uint32_t now) {

  if (hardDisabled || state == Storm32State::EMERGENCY) return;

  uint32_t dt = now - lastAckMs;

  switch (state) {
    case Storm32State::OK:
      if (dt > cfg.ackTimeout1) enterState(Storm32State::DEGRADED);
      break;

    case Storm32State::DEGRADED:
      if (dt > cfg.ackTimeout2) enterState(Storm32State::LOST);
      break;

    case Storm32State::LOST:
      if (dt > cfg.ackTimeout3) enterState(Storm32State::EMERGENCY);
      break;

    default:
      break;
  }
}

// =================================================
// Enter State (EMERGENCY = latch แข็ง)
// =================================================
void Storm32Controller::enterState(Storm32State newState) {

  if (state == Storm32State::EMERGENCY) return;

  state = newState;
  stateEnterMs = millis();

  if (state == Storm32State::EMERGENCY) {
    sendDriveOff();
  }
}

// =================================================
// iBUS → Target
// =================================================
void Storm32Controller::updateTargetFromIBUS() {

  uint16_t rcPitch = ibus.readChannel(STORM32_CH_PITCH);
  uint16_t rcYaw   = ibus.readChannel(STORM32_CH_YAW);

  rcPitch = constrain(rcPitch, 1000, 2000);
  rcYaw   = constrain(rcYaw,   1000, 2000);

  targetPitch = mapRCtoDeg(rcPitch, cfg.pitchLimitDeg, cfg.invertPitch);
  targetYaw   = mapRCtoDeg(rcYaw,   cfg.yawLimitDeg,   cfg.invertYaw);
}

// =================================================
// Motion Apply
// =================================================
void Storm32Controller::applyMotion(uint32_t) {

  float slew = (state == Storm32State::DEGRADED)
                 ? cfg.slewDegraded
                 : cfg.slewNormal;

  currentPitch = stepToward(currentPitch, targetPitch, slew);
  currentYaw   = stepToward(currentYaw,   targetYaw,   slew);
}

// =================================================
// Utility
// =================================================
float Storm32Controller::mapRCtoDeg(uint16_t rc, float limit, bool invert) {
  float v = map(rc, 1000, 2000, -limit * 100, limit * 100) / 100.0f;
  return invert ? -v : v;
}

float Storm32Controller::stepToward(float cur, float tgt, float step) {
  if (fabs(tgt - cur) <= step) return tgt;
  return cur + (tgt > cur ? step : -step);
}

// =================================================
// Storm32 Binary Commands
// =================================================
uint8_t Storm32Controller::calcChecksum(const uint8_t* buf, uint8_t len) {
  uint8_t s = 0;
  for (uint8_t i = 0; i < len; i++) s += buf[i];
  return s;
}

void Storm32Controller::sendBinaryControl(int16_t pitch, int16_t yaw) {

  uint8_t pkt[7];

  pkt[0] = STORM32_START_BYTE;
  pkt[1] = STORM32_CMD_CONTROL;
  pkt[2] = pitch & 0xFF;
  pkt[3] = pitch >> 8;
  pkt[4] = yaw & 0xFF;
  pkt[5] = yaw >> 8;
  pkt[6] = calcChecksum(pkt, 6);

  stormSerial.write(pkt, sizeof(pkt));
  lastTxMs = millis();
}

void Storm32Controller::sendDriveOff() {

  uint8_t pkt[7] = {
    STORM32_START_BYTE,
    STORM32_CMD_CONTROL,
    0, 0, 0, 0,
    0
  };

  pkt[6] = calcChecksum(pkt, 6);
  stormSerial.write(pkt, sizeof(pkt));
}

// =================================================
// EEPROM
// =================================================
void Storm32Controller::loadConfig() {
  EEPROM.get(EEPROM_STORM32_BASE, cfg);

  if (cfg.magic != STORM32_MAGIC) {
    setDefaultConfig();
    saveConfig();
  }

  cfg.pitchLimitDeg = constrain(cfg.pitchLimitDeg, 5.0f, 45.0f);
  cfg.yawLimitDeg   = constrain(cfg.yawLimitDeg,   5.0f, 90.0f);

  cfg.slewNormal    = constrain(cfg.slewNormal,    0.5f, 5.0f);
  cfg.slewDegraded  = constrain(cfg.slewDegraded,  0.2f, 3.0f);
}

void Storm32Controller::saveConfig() {
  cfg.magic = STORM32_MAGIC;
  EEPROM.put(EEPROM_STORM32_BASE, cfg);
}

void Storm32Controller::setDefaultConfig() {
  cfg.invertPitch = false;
  cfg.invertYaw   = false;

  cfg.pitchLimitDeg = 30.0f;
  cfg.yawLimitDeg   = 45.0f;

  cfg.slewNormal    = 3.0f;
  cfg.slewDegraded  = 1.0f;

  cfg.ackTimeout1  = 400;
  cfg.ackTimeout2  = 900;
  cfg.ackTimeout3  = 2000;

  cfg.magic = STORM32_MAGIC;
}

// =================================================
// Status
// =================================================
Storm32State Storm32Controller::getState() const {
  return state;
}

bool Storm32Controller::isLocked() const {
  return hardDisabled ||
         !systemEnabled ||
         state == Storm32State::INIT ||
         state == Storm32State::EMERGENCY;
}
