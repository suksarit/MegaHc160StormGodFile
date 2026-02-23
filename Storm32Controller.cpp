#include "Storm32Controller.h"

// =================================================
Storm32Controller::Storm32Controller(HardwareSerial& serial,
                                     IBusBM& ib)
  : stormSerial(serial), ibus(ib) {}

// =================================================
void Storm32Controller::begin() {

  stormSerial.begin(115200);
  loadConfig();

  state = Storm32State::INIT;

  uint32_t now = millis();

  lastAckMs = now;
  lastTxMs = now;
  lastUpdateMs = now;
  lastPhaseSwitchMs = now;   // ✅ FIX

  targetPitch = targetYaw = 0.0f;
  currentPitch = currentYaw = 0.0f;

  hardDisabled = false;
  systemEnabled = false;

  frameIndex = 0;
  frameLength = 0;
  frameActive = false;
}

// =================================================
// ================== UPDATE ======================
// STRICT DETERMINISTIC VERSION
// =================================================
void Storm32Controller::update(uint32_t now)
{
  uint32_t dtMs = now - lastUpdateMs;
  lastUpdateMs = now;

  float dt = dtMs * 0.001f;
  if (dt <= 0) dt = 0.001f;
  if (dt > 0.1f) dt = 0.1f;

  // ------------------------------------------------
  // HARD LOCK CONDITIONS
  // ------------------------------------------------
  if (state == Storm32State::INIT ||
      !systemEnabled ||
      hardDisabled ||
      state == Storm32State::EMERGENCY)
  {
    sendDriveOff();
    return;
  }

  // ------------------------------------------------
  // PHASE SWITCH
  // ------------------------------------------------
  if (now - lastPhaseSwitchMs >=
      (CONTROL_WINDOW_MS + COMM_WINDOW_MS))
  {
    lastPhaseSwitchMs = now;
  }

  uint32_t phaseTime = now - lastPhaseSwitchMs;
  bool controlPhase = (phaseTime < CONTROL_WINDOW_MS);

  // ------------------------------------------------
  // CONTROL WINDOW
  // ------------------------------------------------
  if (controlPhase)
  {
    updateTargetFromIBUS();
    applyMotion(dt);
    return;
  }

  // ------------------------------------------------
  // COMM WINDOW
  // ------------------------------------------------

  // ---------- RX BUDGET ----------
  uint32_t startRx = micros();

  while (stormSerial.available())
  {
    if (micros() - startRx >= SERIAL_BUDGET_US)
      break;

    uint8_t b = stormSerial.read();
    parseByte(b, now);
  }

  updateWDT(now);

  if (state == Storm32State::EMERGENCY)
  {
    sendDriveOff();
    return;
  }

  // ---------- STRICT TX RATE ----------
  if (now - lastTxMs >= TX_PERIOD_MS)
  {
    uint32_t startTx = micros();

    int16_t p = (int16_t)(currentPitch * 100.0f);
    int16_t y = (int16_t)(currentYaw   * 100.0f);

    sendBinaryControl(p, y);

    if (micros() - startTx > TX_BUDGET_US)
    {
      enterState(Storm32State::DEGRADED);
    }

    lastTxMs = now;   // ✅ single owner
  }
}

// =================================================
// ================== iBUS ========================
// =================================================
void Storm32Controller::updateTargetFromIBUS() {

  uint16_t rcPitch = ibus.readChannel(STORM32_CH_PITCH);
  uint16_t rcYaw   = ibus.readChannel(STORM32_CH_YAW);

  rcPitch = constrain(rcPitch, 1000, 2000);
  rcYaw   = constrain(rcYaw,   1000, 2000);

  targetPitch = mapRCtoDeg(rcPitch,
                           cfg.pitchLimitDeg,
                           cfg.invertPitch);

  targetYaw = mapRCtoDeg(rcYaw,
                         cfg.yawLimitDeg,
                         cfg.invertYaw);
}

// =================================================
// ================== PARSER ======================
// =================================================
void Storm32Controller::parseByte(uint8_t b,
                                  uint32_t now)
{
  if (!frameActive)
  {
    if (b == STORM32_START_BYTE)
    {
      frameActive = true;
      frameIndex = 0;
      frameBuf[frameIndex++] = b;
    }
    return;
  }

  if (frameIndex >= STORM32_MAX_FRAME)
  {
    frameActive = false;
    frameIndex = 0;
    return;
  }

  frameBuf[frameIndex++] = b;

  if (frameIndex == 2)
  {
    frameLength = b;

    if (frameLength > STORM32_MAX_FRAME - 3)
    {
      frameActive = false;
      frameIndex = 0;
    }
  }

  if (frameActive &&
      frameIndex >= (frameLength + 4))
  {
    handleFrame(frameBuf, frameIndex, now);
    frameActive = false;
    frameIndex = 0;
  }
}

void Storm32Controller::handleFrame(uint8_t* buf,
                                    uint8_t len,
                                    uint32_t now)
{
  if (len < 5)
    return;

  uint16_t rx =
    (uint16_t)buf[len - 2] |
    ((uint16_t)buf[len - 1] << 8);

  uint16_t calc = crc16_ccitt(buf, len - 2);

  if (rx != calc)
    return;

  uint8_t cmd = buf[2];

  if (cmd == STORM32_CMD_CONTROL)
  {
    lastAckMs = now;

    if (state == Storm32State::INIT ||
        state == Storm32State::DEGRADED ||
        state == Storm32State::LOST)
    {
      enterState(Storm32State::OK);
    }
  }
}

// =================================================
// ================= WATCHDOG =====================
// =================================================
void Storm32Controller::updateWDT(uint32_t now)
{
  uint32_t dt = now - lastAckMs;

  switch (state)
  {
    case Storm32State::OK:
      if (dt > cfg.ackTimeout1)
        enterState(Storm32State::DEGRADED);
      break;

    case Storm32State::DEGRADED:
      if (dt > cfg.ackTimeout2)
        enterState(Storm32State::LOST);
      break;

    case Storm32State::LOST:
      if (dt > cfg.ackTimeout3)
        enterState(Storm32State::EMERGENCY);
      break;

    default:
      break;
  }
}

void Storm32Controller::enterState(Storm32State newState)
{
  if (state == Storm32State::EMERGENCY)
    return;

  state = newState;

  if (state == Storm32State::EMERGENCY)
    sendDriveOff();
}

// =================================================
// ================= MOTION =======================
// =================================================
void Storm32Controller::applyMotion(float dt)
{
  float slew =
    (state == Storm32State::DEGRADED)
      ? cfg.slewDegraded
      : cfg.slewNormal;

  float maxStep = slew * dt;

  currentPitch =
    stepToward(currentPitch, targetPitch, maxStep);

  currentYaw =
    stepToward(currentYaw, targetYaw, maxStep);

  currentPitch =
    constrain(currentPitch,
              -cfg.pitchLimitDeg,
               cfg.pitchLimitDeg);

  currentYaw =
    constrain(currentYaw,
              -cfg.yawLimitDeg,
               cfg.yawLimitDeg);
}

float Storm32Controller::stepToward(float cur,
                                    float tgt,
                                    float maxStep)
{
  float diff = tgt - cur;

  if (fabs(diff) <= maxStep)
    return tgt;

  return cur + (diff > 0 ? maxStep : -maxStep);
}

float Storm32Controller::mapRCtoDeg(uint16_t rc,
                                    float limit,
                                    bool invert)
{
  float v = map(rc, 1000, 2000,
                -limit * 100,
                 limit * 100) / 100.0f;

  return invert ? -v : v;
}

// =================================================
// ================= SEND =========================
// =================================================
void Storm32Controller::sendBinaryControl(int16_t pitch,
                                          int16_t yaw)
{
  int16_t maxP =
    (int16_t)(cfg.pitchLimitDeg * 100);
  int16_t maxY =
    (int16_t)(cfg.yawLimitDeg   * 100);

  pitch = constrain(pitch, -maxP, maxP);
  yaw   = constrain(yaw,   -maxY, maxY);

  uint8_t pkt[9];

  pkt[0] = STORM32_START_BYTE;
  pkt[1] = 4;
  pkt[2] = STORM32_CMD_CONTROL;
  pkt[3] = pitch & 0xFF;
  pkt[4] = pitch >> 8;
  pkt[5] = yaw & 0xFF;
  pkt[6] = yaw >> 8;

  uint16_t crc =
    crc16_ccitt(pkt, 7);

  pkt[7] = crc & 0xFF;
  pkt[8] = crc >> 8;

  stormSerial.write(pkt, 9);
}

void Storm32Controller::sendDriveOff()
{
  sendBinaryControl(0, 0);
}

// =================================================
// ================= CRC16 ========================
// =================================================
uint16_t Storm32Controller::crc16_ccitt(const uint8_t* data,
                                        uint8_t len)
{
  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;

    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }

  return crc;
}