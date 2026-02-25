#ifndef STORM32_CONTROLLER_H
#define STORM32_CONTROLLER_H

#include <Arduino.h>
#include <IBusBM.h>
#include <EEPROM.h>

// ============================================================================
// EEPROM / PROTOCOL CONSTANTS
// ============================================================================
#define EEPROM_STORM32_BASE 200
#define STORM32_MAGIC 0x32B3

#define STORM32_START_BYTE 0xFA
#define STORM32_CMD_CONTROL 0x19

#define STORM32_CH_PITCH 3
#define STORM32_CH_YAW 9

#define STORM32_MAX_FRAME 32

// ============================================================================
// STORM STATE
// ============================================================================
enum class Storm32State : uint8_t {
  INIT,
  OK,
  DEGRADED,
  LOST,
  EMERGENCY
};

// ============================================================================
// CONFIG STRUCT (EEPROM STORED)
// ============================================================================
struct Storm32Config {
  bool invertPitch;
  bool invertYaw;

  float pitchLimitDeg;
  float yawLimitDeg;

  float slewNormal;    // deg/sec
  float slewDegraded;  // deg/sec

  uint16_t ackTimeout1;
  uint16_t ackTimeout2;
  uint16_t ackTimeout3;

  uint16_t magic;
};

// ============================================================================
// STORM32 CONTROLLER CLASS
// ============================================================================
class Storm32Controller {
public:
  Storm32Controller(HardwareSerial& serial, IBusBM& ibus);

  void begin();
  void update(uint32_t now);

  void forceOff();
  void hardDisable(bool enable);
  void setSystemEnabled(bool enable);
  void clearEmergency();

  Storm32State getState() const;
  bool isLocked() const;

private:
  // --------------------------------------------------------------------------
  // SERIAL / RC
  // --------------------------------------------------------------------------
  HardwareSerial& stormSerial;
  IBusBM& ibus;

  // --------------------------------------------------------------------------
  // CONFIG / STATE
  // --------------------------------------------------------------------------
  Storm32Config cfg;
  Storm32State state;

  bool hardDisabled;
  bool systemEnabled;

  // --------------------------------------------------------------------------
  // TIMING
  // --------------------------------------------------------------------------
  uint32_t lastAckMs;
  uint32_t lastTxMs;
  uint32_t lastUpdateMs;
  uint32_t lastPhaseSwitchMs;

  // --------------------------------------------------------------------------
  // MOTION
  // --------------------------------------------------------------------------
  float targetPitch;
  float targetYaw;
  float currentPitch;
  float currentYaw;

  // --------------------------------------------------------------------------
  // FRAME PARSER
  // --------------------------------------------------------------------------
  uint8_t frameBuf[STORM32_MAX_FRAME];
  uint8_t frameIndex;
  uint8_t frameLength;
  bool frameActive;

  // --------------------------------------------------------------------------
  // ===== DETERMINISTIC SCHEDULER PARAMETERS =====
  // --------------------------------------------------------------------------

  // ----- Dual Phase Window -----
  static constexpr uint16_t CONTROL_WINDOW_MS = 10;  // control phase
  static constexpr uint16_t COMM_WINDOW_MS = 10;     // comm phase

  // ----- Strict TX Rate -----
  static constexpr uint16_t TX_PERIOD_MS = 100;  // 10 Hz

  // ----- Budget Guards -----
  static constexpr uint32_t TX_BUDGET_US = 600;      // max TX time per cycle
  static constexpr uint32_t SERIAL_BUDGET_US = 800;  // max RX time per cycle

  // --------------------------------------------------------------------------
  // INTERNAL METHODS
  // --------------------------------------------------------------------------
  void processSerial(uint32_t now);
  void parseByte(uint8_t b, uint32_t now);
  void handleFrame(uint8_t* buf, uint8_t len, uint32_t now);

  void updateWDT(uint32_t now);
  void enterState(Storm32State newState);

  void updateTargetFromIBUS();
  void applyMotion(float dt);

  float mapRCtoDeg(uint16_t rc, float limit, bool invert);
  float stepToward(float cur, float tgt, float maxStep);

  void sendBinaryControl(int16_t pitch, int16_t yaw);
  void sendDriveOff();

  uint16_t crc16_ccitt(const uint8_t* data, uint8_t len);

  void loadConfig();
  void saveConfig();
  void setDefaultConfig();
};

#endif


