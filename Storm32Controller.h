#ifndef STORM32_CONTROLLER_H
#define STORM32_CONTROLLER_H

#include <Arduino.h>
#include <IBusBM.h>
#include <EEPROM.h>

#define EEPROM_STORM32_BASE 200
#define STORM32_MAGIC 0x32B3

#define STORM32_START_BYTE  0xFA
#define STORM32_CMD_CONTROL 0x19

#define STORM32_CH_PITCH 3
#define STORM32_CH_YAW   9

#define STORM32_MAX_FRAME 32

enum class Storm32State : uint8_t {
  INIT,
  OK,
  DEGRADED,
  LOST,
  EMERGENCY
};

struct Storm32Config {
  bool invertPitch;
  bool invertYaw;

  float pitchLimitDeg;
  float yawLimitDeg;

  float slewNormal;     // deg/sec
  float slewDegraded;   // deg/sec

  uint16_t ackTimeout1;
  uint16_t ackTimeout2;
  uint16_t ackTimeout3;

  uint16_t magic;
};

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
  HardwareSerial& stormSerial;
  IBusBM& ibus;

  Storm32Config cfg;
  Storm32State state;

  bool hardDisabled;
  bool systemEnabled;

  uint32_t lastAckMs;
  uint32_t lastTxMs;
  uint32_t lastUpdateMs;

  float targetPitch;
  float targetYaw;
  float currentPitch;
  float currentYaw;

  // ===== FRAME PARSER =====
  uint8_t frameBuf[STORM32_MAX_FRAME];
  uint8_t frameIndex;
  uint8_t frameLength;
  bool    frameActive;

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