// ========================================================================================
// Storm32Controller.h  
// ========================================================================================

#ifndef STORM32_CONTROLLER_H
#define STORM32_CONTROLLER_H

#include <Arduino.h>
#include <IBusBM.h>
#include <EEPROM.h>

// ================= EEPROM =================
#define EEPROM_STORM32_BASE 200
#define STORM32_MAGIC 0x32B3

// ================= iBUS Channels =================
#define STORM32_CH_PITCH 3
#define STORM32_CH_YAW   9

// ================= Storm32 Binary =================
#define STORM32_CMD_CONTROL 0x19
#define STORM32_START_BYTE  0xFA

// ================= State =================
// NOTE:
// - EMERGENCY = LATCHED (ออกไม่ได้เอง)
// - ต้องปลดด้วย clearEmergency() จากระบบรถเท่านั้น
enum class Storm32State : uint8_t {
  INIT,        // boot / ยังไม่อนุญาตให้ขยับ
  OK,          // ปกติ
  DEGRADED,    // ack ช้า / ลด slew
  LOST,        // เกือบขาดการสื่อสาร
  EMERGENCY    // LOCKED HARD
};

// ================= Config =================
struct Storm32Config {
  bool invertPitch;
  bool invertYaw;

  float pitchLimitDeg;
  float yawLimitDeg;

  float slewNormal;
  float slewDegraded;

  uint16_t ackTimeout1;
  uint16_t ackTimeout2;
  uint16_t ackTimeout3;

  uint16_t magic;
};

class Storm32Controller {
public:
  // ------------------------------------------------
  // Constructor / lifecycle
  // ------------------------------------------------
  Storm32Controller(HardwareSerial& serial, IBusBM& ibus);
  void begin();

  // update() จะทำงานได้ก็ต่อเมื่อ systemEnabled == true
  void update(uint32_t now);

  // ------------------------------------------------
  // Safety / Control API (MASTER = ระบบรถ)
  // ------------------------------------------------

  // HARD LATCH: เข้าสู่ EMERGENCY และล็อกถาวร
  void forceOff();

  // Override ระดับ hardware / logic (TRUE = ปิด 100%)
  void hardDisable(bool enable);

  // เปิด/ปิดการอนุญาตจากระบบรถ (ACTIVE / NOT ACTIVE)
  void setSystemEnabled(bool enable);

  // ปลด EMERGENCY (อนุญาตเฉพาะระบบรถ)
  void clearEmergency();

  // ------------------------------------------------
  // Status
  // ------------------------------------------------
  Storm32State getState() const;

  // true = ห้าม gimbal ขยับ (รวม INIT / EMERGENCY / hardDisable / systemDisabled)
  bool isLocked() const;

private:
  // ------------------------------------------------
  // External interfaces
  // ------------------------------------------------
  HardwareSerial& stormSerial;
  IBusBM& ibus;

  // ------------------------------------------------
  // Config / State
  // ------------------------------------------------
  Storm32Config cfg;
  Storm32State state;

  bool hardDisabled;     // override จากระบบหลัก
  bool systemEnabled;   // อนุญาตจาก systemState (ACTIVE เท่านั้น)

  uint32_t lastAckMs;
  uint32_t stateEnterMs;
  uint32_t lastTxMs;

  // ------------------------------------------------
  // Motion state
  // ------------------------------------------------
  float targetPitch;
  float targetYaw;
  float currentPitch;
  float currentYaw;

  // ------------------------------------------------
  // Core
  // ------------------------------------------------
  void processSerial();
  void updateWDT(uint32_t now);

  // enterState() จะไม่อนุญาตออกจาก EMERGENCY
  void enterState(Storm32State newState);

  // ------------------------------------------------
  // Control
  // ------------------------------------------------
  void updateTargetFromIBUS();
  void applyMotion(uint32_t now);

  // ------------------------------------------------
  // Utility
  // ------------------------------------------------
  float mapRCtoDeg(uint16_t rc, float limit, bool invert);
  float stepToward(float cur, float tgt, float step);

  // ------------------------------------------------
  // Storm32 Binary Commands
  // ------------------------------------------------
  void sendBinaryControl(int16_t pitch, int16_t yaw);
  void sendDriveOff();
  uint8_t calcChecksum(const uint8_t* buf, uint8_t len);

  // ------------------------------------------------
  // EEPROM
  // ------------------------------------------------
  void loadConfig();
  void saveConfig();
  void setDefaultConfig();
};

#endif  // STORM32_CONTROLLER_H
