// ========================================================================================
// SafetyManager.h  (TESTABLE / FIELD-SAFE / NO GLOBAL STATE)
// ========================================================================================

#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <stdint.h>
#include "SystemTypes.h"  // SafetyState, DriveEvent

// ============================================================================
// SAFETY INPUT SNAPSHOT (PURE DATA, NO GLOBAL DEPENDENCY)
// ============================================================================
struct SafetyInput {
  float curA[4];        // กระแสแต่ละช่อง
  int16_t tempDriverL;  // อุณหภูมิไดรเวอร์ซ้าย
  int16_t tempDriverR;  // อุณหภูมิไดรเวอร์ขวา
  bool faultLatched;    // fault ระดับระบบ
  DriveEvent driveEvent;
};

// ============================================================================
// SAFETY THRESHOLDS (INJECTABLE → UNIT TESTABLE)
// ============================================================================
struct SafetyThresholds {
  int16_t CUR_WARN_A;
  int16_t CUR_LIMP_A;
  int16_t TEMP_WARN_C;
  int16_t TEMP_LIMP_C;
};

// ============================================================================
// PUBLIC API
// ============================================================================

// --------------------------------------------------------------------------
// PURE LOGIC
// ไม่มี side-effect
// ไม่มี global access
// ใช้สำหรับ unit test ได้โดยตรง
// --------------------------------------------------------------------------
SafetyState evaluateSafetyRaw(
  const SafetyInput& in,
  const SafetyThresholds& th);

// --------------------------------------------------------------------------
// STATE MACHINE
// จัดการ hysteresis + stability timing
// ไม่มี global extern อีกต่อไป
// --------------------------------------------------------------------------
void updateSafetyStability(
  SafetyState raw,
  uint32_t now,
  uint8_t& autoReverseCount,
  bool& autoReverseActive,
  DriveEvent& lastDriveEvent);

// --------------------------------------------------------------------------
// SAFETY STATE ACCESS (ENCAPSULATED)
// --------------------------------------------------------------------------

// อ่านค่า safety ปัจจุบัน (read-only access)
SafetyState getDriveSafety();

// ใช้เฉพาะกรณี system-level reset เช่น ignition fault reset
void forceSafetyState(SafetyState s);

#endif  // SAFETY_MANAGER_H


