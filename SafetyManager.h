// ========================================================================================
// SafetyManager.h  (TESTABLE / FIELD-SAFE)
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
// PUBLIC RUNTIME STATE (OWNED BY SafetyManager)
// ============================================================================
extern SafetyState driveSafety;

// ============================================================================
// PUBLIC API
// ============================================================================

// PURE LOGIC: ไม่มี side-effect, ไม่มี global access
SafetyState evaluateSafetyRaw(
  const SafetyInput& in,
  const SafetyThresholds& th);

// STATE MACHINE: hysteresis + recovery control
void updateSafetyStability(
  SafetyState raw,
  uint32_t now,
  uint8_t& autoReverseCount,
  bool& autoReverseActive,
  volatile DriveEvent& lastDriveEvent);

#endif  // SAFETY_MANAGER_H


