// ========================================================================================
// SafetyManager.h  (PURE RAW + STABILITY LAYER SEPARATED)
// ========================================================================================

#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <stdint.h>
#include "SystemTypes.h"  // SafetyState, DriveEvent

// ============================================================================
// SAFETY INPUT SNAPSHOT (PURE DATA)
// ============================================================================
struct SafetyInput {
  float curA[4];        // กระแสแต่ละช่อง
  int16_t tempDriverL;  // อุณหภูมิไดรเวอร์ซ้าย
  int16_t tempDriverR;  // อุณหภูมิไดรเวอร์ขวา
  bool faultLatched;    // fault ระดับระบบ
  DriveEvent driveEvent;
};

// ============================================================================
// SAFETY THRESHOLDS (INJECTABLE)
// ============================================================================
struct SafetyThresholds {
  int16_t CUR_WARN_A;
  int16_t CUR_LIMP_A;
  int16_t TEMP_WARN_C;
  int16_t TEMP_LIMP_C;
};

// ============================================================================
// PURE RAW LOGIC (NO STATE / NO STATIC / NO SIDE EFFECT)
// ============================================================================
SafetyState evaluateSafetyRaw(
  const SafetyInput& in,
  const SafetyThresholds& th);

// ============================================================================
// STABILITY + HYSTERESIS LAYER
// ============================================================================
void updateSafetyStability(
  SafetyState raw,
  uint32_t now,
  uint8_t& autoReverseCount,
  bool& autoReverseActive,
  DriveEvent& lastDriveEvent);

// ============================================================================
// ACCESSORS
// ============================================================================
SafetyState getDriveSafety();
void forceSafetyState(SafetyState s);

#endif

