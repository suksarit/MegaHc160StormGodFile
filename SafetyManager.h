// ========================================================================================
// SafetyManager.h  (PURE RAW + STABILITY LAYER SEPARATED)
// INDUSTRIAL SAFE VERSION - ENUM EXTENDABLE
// ========================================================================================

#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <stdint.h>
#include "SystemTypes.h"   // SafetyState, DriveEvent

// ============================================================================
// SAFETY INPUT SNAPSHOT (PURE DATA - NO SIDE EFFECT)
// ============================================================================
struct SafetyInput {
  float curA[4];        // กระแสแต่ละช่อง
  int16_t tempDriverL;  // อุณหภูมิไดรเวอร์ซ้าย
  int16_t tempDriverR;  // อุณหภูมิไดรเวอร์ขวา
  bool faultLatched;    // fault ระดับระบบ
  DriveEvent driveEvent;
};

// ============================================================================
// SAFETY THRESHOLDS (INJECTABLE / TESTABLE)
// ============================================================================
struct SafetyThresholds {
  int16_t CUR_WARN_A;
  int16_t CUR_LIMP_A;
  int16_t TEMP_WARN_C;
  int16_t TEMP_LIMP_C;
};

// ============================================================================
// PURE RAW SAFETY EVALUATION
// - ไม่มี static
// - ไม่มี global
// - deterministic 100%
// ============================================================================
SafetyState evaluateSafetyRaw(
  const SafetyInput& in,
  const SafetyThresholds& th);

// ============================================================================
// STABILITY LAYER (HYSTERESIS / ANTI-FLAP)
// - มี internal static
// - ไม่ยุ่งกับ hardware
// ============================================================================
void updateSafetyStability(
  SafetyState raw,
  uint32_t now,
  uint8_t& autoReverseCount,
  bool& autoReverseActive,
  DriveEvent& lastDriveEvent);

// ============================================================================
// ACCESSORS (GLOBAL SAFETY STATE)
// ============================================================================
SafetyState getDriveSafety();
void forceSafetyState(SafetyState s);

// ============================================================================
// OPTIONAL: STRING CONVERSION (FOR DEBUG / LOG)
// ============================================================================
const char* safetyStateToString(SafetyState s);

#endif