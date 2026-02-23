// ========================================================================================
// SafetyManager.cpp  (TESTABLE / FIELD-SAFE)
// ========================================================================================

#include "SafetyManager.h"
#include <Arduino.h>   // for max()

// ============================================================================
// INTERNAL SAFETY STABILITY STATE (PRIVATE)
// ============================================================================
enum class SafetyStabilityState : uint8_t {
  SAFE_TRANSIENT = 0,
  SAFE_STABLE
};

// ============================================================================
// INTERNAL FILTER COUNTERS
// ============================================================================
static uint8_t limpCnt = 0;
static uint8_t warnCnt = 0;

static constexpr uint8_t LIMP_CONFIRM_CNT = 3;
static constexpr uint8_t WARN_CONFIRM_CNT = 2;

// ============================================================================
// INTERNAL STABILITY TRACKER
// ============================================================================
static SafetyStabilityState safetyStability = SafetyStabilityState::SAFE_TRANSIENT;
static uint32_t safeStableStart_ms = 0;
static constexpr uint32_t SAFE_STABLE_TIME_MS = 2000;

// ============================================================================
// RAW SAFETY EVALUATION (PURE FUNCTION)
// ============================================================================
SafetyState evaluateSafetyRaw(
  const SafetyInput& in,
  const SafetyThresholds& th
) {
  // --------------------------------------------------
  // HARD FAULT ALWAYS WINS
  // --------------------------------------------------
  if (in.faultLatched) {
    limpCnt = 0;
    warnCnt = 0;
    return SafetyState::EMERGENCY;
  }

  float curMax = max(
    max(in.curA[0], in.curA[1]),
    max(in.curA[2], in.curA[3])
  );

  // --------------------------------------------------
  // LIMP CONDITION
  // --------------------------------------------------
  if (curMax > th.CUR_LIMP_A ||
      in.tempDriverL > th.TEMP_LIMP_C ||
      in.tempDriverR > th.TEMP_LIMP_C) {

    warnCnt = 0;

    if (++limpCnt >= LIMP_CONFIRM_CNT) {
      limpCnt = LIMP_CONFIRM_CNT;
      return SafetyState::LIMP;
    }

    return SafetyState::WARN;
  }

  // --------------------------------------------------
  // WARN CONDITION
  // --------------------------------------------------
  if (curMax > th.CUR_WARN_A ||
      in.tempDriverL > th.TEMP_WARN_C ||
      in.tempDriverR > th.TEMP_WARN_C) {

    limpCnt = 0;

    if (++warnCnt >= WARN_CONFIRM_CNT) {
      warnCnt = WARN_CONFIRM_CNT;
      return SafetyState::WARN;
    }

    return SafetyState::SAFE;
  }

  // --------------------------------------------------
  // SAFE
  // --------------------------------------------------
  limpCnt = 0;
  warnCnt = 0;
  return SafetyState::SAFE;
}

// ============================================================================
// SAFETY STABILITY STATE MACHINE
// ============================================================================
void updateSafetyStability(
  SafetyState raw,
  uint32_t now,
  uint8_t& autoReverseCount,
  bool& autoReverseActive,
  volatile DriveEvent& lastDriveEvent
) {
  // --------------------------------------------------
  // PUBLISH RAW SAFETY
  // --------------------------------------------------
  driveSafety = raw;

  // --------------------------------------------------
  // NOT SAFE → RESET STABILITY
  // --------------------------------------------------
  if (raw != SafetyState::SAFE) {
    safetyStability    = SafetyStabilityState::SAFE_TRANSIENT;
    safeStableStart_ms = 0;
    return;
  }

  // --------------------------------------------------
  // SAFE → WAIT FOR STABLE TIME
  // --------------------------------------------------
  if (safetyStability == SafetyStabilityState::SAFE_TRANSIENT) {

    if (safeStableStart_ms == 0) {
      safeStableStart_ms = now;
      return;
    }

    if (now - safeStableStart_ms >= SAFE_STABLE_TIME_MS) {

      safetyStability = SafetyStabilityState::SAFE_STABLE;

      // RESET AUTO-REVERSE ONLY WHEN FULLY SAFE
      autoReverseCount  = 0;
      autoReverseActive = false;
      lastDriveEvent    = DriveEvent::NONE;
    }
  }
}



