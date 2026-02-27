// ========================================================================================
// SafetyManager.cpp  (PURE RAW + FILTER + STABILITY SEPARATED)
// ========================================================================================

#include "SafetyManager.h"
#include <Arduino.h>  // for max()

// ============================================================================
// INTERNAL SAFETY STATE (ENCAPSULATED)
// ============================================================================
static SafetyState driveSafetyInternal = SafetyState::SAFE;

// ============================================================================
// HYSTERESIS FILTER COUNTERS
// ============================================================================
static uint8_t limpConfirmCnt = 0;
static uint8_t warnConfirmCnt = 0;

static constexpr uint8_t LIMP_CONFIRM_CNT = 3;
static constexpr uint8_t WARN_CONFIRM_CNT = 2;

// ============================================================================
// STABILITY TRACKER
// ============================================================================
enum class SafetyStabilityState : uint8_t {
  SAFE_TRANSIENT = 0,
  SAFE_STABLE
};

static SafetyStabilityState safetyStability =
  SafetyStabilityState::SAFE_TRANSIENT;

static uint32_t safeStableStart_ms = 0;
static constexpr uint32_t SAFE_STABLE_TIME_MS = 2000;

// ============================================================================
// PURE RAW SAFETY EVALUATION
// ============================================================================
SafetyState evaluateSafetyRaw(
  const SafetyInput& in,
  const SafetyThresholds& th)
{
  // --------------------------------------------------
  // HARD FAULT ALWAYS WINS
  // --------------------------------------------------
  if (in.faultLatched)
    return SafetyState::EMERGENCY;

  float curMax = max(
    max(in.curA[0], in.curA[1]),
    max(in.curA[2], in.curA[3]));

  // --------------------------------------------------
  // LIMP CONDITION
  // --------------------------------------------------
  if (curMax > th.CUR_LIMP_A ||
      in.tempDriverL > th.TEMP_LIMP_C ||
      in.tempDriverR > th.TEMP_LIMP_C)
  {
    return SafetyState::LIMP;
  }

  if (in.driveEvent == DriveEvent::STUCK_LEFT ||
      in.driveEvent == DriveEvent::STUCK_RIGHT)
  {
    return SafetyState::LIMP;
  }

  if (in.driveEvent == DriveEvent::IMBALANCE)
    return SafetyState::WARN;

  // --------------------------------------------------
  // WARN CONDITION
  // --------------------------------------------------
  if (curMax > th.CUR_WARN_A ||
      in.tempDriverL > th.TEMP_WARN_C ||
      in.tempDriverR > th.TEMP_WARN_C)
  {
    return SafetyState::WARN;
  }

  return SafetyState::SAFE;
}

// ============================================================================
// STABILITY + HYSTERESIS LAYER
// ============================================================================
void updateSafetyStability(
  SafetyState raw,
  uint32_t now,
  uint8_t& autoReverseCount,
  bool& autoReverseActive,
  DriveEvent& lastDriveEvent)
{
  SafetyState filtered = raw;

  // --------------------------------------------------
  // HYSTERESIS FILTER
  // --------------------------------------------------
  if (raw == SafetyState::LIMP) {

    warnConfirmCnt = 0;

    if (++limpConfirmCnt >= LIMP_CONFIRM_CNT) {
      filtered = SafetyState::LIMP;
      limpConfirmCnt = LIMP_CONFIRM_CNT;
    } else {
      filtered = SafetyState::WARN;
    }
  }
  else if (raw == SafetyState::WARN) {

    limpConfirmCnt = 0;

    if (++warnConfirmCnt >= WARN_CONFIRM_CNT) {
      filtered = SafetyState::WARN;
      warnConfirmCnt = WARN_CONFIRM_CNT;
    } else {
      filtered = SafetyState::SAFE;
    }
  }
  else {

    limpConfirmCnt = 0;
    warnConfirmCnt = 0;
  }

  // --------------------------------------------------
  // PUBLISH FILTERED RESULT
  // --------------------------------------------------
  driveSafetyInternal = filtered;

  // --------------------------------------------------
  // STABILITY TIMER (SAFE HOLD)
  // --------------------------------------------------
  if (filtered != SafetyState::SAFE) {

    safetyStability = SafetyStabilityState::SAFE_TRANSIENT;
    safeStableStart_ms = 0;
    return;
  }

  if (safetyStability == SafetyStabilityState::SAFE_TRANSIENT) {

    if (safeStableStart_ms == 0) {
      safeStableStart_ms = now;
      return;
    }

    if (now - safeStableStart_ms >= SAFE_STABLE_TIME_MS) {

      safetyStability = SafetyStabilityState::SAFE_STABLE;

      // RESET AUTO-REVERSE ONLY AFTER FULL SAFE WINDOW
      autoReverseCount = 0;
      autoReverseActive = false;
      lastDriveEvent = DriveEvent::NONE;
    }
  }
}

// ============================================================================
// ACCESSORS
// ============================================================================
SafetyState getDriveSafety()
{
  return driveSafetyInternal;
}

void forceSafetyState(SafetyState s)
{
  driveSafetyInternal = s;

  limpConfirmCnt = 0;
  warnConfirmCnt = 0;

  safetyStability = SafetyStabilityState::SAFE_TRANSIENT;
  safeStableStart_ms = 0;
}

