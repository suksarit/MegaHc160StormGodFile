// ========================================================================================
// SafetyManager.cpp  
// ========================================================================================

#include "SafetyManager.h"

// ============================================================================
// INTERNAL SAFETY STABILITY STATE (PRIVATE TO THIS FILE)
// ============================================================================
enum class SafetyStabilityState : uint8_t {
  SAFE_TRANSIENT = 0,
  SAFE_STABLE
};

// ============================================================================
// INTERNAL FILTER COUNTERS (FIELD HARDENING)
// ============================================================================
static uint8_t limpCnt = 0;
static uint8_t warnCnt = 0;

// ต้องเกิดต่อเนื่องกี่รอบจึงถือว่า “จริง”
static constexpr uint8_t LIMP_CONFIRM_CNT = 3;
static constexpr uint8_t WARN_CONFIRM_CNT = 2;

// ============================================================================
// INTERNAL SAFETY STABILITY STATE
// ============================================================================
static SafetyStabilityState safetyStability = SafetyStabilityState::SAFE_TRANSIENT;
static uint32_t safeStableStart_ms = 0;
static constexpr uint32_t SAFE_STABLE_TIME_MS = 2000;

// ============================================================================
// RAW SAFETY EVALUATION (PURE LOGIC, FILTERED, FIELD SAFE)
// ============================================================================
SafetyState evaluateSafetyRaw() {

  // --------------------------------------------------
  // HARD FAULT ALWAYS WINS
  // --------------------------------------------------
  if (faultLatched) {
    limpCnt = 0;
    warnCnt = 0;
    return SafetyState::EMERGENCY;
  }

  float curMax = max(
    max(curA[0], curA[1]),
    max(curA[2], curA[3])
  );

  // --------------------------------------------------
  // LIMP CONDITION (REQUIRES CONFIRMATION)
  // --------------------------------------------------
  if (curMax > CUR_LIMP_A ||
      tempDriverL > TEMP_LIMP_C ||
      tempDriverR > TEMP_LIMP_C) {

    warnCnt = 0;

    if (++limpCnt >= LIMP_CONFIRM_CNT) {
      limpCnt = LIMP_CONFIRM_CNT;
      return SafetyState::LIMP;
    }

    return SafetyState::WARN;   // ยังไม่ถึง LIMP จริง
  }

  // --------------------------------------------------
  // WARN CONDITION (REQUIRES CONFIRMATION)
  // --------------------------------------------------
  if (curMax > CUR_WARN_A ||
      tempDriverL > TEMP_WARN_C ||
      tempDriverR > TEMP_WARN_C) {

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
// SAFETY STABILITY STATE MACHINE (HYSTERESIS + AUTO-RECOVERY CONTROL)
// ============================================================================
void updateSafetyStability(SafetyState raw, uint32_t now) {

  // --------------------------------------------------
  // PUBLISH RAW SAFETY
  // --------------------------------------------------
  driveSafety = raw;

  // --------------------------------------------------
  // NOT SAFE → IMMEDIATE RESET STABILITY
  // --------------------------------------------------
  if (raw != SafetyState::SAFE) {
    safetyStability    = SafetyStabilityState::SAFE_TRANSIENT;
    safeStableStart_ms = 0;
    return;
  }

  // --------------------------------------------------
  // SAFE → WAIT FOR STABILITY TIME
  // --------------------------------------------------
  if (safetyStability == SafetyStabilityState::SAFE_TRANSIENT) {

    if (safeStableStart_ms == 0) {
      safeStableStart_ms = now;
      return;
    }

    if (now - safeStableStart_ms >= SAFE_STABLE_TIME_MS) {

      safetyStability = SafetyStabilityState::SAFE_STABLE;

      // ===== RESET AUTO-REVERSE (ONLY WHEN FULLY STABLE) =====
      autoReverseCount  = 0;
      autoReverseActive = false;
      lastDriveEvent    = DriveEvent::NONE;
    }
  }

  // SAFE_STABLE → hold state
}
