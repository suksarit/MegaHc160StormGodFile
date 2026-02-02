// ========================================================================================
// SafetyManager.h
// ========================================================================================

#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include "SystemTypes.h"   // SafetyState, DriveEvent

// ============================================================================
// EXTERN RUNTIME STATES (owned by SafetyManager.cpp)
// ============================================================================
extern SafetyState driveSafety;

// ============================================================================
// EXTERN SENSOR RUNTIME VALUES (owned by .ino)
// ============================================================================
extern float   curA[4];
extern int16_t tempDriverL;
extern int16_t tempDriverR;

// ============================================================================
// EXTERN AUTO-REVERSE CONTROL
// ============================================================================
extern uint8_t autoReverseCount;
extern bool    autoReverseActive;
extern volatile DriveEvent lastDriveEvent;
extern bool    faultLatched;

// ============================================================================
// EXTERN THRESHOLDS (defined in .ino, NOT constexpr)
// ============================================================================
extern const int16_t CUR_WARN_A;
extern const int16_t CUR_LIMP_A;
extern const int16_t TEMP_WARN_C;
extern const int16_t TEMP_LIMP_C;

// ============================================================================
// PUBLIC API
// ============================================================================
SafetyState evaluateSafetyRaw();
void updateSafetyStability(SafetyState raw, uint32_t now);

#endif  // SAFETY_MANAGER_H
