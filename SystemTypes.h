// ========================================================================================
// SystemTypes.h
// ========================================================================================

#ifndef SYSTEM_TYPES_H
#define SYSTEM_TYPES_H

#include <Arduino.h>

// ============================================================================
// SAFETY / EVENT
// ============================================================================
enum class SafetyState : uint8_t {
  SAFE,
  WARN,
  LIMP,
  EMERGENCY
};

enum class DriveEvent : uint8_t {
  NONE,
  IMBALANCE,
  STUCK_LEFT,
  STUCK_RIGHT,
  WHEEL_LOCK,
  AUTO_REVERSE
};

// ============================================================================
// SYSTEM STATE MACHINES
// ============================================================================
enum class SystemState : uint8_t {
  INIT,
  WAIT_NEUTRAL,
  WAIT_BLADE_ARM,
  ACTIVE,
  FAULT
};

enum class DriveState : uint8_t {
  IDLE,
  RUN,
  LIMP,
  SOFT_STOP,
  LOCKED
};

enum class BladeState : uint8_t {
  IDLE,
  RUN,
  SOFT_STOP,
  LOCKED
};

#endif


