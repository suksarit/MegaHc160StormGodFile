// ========================================================================================
// SystemTypes.h  (FINAL - SAFE / AVR COMPATIBLE)
// ========================================================================================

#pragma once
#ifndef SYSTEM_TYPES_H
#define SYSTEM_TYPES_H

#include <stdint.h>

// ============================================================================
// SAFETY STATE
// ============================================================================
enum class SafetyState : uint8_t {
  SAFE      = 0,
  WARN      = 1,
  LIMP      = 2,
  EMERGENCY = 3,
  _COUNT
};

// ============================================================================
// DRIVE EVENT
// ============================================================================
enum class DriveEvent : uint8_t {
  NONE         = 0,
  IMBALANCE    = 1,
  STUCK_LEFT   = 2,
  STUCK_RIGHT  = 3,
  WHEEL_LOCK   = 4,
  AUTO_REVERSE = 5,
  _COUNT
};

// ============================================================================
// SYSTEM STATE
// ============================================================================
enum class SystemState : uint8_t {
  INIT   = 0,
  ACTIVE = 1,
  FAULT  = 2,
  _COUNT
};

// ============================================================================
// DRIVE STATE
// ============================================================================
enum class DriveState : uint8_t {
  IDLE      = 0,
  RUN       = 1,
  LIMP      = 2,
  SOFT_STOP = 3,
  LOCKED    = 4,
  _COUNT
};

// ============================================================================
// BLADE STATE
// ============================================================================
enum class BladeState : uint8_t {
  IDLE      = 0,
  RUN       = 1,
  SOFT_STOP = 2,
  LOCKED    = 3,
  _COUNT
};

enum class FaultCode : uint8_t {
  NONE = 0,
  IBUS_LOST,
  COMMS_TIMEOUT,
  LOGIC_WATCHDOG,
  CUR_SENSOR_FAULT,
  VOLT_SENSOR_FAULT,
  TEMP_SENSOR_FAULT,
  OVER_CURRENT,
  OVER_TEMP,
  DRIVE_TIMEOUT,
  BLADE_TIMEOUT,
  LOOP_OVERRUN,
  _COUNT
};

// ============================================================================
// ENUM VALIDATION
// ============================================================================
template<typename E>
inline bool isValidEnum(uint8_t raw) {
  return raw < static_cast<uint8_t>(E::_COUNT);
}

#endif



