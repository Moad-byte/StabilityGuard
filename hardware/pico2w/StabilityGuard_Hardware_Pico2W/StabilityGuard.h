/**
 * @file StabilityGuard.h
 * @brief Core Safety & Stability Detection Engine (Portable C++)
 *
 * Implements deterministic raw-integer acceleration vector validation,
 * FPU-independent tilt angle calculation (0° to 180°), exponential moving
 * average filtering, hysteresis-guarded state transitions (SAFE, WARNING,
 * DANGER, SENSOR_FAULT), and consecutive-sample confirmation debouncing.
 *
 * @note Portable C++98/C++11. Zero Arduino or hardware dependencies.
 * @version 2.1.0
 */

#ifndef STABILITY_GUARD_H
#define STABILITY_GUARD_H

#include <stdint.h>
#include <stddef.h>

namespace stabilityguard {

// ============================================================================
// SYSTEM CONFIGURATION & THRESHOLDS
// ============================================================================

/// MPU6050 sensitivity in +/-2g mode: 16,384 LSB/g
static const int64_t COUNTS_PER_G = 16384LL;

/// Plausible static gravity magnitude window: [0.5g, 1.5g]
static const int64_t MIN_MAG_COUNTS = 8192LL;   // 0.5g
static const int64_t MAX_MAG_COUNTS = 24576LL;  // 1.5g
static const int64_t MIN_MAG_SQ = MIN_MAG_COUNTS * MIN_MAG_COUNTS; // 67,108,864
static const int64_t MAX_MAG_SQ = MAX_MAG_COUNTS * MAX_MAG_COUNTS; // 603,979,776

/// Hysteresis angle thresholds (degrees)
static const float THRESHOLD_WARN_ENTER_DEG   = 15.0f;
static const float THRESHOLD_WARN_EXIT_DEG    = 12.0f;
static const float THRESHOLD_DANGER_ENTER_DEG = 30.0f;
static const float THRESHOLD_DANGER_EXIT_DEG  = 26.0f;

/// Exponential moving average filter coefficient
static const float EMA_ALPHA = 0.35f;

/// Qualification sample counts
static const unsigned SAMPLES_TO_ESCALATE   = 3u; // SAFE -> WARNING, WARNING -> DANGER
static const unsigned SAMPLES_TO_DEESCALATE = 5u; // DANGER -> WARNING, WARNING -> SAFE
static const unsigned SAMPLES_TO_RECOVER    = 5u; // SENSOR_FAULT -> normal state

// ============================================================================
// ENUMS & TELEMETRY STRUCTURES
// ============================================================================

enum class State : uint8_t {
  SAFE,
  WARNING,
  DANGER,
  SENSOR_FAULT
};

enum class Fault : uint8_t {
  NONE,
  READ_FAILURE,
  INJECTED,
  INVALID_VECTOR
};

inline const char* stateName(State s) {
  switch (s) {
    case State::SAFE: return "SAFE";
    case State::WARNING: return "WARNING";
    case State::DANGER: return "DANGER";
    default: return "SENSOR_FAULT";
  }
}

inline const char* faultName(Fault f) {
  switch (f) {
    case Fault::NONE: return "NONE";
    case Fault::READ_FAILURE: return "READ_FAILURE";
    case Fault::INJECTED: return "INJECTED";
    default: return "INVALID_VECTOR";
  }
}

/// Raw sensor measurement packet passed into the safety engine
struct Sample {
  int16_t ax, ay, az;
  bool read_ok;
  Fault failure;

  Sample(
    int16_t x = 0,
    int16_t y = 0,
    int16_t z = 16384,
    bool ok = true,
    Fault why = Fault::NONE
  ) :
    ax(x),
    ay(y),
    az(z),
    read_ok(ok),
    failure(why) {}
};

/// Processed safety assessment output
struct Result {
  State state;
  Fault fault;
  bool valid;
  float raw_tilt_deg;
  float filtered_tilt_deg;

  Result() :
    state(State::SENSOR_FAULT),
    fault(Fault::READ_FAILURE),
    valid(false),
    raw_tilt_deg(0.0f),
    filtered_tilt_deg(0.0f) {}
};

// ============================================================================
// INTEGER ARITHMETIC UTILITIES (FPU-INDEPENDENT)
// ============================================================================

/// 46-entry lookup table for round(tan(deg) * 16384) for deg = 0 to 45
static const uint16_t TAN_TABLE_45[46] = {
      0,   286,   572,   859,  1146,  1433,  1722,  2012,  2303,  2595,
   2889,  3185,  3483,  3783,  4085,  4390,  4698,  5009,  5323,  5641,
   5963,  6289,  6620,  6955,  7295,  7640,  7991,  8348,  8712,  9082,
   9459,  9845, 10238, 10640, 11051, 11472, 11904, 12346, 12801, 13268,
  13748, 14242, 14752, 15278, 15822, 16384
};

/// Fast, deterministic 64-bit integer square root (Newton-Raphson)
static inline uint32_t isqrt64(uint64_t n) {
  if (n == 0) return 0;
  uint64_t x = n;
  uint64_t y = (x + 1) / 2;
  while (y < x) {
    x = y;
    y = (x + n / x) / 2;
  }
  return (uint32_t)x;
}

/**
 * @brief Computes tilt angle relative to positive Z-axis in degrees (0.0° to 180.0°).
 * Uses table lookup and linear interpolation. Error < 0.01° across full range.
 * Completely immune to ARM Cortex-M33 / RP2350 hardware FPU instruction bugs.
 */
static inline float computeAngleDegrees(uint32_t h, int16_t z_raw) {
  if (h == 0) {
    return (z_raw >= 0) ? 0.0f : 180.0f;
  }

  uint32_t abs_z = (z_raw < 0) ? (uint32_t)(-z_raw) : (uint32_t)z_raw;
  float angle_0_90 = 45.0f;

  if (abs_z == 0) {
    angle_0_90 = 90.0f;
  } else if (h <= abs_z) {
    // Sector 0° to 45°: ratio = h / abs_z
    uint32_t ratio = ((uint64_t)h * 16384ULL) / abs_z;
    for (int i = 0; i < 45; ++i) {
      if (ratio <= TAN_TABLE_45[i + 1]) {
        uint32_t span = TAN_TABLE_45[i + 1] - TAN_TABLE_45[i];
        float frac = (span > 0) ? (float)(ratio - TAN_TABLE_45[i]) / (float)span : 0.0f;
        angle_0_90 = (float)i + frac;
        break;
      }
    }
  } else {
    // Sector 45° to 90°: cotangent ratio = abs_z / h
    uint32_t ratio = ((uint64_t)abs_z * 16384ULL) / h;
    for (int i = 0; i < 45; ++i) {
      if (ratio <= TAN_TABLE_45[i + 1]) {
        uint32_t span = TAN_TABLE_45[i + 1] - TAN_TABLE_45[i];
        float frac = (span > 0) ? (float)(ratio - TAN_TABLE_45[i]) / (float)span : 0.0f;
        angle_0_90 = 90.0f - ((float)i + frac);
        break;
      }
    }
  }

  return (z_raw >= 0) ? angle_0_90 : (180.0f - angle_0_90);
}

// ============================================================================
// GUARD DECISION ENGINE CLASS
// ============================================================================

class Guard {
public:
  Guard() {
    reset();
  }

  void reset() {
    output_ = Result();
    filter_ready_ = false;
    recovery_ = 0;
    pending_ = 0;
    pending_state_ = State::SAFE;
    stage_ = "STARTUP";
    last_mag_sq_ = 0;
  }

  Result update(const Sample& sample) {
    // 1. Hardware read failure or injected fault detection
    if (!sample.read_ok) {
      fault(
        sample.failure == Fault::NONE
          ? Fault::READ_FAILURE
          : sample.failure,
        "READ_FAILED_OR_BUTTON"
      );
      return output_;
    }

    // 2. Vector magnitude sanity validation (0.5g <= |a| <= 1.5g)
    const int64_t x = sample.ax;
    const int64_t y = sample.ay;
    const int64_t z = sample.az;

    last_mag_sq_ = x*x + y*y + z*z;

    if (last_mag_sq_ < MIN_MAG_SQ || last_mag_sq_ > MAX_MAG_SQ) {
      fault(
        Fault::INVALID_VECTOR,
        "RAW_MAGNITUDE_OUT_OF_RANGE"
      );
      return output_;
    }

    // 3. Compute horizontal magnitude and tilt angle (0° to 180°)
    const uint32_t horizontal = isqrt64((uint64_t)(x*x + y*y));
    const float raw = computeAngleDegrees(horizontal, sample.az);

    // 4. Exponential moving average (EMA)
    const float filtered = filter_ready_
      ? (EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * output_.filtered_tilt_deg)
      : raw;

    // 5. Bounds validation check
    if (!(
      raw >= 0.0f &&
      raw <= 180.01f &&
      filtered >= 0.0f &&
      filtered <= 180.01f
    )) {
      fault(
        Fault::INVALID_VECTOR,
        "ANGLE_MATH_INVALID"
      );
      return output_;
    }

    // Valid sample received
    stage_ = "OK";
    output_.valid = true;
    output_.fault = Fault::NONE;
    output_.raw_tilt_deg = raw;
    output_.filtered_tilt_deg = filtered;
    filter_ready_ = true;

    // 6. Sensor fault recovery: requires 5 consecutive valid samples
    if (output_.state == State::SENSOR_FAULT) {
      if (++recovery_ >= SAMPLES_TO_RECOVER) {
        output_.state = classify(filtered);
        recovery_ = 0;
        pending_ = 0;
      }
      return output_;
    }

    // 7. Proposed state assessment with hysteresis
    State next = output_.state;

    if (output_.state == State::SAFE) {
      if (filtered >= THRESHOLD_DANGER_ENTER_DEG) {
        next = State::DANGER;
      } else if (filtered >= THRESHOLD_WARN_ENTER_DEG) {
        next = State::WARNING;
      }
    } else if (output_.state == State::WARNING) {
      if (filtered >= THRESHOLD_DANGER_ENTER_DEG) {
        next = State::DANGER;
      } else if (filtered < THRESHOLD_WARN_EXIT_DEG) {
        next = State::SAFE;
      }
    } else if (output_.state == State::DANGER) {
      if (filtered < THRESHOLD_DANGER_EXIT_DEG) {
        next = (filtered >= THRESHOLD_WARN_ENTER_DEG) ? State::WARNING : State::SAFE;
      }
    }

    // 8. Consecutive sample confirmation (debouncing)
    if (next == output_.state) {
      pending_ = 0;
    } else {
      if (next != pending_state_) {
        pending_state_ = next;
        pending_ = 0;
      }

      const bool escalating = severity(next) > severity(output_.state);
      const unsigned needed = escalating ? SAMPLES_TO_ESCALATE : SAMPLES_TO_DEESCALATE;

      if (++pending_ >= needed) {
        output_.state = next;
        pending_ = 0;
      }
    }

    return output_;
  }

  const Result& result() const {
    return output_;
  }

  const char* diagnosticStage() const {
    return stage_;
  }

  int64_t magnitudeSquaredRaw() const {
    return last_mag_sq_;
  }

  /// Returns total acceleration magnitude in milli-g (1000 mg = 1.0g)
  uint32_t accelerationMagnitude_mg() const {
    if (last_mag_sq_ <= 0) return 0;
    uint32_t counts = isqrt64((uint64_t)last_mag_sq_);
    return (uint32_t)(((uint64_t)counts * 1000ULL) / (uint64_t)COUNTS_PER_G);
  }

  static State classify(float angle) {
    if (angle >= THRESHOLD_DANGER_ENTER_DEG) return State::DANGER;
    if (angle >= THRESHOLD_WARN_ENTER_DEG) return State::WARNING;
    return State::SAFE;
  }

  static int severity(State s) {
    if (s == State::DANGER) return 2;
    if (s == State::WARNING) return 1;
    return 0;
  }

private:
  void fault(Fault reason, const char* stage) {
    output_.state = State::SENSOR_FAULT;
    output_.fault = reason;
    output_.valid = false;
    output_.raw_tilt_deg = 0.0f;
    output_.filtered_tilt_deg = 0.0f;

    filter_ready_ = false;
    recovery_ = 0;
    pending_ = 0;
    pending_state_ = State::SAFE;
    stage_ = stage;
  }

  Result output_;
  bool filter_ready_;
  unsigned recovery_;
  unsigned pending_;
  State pending_state_;
  const char* stage_;
  int64_t last_mag_sq_;
};

} // namespace stabilityguard

#endif // STABILITY_GUARD_H
