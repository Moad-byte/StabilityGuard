#ifndef STABILITY_GUARD_H
#define STABILITY_GUARD_H

#include <stdint.h>
#include <stddef.h>

namespace stabilityguard {

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

struct Sample {
  int16_t ax, ay, az;
  bool read_ok;
  Fault failure;

  Sample(
    int16_t x,
    int16_t y,
    int16_t z,
    bool ok,
    Fault why
  ) :
    ax(x),
    ay(y),
    az(z),
    read_ok(ok),
    failure(why) {}
};

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

static const uint16_t TAN_TABLE_45[46] = {
      0,   286,   572,   859,  1146,  1433,  1722,  2012,  2303,  2595,
   2889,  3185,  3483,  3783,  4085,  4390,  4698,  5009,  5323,  5641,
   5963,  6289,  6620,  6955,  7295,  7640,  7991,  8348,  8712,  9082,
   9459,  9845, 10238, 10640, 11051, 11472, 11904, 12346, 12801, 13268,
  13748, 14242, 14752, 15278, 15822, 16384
};

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

static inline float computeAngleDegrees(uint32_t h, int16_t z_raw) {
  if (h == 0) {
    return (z_raw >= 0) ? 0.0f : 180.0f;
  }

  uint32_t abs_z = (z_raw < 0) ? (uint32_t)(-z_raw) : (uint32_t)z_raw;
  float angle_0_90 = 45.0f;

  if (abs_z == 0) {
    angle_0_90 = 90.0f;
  } else if (h <= abs_z) {
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
    if (!sample.read_ok) {
      fault(
        sample.failure == Fault::NONE
          ? Fault::READ_FAILURE
          : sample.failure,
        "READ_FAILED_OR_BUTTON"
      );
      return output_;
    }

    const int64_t x = sample.ax;
    const int64_t y = sample.ay;
    const int64_t z = sample.az;

    last_mag_sq_ = x*x + y*y + z*z;

    if (
      last_mag_sq_ < 8192LL*8192LL ||
      last_mag_sq_ > 24576LL*24576LL
    ) {
      fault(
        Fault::INVALID_VECTOR,
        "RAW_MAGNITUDE_OUT_OF_RANGE"
      );
      return output_;
    }

    const uint32_t horizontal = isqrt64((uint64_t)(x*x + y*y));
    const float raw = computeAngleDegrees(horizontal, sample.az);

    const float filtered = filter_ready_
      ? (0.35f * raw + 0.65f * output_.filtered_tilt_deg)
      : raw;

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

    stage_ = "OK";
    output_.valid = true;
    output_.fault = Fault::NONE;
    output_.raw_tilt_deg = raw;
    output_.filtered_tilt_deg = filtered;
    filter_ready_ = true;

    if (output_.state == State::SENSOR_FAULT) {
      if (++recovery_ >= 5) {
        output_.state = classify(filtered);
        recovery_ = 0;
        pending_ = 0;
      }
      return output_;
    }

    State next = output_.state;

    if (output_.state == State::SAFE) {
      if (filtered >= 30.0f) {
        next = State::DANGER;
      } else if (filtered >= 15.0f) {
        next = State::WARNING;
      }
    } else if (output_.state == State::WARNING) {
      if (filtered >= 30.0f) {
        next = State::DANGER;
      } else if (filtered < 12.0f) {
        next = State::SAFE;
      }
    } else if (output_.state == State::DANGER) {
      if (filtered < 26.0f) {
        next = (filtered >= 15.0f) ? State::WARNING : State::SAFE;
      }
    }

    if (next == output_.state) {
      pending_ = 0;
    } else {
      if (next != pending_state_) {
        pending_state_ = next;
        pending_ = 0;
      }

      const bool escalating = severity(next) > severity(output_.state);
      const unsigned needed = escalating ? 3u : 5u;

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

  static int severity(State s) {
    if (s == State::DANGER) return 2;
    if (s == State::WARNING) return 1;
    return 0;
  }

  static State classify(float angle) {
    if (angle >= 30.0f) return State::DANGER;
    if (angle >= 15.0f) return State::WARNING;
    return State::SAFE;
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
