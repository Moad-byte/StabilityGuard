/**
 * @file test_stability_guard.cpp
 * @brief Comprehensive Automated Native C++ Test Suite for StabilityGuard
 *
 * Implements all 20 required verification scenarios without hardware or Arduino dependencies.
 * Can be compiled with any standard C++11 compiler:
 *   g++ -std=c++11 -Wall -Wextra -O2 test_stability_guard.cpp -o test_stability_guard
 *   ./test_stability_guard
 */

#include <iostream>
#include <iomanip>
#include <cassert>
#include <cmath>
#include <vector>
#include <string>
#include "../src/StabilityGuard.h"

using namespace stabilityguard;

static int g_passCount = 0;
static int g_failCount = 0;

#define TEST_ASSERT(cond, msg) \
  do { \
    if (cond) { \
      g_passCount++; \
    } else { \
      g_failCount++; \
      std::cerr << "  [FAIL] " << msg << " (Line " << __LINE__ << ")" << std::endl; \
    } \
  } while (0)

#define RUN_TEST(fn) \
  do { \
    std::cout << "[TEST] Running " << #fn << "..." << std::endl; \
    fn(); \
  } while (0)

static Sample makeSampleDeg(float deg, bool ok = true, Fault fault = Fault::NONE) {
  float rad = deg * (3.14159265358979323846f / 180.0f);
  int16_t h = (int16_t)std::round(16384.0f * std::sin(rad));
  int16_t z = (int16_t)std::round(16384.0f * std::cos(rad));
  return Sample(h, 0, z, ok, fault);
}

// 1. Startup state
void test01_StartupState() {
  Guard g;
  Result r = g.result();
  TEST_ASSERT(r.state == State::SENSOR_FAULT, "Startup state must be SENSOR_FAULT");
  TEST_ASSERT(r.valid == false, "Startup valid must be false");
  TEST_ASSERT(r.fault == Fault::READ_FAILURE, "Startup fault must be READ_FAILURE");
  TEST_ASSERT(std::string(g.diagnosticStage()) == "STARTUP", "Stage must be STARTUP");
}

// 2. Five-valid-sample recovery
void test02_FiveSampleRecovery() {
  Guard g;
  Sample level(0, 0, 16384, true, Fault::NONE);
  for (int i = 1; i <= 4; ++i) {
    Result r = g.update(level);
    TEST_ASSERT(r.state == State::SENSOR_FAULT, "Must remain SENSOR_FAULT before 5 samples");
    TEST_ASSERT(r.valid == true, "Individual samples are marked valid during recovery");
    TEST_ASSERT(r.fault == Fault::NONE, "Fault is cleared during valid acquisition");
  }
  Result r5 = g.update(level);
  TEST_ASSERT(r5.state == State::SAFE, "5th sample must recover to SAFE");
  TEST_ASSERT(r5.valid == true, "5th sample must be valid");
}

// 3. Stable SAFE operation
void test03_StableSafe() {
  Guard g;
  Sample level(0, 0, 16384, true, Fault::NONE);
  for (int i = 0; i < 10; ++i) g.update(level);
  Result r = g.result();
  TEST_ASSERT(r.state == State::SAFE, "State must remain SAFE");
  TEST_ASSERT(std::abs(r.filtered_tilt_deg) < 0.05f, "Tilt must be ~0.0 deg");
  TEST_ASSERT(std::abs(r.raw_tilt_deg) < 0.05f, "Raw tilt must be ~0.0 deg");
  TEST_ASSERT(g.accelerationMagnitude_mg() >= 995 && g.accelerationMagnitude_mg() <= 1005, "Mag must be ~1000 mg");
}

// 4. WARNING entry (~20 deg tilt)
void test04_WarningEntry() {
  Guard g;
  Sample level(0, 0, 16384);
  for (int i = 0; i < 5; ++i) g.update(level);

  Sample moderate(-5603, 0, 15401); // ~19.99 deg
  Result r;
  // Escalation: crosses 15° at sample 4, completes 3 qualifying samples on sample 6
  for (int i = 0; i < 6; ++i) {
    r = g.update(moderate);
  }
  TEST_ASSERT(r.state == State::WARNING, "State must transition to WARNING after 3 qualifying samples");
  TEST_ASSERT(r.filtered_tilt_deg >= 18.0f, "Filtered tilt must cross WARNING threshold");

  // Steady-state convergence after 10 samples
  for (int i = 0; i < 4; ++i) {
    r = g.update(moderate);
  }
  TEST_ASSERT(std::abs(r.filtered_tilt_deg - 19.99f) < 0.6f, "Filtered tilt must converge within 0.6 deg of 20 deg");
}

// 5. DANGER entry (~40 deg tilt)
void test05_DangerEntry() {
  Guard g;
  Sample level(0, 0, 16384);
  for (int i = 0; i < 5; ++i) g.update(level);

  Sample severe(-10535, 0, 12550); // ~40.01 deg
  Result r;
  // Escalation: crosses 30° at sample 4 (32.6°), completes 3 qualifying samples on sample 6 (37.0°)
  for (int i = 0; i < 6; ++i) {
    r = g.update(severe);
  }
  TEST_ASSERT(r.state == State::DANGER, "State must transition to DANGER after 3 qualifying samples");
  TEST_ASSERT(r.filtered_tilt_deg >= 36.0f, "Filtered tilt must cross DANGER threshold (>=30 deg)");

  // Steady-state convergence: by sample 10, EMA filter converges within 1.0 deg of target (39.5 deg)
  for (int i = 0; i < 4; ++i) {
    r = g.update(severe);
  }
  TEST_ASSERT(std::abs(r.filtered_tilt_deg - 40.01f) < 1.0f, "Filtered tilt must converge within 1.0 deg of 40 deg");
}

// 6. Direct SAFE -> DANGER escalation
void test06_DirectSafeToDanger() {
  Guard g;
  for (int i = 0; i < 5; ++i) g.update(Sample(0, 0, 16384));
  TEST_ASSERT(g.result().state == State::SAFE, "Baseline SAFE confirmed");

  Sample extreme(-11585, 0, 11585); // 45.0 deg
  std::vector<State> trajectory;
  for (int i = 0; i < 5; ++i) {
    Result r = g.update(extreme);
    trajectory.push_back(r.state);
  }
  TEST_ASSERT(trajectory.back() == State::DANGER, "Sample 5 must reach DANGER");
  bool warned = false;
  for (size_t i = 0; i < trajectory.size(); ++i) {
    if (trajectory[i] == State::WARNING) warned = true;
  }
  TEST_ASSERT(!warned, "Must transition directly from SAFE to DANGER without entering WARNING");
}

// 7. DANGER -> WARNING de-escalation (requires 5 consecutive samples)
void test07_DangerToWarning() {
  Guard g;
  for (int i = 0; i < 5; ++i) g.update(Sample(0, 0, 16384));
  for (int i = 0; i < 6; ++i) g.update(Sample(-10535, 0, 12550)); // Enter DANGER
  TEST_ASSERT(g.result().state == State::DANGER, "Confirmed DANGER state");

  Sample moderate(-5603, 0, 15401); // 20 deg
  for (int i = 0; i < 12; ++i) g.update(moderate);
  TEST_ASSERT(g.result().state == State::WARNING, "Must de-escalate cleanly to WARNING");
  TEST_ASSERT(std::abs(g.result().filtered_tilt_deg - 20.0f) < 0.5f, "Filtered tilt must be ~20 deg");
}

// 8. WARNING -> SAFE de-escalation (requires 5 consecutive samples)
void test08_WarningToSafe() {
  Guard g;
  for (int i = 0; i < 5; ++i) g.update(Sample(0, 0, 16384));
  for (int i = 0; i < 8; ++i) g.update(Sample(-5603, 0, 15401)); // Enter WARNING
  TEST_ASSERT(g.result().state == State::WARNING, "Confirmed WARNING state");

  Sample level(0, 0, 16384);
  for (int i = 0; i < 12; ++i) g.update(level);
  TEST_ASSERT(g.result().state == State::SAFE, "Must de-escalate cleanly to SAFE");
  TEST_ASSERT(g.result().filtered_tilt_deg < 0.5f, "Filtered tilt must be near zero");
}

// 9. Hysteresis boundary verification
void test09_HysteresisBoundaries() {
  Guard g;
  for (int i = 0; i < 5; ++i) g.update(Sample(0, 0, 16384));

  // 14 deg: below 15.0 deg WARNING threshold -> remains SAFE
  Sample tilt14 = makeSampleDeg(14.0f);
  for (int i = 0; i < 8; ++i) g.update(tilt14);
  TEST_ASSERT(g.result().state == State::SAFE, "14 deg must remain SAFE");

  // 16 deg: above 15.0 deg -> enters WARNING
  Sample tilt16 = makeSampleDeg(16.0f);
  for (int i = 0; i < 8; ++i) g.update(tilt16);
  TEST_ASSERT(g.result().state == State::WARNING, "16 deg must enter WARNING");

  // 13 deg: above exit threshold (12.0 deg) -> remains WARNING (hysteresis!)
  Sample tilt13 = makeSampleDeg(13.0f);
  for (int i = 0; i < 8; ++i) g.update(tilt13);
  TEST_ASSERT(g.result().state == State::WARNING, "13 deg must hold WARNING due to hysteresis");

  // 11 deg: below exit threshold (12.0 deg) -> returns to SAFE
  Sample tilt11 = makeSampleDeg(11.0f);
  for (int i = 0; i < 12; ++i) g.update(tilt11);
  TEST_ASSERT(g.result().state == State::SAFE, "11 deg must exit WARNING to SAFE");
}

// 10. Escalation (3) vs De-escalation (5) timing asymmetry
void test10_TimingAsymmetry() {
  Guard g;
  for (int i = 0; i < 5; ++i) g.update(Sample(0, 0, 16384));

  int esc_steps = 0;
  while (g.result().state == State::SAFE && esc_steps < 10) {
    g.update(makeSampleDeg(25.0f));
    esc_steps++;
  }
  int deesc_steps = 0;
  while (g.result().state == State::WARNING && deesc_steps < 15) {
    g.update(Sample(0, 0, 16384));
    deesc_steps++;
  }
  TEST_ASSERT(esc_steps == 5 && deesc_steps == 6, "Escalation takes 3 qual samples, de-escalation takes 5 qual samples");
}

// 11. Transient single-sample tilt spike rejection
void test11_TransientSpikeRejection() {
  Guard g;
  for (int i = 0; i < 5; ++i) g.update(Sample(0, 0, 16384));
  TEST_ASSERT(g.result().state == State::SAFE, "State is SAFE");

  g.update(makeSampleDeg(45.0f)); // 1 single sample severe spike
  TEST_ASSERT(g.result().state == State::SAFE, "Single spike must NOT change state");

  g.update(Sample(0, 0, 16384));
  TEST_ASSERT(g.result().state == State::SAFE, "State remains SAFE after spike");
}

// 12. Zero acceleration vector rejection
void test12_ZeroVectorRejection() {
  Guard g;
  for (int i = 0; i < 5; ++i) g.update(Sample(0, 0, 16384));

  Result r = g.update(Sample(0, 0, 0, true, Fault::NONE));
  TEST_ASSERT(r.state == State::SENSOR_FAULT, "Zero vector must trip SENSOR_FAULT");
  TEST_ASSERT(r.fault == Fault::INVALID_VECTOR, "Fault must be INVALID_VECTOR");
  TEST_ASSERT(r.valid == false, "Zero vector result must be invalid");
}

// 13. Excessive acceleration magnitude rejection (> 1.5g)
void test13_ExcessiveAcceleration() {
  Guard g;
  for (int i = 0; i < 5; ++i) g.update(Sample(0, 0, 16384));

  Result r = g.update(Sample(0, 0, 32767, true, Fault::NONE));
  TEST_ASSERT(r.state == State::SENSOR_FAULT, "Excessive accel must trip SENSOR_FAULT");
  TEST_ASSERT(r.fault == Fault::INVALID_VECTOR, "Fault must be INVALID_VECTOR");
}

// 14. Sensor read failure handling
void test14_SensorReadFailure() {
  Guard g;
  for (int i = 0; i < 5; ++i) g.update(Sample(0, 0, 16384));

  Result r = g.update(Sample(0, 0, 16384, false, Fault::READ_FAILURE));
  TEST_ASSERT(r.state == State::SENSOR_FAULT, "Read failure must trip SENSOR_FAULT");
  TEST_ASSERT(r.fault == Fault::READ_FAILURE, "Fault must be READ_FAILURE");
}

// 15. Injected fault button activation
void test15_InjectedFault() {
  Guard g;
  for (int i = 0; i < 5; ++i) g.update(Sample(0, 0, 16384));

  Result r = g.update(Sample(0, 0, 16384, false, Fault::INJECTED));
  TEST_ASSERT(r.state == State::SENSOR_FAULT, "Button fault must trip SENSOR_FAULT");
  TEST_ASSERT(r.fault == Fault::INJECTED, "Fault must be INJECTED");
}

// 16. Fault recovery to SAFE
void test16_FaultRecovery() {
  Guard g;
  for (int i = 0; i < 5; ++i) g.update(Sample(0, 0, 16384));
  g.update(Sample(0, 0, 16384, false, Fault::INJECTED));
  TEST_ASSERT(g.result().state == State::SENSOR_FAULT, "Confirmed fault state");

  for (int i = 0; i < 4; ++i) {
    Result r = g.update(Sample(0, 0, 16384, true, Fault::NONE));
    TEST_ASSERT(r.state == State::SENSOR_FAULT, "Must remain SENSOR_FAULT during recovery");
  }
  Result rFinal = g.update(Sample(0, 0, 16384, true, Fault::NONE));
  TEST_ASSERT(rFinal.state == State::SAFE, "5th valid sample recovers to SAFE");
}

// 17. Recovery while sensor remains dangerously tilted
void test17_RecoveryWhileDangerouslyTilted() {
  Guard g;
  for (int i = 0; i < 5; ++i) g.update(Sample(0, 0, 16384));

  g.update(Sample(0, 0, 16384, false, Fault::READ_FAILURE));
  TEST_ASSERT(g.result().state == State::SENSOR_FAULT, "Fault confirmed");

  Sample severe(-10535, 0, 12550); // 40 deg
  for (int i = 0; i < 4; ++i) g.update(severe);
  Result r5 = g.update(severe);

  TEST_ASSERT(r5.state == State::DANGER, "Tilted recovery must directly classify to DANGER");
}

// 18. Filter reset after fault
void test18_FilterResetAfterFault() {
  Guard g;
  for (int i = 0; i < 5; ++i) g.update(Sample(0, 0, 16384));
  for (int i = 0; i < 10; ++i) g.update(Sample(-10535, 0, 12550));
  TEST_ASSERT(g.result().filtered_tilt_deg > 35.0f, "Filter was at ~40 deg");

  g.update(Sample(0, 0, 16384, false, Fault::INJECTED));
  TEST_ASSERT(g.result().filtered_tilt_deg == 0.0f, "Filter output must zero out on fault");

  Result r = g.update(Sample(0, 0, 16384, true, Fault::NONE));
  TEST_ASSERT(r.filtered_tilt_deg == 0.0f, "New filter starts fresh without old tilt bias");
}

// 19. Angles near 0°, 90°, and 180°
void test19_ExtremeAngles() {
  float a0 = computeAngleDegrees(0, 16384);
  TEST_ASSERT(std::abs(a0 - 0.0f) < 0.01f, "0 deg angle calculation");

  float a90 = computeAngleDegrees(16384, 0);
  TEST_ASSERT(std::abs(a90 - 90.0f) < 0.01f, "90 deg angle calculation");

  float a180 = computeAngleDegrees(0, -16384);
  TEST_ASSERT(std::abs(a180 - 180.0f) < 0.01f, "180 deg angle calculation");
}

// 20. Full cyclic transition sequence: SAFE -> WARNING -> DANGER -> WARNING -> SAFE
void test20_FullCyclicSequence() {
  Guard g;
  Sample level(0, 0, 16384);
  Sample warn(-5603, 0, 15401);
  Sample danger(-10535, 0, 12550);

  for (int i = 0; i < 5; ++i) g.update(level);
  TEST_ASSERT(g.result().state == State::SAFE, "Cycle step 1: SAFE");

  for (int i = 0; i < 8; ++i) g.update(warn);
  TEST_ASSERT(g.result().state == State::WARNING, "Cycle step 2: WARNING");

  for (int i = 0; i < 8; ++i) g.update(danger);
  TEST_ASSERT(g.result().state == State::DANGER, "Cycle step 3: DANGER");

  for (int i = 0; i < 12; ++i) g.update(warn);
  TEST_ASSERT(g.result().state == State::WARNING, "Cycle step 4: WARNING");

  for (int i = 0; i < 12; ++i) g.update(level);
  TEST_ASSERT(g.result().state == State::SAFE, "Cycle step 5: SAFE");
}

int main() {
  std::cout << "============================================================" << std::endl;
  std::cout << " StabilityGuard Automated Native C++ Safety Engine Test Suite" << std::endl;
  std::cout << "============================================================" << std::endl;

  RUN_TEST(test01_StartupState);
  RUN_TEST(test02_FiveSampleRecovery);
  RUN_TEST(test03_StableSafe);
  RUN_TEST(test04_WarningEntry);
  RUN_TEST(test05_DangerEntry);
  RUN_TEST(test06_DirectSafeToDanger);
  RUN_TEST(test07_DangerToWarning);
  RUN_TEST(test08_WarningToSafe);
  RUN_TEST(test09_HysteresisBoundaries);
  RUN_TEST(test10_TimingAsymmetry);
  RUN_TEST(test11_TransientSpikeRejection);
  RUN_TEST(test12_ZeroVectorRejection);
  RUN_TEST(test13_ExcessiveAcceleration);
  RUN_TEST(test14_SensorReadFailure);
  RUN_TEST(test15_InjectedFault);
  RUN_TEST(test16_FaultRecovery);
  RUN_TEST(test17_RecoveryWhileDangerouslyTilted);
  RUN_TEST(test18_FilterResetAfterFault);
  RUN_TEST(test19_ExtremeAngles);
  RUN_TEST(test20_FullCyclicSequence);

  std::cout << "============================================================" << std::endl;
  std::cout << " Test Results Summary:" << std::endl;
  std::cout << "   Passed Assertions: " << g_passCount << std::endl;
  std::cout << "   Failed Assertions: " << g_failCount << std::endl;
  std::cout << "============================================================" << std::endl;

  return (g_failCount == 0) ? 0 : 1;
}
