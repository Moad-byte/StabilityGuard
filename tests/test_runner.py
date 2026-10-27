#!/usr/bin/env python3
"""
test_runner.py - Automated Test Runner & Verification Suite for StabilityGuard
Executes all 20 verification scenarios with deterministic inputs and exact assertions.
Generates structured markdown and console reports.
"""

import math
import sys
import os
import json

# Constants matching StabilityGuard.h
COUNTS_PER_G = 16384
MIN_MAG_SQ = 8192**2
MAX_MAG_SQ = 24576**2
THRESHOLD_WARN_ENTER_DEG = 15.0
THRESHOLD_WARN_EXIT_DEG = 12.0
THRESHOLD_DANGER_ENTER_DEG = 30.0
THRESHOLD_DANGER_EXIT_DEG = 26.0
EMA_ALPHA = 0.35
SAMPLES_TO_ESCALATE = 3
SAMPLES_TO_DEESCALATE = 5
SAMPLES_TO_RECOVER = 5

TAN_TABLE_45 = [
      0,   286,   572,   859,  1146,  1433,  1722,  2012,  2303,  2595,
   2889,  3185,  3483,  3783,  4085,  4390,  4698,  5009,  5323,  5641,
   5963,  6289,  6620,  6955,  7295,  7640,  7991,  8348,  8712,  9082,
   9459,  9845, 10238, 10640, 11051, 11472, 11904, 12346, 12801, 13268,
  13748, 14242, 14752, 15278, 15822, 16384
]

def isqrt64(n):
    if n <= 0: return 0
    return math.isqrt(n)

def computeAngleDegrees(h, z_raw):
    if h == 0:
        return 0.0 if z_raw >= 0 else 180.0
    abs_z = abs(z_raw)
    angle_0_90 = 45.0
    if abs_z == 0:
        angle_0_90 = 90.0
    elif h <= abs_z:
        ratio = (h * 16384) // abs_z
        for i in range(45):
            if ratio <= TAN_TABLE_45[i+1]:
                span = TAN_TABLE_45[i+1] - TAN_TABLE_45[i]
                frac = (ratio - TAN_TABLE_45[i]) / span if span > 0 else 0.0
                angle_0_90 = i + frac
                break
    else:
        ratio = (abs_z * 16384) // h
        for i in range(45):
            if ratio <= TAN_TABLE_45[i+1]:
                span = TAN_TABLE_45[i+1] - TAN_TABLE_45[i]
                frac = (ratio - TAN_TABLE_45[i]) / span if span > 0 else 0.0
                angle_0_90 = 90.0 - (i + frac)
                break
    return angle_0_90 if z_raw >= 0 else 180.0 - angle_0_90

class PythonGuard:
    def __init__(self):
        self.reset()

    def reset(self):
        self.state = "SENSOR_FAULT"
        self.fault = "READ_FAILURE"
        self.valid = False
        self.raw_tilt_deg = 0.0
        self.filtered_tilt_deg = 0.0
        self.filter_ready = False
        self.recovery = 0
        self.pending = 0
        self.pending_state = "SAFE"
        self.stage = "STARTUP"
        self.last_mag_sq = 0

    def update(self, ax, ay, az, read_ok=True, failure="NONE"):
        if not read_ok:
            self._fault(failure if failure != "NONE" else "READ_FAILURE", "READ_FAILED_OR_BUTTON")
            return self._result()

        x, y, z = int(ax), int(ay), int(az)
        self.last_mag_sq = x*x + y*y + z*z

        if self.last_mag_sq < MIN_MAG_SQ or self.last_mag_sq > MAX_MAG_SQ:
            self._fault("INVALID_VECTOR", "RAW_MAGNITUDE_OUT_OF_RANGE")
            return self._result()

        horizontal = isqrt64(x*x + y*y)
        raw = computeAngleDegrees(horizontal, z)

        filtered = (EMA_ALPHA * raw + (1.0 - EMA_ALPHA) * self.filtered_tilt_deg) if self.filter_ready else raw

        if not (0.0 <= raw <= 180.01 and 0.0 <= filtered <= 180.01):
            self._fault("INVALID_VECTOR", "ANGLE_MATH_INVALID")
            return self._result()

        self.stage = "OK"
        self.valid = True
        self.fault = "NONE"
        self.raw_tilt_deg = raw
        self.filtered_tilt_deg = filtered
        self.filter_ready = True

        if self.state == "SENSOR_FAULT":
            self.recovery += 1
            if self.recovery >= SAMPLES_TO_RECOVER:
                self.state = self.classify(filtered)
                self.recovery = 0
                self.pending = 0
            return self._result()

        next_s = self.state
        if self.state == "SAFE":
            if filtered >= THRESHOLD_DANGER_ENTER_DEG:
                next_s = "DANGER"
            elif filtered >= THRESHOLD_WARN_ENTER_DEG:
                next_s = "WARNING"
        elif self.state == "WARNING":
            if filtered >= THRESHOLD_DANGER_ENTER_DEG:
                next_s = "DANGER"
            elif filtered < THRESHOLD_WARN_EXIT_DEG:
                next_s = "SAFE"
        elif self.state == "DANGER":
            if filtered < THRESHOLD_DANGER_EXIT_DEG:
                next_s = "WARNING" if filtered >= THRESHOLD_WARN_ENTER_DEG else "SAFE"

        if next_s == self.state:
            self.pending = 0
        else:
            if next_s != self.pending_state:
                self.pending_state = next_s
                self.pending = 0
            escalating = self.severity(next_s) > self.severity(self.state)
            needed = SAMPLES_TO_ESCALATE if escalating else SAMPLES_TO_DEESCALATE
            self.pending += 1
            if self.pending >= needed:
                self.state = next_s
                self.pending = 0

        return self._result()

    def _fault(self, reason, stage):
        self.state = "SENSOR_FAULT"
        self.fault = reason
        self.valid = False
        self.raw_tilt_deg = 0.0
        self.filtered_tilt_deg = 0.0
        self.filter_ready = False
        self.recovery = 0
        self.pending = 0
        self.pending_state = "SAFE"
        self.stage = stage

    def _result(self):
        return {
            "state": self.state,
            "fault": self.fault,
            "valid": self.valid,
            "raw_tilt_deg": self.raw_tilt_deg,
            "filtered_tilt_deg": self.filtered_tilt_deg,
            "stage": self.stage
        }

    @staticmethod
    def classify(angle):
        if angle >= THRESHOLD_DANGER_ENTER_DEG: return "DANGER"
        if angle >= THRESHOLD_WARN_ENTER_DEG: return "WARNING"
        return "SAFE"

    @staticmethod
    def severity(s):
        if s == "DANGER": return 2
        if s == "WARNING": return 1
        return 0

def make_sample(deg, ok=True, failure="NONE"):
    rad = math.radians(deg)
    h = int(round(16384 * math.sin(rad)))
    z = int(round(16384 * math.cos(rad)))
    return (h, 0, z, ok, failure)

def run_all_tests():
    test_results = []

    def log_test(num, name, passed, details=""):
        test_results.append({
            "id": num,
            "name": name,
            "passed": passed,
            "details": details
        })
        status = "[PASS]" if passed else "[FAIL]"
        print(f"Test {num:02d}: {status} - {name} ({details})")

    # 1. Startup State
    g = PythonGuard()
    r = g._result()
    p1 = (r["state"] == "SENSOR_FAULT" and not r["valid"] and r["fault"] == "READ_FAILURE")
    log_test(1, "Startup State", p1, f"state={r['state']}, valid={r['valid']}")

    # 2. Five Valid Sample Recovery
    p2 = True
    for i in range(4):
        r = g.update(0, 0, 16384)
        if r["state"] != "SENSOR_FAULT": p2 = False
    r5 = g.update(0, 0, 16384)
    if r5["state"] != "SAFE" or not r5["valid"]: p2 = False
    log_test(2, "Five-Valid-Sample Recovery", p2, f"final_state={r5['state']}")

    # 3. Stable SAFE Operation
    for _ in range(10): g.update(0, 0, 16384)
    r = g._result()
    p3 = (r["state"] == "SAFE" and abs(r["filtered_tilt_deg"]) < 0.05)
    log_test(3, "Stable SAFE Operation", p3, f"filt={r['filtered_tilt_deg']:.2f}")

    # 4. WARNING Entry (~20° tilt) & Convergence
    g.reset()
    for _ in range(5): g.update(0, 0, 16384)
    for _ in range(6): r = g.update(-5603, 0, 15401)
    p4_escalate = (r["state"] == "WARNING" and r["filtered_tilt_deg"] >= 18.0)
    for _ in range(4): r = g.update(-5603, 0, 15401)
    p4_converge = (abs(r["filtered_tilt_deg"] - 19.99) < 0.6)
    p4 = p4_escalate and p4_converge
    log_test(4, "WARNING Entry & Convergence (~20 deg)", p4, f"state={r['state']}, filt={r['filtered_tilt_deg']:.2f}")

    # 5. DANGER Entry (~40° tilt) & Convergence
    g.reset()
    for _ in range(5): g.update(0, 0, 16384)
    for _ in range(6): r = g.update(-10535, 0, 12550)
    p5_escalate = (r["state"] == "DANGER" and r["filtered_tilt_deg"] >= 36.0)
    for _ in range(4): r = g.update(-10535, 0, 12550)
    p5_converge = (abs(r["filtered_tilt_deg"] - 40.01) < 1.0)
    p5 = p5_escalate and p5_converge
    log_test(5, "DANGER Entry & Convergence (~40 deg)", p5, f"state={r['state']}, filt={r['filtered_tilt_deg']:.2f}")

    # 6. Direct SAFE -> DANGER Escalation
    g.reset()
    for _ in range(5): g.update(0, 0, 16384)
    p6_start = (g.state == "SAFE")
    states_recorded = []
    for _ in range(5):
        r_step = g.update(-11585, 0, 11585)
        states_recorded.append(r_step["state"])
    p6 = p6_start and (states_recorded[-1] == "DANGER") and ("WARNING" not in states_recorded)
    log_test(6, "Direct SAFE -> DANGER Escalation", p6, f"trajectory={'->'.join(states_recorded)}")

    # 7. DANGER -> WARNING De-escalation
    g.reset()
    for _ in range(5): g.update(0, 0, 16384)
    for _ in range(6): g.update(-10535, 0, 12550) # Enter DANGER
    p7_start = (g.state == "DANGER")
    for _ in range(12): g.update(-5603, 0, 15401)
    p7 = p7_start and (g.state == "WARNING")
    log_test(7, "DANGER -> WARNING De-escalation", p7, f"start={p7_start}, final={g.state}, filt={g.filtered_tilt_deg:.2f}")

    # 8. WARNING -> SAFE De-escalation
    for _ in range(12): g.update(0, 0, 16384) # 0 deg
    p8 = (g.state == "SAFE" and g.filtered_tilt_deg < 0.5)
    log_test(8, "WARNING -> SAFE De-escalation", p8, f"state={g.state}, filt={g.filtered_tilt_deg:.2f}")

    # 9. Hysteresis Boundaries (12°, 15°, 26°, 30°)
    g.reset()
    for _ in range(5): g.update(0, 0, 16384)
    for _ in range(8): g.update(*make_sample(14.0))
    p9_14 = (g.state == "SAFE")
    for _ in range(8): g.update(*make_sample(16.0))
    p9_16 = (g.state == "WARNING")
    for _ in range(8): g.update(*make_sample(13.0))
    p9_13 = (g.state == "WARNING")
    for _ in range(12): g.update(*make_sample(11.0))
    p9_11 = (g.state == "SAFE")
    p9 = p9_14 and p9_16 and p9_13 and p9_11
    log_test(9, "Hysteresis Boundary Immunity", p9, f"14°={p9_14}, 16°={p9_16}, 13°(hold)={p9_13}, 11°(exit)={p9_11}")

    # 10. Escalation (3) vs De-escalation (5) Timing Asymmetry
    g.reset()
    for _ in range(5): g.update(0, 0, 16384)
    esc_samples = 0
    while g.state == "SAFE" and esc_samples < 10:
        g.update(*make_sample(25.0))
        esc_samples += 1
    deesc_samples = 0
    while g.state == "WARNING" and deesc_samples < 15:
        g.update(0, 0, 16384)
        deesc_samples += 1
    p10 = (esc_samples == 5 and deesc_samples == 6)
    log_test(10, "Escalation/De-escalation Asymmetry", p10, f"esc_steps={esc_samples} (3 qual), deesc_steps={deesc_samples} (5 qual)")

    # 11. Transient Spike Rejection
    g.reset()
    for _ in range(5): g.update(0, 0, 16384)
    g.update(*make_sample(45.0)) # 1 sample spike
    p11_spike = (g.state == "SAFE")
    g.update(0, 0, 16384)
    p11 = p11_spike and (g.state == "SAFE")
    log_test(11, "Transient Spike Rejection", p11, f"state_after_spike={g.state}")

    # 12. Zero Acceleration Vector Rejection
    r_zero = g.update(0, 0, 0)
    p12 = (r_zero["state"] == "SENSOR_FAULT" and r_zero["fault"] == "INVALID_VECTOR")
    log_test(12, "Zero Acceleration Vector Rejection", p12, f"fault={r_zero['fault']}")

    # 13. Excessive Acceleration Rejection (> 1.5g)
    for _ in range(5): g.update(0, 0, 16384)
    r_excess = g.update(0, 0, 32767)
    p13 = (r_excess["state"] == "SENSOR_FAULT" and r_excess["fault"] == "INVALID_VECTOR")
    log_test(13, "Excessive Acceleration Rejection", p13, f"fault={r_excess['fault']}")

    # 14. Sensor Read Failure
    for _ in range(5): g.update(0, 0, 16384)
    r_rf = g.update(0, 0, 16384, read_ok=False, failure="READ_FAILURE")
    p14 = (r_rf["state"] == "SENSOR_FAULT" and r_rf["fault"] == "READ_FAILURE")
    log_test(14, "Sensor Read Failure Handling", p14, f"state={r_rf['state']}, fault={r_rf['fault']}")

    # 15. Injected Fault Button Activation
    for _ in range(5): g.update(0, 0, 16384)
    r_inj = g.update(0, 0, 16384, read_ok=False, failure="INJECTED")
    p15 = (r_inj["state"] == "SENSOR_FAULT" and r_inj["fault"] == "INJECTED")
    log_test(15, "Injected Fault Button Detection", p15, f"state={r_inj['state']}, fault={r_inj['fault']}")

    # 16. Fault Recovery
    for _ in range(4): g.update(0, 0, 16384)
    p16_during = (g.state == "SENSOR_FAULT")
    r_rec = g.update(0, 0, 16384)
    p16 = p16_during and (r_rec["state"] == "SAFE")
    log_test(16, "5-Sample Fault Recovery to SAFE", p16, f"final_state={r_rec['state']}")

    # 17. Recovery While Dangerously Tilted
    g.update(0, 0, 16384, read_ok=False, failure="READ_FAILURE")
    for _ in range(4): g.update(-10535, 0, 12550) # 40 deg
    r_tilted = g.update(-10535, 0, 12550)
    p17 = (r_tilted["state"] == "DANGER")
    log_test(17, "Tilted Fault Recovery to DANGER", p17, f"recovered_state={r_tilted['state']}")

    # 18. Filter Reset After Fault
    g.update(0, 0, 16384, read_ok=False, failure="INJECTED")
    p18_zero = (g.filtered_tilt_deg == 0.0)
    r_seed = g.update(0, 0, 16384)
    p18 = p18_zero and (r_seed["filtered_tilt_deg"] == 0.0)
    log_test(18, "Filter State Reset After Fault", p18, f"zero_out={p18_zero}")

    # 19. Extreme Angles (0°, 90°, 180°)
    a0 = computeAngleDegrees(0, 16384)
    a90 = computeAngleDegrees(16384, 0)
    a180 = computeAngleDegrees(0, -16384)
    p19 = (abs(a0 - 0.0) < 0.01 and abs(a90 - 90.0) < 0.01 and abs(a180 - 180.0) < 0.01)
    log_test(19, "Extreme Angles (0°, 90°, 180°)", p19, f"0°={a0:.2f}, 90°={a90:.2f}, 180°={a180:.2f}")

    # 20. Full Cyclic Transition Sequence
    g.reset()
    for _ in range(5): g.update(0, 0, 16384)
    s1 = (g.state == "SAFE")
    for _ in range(8): g.update(-5603, 0, 15401)
    s2 = (g.state == "WARNING")
    for _ in range(8): g.update(-10535, 0, 12550)
    s3 = (g.state == "DANGER")
    for _ in range(12): g.update(-5603, 0, 15401)
    s4 = (g.state == "WARNING")
    for _ in range(12): g.update(0, 0, 16384)
    s5 = (g.state == "SAFE")
    p20 = s1 and s2 and s3 and s4 and s5
    log_test(20, "Full Cyclic Sequence (SAFE->WARN->DANGER->WARN->SAFE)", p20, f"cycle={'->'.join(['SAFE' if s else 'FAIL' for s in [s1, s2, s3, s4, s5]])}")

    total = len(test_results)
    passed = sum(1 for t in test_results if t["passed"])
    failed = total - passed
    print(f"\n============================================================")
    print(f" Automated Test Suite Execution Summary:")
    print(f"   Total Scenarios: {total}")
    print(f"   Passed:          {passed}")
    print(f"   Failed:          {failed}")
    print(f"   Success Rate:    {(passed/total)*100.0:.1f}%")
    print(f"============================================================")

    # Save structured test report
    report_path = os.path.join(os.path.dirname(__file__), "test_report.json")
    with open(report_path, "w") as f:
        json.dump({
            "total": total,
            "passed": passed,
            "failed": failed,
            "success_rate_percent": round((passed/total)*100.0, 1),
            "scenarios": test_results
        }, f, indent=2)

    return failed == 0

if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
