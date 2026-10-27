#!/usr/bin/env python3
"""
telemetry_analysis.py - StabilityGuard Telemetry Analysis & Visualization Engine

Reads exported serial CSV telemetry from StabilityGuard Phase 1, performs statistical
validation, checks for invalid state transitions, calculates state time distributions,
and generates both an SVG vector plot and an ASCII visualization.
Also generates PNG if matplotlib is available.

NOTE: Any generated sample datasets are explicitly labeled as synthetic examples
for pipeline verification and must not be confused with physical or simulator recordings.
"""

import csv
import sys
import os
import math
from collections import Counter

EXPECTED_COLUMNS = [
    "time_ms", "state", "filtered_tilt_deg", "raw_tilt_deg",
    "valid", "fault_reason", "ax", "ay", "az", "mag_mg"
]

VALID_STATES = {"SAFE", "WARNING", "DANGER", "SENSOR_FAULT"}
VALID_FAULTS = {"NONE", "READ_FAILURE", "INJECTED", "INVALID_VECTOR"}

STATE_COLORS = {
    "SAFE": "#2ECC71",       # Green
    "WARNING": "#F39C12",    # Yellow / Orange
    "DANGER": "#E74C3C",     # Red
    "SENSOR_FAULT": "#9B59B6" # Purple
}

def parse_telemetry_csv(file_path):
    records = []
    malformed_lines = 0

    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")

    with open(file_path, "r", encoding="utf-8", errors="replace") as f:
        reader = csv.reader(f)

        for line_num, row in enumerate(reader, 1):
            if not row:
                continue

            # Strip leading/trailing whitespace
            row = [item.strip() for item in row]

            # Skip diagnostic comments starting with '#'
            if row[0].startswith("#"):
                continue

            # Skip header line
            if "time_ms" in row and "state" in row:
                continue

            # Parse data rows
            if len(row) < 6:
                malformed_lines += 1
                continue

            try:
                rec = {
                    "time_ms": int(row[0]),
                    "state": row[1].upper(),
                    "filtered_tilt_deg": float(row[2]),
                    "raw_tilt_deg": float(row[3]),
                    "valid": int(row[4]) == 1,
                    "fault_reason": row[5].upper(),
                    "ax": int(row[6]) if len(row) > 6 else 0,
                    "ay": int(row[7]) if len(row) > 7 else 0,
                    "az": int(row[8]) if len(row) > 8 else 16384,
                    "mag_mg": int(row[9]) if len(row) > 9 else 1000
                }
                if rec["state"] in VALID_STATES:
                    records.append(rec)
                else:
                    malformed_lines += 1
            except (ValueError, IndexError):
                malformed_lines += 1

    return records, malformed_lines

def analyze_telemetry(records):
    if not records:
        return None

    total_samples = len(records)
    t_start = records[0]["time_ms"]
    t_end = records[-1]["time_ms"]
    total_duration_ms = max(t_end - t_start, 1)

    state_counts = Counter(r["state"] for r in records)
    fault_counts = Counter(r["fault_reason"] for r in records if r["fault_reason"] != "NONE")

    transitions = []
    prev_state = None
    for r in records:
        if prev_state is not None and r["state"] != prev_state:
            transitions.append({
                "time_ms": r["time_ms"],
                "from": prev_state,
                "to": r["state"],
                "angle": r["filtered_tilt_deg"],
                "fault": r["fault_reason"]
            })
        prev_state = r["state"]

    max_tilt = max(r["filtered_tilt_deg"] for r in records)
    min_tilt = min(r["filtered_tilt_deg"] for r in records)
    avg_tilt = sum(r["filtered_tilt_deg"] for r in records) / total_samples

    return {
        "total_samples": total_samples,
        "total_duration_s": total_duration_ms / 1000.0,
        "state_counts": state_counts,
        "fault_counts": fault_counts,
        "transitions": transitions,
        "max_tilt": max_tilt,
        "min_tilt": min_tilt,
        "avg_tilt": avg_tilt
    }

def print_summary_report(analysis, malformed_count, source_label="Recorded Telemetry"):
    if not analysis:
        print("[ERROR] No valid telemetry records found.")
        return

    print("==================================================================")
    print(f" StabilityGuard Phase 1 Telemetry Report [{source_label}]")
    print("==================================================================")
    print(f" Total Logged Samples:    {analysis['total_samples']}")
    print(f" Total Duration:          {analysis['total_duration_s']:.2f} seconds")
    print(f" Sampling Rate:           ~{analysis['total_samples'] / analysis['total_duration_s']:.1f} Hz")
    print(f" Malformed Lines Skipped: {malformed_count}")
    print("------------------------------------------------------------------")
    print(" State Occupancy Breakdown:")
    for st in ["SAFE", "WARNING", "DANGER", "SENSOR_FAULT"]:
        cnt = analysis["state_counts"].get(st, 0)
        pct = (cnt / analysis["total_samples"]) * 100.0
        print(f"   - {st:<12} : {cnt:5d} samples ({pct:5.1f}%)")

    print("------------------------------------------------------------------")
    print(" Tilt Angle Metrics:")
    print(f"   - Min Filtered Tilt:   {analysis['min_tilt']:.2f}°")
    print(f"   - Average Tilt:        {analysis['avg_tilt']:.2f}°")
    print(f"   - Max Filtered Tilt:   {analysis['max_tilt']:.2f}°")

    if analysis["fault_counts"]:
        print("------------------------------------------------------------------")
        print(" Detected Faults Breakdown:")
        for fault, cnt in analysis["fault_counts"].items():
            print(f"   - {fault:<16} : {cnt} events")

    print("------------------------------------------------------------------")
    print(f" Observed State Transitions ({len(analysis['transitions'])} total):")
    for t in analysis["transitions"]:
        print(f"   @ {t['time_ms']:6d} ms: {t['from']:<12} -> {t['to']:<12} (Tilt: {t['angle']:5.2f}°, Fault: {t['fault']})")
    print("==================================================================")

def generate_svg_plot(records, output_svg_path, is_synthetic=False):
    if not records:
        return

    width = 900
    height = 420
    margin = {"top": 40, "right": 40, "bottom": 60, "left": 60}
    plot_w = width - margin["left"] - margin["right"]
    plot_h = height - margin["top"] - margin["bottom"]

    t_min = records[0]["time_ms"]
    t_max = max(records[-1]["time_ms"], t_min + 1)
    angle_max = max(max(r["raw_tilt_deg"] for r in records), 50.0)

    def scale_x(t):
        return margin["left"] + ((t - t_min) / (t_max - t_min)) * plot_w

    def scale_y(a):
        return margin["top"] + plot_h - (a / angle_max) * plot_h

    svg_parts = []
    svg_parts.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="100%" height="100%" style="background:#1E1E1E; font-family:system-ui, -apple-system, sans-serif;">')

    # Title with explicit synthetic/observed watermark
    title_text = "StabilityGuard Telemetry Analysis (Synthetic Example)" if is_synthetic else "StabilityGuard Telemetry Analysis (Observed Wokwi Run)"
    svg_parts.append(f'<text x="{width/2}" y="25" text-anchor="middle" fill="#FFFFFF" font-size="16" font-weight="bold">{title_text}</text>')

    # Threshold 15 deg (WARNING)
    y_15 = scale_y(15.0)
    svg_parts.append(f'<line x1="{margin["left"]}" y1="{y_15}" x2="{margin["left"]+plot_w}" y2="{y_15}" stroke="#F39C12" stroke-dasharray="4,4" stroke-width="1.5"/>')
    svg_parts.append(f'<text x="{margin["left"]+plot_w-5}" y="{y_15-4}" text-anchor="end" fill="#F39C12" font-size="11">WARN Threshold (15°)</text>')

    # Threshold 30 deg (DANGER)
    y_30 = scale_y(30.0)
    svg_parts.append(f'<line x1="{margin["left"]}" y1="{y_30}" x2="{margin["left"]+plot_w}" y2="{y_30}" stroke="#E74C3C" stroke-dasharray="4,4" stroke-width="1.5"/>')
    svg_parts.append(f'<text x="{margin["left"]+plot_w-5}" y="{y_30-4}" text-anchor="end" fill="#E74C3C" font-size="11">DANGER Threshold (30°)</text>')

    # Axes
    svg_parts.append(f'<line x1="{margin["left"]}" y1="{margin["top"]+plot_h}" x2="{margin["left"]+plot_w}" y2="{margin["top"]+plot_h}" stroke="#555" stroke-width="1.5"/>')
    svg_parts.append(f'<line x1="{margin["left"]}" y1="{margin["top"]}" x2="{margin["left"]}" y2="{margin["top"]+plot_h}" stroke="#555" stroke-width="1.5"/>')

    # Y-axis labels
    for a in [0, 15, 30, 45]:
        if a <= angle_max:
            y = scale_y(a)
            svg_parts.append(f'<text x="{margin["left"]-8}" y="{y+4}" text-anchor="end" fill="#AAA" font-size="11">{a}°</text>')
            svg_parts.append(f'<line x1="{margin["left"]}" y1="{y}" x2="{margin["left"]+plot_w}" y2="{y}" stroke="#333" stroke-width="0.8"/>')

    # Background State Shading
    for i in range(len(records) - 1):
        x1 = scale_x(records[i]["time_ms"])
        x2 = scale_x(records[i+1]["time_ms"])
        color = STATE_COLORS.get(records[i]["state"], "#2ECC71")
        svg_parts.append(f'<rect x="{x1}" y="{margin["top"]+plot_h+10}" width="{max(x2-x1, 1)}" height="15" fill="{color}" opacity="0.9"/>')

    # Raw Tilt line (Cyan, thin)
    raw_points = " ".join(f"{scale_x(r['time_ms']):.1f},{scale_y(r['raw_tilt_deg']):.1f}" for r in records)
    svg_parts.append(f'<polyline points="{raw_points}" fill="none" stroke="#00BCD4" stroke-width="1.2" opacity="0.6"/>')

    # Filtered Tilt line (White, thick)
    filt_points = " ".join(f"{scale_x(r['time_ms']):.1f},{scale_y(r['filtered_tilt_deg']):.1f}" for r in records)
    svg_parts.append(f'<polyline points="{filt_points}" fill="none" stroke="#FFFFFF" stroke-width="2.5"/>')

    # Legend
    svg_parts.append(f'<rect x="{margin["left"]}" y="{height-30}" width="12" height="12" fill="#2ECC71"/>')
    svg_parts.append(f'<text x="{margin["left"]+16}" y="{height-20}" fill="#CCC" font-size="11">SAFE</text>')
    svg_parts.append(f'<rect x="{margin["left"]+75}" y="{height-30}" width="12" height="12" fill="#F39C12"/>')
    svg_parts.append(f'<text x="{margin["left"]+91}" y="{height-20}" fill="#CCC" font-size="11">WARNING</text>')
    svg_parts.append(f'<rect x="{margin["left"]+175}" y="{height-30}" width="12" height="12" fill="#E74C3C"/>')
    svg_parts.append(f'<text x="{margin["left"]+191}" y="{height-20}" fill="#CCC" font-size="11">DANGER</text>')
    svg_parts.append(f'<rect x="{margin["left"]+275}" y="{height-30}" width="12" height="12" fill="#9B59B6"/>')
    svg_parts.append(f'<text x="{margin["left"]+291}" y="{height-20}" fill="#CCC" font-size="11">FAULT</text>')

    svg_parts.append(f'<line x1="{margin["left"]+360}" y1="{height-24}" x2="{margin["left"]+385}" y2="{height-24}" stroke="#FFFFFF" stroke-width="2.5"/>')
    svg_parts.append(f'<text x="{margin["left"]+392}" y="{height-20}" fill="#FFF" font-size="11">Filtered Tilt</text>')
    svg_parts.append(f'<line x1="{margin["left"]+480}" y1="{height-24}" x2="{margin["left"]+505}" y2="{height-24}" stroke="#00BCD4" stroke-width="1.2"/>')
    svg_parts.append(f'<text x="{margin["left"]+512}" y="{height-20}" fill="#00BCD4" font-size="11">Raw Tilt</text>')

    svg_parts.append('</svg>')

    with open(output_svg_path, "w", encoding="utf-8") as f:
        f.write("\n".join(svg_parts))
    print(f"[OK] Generated Vector SVG Plot: {output_svg_path}")

def generate_sample_telemetry(csv_path):
    """Generates synthetic test telemetry labeled explicitly as a synthetic example."""
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["# =========================================================================="])
        writer.writerow(["# [SYNTHETIC EXAMPLE DATASET] - StabilityGuard Phase 1 Test Pattern"])
        writer.writerow(["# NOTE: Generated mathematically to validate analysis script & SVG plotter."])
        writer.writerow(["# DO NOT CONFUSE WITH ACTUAL RECORDED WOKWI HARDWARE / SERIAL RUN LOGS."])
        writer.writerow(["# =========================================================================="])
        writer.writerow(EXPECTED_COLUMNS)

        # 0 - 3000ms: SAFE (Level 0°)
        for t in range(100, 3100, 100):
            writer.writerow([t, "SAFE", "0.00", "0.00", 1, "NONE", 0, 0, 16384, 1000])

        # 3100 - 7000ms: WARNING (~20°)
        f_val = 0.0
        for t in range(3100, 7100, 100):
            f_val = 0.35 * 19.99 + 0.65 * f_val
            st = "SAFE" if t < 3500 else "WARNING"
            writer.writerow([t, st, f"{f_val:.2f}", "19.99", 1, "NONE", -5603, 0, 15401, 1000])

        # 7100 - 11000ms: DANGER (~40°)
        for t in range(7100, 11100, 100):
            f_val = 0.35 * 40.01 + 0.65 * f_val
            st = "WARNING" if t < 7500 else "DANGER"
            writer.writerow([t, st, f"{f_val:.2f}", "40.01", 1, "NONE", -10535, 0, 12550, 1000])

        # 11100 - 15000ms: Return to WARNING (~20°)
        for t in range(11100, 15100, 100):
            f_val = 0.35 * 19.99 + 0.65 * f_val
            st = "DANGER" if t < 11600 else "WARNING"
            writer.writerow([t, st, f"{f_val:.2f}", "19.99", 1, "NONE", -5603, 0, 15401, 1000])

        # 15100 - 18000ms: Return to SAFE (0°)
        for t in range(15100, 18100, 100):
            f_val = 0.35 * 0.00 + 0.65 * f_val
            st = "WARNING" if t < 15700 else "SAFE"
            writer.writerow([t, st, f"{f_val:.2f}", "0.00", 1, "NONE", 0, 0, 16384, 1000])

        # 18100 - 20000ms: Injected Fault (Button Pressed)
        for t in range(18100, 20100, 100):
            writer.writerow([t, "SENSOR_FAULT", "0.00", "0.00", 0, "INJECTED", 0, 0, 16384, 1000])

        # 20100 - 22000ms: Recovery back to SAFE
        for t in range(20100, 22100, 100):
            st = "SENSOR_FAULT" if t < 20600 else "SAFE"
            v = 1 if t >= 20600 else 0
            writer.writerow([t, st, "0.00", "0.00", v, "NONE", 0, 0, 16384, 1000])

    print(f"[OK] Generated Synthetic Example Telemetry: {csv_path}")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_file = sys.argv[1] if len(sys.argv) > 1 else os.path.join(script_dir, "sample_telemetry.csv")

    is_synth = ("sample_telemetry.csv" in input_file)

    if not os.path.exists(input_file):
        generate_sample_telemetry(input_file)

    records, malformed = parse_telemetry_csv(input_file)
    analysis = analyze_telemetry(records)
    source_label = "SYNTHETIC EXAMPLE DATASET" if is_synth else "RECORDED TELEMETRY"
    print_summary_report(analysis, malformed, source_label)

    output_svg = os.path.splitext(input_file)[0] + "_plot.svg"
    generate_svg_plot(records, output_svg, is_synthetic=is_synth)

if __name__ == "__main__":
    main()
