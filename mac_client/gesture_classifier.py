#!/usr/bin/env python3
"""
gesture_classifier.py — 2-feature rule-based gesture classifier validation

Reads gesture_data.csv (produced by nrf52_voice_client.py gesture collector),
computes per-gesture features, and reports classification accuracy.

Features:
  settled_z        — mean z-acceleration over the entire motion window
  raised_peak      — max |acc| while z >= Z_RAISED threshold

Classification rule (AND):
  settled_z   >= Z_RAISED_THRESH   → arm still raised at settle
  raised_peak >= SPIKE_THRESH      → clench spike while arm was raised

Usage:
  python3 gesture_classifier.py gesture_data.csv
  python3 gesture_classifier.py gesture_data.csv --z-raised 7.5 --spike 11.0
"""

import argparse
import csv
import math
import sys
from collections import defaultdict


def parse_args():
    p = argparse.ArgumentParser(description="2-feature gesture classifier")
    p.add_argument("csv_file", help="Path to gesture_data.csv")
    p.add_argument("--z-raised", type=float, default=7.5,
                   help="z threshold to consider arm raised (m/s², default 7.5)")
    p.add_argument("--spike", type=float, default=11.0,
                   help="min |acc| peak while raised to count as clench (m/s², default 11.0)")
    return p.parse_args()


def load_gestures(csv_file):
    """Return list of (label, samples) where samples = list of (elapsed_ms, x, y, z)."""
    gestures = []
    current_label = None
    current_samples = []

    with open(csv_file, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            label = row["label"].strip()
            elapsed = float(row["elapsed_ms"])
            x = float(row["x"])
            y = float(row["y"])
            z = float(row["z"])

            if current_label is None:
                current_label = label
            elif label != current_label:
                gestures.append((current_label, current_samples))
                current_label = label
                current_samples = []

            current_samples.append((elapsed, x, y, z))

    if current_label is not None and current_samples:
        gestures.append((current_label, current_samples))

    return gestures


def compute_features(samples, z_raised_thresh):
    """Compute (settled_z, raised_peak) for a gesture's sample list."""
    if not samples:
        return 0.0, 0.0

    z_sum = 0.0
    raised_peak = 0.0

    for (_elapsed, x, y, z) in samples:
        z_sum += z
        acc_mag = math.sqrt(x*x + y*y + z*z)
        if z >= z_raised_thresh and acc_mag > raised_peak:
            raised_peak = acc_mag

    settled_z = z_sum / len(samples)
    return settled_z, raised_peak


def classify(settled_z, raised_peak, z_raised_thresh, spike_thresh):
    return settled_z >= z_raised_thresh and raised_peak >= spike_thresh


def main():
    args = parse_args()
    z_thresh = args.z_raised
    spike_thresh = args.spike

    try:
        gestures = load_gestures(args.csv_file)
    except FileNotFoundError:
        print(f"Error: file not found: {args.csv_file}", file=sys.stderr)
        sys.exit(1)
    except KeyError as e:
        print(f"Error: missing column {e} in CSV", file=sys.stderr)
        sys.exit(1)

    if not gestures:
        print("No gestures found in CSV.")
        sys.exit(0)

    print(f"Thresholds: z_raised >= {z_thresh:.1f} m/s²,  spike >= {spike_thresh:.1f} m/s²")
    print()
    print(f"{'#':<4} {'label':<20} {'samples':>7} {'settled_z':>10} {'raised_peak':>12} {'predict':<15} {'correct?'}")
    print("-" * 80)

    total = 0
    correct = 0
    by_label = defaultdict(lambda: {"total": 0, "correct": 0})

    for i, (label, samples) in enumerate(gestures):
        settled_z, raised_peak = compute_features(samples, z_thresh)
        predicted = "recording_start" if classify(settled_z, raised_peak, z_thresh, spike_thresh) else "other_motion"
        is_correct = predicted == label
        marker = "✓" if is_correct else "✗"

        print(f"{i:<4} {label:<20} {len(samples):>7} {settled_z:>10.2f} {raised_peak:>12.2f} {predicted:<15} {marker}")

        total += 1
        if is_correct:
            correct += 1
        by_label[label]["total"] += 1
        if is_correct:
            by_label[label]["correct"] += 1

    print("-" * 80)
    print(f"\nOverall accuracy: {correct}/{total} ({100*correct/total:.1f}%)")
    print()
    for lbl, counts in sorted(by_label.items()):
        n, c = counts["total"], counts["correct"]
        print(f"  {lbl:<20}: {c}/{n} ({100*c/n:.1f}%)")


if __name__ == "__main__":
    main()
