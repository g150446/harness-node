#!/usr/bin/env python3
"""Analyze labelled HarnessNode six-axis collection trials."""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import statistics
import tempfile
from pathlib import Path
from typing import Any

from imu_trajectory import load_trajectory_csv


G = 9.80665
BASELINE_START_MS = 200
BASELINE_END_MS = 900
FINAL_WINDOW_MS = 400


def mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else math.nan


def rms(values: list[float]) -> float:
    return math.sqrt(mean([value * value for value in values]))


def moving_average(values: list[float], width: int = 3) -> list[float]:
    result: list[float] = []
    for index in range(len(values)):
        start = max(0, index - width + 1)
        result.append(mean(values[start : index + 1]))
    return result


def vector_angle_deg(a: tuple[float, float, float],
                     b: tuple[float, float, float]) -> float:
    an = math.sqrt(sum(value * value for value in a))
    bn = math.sqrt(sum(value * value for value in b))
    if an <= 0.01 or bn <= 0.01:
        return math.nan
    cosine = sum(x * y for x, y in zip(a, b)) / (an * bn)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def extract_features(samples: list[dict[str, Any]]) -> dict[str, float]:
    if not samples:
        raise ValueError("empty trajectory")
    times = [int(sample["elapsed_ms"]) for sample in samples]
    ax = [float(sample["ax_ms2"]) for sample in samples]
    ay = [float(sample["ay_ms2"]) for sample in samples]
    az = [float(sample["az_ms2"]) for sample in samples]
    baseline_indices = [
        index for index, elapsed in enumerate(times)
        if BASELINE_START_MS <= elapsed <= BASELINE_END_MS
    ]
    if len(baseline_indices) < 4:
        baseline_indices = list(range(min(len(samples), 20)))
    final_start = times[-1] - FINAL_WINDOW_MS
    final_indices = [
        index for index, elapsed in enumerate(times) if elapsed >= final_start
    ]
    baseline = tuple(
        mean([axis[index] for index in baseline_indices])
        for axis in (ax, ay, az)
    )
    final = tuple(
        mean([axis[index] for index in final_indices])
        for axis in (ax, ay, az)
    )
    ax_smooth = moving_average(ax)
    ay_smooth = moving_average(ay)
    xy_relative = [
        math.hypot(x - baseline[0], y - baseline[1])
        for x, y in zip(ax_smooth, ay_smooth)
    ]
    xy_peak = max(xy_relative)
    xy_peak_index = xy_relative.index(xy_peak)
    xy_final = mean([xy_relative[index] for index in final_indices])
    norms = [math.sqrt(x * x + y * y + z * z) for x, y, z in zip(ax, ay, az)]
    baseline_norm = mean([norms[index] for index in baseline_indices])
    norm_residual = [
        norms[index] - baseline_norm for index in baseline_indices
    ]
    gyro_rows = [sample for sample in samples if sample.get("gyro_read_valid")]
    gyro_peak = {}
    for axis in ("gx_dps", "gy_dps", "gz_dps"):
        values = [abs(float(sample[axis])) for sample in gyro_rows]
        gyro_peak[axis] = max(values, default=math.nan)
    gyro_y_integral = 0.0
    previous_time: int | None = None
    for sample in gyro_rows:
        elapsed = int(sample["elapsed_ms"])
        gy = float(sample["gy_dps"])
        if previous_time is not None and abs(gy) >= 20.0:
            gyro_y_integral += gy * (elapsed - previous_time) / 1000.0
        previous_time = elapsed
    final_norm = math.sqrt(sum(value * value for value in final))
    return {
        "duration_ms": float(times[-1]),
        "sample_count": float(len(samples)),
        "baseline_ax_ms2": baseline[0],
        "baseline_ay_ms2": baseline[1],
        "baseline_az_ms2": baseline[2],
        "baseline_norm_ms2": baseline_norm,
        "baseline_z_ratio": baseline[2] / baseline_norm,
        "baseline_norm_rms_ms2": rms(norm_residual),
        "xy_peak_ms2": xy_peak,
        "xy_peak_ms": float(times[xy_peak_index]),
        "xy_final_ms2": xy_final,
        "xy_drop_ms2": xy_peak - xy_final,
        "xy_drop_ratio": (xy_peak - xy_final) / xy_peak if xy_peak else 0.0,
        "accel_x_min_ms2": min(ax),
        "accel_x_max_ms2": max(ax),
        "accel_y_min_ms2": min(ay),
        "accel_y_max_ms2": max(ay),
        "accel_z_min_ms2": min(az),
        "accel_z_max_ms2": max(az),
        "gyro_x_peak_dps": gyro_peak["gx_dps"],
        "gyro_y_peak_dps": gyro_peak["gy_dps"],
        "gyro_z_peak_dps": gyro_peak["gz_dps"],
        "gyro_xy_peak_ratio": (
            gyro_peak["gx_dps"] / gyro_peak["gy_dps"]
            if gyro_peak["gy_dps"] > 0.0 else math.nan
        ),
        "gyro_y_integral_deg": gyro_y_integral,
        "gyro_y_abs_integral_deg": abs(gyro_y_integral),
        "final_z_ratio": final[2] / final_norm,
        "final_gravity_angle_deg": vector_angle_deg(baseline, final),
    }


def transformed_samples(samples: list[dict[str, Any]], dx: float, dy: float,
                        noise_rms: float, seed: int) -> list[dict[str, Any]]:
    rng = random.Random(seed)
    noise_x = moving_average([rng.gauss(0.0, 1.0) for _ in samples], 5)
    noise_y = moving_average([rng.gauss(0.0, 1.0) for _ in samples], 5)
    scale_x = noise_rms / rms(noise_x) if noise_rms else 0.0
    scale_y = noise_rms / rms(noise_y) if noise_rms else 0.0
    result: list[dict[str, Any]] = []
    for index, sample in enumerate(samples):
        copy = dict(sample)
        copy["ax_ms2"] = float(sample["ax_ms2"]) + dx + noise_x[index] * scale_x
        copy["ay_ms2"] = float(sample["ay_ms2"]) + dy + noise_y[index] * scale_y
        result.append(copy)
    return result


def load_trials(session_dir: Path) -> list[dict[str, Any]]:
    trials: list[dict[str, Any]] = []
    for report_path in sorted(session_dir.glob("trial_*.json")):
        report = json.loads(report_path.read_text(encoding="utf-8"))
        if not report.get("usable", True):
            print(
                f"解析対象外: {report_path.name} "
                f"({report.get('exclusion_reason', '理由未記載')})"
            )
            continue
        csv_path = Path(report["csv"])
        if not csv_path.is_absolute():
            csv_path = session_dir / csv_path
        trajectory = load_trajectory_csv(csv_path)
        samples = trajectory["samples"]
        trials.append(
            {
                "report": report,
                "report_path": report_path,
                "samples": samples,
                "features": extract_features(samples),
            }
        )
    if not trials:
        raise ValueError(f"trial JSONがありません: {session_dir}")
    return trials


def write_feature_csv(path: Path, trials: list[dict[str, Any]]) -> None:
    metadata = ["trial", "hand", "motion", "speed", "classifier_matched"]
    feature_names = list(trials[0]["features"])
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=metadata + feature_names)
        writer.writeheader()
        for trial in trials:
            report = trial["report"]
            row = {name: report.get(name) for name in metadata}
            row.update(trial["features"])
            writer.writerow(row)


def write_overlay(path: Path, trials: list[dict[str, Any]]) -> None:
    import matplotlib.pyplot as plt

    fig, (xy_axis, gyro_axis) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    colors = {"right": "#d62728", "left": "#1f77b4"}
    styles = {"positive": "-", "lift_only": "--", "flip_only": ":", "daily": "-."}
    for trial in trials:
        report = trial["report"]
        samples = trial["samples"]
        features = trial["features"]
        times = [sample["elapsed_ms"] / 1000.0 for sample in samples]
        ax = moving_average([float(sample["ax_ms2"]) for sample in samples])
        ay = moving_average([float(sample["ay_ms2"]) for sample in samples])
        xy = [
            math.hypot(x - features["baseline_ax_ms2"],
                       y - features["baseline_ay_ms2"])
            for x, y in zip(ax, ay)
        ]
        gy = [
            abs(float(sample["gy_dps"]))
            if sample.get("gyro_read_valid") else math.nan
            for sample in samples
        ]
        label = f"{report['hand']} {report['motion']} {report['speed']}"
        kwargs = {
            "color": colors[report["hand"]],
            "linestyle": styles[report["motion"]],
            "linewidth": 1.1,
            "alpha": 0.75,
            "label": label,
        }
        xy_axis.plot(times, xy, **kwargs)
        gyro_axis.plot(times, gy, **kwargs)
    xy_axis.set_ylabel("XY relative acceleration (m/s²)")
    gyro_axis.set_ylabel("|Gyro Y| (dps)")
    gyro_axis.set_xlabel("Capture time (s)")
    for axis in (xy_axis, gyro_axis):
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=7, ncols=2)
    fig.suptitle("HarnessNode labelled trial overlay")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def simulation_rows(trials: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for trial_index, trial in enumerate(trials):
        report = trial["report"]
        baseline = trial["features"]
        for magnitude_g in (0.1, 0.2, 0.3, 0.4, 0.5):
            for angle_deg in (0, 90, 180, 270):
                angle = math.radians(angle_deg)
                for noise in (0.0, 0.5, 1.0, 2.0):
                    samples = transformed_samples(
                        trial["samples"],
                        magnitude_g * G * math.cos(angle),
                        magnitude_g * G * math.sin(angle),
                        noise,
                        20260824 + trial_index * 1000 + angle_deg + int(noise * 10),
                    )
                    features = extract_features(samples)
                    rows.append(
                        {
                            "trial": report["trial"],
                            "hand": report["hand"],
                            "motion": report["motion"],
                            "magnitude_g": magnitude_g,
                            "angle_deg": angle_deg,
                            "noise_rms_ms2": noise,
                            "z_ratio_delta": features["baseline_z_ratio"] - baseline["baseline_z_ratio"],
                            "xy_peak_delta_ms2": features["xy_peak_ms2"] - baseline["xy_peak_ms2"],
                            "xy_drop_ratio_delta": features["xy_drop_ratio"] - baseline["xy_drop_ratio"],
                            "final_angle_delta_deg": features["final_gravity_angle_deg"] - baseline["final_gravity_angle_deg"],
                        }
                    )
    return rows


def write_dict_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def adaptive_flags(trials: list[dict[str, Any]]) -> list[str]:
    flags: list[str] = []
    positive = [
        trial for trial in trials
        if trial["report"]["motion"] == "positive"
        and trial["report"].get("speed") != "slow"
    ]
    negative = [trial for trial in trials if trial not in positive]
    watched = ("xy_peak_ms2", "xy_drop_ratio", "gyro_y_peak_dps",
               "gyro_xy_peak_ratio",
               "gyro_y_abs_integral_deg", "final_gravity_angle_deg")
    for hand in ("right", "left"):
        hand_trials = [trial for trial in positive if trial["report"]["hand"] == hand]
        for feature in watched:
            values = [trial["features"][feature] for trial in hand_trials]
            finite = [value for value in values if math.isfinite(value)]
            if len(finite) >= 2 and min(finite) > 0 and max(finite) / min(finite) > 1.30:
                flags.append(f"{hand}: {feature} の正例ばらつきが30%超")
    for feature in watched:
        positives = [trial["features"][feature] for trial in positive]
        negatives = [trial["features"][feature] for trial in negative]
        positives = [value for value in positives if math.isfinite(value)]
        negatives = [value for value in negatives if math.isfinite(value)]
        if positives and negatives:
            margin = (min(positives) - max(negatives)) / max(min(positives), 1e-6)
            if margin < 0.25:
                flags.append(f"{feature}: 正例と負例の分離が25%未満")
    return flags


def evaluate_motion_shape_gate(trials: list[dict[str, Any]]) -> dict[str, Any]:
    """Evaluate the non-duration 0.0.59 motion-shape gates."""
    rows: list[dict[str, Any]] = []
    tp = tn = fp = fn = 0
    for trial in trials:
        report = trial["report"]
        features = trial["features"]
        expected = report["motion"] == "positive"
        predicted = (
            features["gyro_y_abs_integral_deg"] >= 80.0
            and features["gyro_xy_peak_ratio"] >= 0.42
            and features["final_gravity_angle_deg"] >= 20.0
        )
        if expected and predicted:
            tp += 1
        elif expected:
            fn += 1
        elif predicted:
            fp += 1
        else:
            tn += 1
        rows.append({
            "trial": report["trial"],
            "expected": expected,
            "predicted": predicted,
        })
    return {
        "thresholds": {
            "gyro_y_abs_integral_min_deg": 80.0,
            "gyro_xy_peak_ratio_min": 0.42,
            "final_gravity_angle_min_deg": 20.0,
            "motion_complete_max_ms": 3000.0,
            "duration_available_in_host_capture": False,
        },
        "confusion": {"tp": tp, "tn": tn, "fp": fp, "fn": fn},
        "sensitivity": tp / (tp + fn) if tp + fn else math.nan,
        "specificity": tn / (tn + fp) if tn + fp else math.nan,
        "trials": rows,
    }


def self_test() -> None:
    samples: list[dict[str, Any]] = []
    for index in range(80):
        elapsed_ms = index * 25
        progress = max(0.0, 1.0 - abs(index - 38) / 22.0)
        samples.append(
            {
                "elapsed_ms": elapsed_ms,
                "gyro_read_valid": True,
                "ax_ms2": 0.4 + 3.0 * progress,
                "ay_ms2": -0.3 + 2.0 * progress,
                "az_ms2": 9.7 - 1.2 * progress,
                "gx_dps": 45.0 * progress,
                "gy_dps": 80.0 * progress,
                "gz_dps": 5.0 * progress,
            }
        )
    mirrored = []
    for sample in samples:
        copy = dict(sample)
        copy["ax_ms2"] = -float(sample["ax_ms2"])
        copy["ay_ms2"] = -float(sample["ay_ms2"])
        copy["gx_dps"] = -float(sample["gx_dps"])
        copy["gy_dps"] = -float(sample["gy_dps"])
        mirrored.append(copy)
    original_features = extract_features(samples)
    mirrored_features = extract_features(mirrored)
    for name in (
        "xy_peak_ms2",
        "xy_drop_ms2",
        "xy_drop_ratio",
        "gyro_x_peak_dps",
        "gyro_y_peak_dps",
        "gyro_y_abs_integral_deg",
    ):
        assert math.isclose(
            original_features[name], mirrored_features[name], rel_tol=1e-9
        ), name
    offset_features = extract_features(
        transformed_samples(samples, 0.4 * G, -0.3 * G, 0.0, 1)
    )
    assert math.isclose(
        original_features["xy_peak_ms2"],
        offset_features["xy_peak_ms2"],
        rel_tol=1e-9,
    )
    assert math.isclose(
        original_features["xy_drop_ratio"],
        offset_features["xy_drop_ratio"],
        rel_tol=1e-9,
    )
    trials = [
        {
            "report": {
                "trial": 1,
                "hand": "right",
                "motion": "positive",
                "speed": "natural",
                "classifier_matched": True,
            },
            "samples": samples,
            "features": original_features,
        },
        {
            "report": {
                "trial": 2,
                "hand": "left",
                "motion": "lift_only",
                "speed": "na",
                "classifier_matched": False,
            },
            "samples": mirrored,
            "features": mirrored_features,
        },
    ]
    with tempfile.TemporaryDirectory(prefix="gesture-analyzer-test-") as temp:
        output = Path(temp)
        write_feature_csv(output / "features.csv", trials)
        write_overlay(output / "overlay.png", trials)
        transformed = simulation_rows(trials)
        assert len(transformed) == 160
        write_dict_csv(output / "simulation.csv", transformed)
        for name in ("features.csv", "overlay.png", "simulation.csv"):
            assert (output / name).stat().st_size > 0


def main() -> int:
    parser = argparse.ArgumentParser(description="収集済み6軸データを比較分析")
    parser.add_argument("session_dir", type=Path, nargs="?")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        print("SELF_TEST: PASS")
        return 0
    if args.session_dir is None:
        parser.error("session_dirを指定してください")
    session_dir = args.session_dir.expanduser().resolve()
    trials = load_trials(session_dir)
    feature_path = session_dir / "feature_summary.csv"
    overlay_path = session_dir / "feature_overlay.png"
    simulation_path = session_dir / "simulated_transport_sensitivity.csv"
    analysis_path = session_dir / "analysis.json"
    write_feature_csv(feature_path, trials)
    write_overlay(overlay_path, trials)
    simulations = simulation_rows(trials)
    write_dict_csv(simulation_path, simulations)
    flags = adaptive_flags(trials)
    motion_shape_gate = evaluate_motion_shape_gate(trials)
    analysis_path.write_text(
        json.dumps(
            {
                "trial_count": len(trials),
                "positive_count": sum(t["report"]["motion"] == "positive" for t in trials),
                "negative_count": sum(t["report"]["motion"] != "positive" for t in trials),
                "needs_additional_sampling": bool(flags),
                "adaptive_flags": flags,
                "motion_shape_gate": motion_shape_gate,
                "threshold_policy": (
                    "正例最小と負例最大に25%以上の分離がある特徴だけを"
                    "必須条件候補にする"
                ),
            },
            ensure_ascii=False,
            indent=2,
        ) + "\n",
        encoding="utf-8",
    )
    print(f"分析: {len(trials)} trials")
    print(f"特徴量: {feature_path}")
    print(f"重ね合わせ: {overlay_path}")
    print(f"模擬横加速度: {simulation_path}")
    print(f"判定: {analysis_path}")
    if flags:
        print("追加収集候補:")
        for flag in flags:
            print(f"  - {flag}")
    else:
        print("追加収集フラグなし")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
