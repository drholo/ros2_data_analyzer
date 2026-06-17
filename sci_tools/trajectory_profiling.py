#!/usr/bin/env python3
"""
Generate velocity, angular velocity, trajectory overlap, and spatial deviation
profiles from SLAM TF.json results.

Expected input layout:

    <input-root>/<platform>/results_<measurement><environment>/record_<algorithm>_<amcl/no_amcl>_<iteration>/TF.json

Examples:

    panther/results_1A/record_cartographer_amcl_0/TF.json
    spot/results_2C/record_slam_toolbox_no_amcl_9/TF.json
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import os
import re
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

RESULTS_RE = re.compile(r"^results_(?P<measurement_no>\d+)(?P<environment>[A-Za-z]+)$")
RECORD_RE = re.compile(r"^record_(?P<name>.+)_(?P<iteration>\d+)$")
MODE_COLORS = {"amcl": "#d97706", "no_amcl": "#2563eb"}
PLATFORM_COLORS = {"panther": "#1f77b4", "spot": "#d62728"}
ALGORITHM_COLORS = {"cartographer": "#d62728", "gmapping": "#2ca02c", "slam_toolbox": "#1f77b4"}
PLATFORM_MEASUREMENT_COLORS = {
    ("panther", "1"): "#1f77b4",
    ("panther", "2"): "#17becf",
    ("spot", "1"): "#d62728",
    ("spot", "2"): "#ff7f0e",
}
MEASUREMENT_STYLES = {"1": "-", "2": "--"}

_SHOW_PLOTS: bool = False


@dataclass
class TfRun:
    path: Path
    platform: str
    environment: str
    measurement_no: str
    algorithm: str
    mode: str
    iteration: str
    time_s: np.ndarray
    xy: np.ndarray
    yaw_rad: np.ndarray
    speed_mps: np.ndarray
    angular_velocity_rps: np.ndarray
    normalized_time: np.ndarray
    resampled_xy: np.ndarray
    resampled_speed_mps: np.ndarray
    resampled_angular_velocity_rps: np.ndarray
    path_length_m: float

    @property
    def label(self) -> str:
        return f"{self.platform} {self.measurement_no}{self.environment} {self.algorithm} {self.mode} #{self.iteration}"

    @property
    def group_key(self) -> tuple[str, str, str, str, str]:
        return (self.platform, self.environment, self.measurement_no, self.algorithm, self.mode)

    @property
    def cross_platform_key(self) -> tuple[str, str, str]:
        return (self.environment, self.algorithm, self.mode)

    @property
    def duration_s(self) -> float:
        return float(self.time_s[-1] - self.time_s[0])


def sanitize_filename(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_") or "plot"


def parse_results_dir(name: str) -> tuple[str, str]:
    match = RESULTS_RE.match(name)
    if not match:
        raise ValueError(f"expected results_<measurement><environment>, got {name}")
    return match.group("environment").upper(), match.group("measurement_no")


def parse_record_dir(name: str) -> tuple[str, str, str]:
    match = RECORD_RE.match(name)
    if not match:
        raise ValueError(f"expected record_<algorithm>_<amcl/no_amcl>_<iteration>, got {name}")

    parts = match.group("name").split("_")
    iteration = match.group("iteration")
    if len(parts) >= 3 and parts[-2:] == ["no", "amcl"]:
        return "_".join(parts[:-2]), "no_amcl", iteration
    if parts[-1] == "amcl":
        return "_".join(parts[:-1]), "amcl", iteration
    raise ValueError(f"cannot parse AMCL mode from {name}")


def infer_metadata(input_root: Path, tf_path: Path) -> tuple[str, str, str, str, str, str]:
    rel = tf_path.relative_to(input_root)
    if len(rel.parts) < 4:
        raise ValueError(f"{tf_path}: expected platform/results/record/TF.json")
    platform = rel.parts[0].lower()
    environment, measurement_no = parse_results_dir(rel.parts[1])
    algorithm, mode, iteration = parse_record_dir(rel.parts[2])
    return platform, environment, measurement_no, algorithm, mode, iteration


def xy_from_row(row: dict) -> tuple[float, float]:
    pos = row.get("position")
    if not isinstance(pos, dict):
        raise ValueError("missing position")
    return float(pos["x"]), float(pos["y"])


def yaw_from_row(row: dict) -> float:
    orientation = row.get("orientation")
    if not isinstance(orientation, dict):
        raise ValueError("missing orientation")
    x = float(orientation.get("x", 0.0))
    y = float(orientation.get("y", 0.0))
    z = float(orientation.get("z", 0.0))
    w = float(orientation.get("w", 1.0))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def moving_average(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1 or len(values) < 3:
        return values
    window = min(window, len(values))
    if window % 2 == 0:
        window -= 1
    if window <= 1:
        return values
    kernel = np.ones(window, dtype=float) / window
    if values.ndim == 1:
        padded = np.pad(values, (window // 2, window // 2), mode="edge")
        return np.convolve(padded, kernel, mode="valid")
    return np.column_stack([moving_average(values[:, idx], window) for idx in range(values.shape[1])])


def deduplicate_samples(time_s: np.ndarray, xy: np.ndarray, yaw: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    order = np.argsort(time_s)
    time_s = time_s[order]
    xy = xy[order]
    yaw = yaw[order]
    _, unique_idx = np.unique(time_s, return_index=True)
    unique_idx = np.sort(unique_idx)
    time_s = time_s[unique_idx]
    xy = xy[unique_idx]
    yaw = yaw[unique_idx]
    return time_s, xy, yaw


def path_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return float("nan")
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def resample_by_time(values: np.ndarray, time_s: np.ndarray, target: np.ndarray) -> np.ndarray:
    if values.ndim == 1:
        return np.interp(target, time_s, values)
    return np.column_stack([np.interp(target, time_s, values[:, idx]) for idx in range(values.shape[1])])


def resample_trajectory_arc_length(points: np.ndarray, num_samples: int) -> np.ndarray:
    if len(points) < 2:
        return np.zeros((num_samples, 2), dtype=float)
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.insert(np.cumsum(distances), 0, 0.0)
    if cumulative[-1] <= 0:
        return np.repeat(points[:1], num_samples, axis=0)
    progress = cumulative / cumulative[-1]
    progress, unique_idx = np.unique(progress, return_index=True)
    points = points[unique_idx]
    target = np.linspace(0.0, 1.0, num_samples)
    return np.column_stack((np.interp(target, progress, points[:, 0]), np.interp(target, progress, points[:, 1])))


def align_se2(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    source_centroid = source.mean(axis=0)
    target_centroid = target.mean(axis=0)
    source_centered = source - source_centroid
    target_centered = target - target_centroid
    h = source_centered.T @ target_centered
    u, _, vt = np.linalg.svd(h)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = target_centroid - rotation @ source_centroid
    return (rotation @ source.T).T + translation


def aligned_resampled_trajectories(runs: list[TfRun]) -> list[np.ndarray]:
    if not runs:
        return []
    anchor = runs[0].resampled_xy
    return [anchor] + [align_se2(run.resampled_xy, anchor) for run in runs[1:]]


def estimate_profiles(
    time_s: np.ndarray,
    xy: np.ndarray,
    yaw: np.ndarray,
    smooth_window: int,
    max_dt: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    xy_smooth = moving_average(xy, smooth_window)
    yaw_unwrapped = np.unwrap(yaw)
    yaw_smooth = moving_average(yaw_unwrapped, smooth_window)

    dt = np.diff(time_s)
    valid = dt > 0
    segment_speed = np.zeros(max(0, len(time_s) - 1), dtype=float)
    segment_angular = np.zeros(max(0, len(time_s) - 1), dtype=float)
    if segment_speed.size:
        segment_speed[valid] = np.linalg.norm(np.diff(xy_smooth, axis=0)[valid], axis=1) / dt[valid]
        segment_angular[valid] = np.diff(yaw_smooth)[valid] / dt[valid]
        if max_dt > 0:
            segment_speed[dt > max_dt] = np.nan
            segment_angular[dt > max_dt] = np.nan

    sample_speed = np.zeros(len(time_s), dtype=float)
    sample_angular = np.zeros(len(time_s), dtype=float)
    if len(time_s) > 1:
        sample_speed[0] = segment_speed[0]
        sample_speed[-1] = segment_speed[-1]
        sample_angular[0] = segment_angular[0]
        sample_angular[-1] = segment_angular[-1]
    if len(time_s) > 2:
        speed_pairs = np.column_stack((segment_speed[:-1], segment_speed[1:]))
        angular_pairs = np.column_stack((segment_angular[:-1], segment_angular[1:]))
        speed_counts = np.sum(np.isfinite(speed_pairs), axis=1)
        angular_counts = np.sum(np.isfinite(angular_pairs), axis=1)
        sample_speed[1:-1] = np.divide(
            np.nansum(speed_pairs, axis=1),
            speed_counts,
            out=np.zeros_like(speed_counts, dtype=float),
            where=speed_counts > 0,
        )
        sample_angular[1:-1] = np.divide(
            np.nansum(angular_pairs, axis=1),
            angular_counts,
            out=np.zeros_like(angular_counts, dtype=float),
            where=angular_counts > 0,
        )

    sample_speed = np.nan_to_num(sample_speed, nan=0.0, posinf=0.0, neginf=0.0)
    sample_angular = np.nan_to_num(sample_angular, nan=0.0, posinf=0.0, neginf=0.0)
    normalized_time = (time_s - time_s[0]) / max(time_s[-1] - time_s[0], 1e-9)
    return xy_smooth, yaw_smooth, sample_speed, sample_angular, normalized_time


def load_tf_run(
    input_root: Path,
    path: Path,
    resample: int,
    smooth_window: int,
    max_dt: float,
    min_poses: int,
    max_velocity_mps: float,
) -> TfRun:
    platform, environment, measurement_no, algorithm, mode, iteration = infer_metadata(input_root, path)
    obj = json.loads(path.read_text(encoding="utf-8"))
    data = obj.get("data")
    if not isinstance(data, list):
        raise ValueError(f"{path}: missing non-list data")

    timestamps: list[float] = []
    positions: list[tuple[float, float]] = []
    yaws: list[float] = []
    for row in data:
        if not isinstance(row, dict):
            continue
        timestamps.append(float(row["timestamp"]))
        positions.append(xy_from_row(row))
        yaws.append(yaw_from_row(row))

    if len(timestamps) < min_poses:
        raise ValueError(f"{path}: too few TF samples")

    time_s = np.asarray(timestamps, dtype=float)
    xy = np.asarray(positions, dtype=float)
    yaw = np.asarray(yaws, dtype=float)
    time_s, xy, yaw = deduplicate_samples(time_s, xy, yaw)
    time_s = time_s - time_s[0]
    if len(time_s) < min_poses or time_s[-1] <= 0:
        raise ValueError(f"{path}: too few unique moving poses")

    xy_smooth, yaw_smooth, speed, angular, normalized_time = estimate_profiles(
        time_s, xy, yaw, smooth_window=smooth_window, max_dt=max_dt
    )
    if max_velocity_mps > 0:
        speed = np.minimum(speed, max_velocity_mps)
    target_time = np.linspace(0.0, 1.0, resample)
    return TfRun(
        path=path,
        platform=platform,
        environment=environment,
        measurement_no=measurement_no,
        algorithm=algorithm,
        mode=mode,
        iteration=iteration,
        time_s=time_s,
        xy=xy_smooth,
        yaw_rad=yaw_smooth,
        speed_mps=speed,
        angular_velocity_rps=angular,
        normalized_time=normalized_time,
        resampled_xy=resample_trajectory_arc_length(xy_smooth, resample),
        resampled_speed_mps=np.interp(target_time, normalized_time, speed),
        resampled_angular_velocity_rps=np.interp(target_time, normalized_time, angular),
        path_length_m=path_length(xy_smooth),
    )


def load_runs(args: argparse.Namespace) -> list[TfRun]:
    runs: list[TfRun] = []
    for path in sorted(args.input_root.rglob(args.filename)):
        try:
            runs.append(
                load_tf_run(
                    args.input_root,
                    path,
                    resample=args.resample,
                    smooth_window=args.smooth_window,
                    max_dt=args.max_dt,
                    min_poses=args.min_poses,
                    max_velocity_mps=args.max_velocity_mps,
                )
            )
        except Exception as exc:
            print(f"[WARN] Skipping {path}: {exc}")
    return runs


def write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        print(f"Saved empty table to {path}")
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"Saved table to {path}")


def save_figure(fig: plt.Figure, path: Path, dpi: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    if _SHOW_PLOTS:
        plt.show(block=True)
    fig.savefig(path, dpi=dpi)
    plt.close(fig)
    print(f"Saved plot to {path}")


def line_color(run: TfRun) -> str:
    return PLATFORM_COLORS.get(run.platform, ALGORITHM_COLORS.get(run.algorithm, "#4b5563"))


def line_style(run: TfRun):
    return MEASUREMENT_STYLES.get(run.measurement_no, "-")


def platform_measurement_color(platform: str, measurement_no: str) -> str:
    return PLATFORM_MEASUREMENT_COLORS.get((platform, measurement_no), PLATFORM_COLORS.get(platform, "#4b5563"))


def group_label(key: tuple[str, str, str, str, str]) -> str:
    platform, environment, measurement_no, algorithm, mode = key
    return f"{platform} {measurement_no}{environment} {algorithm} {mode}"


def plot_profile_group(
    runs: list[TfRun],
    out_dir: Path,
    value_attr: str,
    ylabel: str,
    title_metric: str,
    filename_suffix: str,
    dpi: int,
) -> None:
    if not runs:
        return
    fig, ax = plt.subplots(figsize=(10, 5))
    for run in runs:
        values = getattr(run, value_attr)
        ax.plot(run.time_s, values, color=MODE_COLORS.get(run.mode, "#555555"), linewidth=0.7, alpha=0.5)
    max_duration = max(run.time_s[-1] for run in runs)
    time_grid = np.linspace(0.0, max_duration, 500)
    stacked = np.vstack([
        np.interp(time_grid, run.time_s, getattr(run, value_attr), left=np.nan, right=np.nan)
        for run in runs
    ])
    mean_vals = np.nanmean(stacked, axis=0)
    valid = np.sum(np.isfinite(stacked), axis=0) > 0
    ax.plot(time_grid[valid], mean_vals[valid], color="black", linewidth=0.9, label="mean profile")
    ax.grid(True, linestyle="-.", alpha=0.5)
    ax.set_xlabel("time [s]", fontsize=14)
    ax.set_ylabel(ylabel, fontsize=14)
    ax.set_title(f"{title_metric}: {group_label(runs[0].group_key)}", fontweight="bold", fontsize=16)
    ax.legend(loc="best", fontsize=14)
    name = f"{'_'.join(runs[0].group_key)}_{filename_suffix}.png"
    save_figure(fig, out_dir / sanitize_filename(name), dpi)


def plot_trajectory_overlap(runs: list[TfRun], out_dir: Path, dpi: int) -> None:
    if not runs:
        return
    aligned = aligned_resampled_trajectories(runs)
    fig, ax = plt.subplots(figsize=(7, 7))
    color = MODE_COLORS.get(runs[0].mode, "#555555")
    for xy in aligned:
        ax.plot(xy[:, 0], xy[:, 1], color=color, linewidth=0.5, alpha=0.25)
    mean_xy = np.stack(aligned, axis=0).mean(axis=0)
    ax.plot(mean_xy[:, 0], mean_xy[:, 1], color="black", linestyle="--", linewidth=0.75, label="mean")
    ax.set_aspect("equal", "box")
    ax.grid(True, linestyle="-.", alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Trajectory overlap: {group_label(runs[0].group_key)}", fontweight="bold")
    ax.legend(loc="best")
    name = f"{'_'.join(runs[0].group_key)}_trajectory_overlap.png"
    save_figure(fig, out_dir / sanitize_filename(name), dpi)


def pairwise_deviation_rows(runs: list[TfRun]) -> list[dict[str, float | str]]:
    rows: list[dict[str, float | str]] = []
    if len(runs) < 2:
        return rows
    aligned = aligned_resampled_trajectories(runs)
    for (idx_a, run_a), (idx_b, run_b) in itertools.combinations(enumerate(runs), 2):
        distances = np.linalg.norm(aligned[idx_a] - aligned[idx_b], axis=1)
        rows.append(
            {
                "platform": run_a.platform,
                "environment": run_a.environment,
                "measurement_no": run_a.measurement_no,
                "algorithm": run_a.algorithm,
                "mode": run_a.mode,
                "iteration_a": run_a.iteration,
                "iteration_b": run_b.iteration,
                "mean_deviation_m": float(np.mean(distances)),
                "median_deviation_m": float(np.median(distances)),
                "rms_deviation_m": float(np.sqrt(np.mean(distances**2))),
                "max_deviation_m": float(np.max(distances)),
            }
        )
    return rows


def plot_spatial_deviation(runs: list[TfRun], out_dir: Path, dpi: int) -> None:
    if len(runs) < 2:
        return
    aligned = aligned_resampled_trajectories(runs)
    progress = np.linspace(0.0, 100.0, len(aligned[0]))
    fig, ax = plt.subplots(figsize=(10, 5))
    for (idx_a, run_a), (idx_b, run_b) in itertools.combinations(enumerate(runs), 2):
        distances = np.linalg.norm(aligned[idx_a] - aligned[idx_b], axis=1)
        ax.plot(progress, distances, linewidth=0.3, alpha=0.25, color=MODE_COLORS.get(run_a.mode, "#555555"))
    mean_dist = np.mean(
        [
            np.linalg.norm(aligned[idx_a] - aligned[idx_b], axis=1)
            for idx_a, idx_b in itertools.combinations(range(len(aligned)), 2)
        ],
        axis=0,
    )
    ax.plot(progress, mean_dist, color="black", linewidth=0.75, label="mean pairwise deviation")
    ax.grid(True, linestyle="-.", alpha=0.3)
    ax.set_xlabel("normalized run progress [%]")
    ax.set_ylabel("spatial deviation [m]")
    ax.set_title(f"Spatial deviation: {group_label(runs[0].group_key)}", fontweight="bold")
    ax.legend(loc="best")
    name = f"{'_'.join(runs[0].group_key)}_spatial_deviation.png"
    save_figure(fig, out_dir / sanitize_filename(name), dpi)


def aggregate_profile_by_platform_measurement(runs: list[TfRun], attr: str) -> list[tuple[str, np.ndarray, str, str]]:
    groups: dict[tuple[str, str], list[TfRun]] = {}
    for run in runs:
        groups.setdefault((run.platform, run.measurement_no), []).append(run)
    profiles = []
    for (platform, measurement_no), group in sorted(groups.items()):
        arr = np.stack([getattr(run, attr) for run in group], axis=0)
        profiles.append((f"{platform}_{measurement_no}{group[0].environment}", np.nanmean(arr, axis=0), platform, measurement_no))
    return profiles


def plot_cross_platform_group(runs: list[TfRun], out_dir: Path, dpi: int) -> None:
    if len(runs) < 2:
        return
    env, algorithm, mode = runs[0].cross_platform_key
    progress = np.linspace(0.0, 100.0, len(runs[0].resampled_speed_mps))
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    for label, profile, platform, measurement_no in aggregate_profile_by_platform_measurement(
        runs, "resampled_speed_mps"
    ):
        axes[0].plot(
            progress,
            profile,
            color=PLATFORM_COLORS.get(platform, "#555555"),
            linestyle=MEASUREMENT_STYLES.get(measurement_no, "-"),
            linewidth=0.75,
            label=label,
        )
    for label, profile, platform, measurement_no in aggregate_profile_by_platform_measurement(
        runs, "resampled_angular_velocity_rps"
    ):
        axes[1].plot(
            progress,
            profile,
            color=PLATFORM_COLORS.get(platform, "#555555"),
            linestyle=MEASUREMENT_STYLES.get(measurement_no, "-"),
            linewidth=0.75,
            label=label,
        )

    axes[0].set_title(f"Cross-platform velocity: {env} {algorithm} {mode}", fontweight="bold")
    axes[0].set_ylabel("linear speed [m/s]")
    axes[1].set_title(f"Cross-platform angular velocity: {env} {algorithm} {mode}", fontweight="bold")
    axes[1].set_xlabel("normalized run progress [%]")
    axes[1].set_ylabel("yaw rate [rad/s]")
    for ax in axes:
        ax.grid(True, linestyle="-.", alpha=0.3)
        ax.legend(loc="best")

    name = f"{env}_{algorithm}_{mode}_cross_platform_velocity_angular.png"
    save_figure(fig, out_dir / sanitize_filename(name), dpi)


def mean_profile(values: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    finite = np.isfinite(values)
    counts = finite.sum(axis=0)
    sums = np.nansum(values, axis=0)
    mean = np.divide(sums, counts, out=np.full(values.shape[1], np.nan), where=counts > 0)

    centered = np.where(finite, values - mean[None, :], 0.0)
    squared = np.sum(centered * centered, axis=0)
    std = np.sqrt(np.divide(squared, counts - 1, out=np.zeros(values.shape[1]), where=counts > 1))
    return mean, std, counts


def mean_time_profiles(
    runs: list[TfRun],
    value_attr: str,
    time_step_s: float = 1.0,
) -> dict[tuple[str, str, str, str, str], tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]:
    groups: dict[tuple[str, str, str, str, str], list[TfRun]] = {}
    for run in runs:
        groups.setdefault((run.environment, run.algorithm, run.mode, run.platform, run.measurement_no), []).append(run)

    profiles: dict[tuple[str, str, str, str, str], tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = {}
    for key, group in sorted(groups.items()):
        max_duration = max(run.duration_s for run in group)
        time_grid = np.arange(0.0, max_duration + time_step_s, time_step_s)
        stacked = np.vstack(
            [
                np.interp(time_grid, run.time_s, getattr(run, value_attr), left=np.nan, right=np.nan)
                for run in group
            ]
        )
        mean, std, counts = mean_profile(stacked)
        profiles[key] = time_grid, mean, std, counts
    return profiles


def time_profile_rows(
    profiles: dict[tuple[str, str, str, str, str], tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
    mean_col: str,
    std_col: str,
) -> list[dict[str, float | int | str]]:
    rows: list[dict[str, float | int | str]] = []
    for (environment, algorithm, mode, platform, measurement_no), (time_grid, mean, std, counts) in sorted(profiles.items()):
        for time_s, mean_value, std_value, count in zip(time_grid, mean, std, counts):
            if count <= 0 or not np.isfinite(mean_value):
                continue
            rows.append(
                {
                    "environment": environment,
                    "algorithm": algorithm,
                    "mode": mode,
                    "platform": platform,
                    "measurement_no": measurement_no,
                    "time_s": float(time_s),
                    mean_col: float(mean_value),
                    std_col: float(std_value),
                    "run_count": int(count),
                }
            )
    return rows


def plot_custom_time_profiles_by_environment(
    runs: list[TfRun],
    out_dir: Path,
    dpi: int,
    value_attr: str,
    csv_name: str,
    filename_metric: str,
    title_metric: str,
    ylabel: str,
    mean_col: str,
    std_col: str,
    symmetric_y: bool = False,
) -> None:
    profiles = mean_time_profiles(runs, value_attr)
    rows = time_profile_rows(profiles, mean_col=mean_col, std_col=std_col)
    write_csv(out_dir / "custom" / csv_name, rows)
    if not profiles:
        return

    environments = sorted({key[0] for key in profiles})
    algorithms = sorted({key[1] for key in profiles})
    modes = [mode for mode in ("amcl", "no_amcl") if any(key[2] == mode for key in profiles)]
    series = [
        ("panther", "1"),
        ("panther", "2"),
        ("spot", "1"),
        ("spot", "2"),
    ]
    series_labels = [f"{platform} meas. no {measurement_no}" for platform, measurement_no in series]
    series_colors = ["#1f77b4", "#1f77b4", "#d62728", "#d62728"]
    series_styles = ["-", "--", "-", "--"]

    for environment in environments:
        fig, axes = plt.subplots(
            len(algorithms),
            len(modes),
            figsize=(max(10.0, len(modes) * 5.0), max(7.0, len(algorithms) * 2.8)),
            sharey=True,
            squeeze=False,
        )
        max_y = 0.0
        min_y = 0.0

        for algorithm_idx, algorithm in enumerate(algorithms):
            for mode_idx, mode in enumerate(modes):
                ax = axes[algorithm_idx][mode_idx]
                for (platform, measurement_no), label, color, style in zip(
                    series,
                    series_labels,
                    series_colors,
                    series_styles,
                ):
                    profile = profiles.get((environment, algorithm, mode, platform, measurement_no))
                    if profile is None:
                        continue
                    time_grid, mean_speed, _, counts = profile
                    valid = (counts > 0) & np.isfinite(mean_speed)
                    if not np.any(valid):
                        continue
                    max_y = max(max_y, float(np.nanmax(mean_speed[valid])))
                    min_y = min(min_y, float(np.nanmin(mean_speed[valid])))
                    ax.plot(
                        time_grid[valid],
                        mean_speed[valid],
                        color=color,
                        linestyle=style,
                        linewidth=0.75,
                        label=label,
                    )

                ax.set_title(f"{algorithm.title()} {mode.replace('_', ' ').upper()}", fontweight="bold")
                ax.grid(True, linestyle="-.", alpha=0.3)
                ax.set_xlabel("time [s]")
                if mode_idx == 0:
                    ax.set_ylabel(ylabel)
                ax.legend(loc="best", fontsize=8)

        if symmetric_y:
            y_abs = max(abs(min_y), abs(max_y), 0.1)
            y_limits = (-y_abs * 1.08, y_abs * 1.08)
        else:
            y_limits = (0.0, max(0.1, max_y * 1.08))
        for ax in axes.ravel():
            ax.set_ylim(*y_limits)

        fig.suptitle(f"{title_metric}: Environment {environment}", fontweight="bold")
        save_figure(fig, out_dir / "custom" / f"{environment}_{filename_metric}_by_algorithm_measurement.png", dpi)


def plot_custom_mean_velocity_by_environment(runs: list[TfRun], out_dir: Path, dpi: int) -> None:
    plot_custom_time_profiles_by_environment(
        runs,
        out_dir,
        dpi,
        value_attr="speed_mps",
        csv_name="mean_linear_velocity_by_environment.csv",
        filename_metric="mean_linear_velocity",
        title_metric="Mean linear velocity over time",
        ylabel="velocity [m/s]",
        mean_col="mean_speed_mps",
        std_col="std_speed_mps",
    )


def plot_custom_mean_angular_velocity_by_environment(runs: list[TfRun], out_dir: Path, dpi: int) -> None:
    plot_custom_time_profiles_by_environment(
        runs,
        out_dir,
        dpi,
        value_attr="angular_velocity_rps",
        csv_name="mean_angular_velocity_by_environment.csv",
        filename_metric="mean_angular_velocity",
        title_metric="Mean angular velocity over time",
        ylabel="angular velocity [rad/s]",
        mean_col="mean_angular_velocity_rps",
        std_col="std_angular_velocity_rps",
        symmetric_y=True,
    )


def plot_custom_platform_measurement_velocity_pairs(runs: list[TfRun], out_dir: Path, dpi: int) -> None:
    speed_profiles = mean_time_profiles(runs, "speed_mps")
    angular_profiles = mean_time_profiles(runs, "angular_velocity_rps")
    if not speed_profiles and not angular_profiles:
        return

    environments = sorted({key[0] for key in speed_profiles} | {key[0] for key in angular_profiles})
    platform_measurements = sorted({(key[3], key[4]) for key in speed_profiles} | {(key[3], key[4]) for key in angular_profiles})
    mode_styles = {"amcl": "-", "no_amcl": "--"}

    for environment in environments:
        for platform, measurement_no in platform_measurements:
            keys = sorted(
                {
                    (key[1], key[2])
                    for key in set(speed_profiles) | set(angular_profiles)
                    if key[0] == environment and key[3] == platform and key[4] == measurement_no
                }
            )
            if not keys:
                continue

            fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
            max_speed = 0.0
            min_angular = 0.0
            max_angular = 0.0

            for algorithm, mode in keys:
                color = ALGORITHM_COLORS.get(algorithm, "#4b5563")
                style = mode_styles.get(mode, "-")
                label = f"{algorithm} {mode.replace('_', ' ')}"

                speed_profile = speed_profiles.get((environment, algorithm, mode, platform, measurement_no))
                if speed_profile is not None:
                    time_grid, mean_speed, _, counts = speed_profile
                    valid = (counts > 0) & np.isfinite(mean_speed)
                    if np.any(valid):
                        max_speed = max(max_speed, float(np.nanmax(mean_speed[valid])))
                        axes[0].plot(
                            time_grid[valid],
                            mean_speed[valid],
                            color=color,
                            linestyle=style,
                            linewidth=0.75,
                            label=label,
                        )

                angular_profile = angular_profiles.get((environment, algorithm, mode, platform, measurement_no))
                if angular_profile is not None:
                    time_grid, mean_angular, _, counts = angular_profile
                    valid = (counts > 0) & np.isfinite(mean_angular)
                    if np.any(valid):
                        min_angular = min(min_angular, float(np.nanmin(mean_angular[valid])))
                        max_angular = max(max_angular, float(np.nanmax(mean_angular[valid])))
                        axes[1].plot(
                            time_grid[valid],
                            mean_angular[valid],
                            color=color,
                            linestyle=style,
                            linewidth=0.75,
                            label=label,
                        )

            axes[0].set_title(f"Mean linear velocity", fontweight="bold")
            axes[0].set_ylabel("velocity [m/s]")
            axes[0].set_ylim(0.0, max(0.1, max_speed * 1.08))

            angular_abs = max(abs(min_angular), abs(max_angular), 0.1)
            axes[1].set_title(f"Mean angular velocity", fontweight="bold")
            axes[1].set_xlabel("time [s]")
            axes[1].set_ylabel("angular velocity [rad/s]")
            axes[1].set_ylim(-angular_abs * 1.08, angular_abs * 1.08)

            for ax in axes:
                ax.grid(True, linestyle="-.", alpha=0.3)
            axes[0].legend(loc="best", fontsize=8, title=f"{platform} {environment}{measurement_no}")

            filename = f"{environment}_{platform}_{measurement_no}_mean_velocity_profiles.png"
            save_figure(fig, out_dir / "custom" / sanitize_filename(filename), dpi)


def plot_custom_environment_platform_measurement_overlay(runs: list[TfRun], out_dir: Path, dpi: int) -> None:
    speed_profiles = mean_time_profiles(runs, "speed_mps")
    angular_profiles = mean_time_profiles(runs, "angular_velocity_rps")
    all_profile_keys = set(speed_profiles) | set(angular_profiles)
    if not all_profile_keys:
        return

    environments = sorted({key[0] for key in all_profile_keys})
    mode_styles = {"amcl": "-", "no_amcl": "--"}

    for environment in environments:
        keys = sorted(key for key in all_profile_keys if key[0] == environment)
        if not keys:
            continue

        fig, axes = plt.subplots(2, 1, figsize=(18, 7), sharex=True)
        max_speed = 0.0
        min_angular = 0.0
        max_angular = 0.0
        legend_seen: set[tuple[str, str]] = set()

        for _, algorithm, mode, platform, measurement_no in keys:
            color = platform_measurement_color(platform, measurement_no)
            style = mode_styles.get(mode, "-")
            label_key = (platform, measurement_no)
            label = f"{platform}_{measurement_no}" if label_key not in legend_seen else None
            legend_seen.add(label_key)

            speed_profile = speed_profiles.get((environment, algorithm, mode, platform, measurement_no))
            if speed_profile is not None:
                time_grid, mean_speed, _, counts = speed_profile
                valid = (counts > 0) & np.isfinite(mean_speed)
                if np.any(valid):
                    max_speed = max(max_speed, float(np.nanmax(mean_speed[valid])))
                    axes[0].plot(
                        time_grid[valid],
                        mean_speed[valid],
                        color=color,
                        linestyle=style,
                        linewidth=0.8,
                        alpha=0.7,
                        label=label,
                    )

            angular_profile = angular_profiles.get((environment, algorithm, mode, platform, measurement_no))
            if angular_profile is not None:
                time_grid, mean_angular, _, counts = angular_profile
                valid = (counts > 0) & np.isfinite(mean_angular)
                if np.any(valid):
                    min_angular = min(min_angular, float(np.nanmin(mean_angular[valid])))
                    max_angular = max(max_angular, float(np.nanmax(mean_angular[valid])))
                    axes[1].plot(
                        time_grid[valid],
                        mean_angular[valid],
                        color=color,
                        linestyle=style,
                        linewidth=0.8,
                        alpha=0.7,
                    )

        axes[0].set_title(f"Mean linear velocity", fontweight="bold", fontsize=18)
        axes[0].set_ylabel("velocity [m/s]", fontsize=12)
        axes[0].set_ylim(0.0, max(0.1, max_speed * 1.08))

        angular_abs = max(abs(min_angular), abs(max_angular), 0.1)
        axes[1].set_title(f"Mean angular velocity", fontweight="bold", fontsize=18)
        axes[1].set_xlabel("time [s]", fontsize=12)
        axes[1].set_ylabel("angular velocity [rad/s]", fontsize=12)
        axes[1].set_ylim(-angular_abs * 1.08, angular_abs * 1.08)

        for ax in axes:
            ax.grid(True, linestyle="-.", alpha=0.5)
        axes[0].legend(loc="best", fontsize=14, title=f"Environment {environment}", title_fontsize=16)

        filename = f"{environment}_all_platform_measurements_mean_velocity_profiles.png"
        save_figure(fig, out_dir / "custom" / sanitize_filename(filename), dpi)


def plot_custom_environment_trajectory_overlap(
    runs: list[TfRun],
    out_dir: Path,
    dpi: int,
    oriented: bool = False,
) -> None:
    by_environment_group: dict[tuple[str, str, str], list[TfRun]] = {}
    for run in runs:
        by_environment_group.setdefault((run.environment, run.platform, run.measurement_no), []).append(run)

    environments = sorted({key[0] for key in by_environment_group})
    for environment in environments:
        mean_trajectories: list[tuple[str, str, str, np.ndarray]] = []
        for _, platform, measurement_no in sorted(key for key in by_environment_group if key[0] == environment):
            group = sorted(
                by_environment_group[(environment, platform, measurement_no)],
                key=lambda item: (item.algorithm, item.mode, int(item.iteration)),
            )
            aligned = aligned_resampled_trajectories(group)
            if not aligned:
                continue
            mean_xy = np.stack(aligned, axis=0).mean(axis=0)
            mean_trajectories.append((platform, measurement_no, f"{platform}_{measurement_no}", mean_xy))

        if not mean_trajectories:
            continue

        if oriented:
            reference = mean_trajectories[0][3]
            mean_trajectories = [
                (platform, measurement_no, label, mean_xy if idx == 0 else align_se2(mean_xy, reference))
                for idx, (platform, measurement_no, label, mean_xy) in enumerate(mean_trajectories)
            ]

        fig, ax = plt.subplots(figsize=(7, 7))
        all_xy = []
        for platform, measurement_no, label, mean_xy in mean_trajectories:
            all_xy.append(mean_xy)
            ax.plot(
                mean_xy[:, 0],
                mean_xy[:, 1],
                color=platform_measurement_color(platform, measurement_no),
                linestyle=MEASUREMENT_STYLES.get(measurement_no, "-"),
                linewidth=0.9,
                label=label,
            )

        xy_values = np.vstack(all_xy)
        min_x, min_y = np.min(xy_values, axis=0)
        max_x, max_y = np.max(xy_values, axis=0)
        center_x = float((min_x + max_x) / 2.0)
        center_y = float((min_y + max_y) / 2.0)
        half_range = float(max(max_x - min_x, max_y - min_y) / 2.0)
        half_range = max(half_range * 1.08, 0.2)

        ax.set_xlim(center_x - half_range, center_x + half_range)
        ax.set_ylim(center_y - half_range, center_y + half_range)
        ax.set_aspect("equal", "box")
        ax.grid(True, linestyle="-.", alpha=0.5)
        ax.set_xlabel("x [m]", fontsize=14)
        ax.set_ylabel("y [m]", fontsize=14)
        title_suffix = "SE(2) aligned" if oriented else ""
        ax.set_title(f"Mean trajectory overlap{title_suffix}", fontweight="bold", fontsize=16)
        ax.legend(loc="best", fontsize=12, title=f"Environment {environment}", title_fontsize=14)

        filename_suffix = "_aligned" if oriented else ""
        filename = f"{environment}_platform_measurement_mean_trajectory_overlap{filename_suffix}.png"
        save_figure(fig, out_dir / "custom" / sanitize_filename(filename), dpi)


def write_tables(runs: list[TfRun], pairwise_rows: list[dict[str, float | str]], out_dir: Path) -> None:
    metric_rows = []
    for run in runs:
        metric_rows.append(
            {
                "platform": run.platform,
                "environment": run.environment,
                "measurement_no": run.measurement_no,
                "algorithm": run.algorithm,
                "mode": run.mode,
                "iteration": run.iteration,
                "file": str(run.path),
                "sample_count": len(run.time_s),
                "duration_s": run.duration_s,
                "path_length_m": run.path_length_m,
                "mean_speed_mps": float(np.mean(run.speed_mps)),
                "median_speed_mps": float(np.median(run.speed_mps)),
                "max_speed_mps": float(np.max(run.speed_mps)),
                "mean_abs_angular_velocity_rps": float(np.mean(np.abs(run.angular_velocity_rps))),
                "max_abs_angular_velocity_rps": float(np.max(np.abs(run.angular_velocity_rps))),
            }
        )
    write_csv(out_dir / "per_run_tf_metrics.csv", metric_rows)
    write_csv(out_dir / "pairwise_spatial_deviation.csv", pairwise_rows)


def generate_plots(runs: list[TfRun], out_dir: Path, dpi: int, plot_individual_groups: bool) -> list[dict[str, float | str]]:
    by_group: dict[tuple[str, str, str, str, str], list[TfRun]] = {}
    by_cross: dict[tuple[str, str, str], list[TfRun]] = {}
    for run in runs:
        by_group.setdefault(run.group_key, []).append(run)
        by_cross.setdefault(run.cross_platform_key, []).append(run)

    pairwise_rows: list[dict[str, float | str]] = []
    for group in by_group.values():
        ordered = sorted(group, key=lambda item: int(item.iteration))
        pairwise_rows.extend(pairwise_deviation_rows(ordered))
        if plot_individual_groups:
            plot_profile_group(
                ordered,
                out_dir / "platform_measurement" / "velocity",
                "speed_mps",
                "linear speed [m/s]",
                "Velocity profiles",
                "velocity_profiles",
                dpi,
            )
            plot_profile_group(
                ordered,
                out_dir / "platform_measurement" / "angular_velocity",
                "angular_velocity_rps",
                "yaw rate [rad/s]",
                "Angular velocity profiles",
                "angular_velocity_profiles",
                dpi,
            )
            plot_trajectory_overlap(ordered, out_dir / "platform_measurement" / "trajectory_overlap", dpi)
            plot_spatial_deviation(ordered, out_dir / "platform_measurement" / "spatial_deviation", dpi)

    for group in by_cross.values():
        ordered = sorted(group, key=lambda item: (item.platform, item.measurement_no, item.algorithm, item.mode, int(item.iteration)))
        plot_cross_platform_group(ordered, out_dir / "cross_platform", dpi)

    plot_custom_mean_velocity_by_environment(runs, out_dir, dpi)
    plot_custom_mean_angular_velocity_by_environment(runs, out_dir, dpi)
    plot_custom_platform_measurement_velocity_pairs(runs, out_dir, dpi)
    plot_custom_environment_platform_measurement_overlay(runs, out_dir, dpi)
    plot_custom_environment_trajectory_overlap(runs, out_dir, dpi)
    plot_custom_environment_trajectory_overlap(runs, out_dir, dpi, oriented=True)

    return pairwise_rows


def build_arg_parser() -> argparse.ArgumentParser:
    default_input = Path("/home/andrii/games/ROS/bags/mdpi_results/slam_results")
    default_output = Path(__file__).resolve().parent / "output" / "tf_profiles"
    parser = argparse.ArgumentParser(description="Generate velocity and repeatability profiles from TF.json results.")
    parser.add_argument("--input-root", type=Path, default=default_input, help="Root containing platform result folders.")
    parser.add_argument("--output-dir", type=Path, default=default_output, help="Directory for plots and CSV outputs.")
    parser.add_argument("--filename", default="TF.json", help="TF filename to search for.")
    parser.add_argument("--resample", type=int, default=1000, help="Samples used for normalized profiles and trajectories.")
    parser.add_argument("--smooth-window", type=int, default=5, help="Odd moving-average window for TF position/yaw.")
    parser.add_argument("--max-dt", type=float, default=1.0, help="Ignore velocity over timestamp gaps larger than this.")
    parser.add_argument(
        "--max-velocity-mps",
        type=float,
        default=3.0,
        help="Cut off linear speed values above this threshold. Use <= 0 to disable.",
    )
    parser.add_argument("--min-poses", type=int, default=5, help="Minimum unique TF timestamps per run.")
    parser.add_argument("--dpi", type=int, default=600, help="Output plot DPI.")
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="Only create cross-platform plots and CSVs; skip per platform/measurement group plots.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open an interactive window for every plot before saving it.",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    runs = load_runs(args)
    if not runs:
        raise SystemExit(f"No valid {args.filename} files found below {args.input_root}")

    runs = sorted(
        runs,
        key=lambda item: (
            item.platform,
            item.environment,
            int(item.measurement_no),
            item.algorithm,
            item.mode,
            int(item.iteration),
        ),
    )
    print(f"Loaded {len(runs)} TF runs from {args.input_root}")
    if args.show:
        global _SHOW_PLOTS
        _SHOW_PLOTS = True
        plt.switch_backend("TkAgg")
    pairwise_rows = generate_plots(runs, args.output_dir, dpi=args.dpi, plot_individual_groups=not args.summary_only)
    write_tables(runs, pairwise_rows, args.output_dir)


if __name__ == "__main__":
    main()
