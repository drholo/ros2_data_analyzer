#!/usr/bin/env python3
"""
Plot all TF.json trajectories found under a directory and overlay their mean trajectory.

If the input directory name matches `results_<measurement_no><env>` and the default
Panther CSV is available, the figure title is annotated with the corresponding mean
path length from `output/panther/raw_path_length_by_env_and_measurement.csv`.
"""

from __future__ import annotations

import argparse
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

LINE_WIDTH = 0.8


@dataclass
class TrajectoryRun:
    path: Path
    algorithm: str
    mode: str
    run_id: str
    raw: np.ndarray
    resampled: np.ndarray
    raw_path_length_m: float


def load_positions_from_json(path: Path, min_poses: int = 5) -> np.ndarray:
    obj = json.loads(path.read_text(encoding="utf-8"))
    data = obj.get("data", None)
    if not isinstance(data, list):
        raise ValueError(f"{path}: missing or non-list 'data'")

    def try_xy(v):
        if v is None:
            return None
        if isinstance(v, dict):
            for key in ("position", "translation"):
                if key in v:
                    got = try_xy(v[key])
                    if got is not None:
                        return got
            if "x" in v and "y" in v:
                return float(v["x"]), float(v["y"])
            if "X" in v and "Y" in v:
                return float(v["X"]), float(v["Y"])
        if isinstance(v, (list, tuple)) and len(v) >= 2:
            return float(v[0]), float(v[1])
        return None

    pts = []
    for row in data:
        xy = try_xy(row)
        if xy is not None:
            pts.append(xy)

    raw = np.asarray(pts, dtype=float)
    if len(raw) < min_poses:
        raise ValueError(f"too few valid poses in {path}")

    if len(raw) >= 2:
        step = np.linalg.norm(np.diff(raw, axis=0), axis=1)
        keep = np.concatenate([[True], step > 0])
        raw = raw[keep]

    return raw


def path_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return float("nan")
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


def resample_trajectory_arc_length(points: np.ndarray, num_samples: int = 1000) -> np.ndarray:
    if len(points) < 2:
        return np.zeros((num_samples, 2))

    diffs = np.diff(points, axis=0)
    distances = np.linalg.norm(diffs, axis=1)
    cumulative = np.insert(np.cumsum(distances), 0, 0.0)
    total = cumulative[-1]
    if total == 0:
        return np.zeros((num_samples, 2))

    progress = cumulative / total
    progress, unique_idx = np.unique(progress, return_index=True)
    unique_points = points[unique_idx]
    target = np.linspace(0.0, 1.0, num_samples)

    x = np.interp(target, progress, unique_points[:, 0])
    y = np.interp(target, progress, unique_points[:, 1])
    return np.column_stack((x, y))


def align_se2(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    centroid_source = np.mean(source, axis=0)
    centroid_target = np.mean(target, axis=0)

    source_centered = source - centroid_source
    target_centered = target - centroid_target

    h = source_centered.T @ target_centered
    u, _, vt = np.linalg.svd(h)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[1, :] *= -1
        rotation = vt.T @ u.T

    translation = centroid_target - rotation @ centroid_source
    return (rotation @ source.T).T + translation


def mean_trajectory(runs: list[np.ndarray]) -> np.ndarray:
    anchor = runs[0]
    aligned = [anchor]
    for run in runs[1:]:
        aligned.append(align_se2(run, anchor))
    return np.stack(aligned, axis=0).mean(axis=0)


def parse_results_folder_name(name: str) -> Optional[tuple[str, str]]:
    match = re.match(r"^results_(\d+)([a-zA-Z]+)$", name)
    if not match:
        return None
    measurement_no = match.group(1)
    env = match.group(2).upper()
    return env, measurement_no


def parse_record_folder(name: str) -> Optional[tuple[str, str, str]]:
    if not name.startswith("record_"):
        return None
    parts = name.removeprefix("record_").split("_")
    if len(parts) < 3 or not parts[-1].isdigit():
        return None
    if len(parts) >= 4 and parts[-3] == "no" and parts[-2] == "amcl":
        return "_".join(parts[:-3]), "no_amcl", parts[-1]
    if parts[-2] == "amcl":
        return "_".join(parts[:-2]), "amcl", parts[-1]
    return None


def infer_run_metadata(root: Path, path: Path) -> tuple[str, str, str]:
    relative_parts = path.relative_to(root).parts
    for part in relative_parts[:-1]:
        parsed = parse_record_folder(part)
        if parsed is not None:
            return parsed
    return "unknown", "unknown", path.stem


def load_runs(root: Path, filename: str, resample: int, min_poses: int) -> list[TrajectoryRun]:
    runs: list[TrajectoryRun] = []
    for path in sorted(root.rglob(filename)):
        try:
            raw = load_positions_from_json(path, min_poses=min_poses)
            algorithm, mode, run_id = infer_run_metadata(root, path)
            runs.append(
                TrajectoryRun(
                    path=path,
                    algorithm=algorithm,
                    mode=mode,
                    run_id=run_id,
                    raw=raw,
                    resampled=resample_trajectory_arc_length(raw, num_samples=resample),
                    raw_path_length_m=path_length(raw),
                )
            )
        except Exception as exc:
            print(f"[WARN] Skipping {path}: {exc}")
    return runs


def aligned_runs(runs: list[TrajectoryRun]) -> list[np.ndarray]:
    anchor = runs[0].resampled
    aligned = [anchor]
    for run in runs[1:]:
        aligned.append(align_se2(run.resampled, anchor))
    return aligned


def panther_mean_summary(csv_path: Path, input_dir: Path) -> Optional[dict[str, float | str]]:
    parsed = parse_results_folder_name(input_dir.name)
    if not parsed or not csv_path.exists():
        return None

    env, measurement_no = parsed
    df = pd.read_csv(csv_path, dtype={"env": str, "measurement_no": str})
    row = df[
        (df["env"].str.upper() == env)
        & (df["measurement_no"] == measurement_no)
    ]
    if row.empty:
        return None

    item = row.iloc[0]
    return {
        "env": env,
        "measurement_no": measurement_no,
        "mean_path_length_m": float(item["mean_path_length_m"]),
        "ci95_low_m": float(item["ci95_low_m"]),
        "ci95_high_m": float(item["ci95_high_m"]),
    }


def build_title(input_dir: Path) -> str:
    return f"2D Trajectory Comparison: {input_dir.name}"


def compute_plot_limits(aligned: list[np.ndarray], mean_xy: np.ndarray, padding: float = 0.2) -> tuple[float, float]:
    all_points = np.concatenate(aligned + [mean_xy], axis=0)
    return float(all_points.min()) - padding, float(all_points.max()) + padding


def sanitize_filename(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_") or "plot"


def algorithm_color(algorithm: str) -> str:
    color_map = {
        "gmapping": "green",
        "cartographer": "red",
        "slam_toolbox": "blue",
    }
    return color_map.get(algorithm, "gray")


def style_for_index(index: int):
    styles = ["--", ":", "-.", (0, (3, 1, 1, 1)), (0, (5, 2)), (0, (1, 1))]
    return styles[index % len(styles)]


def style_axes(ax: plt.Axes, title: str, limit_low: float, limit_high: float) -> None:
    ax.set_aspect("equal", "box")
    ax.grid(True, linestyle="-.", alpha=0.3)
    ax.set_xlabel("x [m]", fontsize=12)
    ax.set_ylabel("y [m]", fontsize=12)
    ax.set_title(title, fontsize=14, fontweight="bold")
    ax.set_xlim(limit_low, limit_high)
    ax.set_ylim(limit_low, limit_high)


def _create_figure(title: str, limit_low: float, limit_high: float) -> tuple[plt.Figure, plt.Axes]:
    fig, ax = plt.subplots(figsize=(10, 8))
    fig.patch.set_facecolor("white")
    style_axes(ax, title, limit_low, limit_high)
    return fig, ax


def _save_figure(fig: plt.Figure, path: Path) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"Saved plot to {path}")


def plot_algorithm_group(
    input_dir: Path,
    output_dir: Path,
    algorithm: str,
    mode: str,
    runs: list[TrajectoryRun],
    summary: Optional[dict[str, float | str]],
) -> np.ndarray:
    aligned = aligned_runs(runs)
    mean_xy = mean_trajectory([run.resampled for run in runs])
    limit_low, limit_high = compute_plot_limits(aligned, mean_xy)

    fig, ax = _create_figure(
        f"{build_title(input_dir)} {algorithm} {mode}",
        limit_low,
        limit_high,
    )

    for aligned_xy in aligned:
        ax.plot(aligned_xy[:, 0], aligned_xy[:, 1], color="gray", alpha=0.3, linewidth=LINE_WIDTH)

    ax.plot(mean_xy[:, 0], mean_xy[:, 1], color="black", linestyle="--", linewidth=LINE_WIDTH)

    out_path = output_dir / f"tf_trajectories_{sanitize_filename(algorithm)}_{sanitize_filename(mode)}.png"
    _save_figure(fig, out_path)
    return mean_xy


def plot_mode_summary(
    input_dir: Path,
    output_dir: Path,
    mode: str,
    runs: list[TrajectoryRun],
    summary: Optional[dict[str, float | str]],
) -> None:
    by_algorithm: dict[str, list[TrajectoryRun]] = {}
    for run in runs:
        by_algorithm.setdefault(run.algorithm, []).append(run)

    algo_aligned: dict[str, list[np.ndarray]] = {}
    algorithm_means: dict[str, np.ndarray] = {}
    all_curves: list[np.ndarray] = []

    for algorithm, algo_runs in sorted(by_algorithm.items()):
        curves = aligned_runs(algo_runs)
        mean_xy = mean_trajectory([run.resampled for run in algo_runs])
        algo_aligned[algorithm] = curves
        algorithm_means[algorithm] = mean_xy
        all_curves.extend(curves)

    if not algorithm_means:
        return

    mean_values = list(algorithm_means.values())
    mean_of_means = mean_trajectory(mean_values)
    limit_low, limit_high = compute_plot_limits(all_curves + mean_values, mean_of_means)
    fig, ax = _create_figure(
        f"{build_title(input_dir)} all algorithms {mode}",
        limit_low,
        limit_high,
    )

    for idx, (algorithm, curves) in enumerate(sorted(algo_aligned.items())):
        color = algorithm_color(algorithm)
        for aligned_xy in curves:
            ax.plot(aligned_xy[:, 0], aligned_xy[:, 1], color=color, alpha=0.3, linewidth=LINE_WIDTH)
        mean_xy = algorithm_means[algorithm]
        ax.plot(
            mean_xy[:, 0],
            mean_xy[:, 1],
            color="black",
            linestyle=style_for_index(idx),
            linewidth=LINE_WIDTH,
            label=f"{algorithm} mean",
        )

    ax.legend(loc="best", fontsize=11)
    out_path = output_dir / f"tf_trajectories_all_algorithms_{sanitize_filename(mode)}.png"
    _save_figure(fig, out_path)


def plot_mean_summary(
    input_dir: Path,
    output_dir: Path,
    algorithm_means: dict[str, np.ndarray],
    summary: Optional[dict[str, float | str]],
) -> None:
    ordered_names = sorted(algorithm_means)
    ordered_means = [algorithm_means[name] for name in ordered_names]
    reference_mean = ordered_means[0]
    limit_low, limit_high = compute_plot_limits(ordered_means, reference_mean)
    fig, ax = _create_figure(
        f"{build_title(input_dir)} algorithm means",
        limit_low,
        limit_high,
    )

    for idx, (name, mean_xy) in enumerate(zip(ordered_names, ordered_means)):
        ax.plot(
            mean_xy[:, 0],
            mean_xy[:, 1],
            color="black",
            linestyle=style_for_index(idx),
            label=f"{name} mean",
            linewidth=LINE_WIDTH,
        )
    ax.legend(loc="best", fontsize=11)
    _save_figure(fig, output_dir / "tf_trajectories_algorithm_means.png")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("input_dir", help="Directory to scan recursively for TF.json files")
    ap.add_argument("--filename", default="TF.json", help="Trajectory filename to search for")
    ap.add_argument("--out-dir", help="Output directory; defaults to <input_dir>/trajectory_plots")
    ap.add_argument("--resample", type=int, default=1000, help="Samples per trajectory for averaging")
    ap.add_argument("--min-poses", type=int, default=5, help="Minimum valid poses required to keep a run")
    ap.add_argument(
        "--panther-csv",
        default="output/panther/raw_path_length_by_env_and_measurement.csv",
        help="CSV file with Panther mean path length by env and measurement",
    )
    args = ap.parse_args()

    input_dir = Path(args.input_dir).resolve()
    if not input_dir.exists() or not input_dir.is_dir():
        raise SystemExit(f"Input directory does not exist or is not a directory: {input_dir}")

    runs = load_runs(input_dir, args.filename, args.resample, args.min_poses)
    if not runs:
        raise SystemExit(f"No valid {args.filename} files found under {input_dir}")

    summary = panther_mean_summary(Path(args.panther_csv), input_dir)
    output_dir = Path(args.out_dir).resolve() if args.out_dir else input_dir / "trajectory_plots"
    output_dir.mkdir(parents=True, exist_ok=True)

    by_algo_mode: dict[tuple[str, str], list[TrajectoryRun]] = {}
    by_mode: dict[str, list[TrajectoryRun]] = {}
    for run in runs:
        by_algo_mode.setdefault((run.algorithm, run.mode), []).append(run)
        by_mode.setdefault(run.mode, []).append(run)

    algorithm_means: dict[str, np.ndarray] = {}
    for algorithm, mode in sorted(by_algo_mode):
        group_mean = plot_algorithm_group(
            input_dir,
            output_dir,
            algorithm,
            mode,
            by_algo_mode[(algorithm, mode)],
            summary,
        )
        algorithm_means[f"{algorithm}_{mode}"] = group_mean

    for mode in sorted(by_mode):
        plot_mode_summary(input_dir, output_dir, mode, by_mode[mode], summary)

    if algorithm_means:
        plot_mean_summary(input_dir, output_dir, algorithm_means, summary)


if __name__ == "__main__":
    main()
