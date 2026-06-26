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
MODE_COLORS = {"amcl": "#e07b39", "no_amcl": "#5b8db8"}
MODE_LABELS = {"amcl": "AMCL", "no_amcl": "no-AMCL"}


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


def bootstrap_ci_mean(
    x: np.ndarray,
    n_boot: int = 2000,
    alpha: float = 0.05,
    seed: int = 0,
) -> tuple[float, float]:
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan"), float("nan")
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(x), size=(n_boot, len(x)))
    boots = x[idx].mean(axis=1)
    return float(np.quantile(boots, alpha / 2)), float(np.quantile(boots, 1 - alpha / 2))


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


def platform_mean_summary(csv_path: Path, input_dir: Path) -> Optional[dict[str, float | str]]:
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


def _slice_sort_key(env: str, measurement_no: str) -> tuple[str, int | str]:
    if str(measurement_no).isdigit():
        return env, int(measurement_no)
    return env, measurement_no


def _slice_label(env: str, measurement_no: str) -> str:
    return f"{env}-{measurement_no}"


def plot_delta_heatmap(
    summary_df: pd.DataFrame,
    metric_col: str,
    title: str,
    out_path: Path,
) -> None:
    required = {"env", "measurement_no", "algorithm", metric_col}
    if not required.issubset(summary_df.columns):
        print(f"[WARN] Missing columns for heatmap {metric_col}, skipping")
        return

    df = summary_df.copy()
    df["slice"] = df["env"].astype(str) + "-" + df["measurement_no"].astype(str)
    algos = sorted(df["algorithm"].unique())
    slices = sorted(
        df[["env", "measurement_no"]].drop_duplicates().itertuples(index=False, name=None),
        key=lambda v: _slice_sort_key(v[0], str(v[1])),
    )
    slice_labels = [_slice_label(e, str(m)) for e, m in slices]

    grid = pd.DataFrame(np.nan, index=algos, columns=slice_labels)
    for _, row in df.iterrows():
        grid.loc[row["algorithm"], f"{row['env']}-{row['measurement_no']}"] = float(row[metric_col])

    data = grid.to_numpy(dtype=float)
    fig, ax = plt.subplots(
        figsize=(max(4.0, len(slice_labels) * 0.95), max(2.5, len(algos) * 0.8)),
        dpi=300,
    )
    im = ax.imshow(data, vmin=-1.0, vmax=1.0, cmap="RdBu_r", aspect="auto")
    ax.set_xticks(range(len(slice_labels)))
    ax.set_yticks(range(len(algos)))
    ax.set_xticklabels(slice_labels, rotation=35, ha="right")
    ax.set_yticklabels(algos)
    ax.set_title(title, fontsize=12, fontweight="bold")
    # ax.set_xlabel("Environment - measurement")
    # ax.set_ylabel("Algorithm")
    plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label="Cliff's delta")

    for i in range(len(algos)):
        for j in range(len(slice_labels)):
            val = data[i, j]
            if np.isfinite(val):
                ax.text(
                    j,
                    i,
                    f"{val:+.2f}",
                    ha="center",
                    va="center",
                    fontsize=10,
                    color="white" if abs(val) > 0.5 else "black",
                )

    fig.tight_layout()
    fig.savefig(out_path, dpi=300)
    plt.close(fig)
    print(f"Saved plot to {out_path}")


def plot_faceted_mean_ci(
    summary_df: pd.DataFrame,
    amcl_col: str,
    no_amcl_col: str,
    title: str,
    out_path: Path,
) -> None:
    required = {"env", "measurement_no", "algorithm", amcl_col, no_amcl_col}
    if not required.issubset(summary_df.columns):
        print(f"[WARN] Missing columns for faceted plot {amcl_col}/{no_amcl_col}, skipping")
        return

    slices = sorted(
        summary_df[["env", "measurement_no"]].drop_duplicates().itertuples(index=False, name=None),
        key=lambda v: _slice_sort_key(v[0], str(v[1])),
    )
    algos = sorted(summary_df["algorithm"].unique())
    if not slices or not algos:
        return

    fig, axes = plt.subplots(
        len(slices),
        len(algos),
        figsize=(max(3.0, len(algos) * 3.0), max(2.4, len(slices) * 2.2)),
        dpi=300,
        squeeze=False,
    )

    for r, (env, measurement_no) in enumerate(slices):
        row_vals: list[float] = []
        for c, algo in enumerate(algos):
            ax = axes[r][c]
            row = summary_df[
                (summary_df["env"] == env)
                & (summary_df["measurement_no"].astype(str) == str(measurement_no))
                & (summary_df["algorithm"] == algo)
            ]
            if row.empty:
                ax.text(0.5, 0.5, "no data", transform=ax.transAxes, ha="center", va="center", color="gray")
            else:
                item = row.iloc[0]
                y_amcl = float(item[amcl_col])
                y_no = float(item[no_amcl_col])
                has_amcl = np.isfinite(y_amcl)
                has_no = np.isfinite(y_no)
                if not has_amcl and not has_no:
                    ax.text(0.5, 0.5, "no data", transform=ax.transAxes, ha="center", va="center", color="gray")
                else:
                    if has_amcl:
                        ax.scatter([0], [y_amcl], color=MODE_COLORS["amcl"], s=34)
                        row_vals.append(y_amcl)
                    if has_no:
                        ax.scatter([1], [y_no], color=MODE_COLORS["no_amcl"], s=34)
                        row_vals.append(y_no)
                    if has_amcl and has_no:
                        ax.plot([0, 1], [y_amcl, y_no], color="gray", alpha=0.35, linewidth=1.0)

            ax.set_xlim(-0.6, 1.6)
            ax.set_xticks([0, 1])
            ax.set_xticklabels(["AMCL", "no-AMCL"], fontsize=10, fontweight="medium")
            ax.grid(True, axis="y", linestyle="-.", alpha=0.3)
            if r == 0:
                ax.set_title(algo, fontsize=10, fontweight="medium")
            if c == 0:
                ax.set_ylabel(f"{env}-{measurement_no}")

        # set consistent y-limits for the whole row based on actual data
        if row_vals:
            y_max = max(row_vals)
            y_pad = max(y_max * 0.1, 0.05)
            for ax in axes[r]:
                ax.set_ylim(0.0, y_max + y_pad)
        else:
            for ax in axes[r]:
                ax.set_ylim(bottom=0.0)

    fig.suptitle(title, fontsize=12, fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_path, dpi=300)
    plt.close(fig)
    print(f"Saved plot to {out_path}")


def plot_mean_ci_by_slice(
    summary_df: pd.DataFrame,
    amcl_col: str,
    no_amcl_col: str,
    title_prefix: str,
    out_dir: Path,
) -> None:
    required = {"env", "measurement_no", "algorithm", amcl_col, no_amcl_col}
    if not required.issubset(summary_df.columns):
        print(f"[WARN] Missing columns for mean-by-slice {amcl_col}/{no_amcl_col}, skipping")
        return

    slices = sorted(
        summary_df[["env", "measurement_no"]].drop_duplicates().itertuples(index=False, name=None),
        key=lambda v: _slice_sort_key(v[0], str(v[1])),
    )
    slice_labels = [_slice_label(e, str(m)) for e, m in slices]

    for algo in sorted(summary_df["algorithm"].unique()):
        fig, ax = plt.subplots(figsize=(max(4.0, len(slice_labels) * 1.2), 3.8), dpi=300)
        algo_rows = summary_df[summary_df["algorithm"] == algo].copy()
        means_amcl, means_no = [], []
        for env, measurement_no in slices:
            row = algo_rows[
                (algo_rows["env"] == env)
                & (algo_rows["measurement_no"].astype(str) == str(measurement_no))
            ]
            if row.empty:
                means_amcl.append(float("nan"))
                means_no.append(float("nan"))
            else:
                means_amcl.append(float(row.iloc[0][amcl_col]))
                means_no.append(float(row.iloc[0][no_amcl_col]))

        x = np.arange(len(slice_labels))
        offset = 0.14
        means_amcl_arr = np.asarray(means_amcl, dtype=float)
        means_no_arr = np.asarray(means_no, dtype=float)
        valid_amcl = np.isfinite(means_amcl_arr)
        valid_no = np.isfinite(means_no_arr)

        ax.plot(x[valid_amcl] - offset, means_amcl_arr[valid_amcl], "o", color=MODE_COLORS["amcl"], label="AMCL")
        ax.plot(x[valid_no] + offset, means_no_arr[valid_no], "o", color=MODE_COLORS["no_amcl"], label="no AMCL")

        for i in range(len(x)):
            if np.isfinite(means_amcl_arr[i]) and np.isfinite(means_no_arr[i]):
                ax.plot([i - offset, i + offset], [means_amcl_arr[i], means_no_arr[i]], color="gray", alpha=0.35, linewidth=1.0)

        ax.set_xticks(x)
        ax.set_xticklabels(slice_labels, rotation=35, ha="right", fontsize=8)
        ax.set_ylim(bottom=0.0)
        ax.set_ylabel("Mean value")
        ax.set_title(f"{title_prefix} by slice - {algo}", fontsize=11, fontweight="bold")
        ax.grid(True, axis="y", linestyle="-.", alpha=0.3)
        ax.legend(frameon=False)
        fig.tight_layout()
        out_path = out_dir / f"trajectory_{sanitize_filename(title_prefix)}_{sanitize_filename(algo)}_mean_by_slice.png"
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        print(f"Saved plot to {out_path}")


def plot_per_run_metric_boxplots(
    per_run_df: pd.DataFrame,
    metric_col: str,
    y_label: str,
    title: str,
    out_path: Path,
) -> None:
    required = {"algorithm", "mode", metric_col}
    if not required.issubset(per_run_df.columns):
        print(f"[WARN] Missing columns for per-run boxplot {metric_col}, skipping")
        return

    algorithms = sorted(per_run_df["algorithm"].unique())
    if not algorithms:
        return

    n = len(algorithms)
    ncols = min(3, n)
    nrows = (n + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.2 * ncols, 3.6 * nrows), dpi=300, squeeze=False)

    for idx, algo in enumerate(algorithms):
        ax = axes[idx // ncols][idx % ncols]
        subset = per_run_df[per_run_df["algorithm"] == algo]
        groups = []
        labels = []
        for mode in ("amcl", "no_amcl"):
            vals = subset[subset["mode"] == mode][metric_col].to_numpy(dtype=float)
            vals = vals[np.isfinite(vals)]
            if vals.size:
                groups.append(vals)
                labels.append(MODE_LABELS[mode])

        if not groups:
            ax.set_visible(False)
            continue

        bp = ax.boxplot(
            groups,
            patch_artist=True,
            widths=0.46,
            medianprops=dict(color="black", linewidth=1.4),
            whiskerprops=dict(linewidth=1.0),
            capprops=dict(linewidth=1.0),
            flierprops=dict(marker="o", markersize=2.8, alpha=0.45),
        )
        mode_order = ["amcl", "no_amcl"][: len(groups)]
        for patch, mode in zip(bp["boxes"], mode_order):
            patch.set_facecolor(MODE_COLORS[mode])
            patch.set_alpha(0.72)

        ax.set_xticks(range(1, len(labels) + 1))
        ax.set_xticklabels(labels, fontsize=14, fontweight="medium")
        ax.tick_params(axis="y", labelsize=14)
        ax.set_title(algo, fontsize=15, fontweight="medium")
        ax.set_ylabel(y_label, fontsize=14, fontweight="medium")
        ax.grid(True, axis="y", linestyle="-.", alpha=0.5)

    for idx in range(n, nrows * ncols):
        axes[idx // ncols][idx % ncols].set_visible(False)

    fig.suptitle(title, fontsize=16, fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_path, dpi=300)
    plt.close(fig)
    print(f"Saved plot to {out_path}")


def plot_mode_comparison_boxplot_metric(
    data: dict[tuple[str, str], np.ndarray],
    ylabel: str,
    title: str,
    out_path: Path,
) -> None:
    algorithms = sorted({k[0] for k in data})
    if not algorithms:
        return

    n = len(algorithms)
    ncols = min(n, 3)
    nrows = (n + ncols - 1) // ncols
    fig, axes = plt.subplots(
        nrows,
        ncols,
        figsize=(4.0 * ncols, 3.5 * nrows),
        dpi=300,
        squeeze=False,
    )

    for ax_idx, algo in enumerate(algorithms):
        ax = axes[ax_idx // ncols][ax_idx % ncols]
        groups = []
        labels = []
        for mode in ("amcl", "no_amcl"):
            vals = data.get((algo, mode), np.array([]))
            vals = np.asarray(vals, dtype=float)
            vals = vals[np.isfinite(vals)]
            if vals.size:
                groups.append(vals)
                labels.append(MODE_LABELS[mode])

        if not groups:
            ax.set_visible(False)
            continue

        bp = ax.boxplot(
            groups,
            patch_artist=True,
            widths=0.45,
            medianprops=dict(color="black", linewidth=1.5),
            whiskerprops=dict(linewidth=1.0),
            capprops=dict(linewidth=1.0),
            flierprops=dict(marker="o", markersize=3, alpha=0.5),
        )
        mode_order = ["amcl", "no_amcl"][: len(groups)]
        for patch, mode in zip(bp["boxes"], mode_order):
            patch.set_facecolor(MODE_COLORS[mode])
            patch.set_alpha(0.7)

        ax.set_xticks(range(1, len(labels) + 1))
        ax.set_xticklabels(labels)
        ax.set_ylabel(ylabel)
        ax.set_title(algo)
        ax.grid(True, axis="y", linestyle="-.", alpha=0.3, linewidth=0.5)

    for idx in range(n, nrows * ncols):
        axes[idx // ncols][idx % ncols].set_visible(False)

    fig.suptitle(title, fontsize=11, fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_path, dpi=300)
    plt.close(fig)
    print(f"Saved plot to {out_path}")


def plot_mean_summary_metric(
    data: dict[tuple[str, str], np.ndarray],
    ylabel: str,
    title: str,
    out_path: Path,
) -> None:
    algorithms = sorted({k[0] for k in data})
    if not algorithms:
        return

    x = np.arange(len(algorithms))
    fig, ax = plt.subplots(figsize=(max(4.0, len(algorithms) * 1.4), 3.5), dpi=300)
    offset = 0.15

    for mode_idx, mode in enumerate(("amcl", "no_amcl")):
        means = []
        lo_err = []
        hi_err = []
        for algo in algorithms:
            vals = np.asarray(data.get((algo, mode), np.array([])), dtype=float)
            vals = vals[np.isfinite(vals)]
            m = float(np.nanmean(vals)) if vals.size else float("nan")
            lo, hi = bootstrap_ci_mean(vals) if vals.size else (float("nan"), float("nan"))
            means.append(m)
            lo_err.append(max(0.0, m - lo) if np.isfinite(m) and np.isfinite(lo) else float("nan"))
            hi_err.append(max(0.0, hi - m) if np.isfinite(m) and np.isfinite(hi) else float("nan"))

        means_arr = np.asarray(means, dtype=float)
        yerr = np.vstack([lo_err, hi_err])
        valid = np.isfinite(means_arr)
        xs = x + offset * (mode_idx - 0.5)
        ax.errorbar(
            xs[valid],
            means_arr[valid],
            yerr=yerr[:, valid],
            fmt="o",
            color=MODE_COLORS[mode],
            label=MODE_LABELS[mode],
            capsize=4,
            linewidth=1.2,
            markersize=5,
        )

    ax.set_xticks(x)
    ax.set_xticklabels(algorithms, rotation=15, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, axis="y", linestyle="-.", alpha=0.3, linewidth=0.5)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(out_path, dpi=300)
    plt.close(fig)
    print(f"Saved plot to {out_path}")


def _algo_mode_metric_values(
    df: pd.DataFrame,
    metric_col: str,
) -> dict[tuple[str, str], np.ndarray]:
    data: dict[tuple[str, str], np.ndarray] = {}
    for algo in sorted(df["algorithm"].unique()):
        for mode in ("amcl", "no_amcl"):
            vals = df[
                (df["algorithm"] == algo)
                & (df["mode"] == mode)
            ][metric_col].to_numpy(dtype=float)
            data[(algo, mode)] = vals[np.isfinite(vals)]
    return data


def generate_platform_summary_plots(platform_dir: Path, out_dir: Path) -> None:
    summary_path = platform_dir / "summary_by_slice.csv"
    per_run_path = platform_dir / "per_run_metrics.csv"
    if not summary_path.exists():
        print(f"[WARN] {summary_path} not found, skipping trajectory summary plots")
        return

    out_dir.mkdir(parents=True, exist_ok=True)
    summary_df = pd.read_csv(summary_path, dtype={"env": str, "measurement_no": str})

    summary_specs = [
        {
            "slug": "pairwise_ate",
            "delta_col": "pairwise_ATE_cliffs_delta",
            "amcl_col": "pairwise_ATE_amcl_mean",
            "no_col": "pairwise_ATE_no_amcl_mean",
            "title_base": "Pairwise ATE",
        },
        {
            "slug": "centroid_ate",
            "delta_col": "centroid_ATE_cliffs_delta",
            "amcl_col": "centroid_ATE_amcl_mean",
            "no_col": "centroid_ATE_no_amcl_mean",
            "title_base": "Centroid ATE",
        },
    ]

    for spec in summary_specs:
        plot_delta_heatmap(
            summary_df,
            metric_col=spec["delta_col"],
            title=f"{spec['title_base']} Cliff's delta: AMCL vs no-AMCL",
            out_path=out_dir / f"combined_cliffs_delta_heatmap_{spec['slug']}.png",
        )
        plot_faceted_mean_ci(
            summary_df,
            amcl_col=spec["amcl_col"],
            no_amcl_col=spec["no_col"],
            title=f"{spec['title_base']} means faceted by env / measurement",
            out_path=out_dir / f"combined_faceted_mean_{spec['slug']}.png",
        )
        plot_mean_ci_by_slice(
            summary_df,
            amcl_col=spec["amcl_col"],
            no_amcl_col=spec["no_col"],
            title_prefix=spec["slug"],
            out_dir=out_dir,
        )

    if per_run_path.exists():
        per_run_df = pd.read_csv(per_run_path, dtype={"env": str, "measurement_no": str})
        metrics = [
            ("centroid_ATE_m", "Centroid ATE [m]", "centroid_ate"),
            ("RPE_trans_m", "RPE trans proxy [m]", "rpe_trans"),
            ("completeness_Ck", "Completeness Ck", "completeness"),
        ]

        for metric_col, y_label, slug in metrics:
            plot_per_run_metric_boxplots(
                per_run_df,
                metric_col=metric_col,
                y_label=y_label,
                title=f"Per-run {y_label} by algorithm",
                out_path=out_dir / f"per_run_boxplot_{slug}.png",
            )

        # per-slice map-style summary folders: mode-comparison + mean/CI summary
        slices = sorted(
            per_run_df[["env", "measurement_no"]].drop_duplicates().itertuples(index=False, name=None),
            key=lambda v: _slice_sort_key(v[0], str(v[1])),
        )
        for env, measurement_no in slices:
            slice_slug = f"{sanitize_filename(str(env))}_{sanitize_filename(str(measurement_no))}"
            slice_out = out_dir / slice_slug
            slice_out.mkdir(parents=True, exist_ok=True)

            df_slice = per_run_df[
                (per_run_df["env"] == env)
                & (per_run_df["measurement_no"].astype(str) == str(measurement_no))
            ]
            for metric_col, ylabel, slug in metrics:
                data = _algo_mode_metric_values(df_slice, metric_col)

                plot_mode_comparison_boxplot_metric(
                    data,
                    ylabel=ylabel,
                    title=f"Trajectory {ylabel}: AMCL vs no-AMCL ({env}-{measurement_no})",
                    out_path=slice_out / f"mode_comparison_boxplot_{slug}.png",
                )
                plot_mean_summary_metric(
                    data,
                    ylabel=ylabel,
                    title=f"Trajectory {ylabel} mean with 95% CI ({env}-{measurement_no})",
                    out_path=slice_out / f"mean_ci_summary_{slug}.png",
                )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("input_dir", help="Directory to scan recursively for TF.json files")
    ap.add_argument("--filename", default="TF.json", help="Trajectory filename to search for")
    ap.add_argument("--out-dir", help="Output directory; defaults to <input_dir>/trajectory_plots")
    ap.add_argument("--resample", type=int, default=1000, help="Samples per trajectory for averaging")
    ap.add_argument("--min-poses", type=int, default=5, help="Minimum valid poses required to keep a run")
    ap.add_argument(
        "--platform-csv",
        default="output/raw_path_length_by_env_and_measurement.csv",
        help="CSV file with platform mean path length by env and measurement",
    )
    ap.add_argument(
        "--platform-dir",
        default="results/traj",
        help="Directory with trajectory evaluation CSVs (summary_by_slice.csv, per_run_metrics.csv)",
    )
    ap.add_argument(
        "--csvs-only",
        action="store_true",
        help="Generate only trajectory-summary plots from platform CSVs (skip TF.json drawing)",
    )
    ap.add_argument(
        "--no-platform-summary",
        action="store_true",
        help="Disable additional summary plots from platform CSVs",
    )
    args = ap.parse_args()

    input_dir = Path(args.input_dir).resolve()
    if not input_dir.exists() or not input_dir.is_dir():
        raise SystemExit(f"Input directory does not exist or is not a directory: {input_dir}")

    output_dir = Path(args.out_dir).resolve() if args.out_dir else input_dir / "trajectory_plots"
    output_dir.mkdir(parents=True, exist_ok=True)

    if not args.csvs_only:
        runs = load_runs(input_dir, args.filename, args.resample, args.min_poses)
        if not runs:
            raise SystemExit(f"No valid {args.filename} files found under {input_dir}")

        summary = platform_mean_summary(Path(args.platform_csv), input_dir)

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

    if not args.no_platform_summary:
        platform_dir = Path(args.platform_dir)
        if not platform_dir.is_absolute():
            platform_dir = (Path.cwd() / platform_dir).resolve()
        summary_out = output_dir / f"platform_{platform_dir.name}_summary"
        generate_platform_summary_plots(platform_dir, summary_out)


if __name__ == "__main__":
    main()
