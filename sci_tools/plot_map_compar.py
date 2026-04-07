#!/usr/bin/env python3
"""
plot_map_compar.py

Visualise map-repeatability results produced by calculate_map_repeatability.py.

Two input formats are supported and detected automatically:

  SIMPLE format  (output-map-test/)
    Files: <algo>_pairwise_IoU_<mode>.csv  (NxN square matrix with run labels)
    One environment / one measurement batch.

  COMBINED format  (output-map-combined/)
    Files: pairwise_IoU_amcl.csv, pairwise_IoU_no_amcl.csv  (or per_pair_IoU.csv)
    Long-form with columns: env, measurement_no, algorithm, mode, run_i, run_j, IoU
    Multiple environments and measurement batches in a single directory.

For every (algorithm, mode) pair [simple] or
    (env, measurement_no, algorithm, mode) slice [combined] the script produces:

  1. Heatmap          – colour-coded NxN pairwise-IoU matrix.
  2. IoU-profile plot – each run's sorted IoU vector (its "map edge") plus
                        the mean IoU profile ("mean edge"), analogous to the
                        trajectory overlay in plot_tf_trajectories.py.
  3. MDS scatter      – 2-D projection via classical MDS of the distance
                        matrix (1 - IoU); each dot is one run, the centroid
                        marks the "mean map".

Across algorithms the script additionally creates:

  4. Mode-comparison boxplot  – AMCL vs no-AMCL distribution side by side
                                for every algorithm.
  5. Mean+CI point plot       – per algorithm/mode, mean IoU ± 95% bootstrap CI.

Combined format adds:

  6. Faceted mean+CI grid     – rows = environments, columns = algorithms,
                                AMCL vs no-AMCL per cell.
  7. Cross-env profiles       – AMCL and no-AMCL mean IoU profiles overlaid
                                across all (env, measurement_no) slices per algo.
  8. Cliff's delta heatmap    – algorithm × (env, measurement) grid showing
                                effect-size direction from summary CSV.

Usage
-----
python plot_map_compar.py <results_dir>
    [--out-dir <output_dir>]
    [--dpi 200]
    [--no-mds]    # skip MDS (slow for large N)
    [--palette <matplotlib colormap name>]

Examples
--------
# simple (per-algorithm NxN matrices)
python plot_map_compar.py sci_tools/output-map-test --out-dir sci_tools/output-map-test/plots

# combined (long-form multi-env)
python plot_map_compar.py sci_tools/output-map-combined --out-dir sci_tools/output-map-combined/plots
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Optional

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

matplotlib.rcParams.update(
    {
        "font.size": 9,
        "axes.titlesize": 10,
        "axes.labelsize": 9,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "legend.fontsize": 8,
        "figure.dpi": 150,
    }
)

LINE_WIDTH = 0.9

# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

ALGO_COLORS = {
    "cartographer": "#d62728",
    "slam_toolbox": "#1f77b4",
    "gmapping": "#2ca02c",
}
MODE_COLORS = {"amcl": "#e07b39", "no_amcl": "#5b8db8"}
MODE_LABELS = {"amcl": "AMCL", "no_amcl": "no AMCL"}


def algo_color(name: str) -> str:
    return ALGO_COLORS.get(name, "#7f7f7f")


def sanitize(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_") or "plot"


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


def upper_triangle_values(matrix: np.ndarray) -> np.ndarray:
    """Return the strictly upper-triangle values of a square matrix."""
    idx = np.triu_indices(len(matrix), k=1)
    vals = matrix[idx]
    return vals[np.isfinite(vals)]


def parse_ci_string(s: str) -> tuple[float, float]:
    """Parse '[lo, hi]' strings from summary CSV."""
    m = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", s)
    if len(m) >= 2:
        return float(m[0]), float(m[1])
    return float("nan"), float("nan")


# ─────────────────────────────────────────────────────────────────────────────
# Data loading  –  format detection + loaders
# ─────────────────────────────────────────────────────────────────────────────

# Key types
#   simple   : (algo, mode)                        → (matrix, labels)
#   combined : (env, measurement_no, algo, mode)   → (matrix, labels)

def detect_format(results_dir: Path) -> str:
    """
    Return 'combined' if the directory contains long-form pairwise IoU CSVs
    (pairwise_IoU_amcl.csv / pairwise_IoU_no_amcl.csv / per_pair_IoU.csv with
    an 'env' column), otherwise return 'simple'.
    """
    candidates = [
        results_dir / "pairwise_IoU_amcl.csv",
        results_dir / "pairwise_IoU_no_amcl.csv",
        results_dir / "per_pair_IoU.csv",
    ]
    for p in candidates:
        if p.exists():
            try:
                header = pd.read_csv(p, nrows=0).columns.tolist()
                if "env" in header and "measurement_no" in header:
                    return "combined"
            except Exception:
                pass
    return "simple"


# ── simple format ─────────────────────────────────────────────────────────────

def discover_matrices(results_dir: Path) -> dict[tuple[str, str], Path]:
    """
    Scan results_dir for files matching <algo>_pairwise_IoU_<mode>.csv.
    Returns {(algo, mode): path}.
    """
    pattern = re.compile(r"^(.+)_pairwise_IoU_(amcl|no_amcl)\.csv$")
    found: dict[tuple[str, str], Path] = {}
    for p in sorted(results_dir.glob("*.csv")):
        m = pattern.match(p.name)
        if m:
            found[(m.group(1), m.group(2))] = p
    return found


def load_matrix(path: Path) -> tuple[np.ndarray, list[str]]:
    """
    Load a pairwise IoU CSV with row/column labels (simple NxN format).
    Returns (matrix NxN, run_labels).
    """
    df = pd.read_csv(path, index_col=0)
    labels = list(df.index)
    M = df.to_numpy(dtype=float)
    np.fill_diagonal(M, 1.0)  # ensure self-similarity = 1
    return M, labels


# ── combined format ───────────────────────────────────────────────────────────

def load_combined(
    results_dir: Path,
) -> tuple[
    dict[tuple[str, str, str, str], tuple[np.ndarray, list[str]]],
    Optional[pd.DataFrame],
]:
    """
    Load a combined long-form directory.

    Reads pairwise_IoU_amcl.csv + pairwise_IoU_no_amcl.csv (or per_pair_IoU.csv
    as fallback), pivots each (env, measurement_no, algo, mode) slice into a
    square IoU matrix, and also returns the summary DataFrame if present.

    Returns
    -------
    matrices : dict keyed by (env, measurement_no, algo, mode)
                value: (NxN ndarray, run_labels list)
    summary  : pd.DataFrame or None
    """
    # prefer the two split files; fall back to unified per_pair_IoU.csv
    amcl_path  = results_dir / "pairwise_IoU_amcl.csv"
    no_path    = results_dir / "pairwise_IoU_no_amcl.csv"
    pair_path  = results_dir / "per_pair_IoU.csv"

    dfs: list[pd.DataFrame] = []
    for p in (amcl_path, no_path, pair_path):
        if p.exists():
            try:
                dfs.append(pd.read_csv(p, dtype={"env": str, "measurement_no": str}))
            except Exception as exc:
                print(f"  [WARN] Could not read {p.name}: {exc}")

    if not dfs:
        return {}, None

    df = pd.concat(dfs, ignore_index=True).drop_duplicates(
        subset=["env", "measurement_no", "algorithm", "mode", "run_i", "run_j"]
    )

    matrices: dict[tuple[str, str, str, str], tuple[np.ndarray, list[str]]] = {}

    for (env, meas, algo, mode), grp in df.groupby(
        ["env", "measurement_no", "algorithm", "mode"], sort=True
    ):
        runs = sorted(set(grp["run_i"]).union(set(grp["run_j"])))
        idx_map = {r: i for i, r in enumerate(runs)}
        N = len(runs)
        M = np.full((N, N), np.nan)
        np.fill_diagonal(M, 1.0)
        for _, row in grp.iterrows():
            i, j = idx_map[row["run_i"]], idx_map[row["run_j"]]
            M[i, j] = row["IoU"]
            M[j, i] = row["IoU"]   # symmetric
        matrices[(str(env), str(meas), str(algo), str(mode))] = (M, runs)

    summary_path = results_dir / "summary_by_algorithm.csv"
    summary: Optional[pd.DataFrame] = None
    if summary_path.exists():
        try:
            summary = pd.read_csv(
                summary_path,
                dtype={"env": str, "measurement_no": str},
            )
        except Exception as exc:
            print(f"  [WARN] Could not read summary_by_algorithm.csv: {exc}")

    return matrices, summary


# ─────────────────────────────────────────────────────────────────────────────
# Analysis helpers
# ─────────────────────────────────────────────────────────────────────────────

def iou_profiles(M: np.ndarray) -> np.ndarray:
    """
    Each run's "IoU profile": for run i, sort its off-diagonal IoU values in
    ascending order.  Returns shape (N, N-1).
    """
    N = len(M)
    profiles = np.zeros((N, N - 1), dtype=float)
    for i in range(N):
        row = np.concatenate([M[i, :i], M[i, i + 1 :]])
        profiles[i] = np.sort(row)
    return profiles


def classical_mds(D: np.ndarray, n_components: int = 2) -> np.ndarray:
    """
    Classical (metric) MDS given a distance matrix D (NxN symmetric, diagonal 0).
    Returns embedding of shape (N, n_components).
    """
    N = len(D)
    D2 = D ** 2
    J = np.eye(N) - np.ones((N, N)) / N
    B = -0.5 * J @ D2 @ J
    vals, vecs = np.linalg.eigh(B)
    # sort descending
    order = np.argsort(vals)[::-1]
    vals = vals[order]
    vecs = vecs[:, order]
    # keep positive eigenvalues only
    pos = vals > 0
    if pos.sum() < n_components:
        n_components = pos.sum()
    coords = vecs[:, :n_components] * np.sqrt(np.maximum(vals[:n_components], 0))
    return coords


# ─────────────────────────────────────────────────────────────────────────────
# Individual (algo, mode) plots
# ─────────────────────────────────────────────────────────────────────────────

def _short_label(label: str) -> str:
    """Strip 'record_<algo>_<mode>_' prefix, keep only run number."""
    m = re.search(r"(\d+)$", label)
    return m.group(1) if m else label


def plot_heatmap(
    M: np.ndarray,
    labels: list[str],
    algo: str,
    mode: str,
    out_dir: Path,
    palette: str,
    dpi: int,
) -> None:
    """Heat-map of the NxN pairwise IoU matrix."""
    short = [_short_label(lb) for lb in labels]
    N = len(M)
    size = max(3.5, N * 0.45)
    fig, ax = plt.subplots(figsize=(size, size * 0.85), dpi=dpi)
    im = ax.imshow(M, vmin=0, vmax=1, cmap=palette, aspect="auto")
    ax.set_xticks(range(N))
    ax.set_yticks(range(N))
    ax.set_xticklabels(short, rotation=90)
    ax.set_yticklabels(short)
    ax.set_title(f"Pairwise IoU – {algo} / {MODE_LABELS.get(mode, mode)}")
    plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label="IoU")
    # annotate cells
    if N <= 12:
        for i in range(N):
            for j in range(N):
                v = M[i, j]
                text_color = "white" if v < 0.5 else "black"
                ax.text(j, i, f"{v:.2f}", ha="center", va="center",
                        fontsize=6, color=text_color)
    fig.tight_layout()
    fig.savefig(out_dir / f"{sanitize(algo)}_{sanitize(mode)}_heatmap.png", dpi=dpi)
    plt.close(fig)
    print(f"  saved heatmap: {algo} / {mode}")


def plot_iou_profiles(
    M: np.ndarray,
    labels: list[str],
    algo: str,
    mode: str,
    out_dir: Path,
    dpi: int,
) -> None:
    """
    IoU-profile plot – each run's sorted pairwise IoU vector ("map edge")
    plus the mean profile ("mean edge").  Analogous to aligned trajectory
    overlays in plot_tf_trajectories.py.
    """
    profiles = iou_profiles(M)  # (N, N-1)
    N, K = profiles.shape
    x = np.arange(K)  # rank axis (0 = most dissimilar neighbour)
    mean_profile = profiles.mean(axis=0)

    color = algo_color(algo)

    fig, ax = plt.subplots(figsize=(5.5, 3.5), dpi=dpi)

    # individual run profiles (map edges)
    for i in range(N):
        ax.plot(
            x,
            profiles[i],
            color=color,
            alpha=0.30,
            linewidth=LINE_WIDTH,
            label="Run profile" if i == 0 else None,
        )

    # mean profile (mean edge)
    ax.plot(
        x,
        mean_profile,
        color="black",
        linestyle="--",
        linewidth=LINE_WIDTH * 1.8,
        label="Mean profile",
        zorder=5,
    )

    # shaded ±1σ band around mean
    std_profile = profiles.std(axis=0)
    ax.fill_between(
        x,
        mean_profile - std_profile,
        mean_profile + std_profile,
        color=color,
        alpha=0.12,
        label="±1σ",
    )

    ax.set_xlim(-0.5, K - 0.5)
    ax.set_ylim(-0.02, 1.05)
    ax.set_xlabel("Neighbour rank (ascending IoU)")
    ax.set_ylabel("Pairwise IoU")
    ax.set_title(f"IoU profiles – {algo} / {MODE_LABELS.get(mode, mode)}")
    ax.grid(True, linestyle="-.", alpha=0.3, linewidth=0.5)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(out_dir / f"{sanitize(algo)}_{sanitize(mode)}_iou_profiles.png", dpi=dpi)
    plt.close(fig)
    print(f"  saved IoU profiles: {algo} / {mode}")


def plot_mds_scatter(
    M: np.ndarray,
    labels: list[str],
    algo: str,
    mode: str,
    out_dir: Path,
    dpi: int,
) -> None:
    """
    Classical MDS scatter of runs in 2-D IoU-distance space.
    Distance = 1 - IoU.  The centroid marks the "mean map" position.
    """
    D = 1.0 - M
    np.fill_diagonal(D, 0.0)

    coords = classical_mds(D, n_components=2)
    if coords.shape[1] < 2:
        print(f"  [WARN] MDS failed for {algo}/{mode} (not enough positive eigenvalues), skipping")
        return

    short_labels = [_short_label(lb) for lb in labels]
    color = algo_color(algo)
    centroid = coords.mean(axis=0)

    fig, ax = plt.subplots(figsize=(5, 4.5), dpi=dpi)

    ax.scatter(coords[:, 0], coords[:, 1], c=color, s=60, alpha=0.85,
               edgecolors="white", linewidths=0.5, zorder=4)

    # label each run
    for i, lbl in enumerate(short_labels):
        ax.annotate(lbl, (coords[i, 0], coords[i, 1]),
                    textcoords="offset points", xytext=(5, 3), fontsize=7, color="dimgray")

    # centroid ("mean map")
    ax.scatter(*centroid, marker="*", s=180, c="black", zorder=5, label="Mean map")

    # draw spokes from centroid to each run
    for i in range(len(coords)):
        ax.plot(
            [centroid[0], coords[i, 0]],
            [centroid[1], coords[i, 1]],
            color="gray",
            alpha=0.30,
            linewidth=0.8,
            zorder=2,
        )

    ax.set_xlabel("MDS dim 1")
    ax.set_ylabel("MDS dim 2")
    ax.set_title(f"MDS map-space – {algo} / {MODE_LABELS.get(mode, mode)}")
    ax.grid(True, linestyle="-.", alpha=0.3, linewidth=0.5)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(out_dir / f"{sanitize(algo)}_{sanitize(mode)}_mds.png", dpi=dpi)
    plt.close(fig)
    print(f"  saved MDS scatter: {algo} / {mode}")


# ─────────────────────────────────────────────────────────────────────────────
# Cross-algorithm / cross-mode plots
# ─────────────────────────────────────────────────────────────────────────────

def plot_mode_comparison_boxplot(
    data: dict[tuple[str, str], np.ndarray],
    out_dir: Path,
    dpi: int,
) -> None:
    """
    Side-by-side box plots: for each algorithm, AMCL vs no-AMCL upper-triangle IoU values.
    One subplot per algorithm.
    """
    algos = sorted({k[0] for k in data})
    n = len(algos)
    if n == 0:
        return

    ncols = min(n, 3)
    nrows = (n + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.0 * ncols, 3.5 * nrows), dpi=dpi,
                             squeeze=False)

    for ax_idx, algo in enumerate(algos):
        ax = axes[ax_idx // ncols][ax_idx % ncols]
        groups = []
        tick_labels = []
        for mode in ("amcl", "no_amcl"):
            key = (algo, mode)
            if key in data:
                groups.append(data[key])
                tick_labels.append(MODE_LABELS[mode])

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
        for patch, mode in zip(bp["boxes"], ("amcl", "no_amcl")):
            patch.set_facecolor(MODE_COLORS[mode])
            patch.set_alpha(0.7)

        ax.set_xticks([1, 2][: len(groups)])
        ax.set_xticklabels(tick_labels)
        ax.set_ylabel("Pairwise IoU")
        ax.set_title(algo)
        ax.grid(True, axis="y", linestyle="-.", alpha=0.3, linewidth=0.5)
        ax.set_ylim(-0.02, 1.05)

    # hide unused subplots
    for idx in range(n, nrows * ncols):
        axes[idx // ncols][idx % ncols].set_visible(False)

    fig.suptitle("Map Repeatability: AMCL vs no-AMCL", fontsize=11, fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_dir / "mode_comparison_boxplot.png", dpi=dpi)
    plt.close(fig)
    print("  saved mode comparison boxplot")


def plot_mean_ci_summary(
    data: dict[tuple[str, str], np.ndarray],
    out_dir: Path,
    dpi: int,
) -> None:
    """
    Mean IoU ± 95 % bootstrap CI, per algorithm and mode.
    Mirrors the summary figure from calculate_map_repeatability.py.
    """
    algos = sorted({k[0] for k in data})
    if not algos:
        return

    x = np.arange(len(algos))
    fig, ax = plt.subplots(figsize=(max(4.0, len(algos) * 1.4), 3.5), dpi=dpi)

    offset = 0.15
    for mode_idx, mode in enumerate(("amcl", "no_amcl")):
        means, lo_err, hi_err = [], [], []
        for algo in algos:
            vals = data.get((algo, mode), np.array([]))
            m = float(np.nanmean(vals)) if vals.size else float("nan")
            lo, hi = bootstrap_ci_mean(vals) if vals.size else (float("nan"), float("nan"))
            means.append(m)
            lo_err.append(max(0.0, m - lo))
            hi_err.append(max(0.0, hi - m))

        means_arr = np.array(means)
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
    ax.set_xticklabels(algos, rotation=15, ha="right")
    ax.set_ylabel("Mean pairwise IoU")
    ax.set_title("Map Repeatability – Mean IoU with 95% bootstrap CI")
    ax.set_ylim(-0.02, 1.05)
    ax.grid(True, axis="y", linestyle="-.", alpha=0.3, linewidth=0.5)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(out_dir / "mean_ci_summary.png", dpi=dpi)
    plt.close(fig)
    print("  saved mean+CI summary")


def plot_all_profiles_overlay(
    data: dict[tuple[str, str], tuple[np.ndarray, list[str]]],
    out_dir: Path,
    dpi: int,
) -> None:
    """
    One figure per algorithm: overlay AMCL and no-AMCL mean IoU profiles
    so shapes can be compared directly.
    """
    algos = sorted({k[0] for k in data})
    for algo in algos:
        fig, ax = plt.subplots(figsize=(5.5, 3.5), dpi=dpi)
        any_plotted = False

        for mode in ("amcl", "no_amcl"):
            key = (algo, mode)
            if key not in data:
                continue
            M, _labels = data[key]
            profiles = iou_profiles(M)
            K = profiles.shape[1]
            x = np.arange(K)
            mean_profile = profiles.mean(axis=0)
            std_profile = profiles.std(axis=0)
            color = MODE_COLORS[mode]

            # individual run "map edges"
            for i in range(len(profiles)):
                ax.plot(x, profiles[i], color=color, alpha=0.20,
                        linewidth=LINE_WIDTH,
                        label=f"{MODE_LABELS[mode]} run" if i == 0 else None)

            # mean edge
            ax.plot(x, mean_profile, color=color, linestyle="--",
                    linewidth=LINE_WIDTH * 2,
                    label=f"{MODE_LABELS[mode]} mean")

            ax.fill_between(x, mean_profile - std_profile,
                            mean_profile + std_profile,
                            color=color, alpha=0.10)
            any_plotted = True

        if not any_plotted:
            plt.close(fig)
            continue

        ax.set_xlim(-0.5, None)
        ax.set_ylim(-0.02, 1.05)
        ax.set_xlabel("Neighbour rank (ascending IoU)")
        ax.set_ylabel("Pairwise IoU")
        ax.set_title(f"IoU profiles overlay – {algo}")
        ax.grid(True, linestyle="-.", alpha=0.3, linewidth=0.5)
        ax.legend(frameon=False)
        fig.tight_layout()
        fig.savefig(out_dir / f"{sanitize(algo)}_profiles_overlay.png", dpi=dpi)
        plt.close(fig)
        print(f"  saved profiles overlay: {algo}")


# ─────────────────────────────────────────────────────────────────────────────
# Combined-format plots  (multi-env / multi-measurement)
# ─────────────────────────────────────────────────────────────────────────────

def _slice_label(env: str, meas: str) -> str:
    return f"{env}-{meas}"


def plot_combined_faceted_mean_ci(
    matrices: dict[tuple[str, str, str, str], tuple[np.ndarray, list[str]]],
    out_dir: Path,
    dpi: int,
) -> None:
    """
    Faceted mean IoU ± 95% CI grid.
    Rows = unique (env, measurement_no) slices, columns = algorithms.
    Within each cell: two points (AMCL orange, no-AMCL blue).
    """
    slices  = sorted({(e, m) for e, m, *_ in matrices})
    algos   = sorted({a for _, _, a, _ in matrices})
    if not slices or not algos:
        return

    n_rows, n_cols = len(slices), len(algos)
    fig, axes = plt.subplots(
        n_rows, n_cols,
        figsize=(max(3.0, n_cols * 2.8), max(2.5, n_rows * 2.2)),
        dpi=dpi,
        squeeze=False,
        sharey=True,
    )

    for r_idx, (env, meas) in enumerate(slices):
        for c_idx, algo in enumerate(algos):
            ax = axes[r_idx][c_idx]
            plotted = False
            for m_idx, mode in enumerate(("amcl", "no_amcl")):
                key = (env, meas, algo, mode)
                if key not in matrices:
                    continue
                M, _ = matrices[key]
                vals = upper_triangle_values(M)
                if vals.size == 0:
                    continue
                mean_v = float(np.nanmean(vals))
                lo, hi = bootstrap_ci_mean(vals)
                ax.errorbar(
                    [m_idx],
                    [mean_v],
                    yerr=[[max(0.0, mean_v - lo)], [max(0.0, hi - mean_v)]],
                    fmt="o",
                    color=MODE_COLORS[mode],
                    capsize=4,
                    markersize=5,
                    linewidth=1.2,
                    label=MODE_LABELS[mode],
                )
                plotted = True

            ax.set_xlim(-0.6, 1.6)
            ax.set_ylim(-0.02, 1.05)
            ax.set_xticks([0, 1])
            ax.set_xticklabels(["AMCL", "no AMCL"], fontsize=7)
            ax.grid(True, axis="y", linestyle="-.", alpha=0.3, linewidth=0.5)
            if r_idx == 0:
                ax.set_title(algo, fontsize=9, fontweight="bold")
            if c_idx == 0:
                ax.set_ylabel(f"{env}-{meas}\nMean IoU", fontsize=8)
            if not plotted:
                ax.text(0.5, 0.5, "no data", transform=ax.transAxes,
                        ha="center", va="center", color="gray", fontsize=8)

    fig.suptitle("Map Repeatability – faceted by env / measurement",
                 fontsize=11, fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_dir / "combined_faceted_mean_ci.png", dpi=dpi)
    plt.close(fig)
    print("  saved combined faceted mean+CI grid")


def plot_combined_cross_env_profiles(
    matrices: dict[tuple[str, str, str, str], tuple[np.ndarray, list[str]]],
    out_dir: Path,
    dpi: int,
) -> None:
    """
    For each algorithm: one subplot per mode.
    Each (env, measurement_no) slice contributes its mean IoU profile as a
    coloured line.  Lets you see how map-edge shape varies across environments.
    """
    algos  = sorted({a for _, _, a, _ in matrices})
    slices = sorted({(e, m) for e, m, *_ in matrices})
    if not algos or not slices:
        return

    # colour by slice
    cmap = plt.get_cmap("tab10")
    slice_colors = {s: cmap(i % 10) for i, s in enumerate(slices)}

    for algo in algos:
        fig, axes = plt.subplots(
            1, 2,
            figsize=(10.0, 3.8),
            dpi=dpi,
            sharey=True,
        )
        for ax, mode in zip(axes, ("amcl", "no_amcl")):
            any_plotted = False
            for env, meas in slices:
                key = (env, meas, algo, mode)
                if key not in matrices:
                    continue
                M, _ = matrices[key]
                if len(M) < 2:
                    continue
                profiles = iou_profiles(M)
                K = profiles.shape[1]
                x = np.arange(K)
                color = slice_colors[(env, meas)]
                label = _slice_label(env, meas)

                # individual run map edges (faint)
                for i in range(len(profiles)):
                    ax.plot(x, profiles[i], color=color, alpha=0.15,
                            linewidth=LINE_WIDTH * 0.8)

                # mean edge (solid, labelled)
                mean_profile = profiles.mean(axis=0)
                ax.plot(x, mean_profile, color=color, linewidth=LINE_WIDTH * 2,
                        label=label)
                any_plotted = True

            ax.set_xlim(-0.5, None)
            ax.set_ylim(-0.02, 1.05)
            ax.set_xlabel("Neighbour rank (ascending IoU)")
            ax.set_ylabel("Pairwise IoU")
            ax.set_title(f"{MODE_LABELS.get(mode, mode)}")
            ax.grid(True, linestyle="-.", alpha=0.3, linewidth=0.5)
            if any_plotted:
                ax.legend(frameon=False, fontsize=7,
                          title="env-meas", title_fontsize=7)

        fig.suptitle(f"IoU profiles across environments – {algo}",
                     fontsize=11, fontweight="bold")
        fig.tight_layout()
        fig.savefig(out_dir / f"{sanitize(algo)}_cross_env_profiles.png", dpi=dpi)
        plt.close(fig)
        print(f"  saved cross-env profiles: {algo}")


def plot_combined_cliffs_delta_heatmap(
    summary: pd.DataFrame,
    out_dir: Path,
    dpi: int,
) -> None:
    """
    Heatmap of Cliff's delta (AMCL vs no-AMCL IoU).
    Rows = algorithms, columns = (env, measurement_no) slices.
    Positive (orange) = AMCL higher; negative (blue) = no-AMCL higher.
    """
    required = {"env", "measurement_no", "algorithm", "cliffs_delta"}
    if not required.issubset(summary.columns):
        print("  [WARN] summary CSV missing columns for Cliff's delta heatmap, skipping")
        return

    summary = summary.copy()
    summary["slice"] = summary["env"].astype(str) + "-" + summary["measurement_no"].astype(str)
    algos  = sorted(summary["algorithm"].unique())
    slices = sorted(summary["slice"].unique())

    grid = pd.DataFrame(np.nan, index=algos, columns=slices)
    for _, row in summary.iterrows():
        grid.loc[row["algorithm"], row["slice"]] = float(row["cliffs_delta"])

    data = grid.to_numpy(dtype=float)
    n_rows, n_cols = data.shape
    fig, ax = plt.subplots(
        figsize=(max(4.0, n_cols * 0.9), max(2.5, n_rows * 0.7)),
        dpi=dpi,
    )
    im = ax.imshow(data, vmin=-1, vmax=1, cmap="RdBu_r", aspect="auto")
    ax.set_xticks(range(n_cols))
    ax.set_yticks(range(n_rows))
    ax.set_xticklabels(slices, rotation=45, ha="right", fontsize=8)
    ax.set_yticklabels(algos, fontsize=8)
    ax.set_xlabel("env – measurement")
    ax.set_ylabel("Algorithm")
    ax.set_title("Cliff's delta: AMCL vs no-AMCL  (+ = AMCL higher IoU)")
    plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label="Cliff's δ")

    for i in range(n_rows):
        for j in range(n_cols):
            v = data[i, j]
            if np.isfinite(v):
                ax.text(j, i, f"{v:+.2f}", ha="center", va="center",
                        fontsize=7, color="white" if abs(v) > 0.5 else "black")

    fig.tight_layout()
    fig.savefig(out_dir / "combined_cliffs_delta_heatmap.png", dpi=dpi)
    plt.close(fig)
    print("  saved Cliff's delta heatmap")


def plot_combined_algo_mean_ci_by_slice(
    matrices: dict[tuple[str, str, str, str], tuple[np.ndarray, list[str]]],
    out_dir: Path,
    dpi: int,
) -> None:
    """
    One figure per algorithm: mean IoU ± 95% CI for every (env, measurement_no)
    slice, with AMCL and no-AMCL shown side by side.  Gives an at-a-glance
    overview of repeatability stability across measurement campaigns.
    """
    algos  = sorted({a for _, _, a, _ in matrices})
    slices = sorted({(e, m) for e, m, *_ in matrices})
    if not algos or not slices:
        return

    x = np.arange(len(slices))
    slice_labels = [_slice_label(e, m) for e, m in slices]

    for algo in algos:
        fig, ax = plt.subplots(
            figsize=(max(4.0, len(slices) * 1.3), 3.5), dpi=dpi
        )
        offset = 0.14
        for m_idx, mode in enumerate(("amcl", "no_amcl")):
            means, lo_err, hi_err = [], [], []
            for env, meas in slices:
                key = (env, meas, algo, mode)
                if key not in matrices:
                    means.append(float("nan"))
                    lo_err.append(float("nan"))
                    hi_err.append(float("nan"))
                    continue
                M, _ = matrices[key]
                vals = upper_triangle_values(M)
                mean_v = float(np.nanmean(vals)) if vals.size else float("nan")
                lo, hi = bootstrap_ci_mean(vals) if vals.size else (float("nan"), float("nan"))
                means.append(mean_v)
                lo_err.append(max(0.0, mean_v - lo))
                hi_err.append(max(0.0, hi - mean_v))

            means_arr = np.array(means)
            yerr = np.vstack([lo_err, hi_err])
            valid = np.isfinite(means_arr)
            xs = x + offset * (m_idx - 0.5)
            ax.errorbar(
                xs[valid], means_arr[valid],
                yerr=yerr[:, valid],
                fmt="o",
                color=MODE_COLORS[mode],
                label=MODE_LABELS[mode],
                capsize=4, linewidth=1.2, markersize=5,
            )

        ax.set_xticks(x)
        ax.set_xticklabels(slice_labels, rotation=30, ha="right", fontsize=8)
        ax.set_ylabel("Mean pairwise IoU")
        ax.set_title(f"Mean IoU by measurement slice – {algo}")
        ax.set_ylim(-0.02, 1.05)
        ax.grid(True, axis="y", linestyle="-.", alpha=0.3, linewidth=0.5)
        ax.legend(frameon=False)
        fig.tight_layout()
        fig.savefig(out_dir / f"{sanitize(algo)}_mean_ci_by_slice.png", dpi=dpi)
        plt.close(fig)
        print(f"  saved mean+CI by slice: {algo}")


# ─────────────────────────────────────────────────────────────────────────────
# Summary table printout
# ─────────────────────────────────────────────────────────────────────────────

def print_summary_table(
    data: dict[tuple, np.ndarray],
    key_names: Optional[list[str]] = None,
) -> None:
    """
    Print a summary table.  key_names controls the column headers for the
    tuple key fields (defaults to ["algorithm", "mode"]).
    """
    if key_names is None:
        key_names = ["algorithm", "mode"]
    rows = []
    for key, vals in sorted(data.items()):
        if not isinstance(key, tuple):
            key = (key,)
        m = float(np.nanmean(vals)) if vals.size else float("nan")
        lo, hi = bootstrap_ci_mean(vals) if vals.size else (float("nan"), float("nan"))
        row = {}
        for name, val in zip(key_names, key):
            row[name] = MODE_LABELS.get(str(val), str(val)) if name == "mode" else val
        row["n_pairs"] = int(vals.size)
        row["mean_IoU"] = f"{m:.4f}"
        row["CI95"] = f"[{lo:.4f}, {hi:.4f}]"
        rows.append(row)
    if rows:
        print("\n" + pd.DataFrame(rows).to_string(index=False))


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def _run_simple_pipeline(
    results_dir: Path,
    out_dir: Path,
    args: argparse.Namespace,
) -> None:
    """Pipeline for the simple (per-algorithm NxN matrix) format."""
    matrix_paths = discover_matrices(results_dir)
    if not matrix_paths:
        sys.exit(
            "No pairwise IoU CSV files found.\n"
            "Expected pattern: <algo>_pairwise_IoU_<mode>.csv"
        )

    print(f"\nFound {len(matrix_paths)} matrix file(s):")
    for key, path in sorted(matrix_paths.items()):
        print(f"  {key[0]} / {key[1]}  →  {path.name}")

    raw_matrices: dict[tuple[str, str], tuple[np.ndarray, list[str]]] = {}
    upper_vals:   dict[tuple[str, str], np.ndarray] = {}

    for key, path in sorted(matrix_paths.items()):
        try:
            M, labels = load_matrix(path)
            raw_matrices[key] = (M, labels)
            upper_vals[key] = upper_triangle_values(M)
        except Exception as exc:
            print(f"  [WARN] Could not load {path.name}: {exc}")

    if not raw_matrices:
        sys.exit("No matrices loaded successfully.")

    print("\nGenerating per-condition plots …")
    for (algo, mode), (M, labels) in sorted(raw_matrices.items()):
        print(f"\n  [{algo} / {mode}]  N={len(labels)} runs")
        plot_heatmap(M, labels, algo, mode, out_dir, args.palette, args.dpi)
        plot_iou_profiles(M, labels, algo, mode, out_dir, args.dpi)
        if not args.no_mds:
            plot_mds_scatter(M, labels, algo, mode, out_dir, args.dpi)

    print("\nGenerating summary plots …")
    plot_all_profiles_overlay(raw_matrices, out_dir, args.dpi)
    plot_mode_comparison_boxplot(upper_vals, out_dir, args.dpi)
    plot_mean_ci_summary(upper_vals, out_dir, args.dpi)

    print_summary_table(upper_vals)


def _run_combined_pipeline(
    results_dir: Path,
    out_dir: Path,
    args: argparse.Namespace,
) -> None:
    """Pipeline for the combined long-form multi-env/multi-measurement format."""
    matrices, summary = load_combined(results_dir)
    if not matrices:
        sys.exit("No pairwise IoU data loaded from combined format.")

    slices = sorted({(e, m) for e, m, *_ in matrices})
    algos  = sorted({a for _, _, a, _ in matrices})
    print(f"\nLoaded {len(matrices)} (env, meas, algo, mode) slices")
    print(f"  Environments × measurements : {slices}")
    print(f"  Algorithms                  : {algos}")

    # ── per-slice per-condition plots (heatmap, profiles, MDS) ────────────
    print("\nGenerating per-condition plots …")
    for (env, meas, algo, mode), (M, labels) in sorted(matrices.items()):
        slug = f"{sanitize(env)}_{sanitize(meas)}"
        slice_out = out_dir / slug
        slice_out.mkdir(parents=True, exist_ok=True)
        print(f"\n  [{env}/{meas}  {algo} / {mode}]  N={len(labels)} runs")
        plot_heatmap(M, labels, algo, mode, slice_out, args.palette, args.dpi)
        plot_iou_profiles(M, labels, algo, mode, slice_out, args.dpi)
        if not args.no_mds:
            plot_mds_scatter(M, labels, algo, mode, slice_out, args.dpi)

    # ── per-slice summary plots (profiles overlay, boxplot, mean+CI) ──────
    print("\nGenerating per-slice summary plots …")
    for env, meas in slices:
        slug = f"{sanitize(env)}_{sanitize(meas)}"
        slice_out = out_dir / slug
        slice_out.mkdir(parents=True, exist_ok=True)

        slice_matrices: dict[tuple[str, str], tuple[np.ndarray, list[str]]] = {
            (a, mo): v
            for (e, ms, a, mo), v in matrices.items()
            if e == env and ms == meas
        }
        slice_upper: dict[tuple[str, str], np.ndarray] = {
            k: upper_triangle_values(v[0]) for k, v in slice_matrices.items()
        }

        plot_all_profiles_overlay(slice_matrices, slice_out, args.dpi)
        plot_mode_comparison_boxplot(slice_upper, slice_out, args.dpi)
        plot_mean_ci_summary(slice_upper, slice_out, args.dpi)

    # ── combined summary plots ─────────────────────────────────────────────
    print("\nGenerating combined summary plots …")
    plot_combined_faceted_mean_ci(matrices, out_dir, args.dpi)
    plot_combined_cross_env_profiles(matrices, out_dir, args.dpi)
    plot_combined_algo_mean_ci_by_slice(matrices, out_dir, args.dpi)
    if summary is not None:
        plot_combined_cliffs_delta_heatmap(summary, out_dir, args.dpi)

    # ── console summary ───────────────────────────────────────────────────
    upper_vals_combined: dict[tuple, np.ndarray] = {
        key: upper_triangle_values(M)
        for key, (M, _) in matrices.items()
    }
    print_summary_table(
        upper_vals_combined,
        key_names=["env", "measurement_no", "algorithm", "mode"],
    )


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Plot map-repeatability comparison figures from pairwise IoU matrices.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("results_dir", help="Directory containing pairwise IoU CSV files")
    ap.add_argument("--out-dir", default=None,
                    help="Output directory (default: <results_dir>/plots)")
    ap.add_argument("--dpi", type=int, default=200, help="Figure DPI (default: 200)")
    ap.add_argument("--no-mds", action="store_true", help="Skip MDS scatter plots")
    ap.add_argument("--palette", default="viridis",
                    help="Matplotlib colormap for heatmaps (default: viridis)")
    args = ap.parse_args()

    results_dir = Path(args.results_dir).resolve()
    if not results_dir.exists() or not results_dir.is_dir():
        sys.exit(f"results_dir does not exist or is not a directory: {results_dir}")

    out_dir = Path(args.out_dir).resolve() if args.out_dir else results_dir / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)

    fmt = detect_format(results_dir)
    print(f"Results dir : {results_dir}")
    print(f"Output dir  : {out_dir}")
    print(f"Format      : {fmt}")

    if fmt == "combined":
        _run_combined_pipeline(results_dir, out_dir, args)
    else:
        _run_simple_pipeline(results_dir, out_dir, args)

    print(f"\nAll plots saved to: {out_dir}")


if __name__ == "__main__":
    main()
