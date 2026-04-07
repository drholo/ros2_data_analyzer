#!/usr/bin/env python3
"""
plot_map_compar.py

Visualise map-repeatability results produced by calculate_map_repeatability.py.

Input: a directory containing pairwise IoU CSV matrices and (optionally)
       summary_by_algorithm.csv, in the same layout as output-map-test/.

For every (algorithm, mode) pair the script produces:

  1. Heatmap          – colour-coded NxN pairwise-IoU matrix.
  2. IoU-profile plot – each run's sorted IoU vector (its "map edge") plus
                        the mean IoU profile ("mean edge"), analogous to the
                        trajectory overlay in plot_tf_trajectories.py.
  3. MDS scatter      – 2-D projection via classical MDS of the distance
                        matrix (1 - IoU); each dot is one run, the centroid
                        marks the "mean map".

Across algorithms the script additionally creates:

  4. Mode-comparison boxplot  – AMCL vs no-AMCL distribution side by side
                                for every algorithm (upper-triangle IoU values).
  5. Mean+CI bar / point plot – per algorithm, per mode, mean IoU with 95 %
                                bootstrap CI (mirrors the summary figure from
                                calculate_map_repeatability.py but regenerated
                                from raw matrices so the script is self-contained).

Usage
-----
python plot_map_compar.py <results_dir>
    [--out-dir <output_dir>]
    [--summary  summary_by_algorithm.csv]  # optional, auto-detected
    [--dpi 200]
    [--no-mds]    # skip MDS (slow for large N)
    [--palette <matplotlib colormap name>]

Examples
--------
python plot_map_compar.py sci_tools/output-map-test --out-dir sci_tools/output-map-test/plots
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
# Data loading
# ─────────────────────────────────────────────────────────────────────────────

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
    Load a pairwise IoU CSV with row/column labels.
    Returns (matrix NxN, run_labels).
    """
    df = pd.read_csv(path, index_col=0)
    labels = list(df.index)
    M = df.to_numpy(dtype=float)
    np.fill_diagonal(M, 1.0)  # ensure self-similarity = 1
    return M, labels


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
# Summary table printout
# ─────────────────────────────────────────────────────────────────────────────

def print_summary_table(
    data: dict[tuple[str, str], np.ndarray],
) -> None:
    rows = []
    for (algo, mode), vals in sorted(data.items()):
        m = float(np.nanmean(vals)) if vals.size else float("nan")
        lo, hi = bootstrap_ci_mean(vals) if vals.size else (float("nan"), float("nan"))
        rows.append({
            "algorithm": algo,
            "mode": MODE_LABELS.get(mode, mode),
            "n_pairs": int(vals.size),
            "mean_IoU": f"{m:.4f}",
            "CI95": f"[{lo:.4f}, {hi:.4f}]",
        })
    if rows:
        print("\n" + pd.DataFrame(rows).to_string(index=False))


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

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
    print(f"Results dir : {results_dir}")
    print(f"Output dir  : {out_dir}")

    matrix_paths = discover_matrices(results_dir)
    if not matrix_paths:
        sys.exit("No pairwise IoU CSV files found.  Expected pattern: <algo>_pairwise_IoU_<mode>.csv")

    print(f"\nFound {len(matrix_paths)} matrix file(s):")
    for key, path in sorted(matrix_paths.items()):
        print(f"  {key[0]} / {key[1]}  →  {path.name}")

    # ── load all matrices ──────────────────────────────────────────────────
    raw_matrices: dict[tuple[str, str], tuple[np.ndarray, list[str]]] = {}
    upper_vals: dict[tuple[str, str], np.ndarray] = {}

    for key, path in sorted(matrix_paths.items()):
        try:
            M, labels = load_matrix(path)
            raw_matrices[key] = (M, labels)
            upper_vals[key] = upper_triangle_values(M)
        except Exception as exc:
            print(f"  [WARN] Could not load {path.name}: {exc}")

    if not raw_matrices:
        sys.exit("No matrices loaded successfully.")

    # ── per-(algo, mode) plots ─────────────────────────────────────────────
    print("\nGenerating per-condition plots …")
    for (algo, mode), (M, labels) in sorted(raw_matrices.items()):
        print(f"\n  [{algo} / {mode}]  N={len(labels)} runs")
        plot_heatmap(M, labels, algo, mode, out_dir, args.palette, args.dpi)
        plot_iou_profiles(M, labels, algo, mode, out_dir, args.dpi)
        if not args.no_mds:
            plot_mds_scatter(M, labels, algo, mode, out_dir, args.dpi)

    # ── cross-algorithm / cross-mode plots ────────────────────────────────
    print("\nGenerating summary plots …")
    plot_all_profiles_overlay(raw_matrices, out_dir, args.dpi)
    plot_mode_comparison_boxplot(upper_vals, out_dir, args.dpi)
    plot_mean_ci_summary(upper_vals, out_dir, args.dpi)

    # ── console summary ───────────────────────────────────────────────────
    print_summary_table(upper_vals)

    print(f"\nAll plots saved to: {out_dir}")


if __name__ == "__main__":
    main()
