#!/usr/bin/env python3
"""
slam_amcl_map_repeatability.py

2D occupancy map repeatability for SLAM algorithms with AMCL vs no_AMCL.

Input folder structure:
results/
  record_<algorithm>_<amcl|no_amcl>_<run_no>/
     ... <MAP_YAML_NAME>   (e.g., raw_map.yaml or formatted_map.yaml)
     ... corresponding .pgm referenced by YAML "image:" field

Core steps:
1) Load YAML metadata (resolution/origin/thresholds/negate) and the PGM
2) Convert PGM to occupancy prob p(u,v) and then trinary map: occ/free/unk
3) Create:
   - occ mask (binary)
   - known mask (not unknown)
4) Align map A to map B in SE(2):
   - scan rotations in [--rot-min, --rot-max] with --rot-step
   - for each rotation: find best translation using FFT cross-correlation on occ masks
   - score by IoU_occ computed on known∩known
   - optional refinement around best angle
5) Compute pairwise IoU distributions within each (algorithm, mode)
6) Compare AMCL vs no_AMCL distributions with bootstrap CI, Welch, Mann–Whitney, Cliff's delta
7) Save CSV summaries and plots.

Dependencies: numpy, pandas, matplotlib
Optional but recommended: PyYAML, scipy (for rotate and FFT conveniences)
"""

from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, List, Dict

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Optional deps
try:
    import yaml  # PyYAML
    YAML_OK = True
except Exception:
    YAML_OK = False

try:
    from scipy import ndimage
    SCIPY_OK = True
except Exception:
    SCIPY_OK = False

try:
    from PIL import Image
    PIL_OK = True
except Exception:
    PIL_OK = False


# -------------------- parsing record folder names --------------------

def parse_record_folder(name: str):
    """
    Parse folder names like:
      record_<algorithm>_amcl_<run_no>
      record_<algorithm>_no_amcl_<run_no>
    Algorithm may contain underscores.

    Returns (algo, mode, run_no) or None.
    """
    if not name.startswith("record_"):
        return None
    rest = name[len("record_"):]
    parts = rest.split("_")
    if len(parts) < 3:
        return None

    run_no = parts[-1]
    if not run_no.isdigit():
        return None

    # IMPORTANT: check no_amcl first
    if len(parts) >= 4 and parts[-3] == "no" and parts[-2] == "amcl":
        mode = "no_amcl"
        algo_parts = parts[:-3]
    elif parts[-2] == "amcl":
        mode = "amcl"
        algo_parts = parts[:-2]
    else:
        return None

    algo = "_".join(algo_parts)
    if not algo:
        return None
    return algo, mode, run_no


# -------------------- map loading --------------------

@dataclass
class GridMap:
    name: str
    occ: np.ndarray      # bool HxW occupied mask
    known: np.ndarray    # bool HxW known (not unknown) mask
    meta: dict
    pgm_path: Path


def _read_pgm_gray(pgm_path: Path) -> np.ndarray:
    if not PIL_OK:
        raise RuntimeError("Pillow (PIL) not available. Install: pip install pillow")
    img = Image.open(pgm_path)
    arr = np.array(img)
    if arr.ndim == 3:
        arr = arr[..., 0]
    return arr.astype(np.uint8)


def _yaml_load(path: Path) -> dict:
    if not YAML_OK:
        raise RuntimeError("PyYAML not available. Install: pip install pyyaml")
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def load_ros_map(yaml_path: Path) -> GridMap:
    """
    Load ROS map_server YAML + referenced PGM, then build occ/known masks.
    YAML keys typically:
      image, resolution, origin, negate, occupied_thresh, free_thresh
    """
    meta = _yaml_load(yaml_path)

    # Resolve PGM path
    image_rel = meta.get("image", None)
    if image_rel is None:
        raise ValueError(f"{yaml_path}: missing 'image' field")
    pgm_path = (yaml_path.parent / image_rel).resolve()
    if not pgm_path.exists():
        # try relative without resolve edge cases
        pgm_path = (yaml_path.parent / Path(image_rel).name)
    if not pgm_path.exists():
        raise FileNotFoundError(f"{yaml_path}: referenced image not found: {image_rel}")

    I = _read_pgm_gray(pgm_path)  # 0..255
    negate = int(meta.get("negate", 0))
    occ_th = float(meta.get("occupied_thresh", 0.65))
    free_th = float(meta.get("free_thresh", 0.25))

    # Convert intensity to occupancy probability (common ROS convention)
    if negate == 0:
        p = 1.0 - (I.astype(np.float32) / 255.0)
    else:
        p = I.astype(np.float32) / 255.0

    occ = p >= occ_th
    free = p <= free_th
    known = occ | free  # unknown is everything else

    return GridMap(
        name=yaml_path.stem,
        occ=occ.astype(bool),
        known=known.astype(bool),
        meta=meta,
        pgm_path=pgm_path
    )


def find_yaml_in_record_folder(folder: Path, yaml_name: str) -> Path:
    matches = list(folder.rglob(yaml_name))
    if not matches:
        raise FileNotFoundError(f"{yaml_name} not found under {folder}")
    # deterministic pick
    matches = sorted(matches)
    return matches[0]


# -------------------- alignment + metrics --------------------

def iou_occ(occA: np.ndarray, knownA: np.ndarray,
            occB: np.ndarray, knownB: np.ndarray) -> float:
    """
    IoU of occupied cells computed on mask Omega = knownA & knownB.
    """
    Omega = knownA & knownB
    if Omega.sum() == 0:
        return float("nan")
    A = occA & Omega
    B = occB & Omega
    inter = (A & B).sum()
    union = (A | B).sum()
    if union == 0:
        # no occupied cells in overlap region; define IoU as 1 if both empty else 0
        return 1.0 if inter == 0 else 0.0
    return float(inter / union)


def _center_pad_to_shape(mask: np.ndarray, target_shape: Tuple[int, int]) -> np.ndarray:
    """
    Center-pad a 2D mask to target_shape with zeros.
    """
    h, w = mask.shape
    th, tw = target_shape
    if h == th and w == tw:
        return mask
    if h > th or w > tw:
        raise ValueError(f"target_shape {target_shape} must be >= source shape {(h, w)}")

    out = np.zeros((th, tw), dtype=mask.dtype)
    y0 = (th - h) // 2
    x0 = (tw - w) // 2
    out[y0:y0 + h, x0:x0 + w] = mask
    return out


def _prepare_pair_for_alignment(A_occ: np.ndarray, A_known: np.ndarray,
                                B_occ: np.ndarray, B_known: np.ndarray):
    """
    Center-pad A/B occupancy and known masks to a common canvas.
    This allows alignment and IoU when original map sizes differ.
    """
    th = max(A_occ.shape[0], B_occ.shape[0])
    tw = max(A_occ.shape[1], B_occ.shape[1])
    target = (th, tw)

    return (
        _center_pad_to_shape(A_occ, target),
        _center_pad_to_shape(A_known, target),
        _center_pad_to_shape(B_occ, target),
        _center_pad_to_shape(B_known, target),
    )


def _fft_cross_correlation_shift(A: np.ndarray, B: np.ndarray) -> Tuple[int, int]:
    """
    Find integer shift (dy, dx) that maximizes cross-correlation between A and B.
    A, B: float arrays same shape.
    Uses FFT-based circular correlation; we then interpret peak as translation.
    """
    if A.shape != B.shape:
        raise ValueError(f"cross-correlation requires same shape, got {A.shape} vs {B.shape}")

    # FFT correlation: corr = ifft(fft(A) * conj(fft(B)))
    FA = np.fft.rfft2(A)
    FB = np.fft.rfft2(B)
    corr = np.fft.irfft2(FA * np.conj(FB), s=A.shape)

    peak = np.unravel_index(np.argmax(corr), corr.shape)
    dy, dx = peak

    # Convert circular shift to signed shift
    H, W = A.shape
    if dy > H // 2:
        dy -= H
    if dx > W // 2:
        dx -= W
    return int(dy), int(dx)


def _shift_mask(mask: np.ndarray, dy: int, dx: int) -> np.ndarray:
    """
    Shift mask by (dy, dx) with zero-fill (not wrap).
    """
    H, W = mask.shape
    out = np.zeros_like(mask, dtype=mask.dtype)

    y0_src = max(0, -dy)
    y1_src = min(H, H - dy) if dy >= 0 else H
    x0_src = max(0, -dx)
    x1_src = min(W, W - dx) if dx >= 0 else W

    y0_dst = max(0, dy)
    y1_dst = y0_dst + (y1_src - y0_src)
    x0_dst = max(0, dx)
    x1_dst = x0_dst + (x1_src - x0_src)

    if (y1_src - y0_src) <= 0 or (x1_src - x0_src) <= 0:
        return out

    out[y0_dst:y1_dst, x0_dst:x1_dst] = mask[y0_src:y1_src, x0_src:x1_src]
    return out


def _rotate_mask(mask: np.ndarray, angle_deg: float) -> np.ndarray:
    """
    Rotate around center, keep same shape, nearest-neighbor.
    Requires scipy.ndimage.
    """
    if not SCIPY_OK:
        raise RuntimeError("scipy is required for rotation alignment. Install: pip install scipy")
    # ndimage.rotate uses degrees, positive is CCW; reshape=False keeps size
    return ndimage.rotate(mask.astype(np.float32), angle=angle_deg, reshape=False, order=0, mode="constant", cval=0.0) > 0.5


def align_A_to_B_by_search(A_occ, A_known, B_occ, B_known,
                           rot_min: float, rot_max: float, rot_step: float,
                           refine: bool = True) -> Tuple[np.ndarray, np.ndarray, dict]:
    """
    Align A to B by scanning rotations and finding translation by FFT correlation.

    Returns aligned (occ, known) of A and params dict: angle_deg, dy, dx, score.
    """
    best = {"score": -1.0, "angle_deg": 0.0, "dy": 0, "dx": 0}

    def evaluate(angle_deg: float):
        Aor = _rotate_mask(A_occ, angle_deg)
        Akr = _rotate_mask(A_known, angle_deg)

        # use float masks for correlation (occupied only)
        dy, dx = _fft_cross_correlation_shift(Aor.astype(np.float32), B_occ.astype(np.float32))

        Aos = _shift_mask(Aor, dy, dx)
        Aks = _shift_mask(Akr, dy, dx)

        score = iou_occ(Aos, Aks, B_occ, B_known)
        return score, angle_deg, dy, dx, Aos, Aks

    angles = np.arange(rot_min, rot_max + 1e-12, rot_step)
    for ang in angles:
        score, angle_deg, dy, dx, Aos, Aks = evaluate(float(ang))
        if np.isfinite(score) and score > best["score"]:
            best.update({"score": float(score), "angle_deg": float(angle_deg), "dy": int(dy), "dx": int(dx)})
            best_occ, best_known = Aos, Aks

    # refine around best angle
    if refine and rot_step > 0.05:
        ang0 = best["angle_deg"]
        fine_step = rot_step / 5.0
        fine_angles = np.arange(ang0 - 2*rot_step, ang0 + 2*rot_step + 1e-12, fine_step)
        for ang in fine_angles:
            score, angle_deg, dy, dx, Aos, Aks = evaluate(float(ang))
            if np.isfinite(score) and score > best["score"]:
                best.update({"score": float(score), "angle_deg": float(angle_deg), "dy": int(dy), "dx": int(dx)})
                best_occ, best_known = Aos, Aks

    return best_occ, best_known, best


# -------------------- statistics --------------------

def bootstrap_ci_mean(x: np.ndarray, n_boot: int = 2000, alpha: float = 0.05, seed: int = 0) -> Tuple[float, float]:
    rng = np.random.default_rng(seed)
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if len(x) == 0:
        return (float("nan"), float("nan"))
    boots = np.array([rng.choice(x, size=len(x), replace=True).mean() for _ in range(n_boot)])
    return (float(np.quantile(boots, alpha/2)), float(np.quantile(boots, 1-alpha/2)))


def cliffs_delta(a: np.ndarray, b: np.ndarray) -> float:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    a = a[np.isfinite(a)]
    b = b[np.isfinite(b)]
    if len(a) == 0 or len(b) == 0:
        return float("nan")
    gt = 0
    lt = 0
    for x in a:
        gt += np.sum(x > b)
        lt += np.sum(x < b)
    return float((gt - lt) / (len(a) * len(b)))


# -------------------- pipeline --------------------

@dataclass
class RunMap:
    algo: str
    mode: str
    run_id: str
    folder: str
    gm: GridMap


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", required=True, help="Path to results/ folder")
    ap.add_argument("--out", default="map_repeatability_out", help="Output folder")
    ap.add_argument("--map-yaml", required=True, help="Map YAML filename to use (e.g., raw_map.yaml or formatted_map.yaml)")
    ap.add_argument("--min-runs", type=int, default=2, help="Min runs per condition per algorithm")

    # alignment parameters
    ap.add_argument("--rot-min", type=float, default=-10.0, help="Min rotation (deg) when aligning A->B")
    ap.add_argument("--rot-max", type=float, default=10.0, help="Max rotation (deg) when aligning A->B")
    ap.add_argument("--rot-step", type=float, default=0.5, help="Rotation step (deg)")
    ap.add_argument("--no-refine", action="store_true", help="Disable refinement around best angle")

    # IOU details
    ap.add_argument("--save-aligned-example", action="store_true",
                    help="Save one aligned example per algorithm for visual sanity-check")

    args = ap.parse_args()

    root = Path(args.results)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    # Load all runs
    all_runs: List[RunMap] = []
    for rec in sorted(root.glob("record_*")):
        if not rec.is_dir():
            continue
        parsed = parse_record_folder(rec.name)
        if not parsed:
            continue
        algo, mode, run_no = parsed
        try:
            yaml_path = find_yaml_in_record_folder(rec, args.map_yaml)
            gm = load_ros_map(yaml_path)
            all_runs.append(RunMap(algo=algo, mode=mode, run_id=run_no, folder=rec.name, gm=gm))
        except Exception as e:
            print(f"[WARN] Skipping {rec.name}: {e}")

    if not all_runs:
        raise SystemExit("No valid maps found. Check --map-yaml and folder structure.")

    # Report counts
    from collections import Counter
    print("Parsed counts:", Counter((r.algo, r.mode) for r in all_runs))

    algos = sorted(set(r.algo for r in all_runs))
    summary_rows = []

    # Optional SciPy stats
    try:
        from scipy.stats import ttest_ind, mannwhitneyu
        STATS_OK = True
    except Exception:
        STATS_OK = False

    def pairwise_scores(runs: List[RunMap]) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        Return pairwise IoU (upper triangle) and full matrix (optional).
        """
        n = len(runs)
        M = np.full((n, n), np.nan, dtype=float)
        for i in range(n):
            M[i, i] = 1.0
        for i in range(n):
            for j in range(i+1, n):
                A = runs[i].gm
                B = runs[j].gm

                Aocc, Aknown, Bocc, Bknown = _prepare_pair_for_alignment(
                    A.occ, A.known, B.occ, B.known
                )

                Aocc_al, Aknown_al, params = align_A_to_B_by_search(
                    Aocc, Aknown, Bocc, Bknown,
                    rot_min=args.rot_min, rot_max=args.rot_max, rot_step=args.rot_step,
                    refine=(not args.no_refine)
                )
                s = iou_occ(Aocc_al, Aknown_al, Bocc, Bknown)
                M[i, j] = s
                M[j, i] = s  # symmetric score

        iu = np.triu_indices(n, k=1)
        vals = M[iu]
        vals = vals[np.isfinite(vals)]
        return vals, M

    for algo in algos:
        runs_algo = [r for r in all_runs if r.algo == algo]
        amcl_runs = [r for r in runs_algo if r.mode == "amcl"]
        no_runs   = [r for r in runs_algo if r.mode == "no_amcl"]

        if len(amcl_runs) < args.min_runs or len(no_runs) < args.min_runs:
            print(f"[WARN] {algo}: insufficient runs (amcl={len(amcl_runs)}, no_amcl={len(no_runs)}); skipping")
            continue

        # Pairwise distributions
        amcl_vals, amcl_M = pairwise_scores(amcl_runs)
        no_vals, no_M     = pairwise_scores(no_runs)

        # Save matrices (for audit)
        pd.DataFrame(amcl_M, index=[r.folder for r in amcl_runs], columns=[r.folder for r in amcl_runs]).to_csv(
            out / f"{algo}_pairwise_IoU_amcl.csv"
        )
        pd.DataFrame(no_M, index=[r.folder for r in no_runs], columns=[r.folder for r in no_runs]).to_csv(
            out / f"{algo}_pairwise_IoU_no_amcl.csv"
        )

        # CI for mean IoU
        amcl_ci = bootstrap_ci_mean(amcl_vals)
        no_ci   = bootstrap_ci_mean(no_vals)

        # Tests + effect size (IoU higher is better)
        delta = cliffs_delta(amcl_vals, no_vals)  # positive means AMCL tends to be higher IoU
        welch_p = float("nan")
        mw_p = float("nan")
        if STATS_OK and len(amcl_vals) >= 2 and len(no_vals) >= 2:
            welch_p = float(ttest_ind(amcl_vals, no_vals, equal_var=False).pvalue)
            mw_p = float(mannwhitneyu(amcl_vals, no_vals, alternative="two-sided").pvalue)

        summary_rows.append({
            "algorithm": algo,
            "amcl_runs": len(amcl_runs),
            "no_amcl_runs": len(no_runs),

            "pairwise_IoU_amcl_mean": float(np.mean(amcl_vals)) if len(amcl_vals) else float("nan"),
            "pairwise_IoU_no_amcl_mean": float(np.mean(no_vals)) if len(no_vals) else float("nan"),
            "pairwise_IoU_amcl_ci95": f"[{amcl_ci[0]:.4g}, {amcl_ci[1]:.4g}]",
            "pairwise_IoU_no_amcl_ci95": f"[{no_ci[0]:.4g}, {no_ci[1]:.4g}]",

            "cliffs_delta": float(delta),
            "welch_p": float(welch_p),
            "mannwhitney_p": float(mw_p),
            "n_pairs_amcl": int(len(amcl_vals)),
            "n_pairs_no_amcl": int(len(no_vals)),
        })

        # Plots: boxplot distributions
        plt.figure(figsize=(4.5, 3.0), dpi=200)
        plt.boxplot([amcl_vals, no_vals], tick_labels=["AMCL", "no AMCL"])
        plt.ylabel("Occupied IoU (known∩known)")
        plt.title(f"{algo}: map repeatability (pairwise IoU)")
        plt.tight_layout()
        plt.savefig(out / f"{algo}_box_pairwise_IoU.png", dpi=300)
        plt.close()

        # Optional: save one aligned example visualization for sanity
        if args.save_aligned_example and len(amcl_runs) >= 2:
            A = amcl_runs[0].gm
            B = amcl_runs[1].gm
            Aocc_al, Aknown_al, params = align_A_to_B_by_search(
                A.occ, A.known, B.occ, B.known,
                rot_min=args.rot_min, rot_max=args.rot_max, rot_step=args.rot_step,
                refine=(not args.no_refine)
            )
            # visualize overlap: occA in red, occB in blue (saved as grayscale channels)
            ov = np.zeros((*B.occ.shape, 3), dtype=np.uint8)
            ov[..., 0] = (Aocc_al.astype(np.uint8) * 255)
            ov[..., 2] = (B.occ.astype(np.uint8) * 255)
            if PIL_OK:
                Image.fromarray(ov).save(out / f"{algo}_aligned_example_amcl.png")

    if not summary_rows:
        raise SystemExit("No algorithms produced results (check --min-runs, --map-yaml).")

    df_sum = pd.DataFrame(summary_rows).sort_values("pairwise_IoU_amcl_mean", ascending=False)
    df_sum.to_csv(out / "summary_by_algorithm.csv", index=False)

    # IEEE-style mean+CI points plot (single figure for paper)
    # (No hard-coded colors; default matplotlib is fine)
    algos2 = df_sum["algorithm"].tolist()
    x = np.arange(len(algos2))
    am = df_sum["pairwise_IoU_amcl_mean"].to_numpy(float)
    nm = df_sum["pairwise_IoU_no_amcl_mean"].to_numpy(float)

    def parse_ci(s):
        m = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", str(s))
        return (float(m[0]), float(m[1])) if len(m) >= 2 else (float("nan"), float("nan"))

    am_ci = np.array([parse_ci(s) for s in df_sum["pairwise_IoU_amcl_ci95"]], float)
    nm_ci = np.array([parse_ci(s) for s in df_sum["pairwise_IoU_no_amcl_ci95"]], float)
    am_err = np.vstack([am - am_ci[:,0], am_ci[:,1] - am])
    nm_err = np.vstack([nm - nm_ci[:,0], nm_ci[:,1] - nm])

    plt.figure(figsize=(3.5, 2.2), dpi=300)
    off = 0.12
    plt.errorbar(x - off, am, yerr=am_err, fmt='o', capsize=3, label="AMCL")
    plt.errorbar(x + off, nm, yerr=nm_err, fmt='o', capsize=3, label="no AMCL")
    plt.xticks(x, algos2)
    plt.ylabel("Pairwise IoU (occ)")
    plt.xlabel("SLAM back-end")
    plt.grid(True, axis='y', linewidth=0.5, alpha=0.4)
    plt.legend(frameon=False, fontsize=8)
    plt.tight_layout()
    plt.savefig(out / "map_repeatability_mean_ci.pdf")
    plt.savefig(out / "map_repeatability_mean_ci.png", dpi=300)
    plt.close()

    print("\nSaved outputs to:", out.resolve())
    print(df_sum.to_string(index=False))


if __name__ == "__main__":
    main()
