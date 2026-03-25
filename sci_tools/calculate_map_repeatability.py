#!/usr/bin/env python3
"""
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
import os
import re
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, List

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import yaml
from scipy import ndimage
from scipy.stats import ttest_ind, mannwhitneyu
from PIL import Image


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
    img = Image.open(pgm_path)
    arr = np.array(img)
    if arr.ndim == 3:
        arr = arr[..., 0]
    return arr.astype(np.uint8)


def _yaml_load(path: Path) -> dict:
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
    FB_conj = np.conj(np.fft.rfft2(B))
    corr = np.fft.irfft2(FA * FB_conj, s=A.shape)

    peak = np.unravel_index(np.argmax(corr), corr.shape)
    dy, dx = peak
    # Convert circular shift to signed shift
    H, W = A.shape
    if dy > H // 2:
        dy -= H
    if dx > W // 2:
        dx -= W
    return int(dy), int(dx)


def _fft_cross_correlation_shift_with_precomputed_b(A: np.ndarray, B_fft_conj: np.ndarray) -> Tuple[int, int]:
    """
    Same as _fft_cross_correlation_shift, but reuses conj(fft(B))
    for repeated comparisons against a fixed B.
    """
    FA = np.fft.rfft2(A)
    corr = np.fft.irfft2(FA * B_fft_conj, s=A.shape)

    peak = np.unravel_index(np.argmax(corr), corr.shape)
    dy, dx = peak

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
    best_occ = A_occ
    best_known = A_known
    B_occ_f32 = B_occ.astype(np.float32)
    B_occ_fft_conj = np.conj(np.fft.rfft2(B_occ_f32))
    rot_cache: dict[float, Tuple[np.ndarray, np.ndarray]] = {}

    def get_rotated(angle_deg: float) -> Tuple[np.ndarray, np.ndarray]:
        # cache reused angles across coarse and refinement passes
        key = round(float(angle_deg), 6)
        cached = rot_cache.get(key)
        if cached is not None:
            return cached
        Aor = _rotate_mask(A_occ, key)
        Akr = _rotate_mask(A_known, key)
        rot_cache[key] = (Aor, Akr)
        return Aor, Akr

    def evaluate(angle_deg: float):
        Aor, Akr = get_rotated(angle_deg)

        # use float masks for correlation (occupied only)
        dy, dx = _fft_cross_correlation_shift_with_precomputed_b(
            Aor.astype(np.float32),
            B_occ_fft_conj,
        )

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
    idx = rng.integers(0, len(x), size=(n_boot, len(x)))
    boots = x[idx].mean(axis=1)
    return (float(np.quantile(boots, alpha/2)), float(np.quantile(boots, 1-alpha/2)))


def cliffs_delta(a: np.ndarray, b: np.ndarray) -> float:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    a = a[np.isfinite(a)]
    b = b[np.isfinite(b)]
    if len(a) == 0 or len(b) == 0:
        return float("nan")
    b_sorted = np.sort(b)
    # counts for each x: elements in b strictly less/greater than x
    lt = np.searchsorted(b_sorted, a, side="left").sum()
    le = np.searchsorted(b_sorted, a, side="right").sum()
    gt = len(a) * len(b) - le
    return float((gt - lt) / (len(a) * len(b)))


# -------------------- pipeline --------------------

@dataclass
class RunMap:
    algo: str
    mode: str
    run_id: str
    folder: str
    gm: GridMap


def _score_pair_worker(task: tuple):
    """
    Top-level picklable worker for multiprocessing.
    Computes aligned IoU for one map pair (i, j).
    task = (i, j, occ_A, known_A, occ_B, known_B, rot_min, rot_max, rot_step, refine)
    """
    i, j, occ_A, known_A, occ_B, known_B, rot_min, rot_max, rot_step, refine = task
    Aocc, Aknown, Bocc, Bknown = _prepare_pair_for_alignment(occ_A, known_A, occ_B, known_B)
    Aocc_al, Aknown_al, _ = align_A_to_B_by_search(
        Aocc, Aknown, Bocc, Bknown,
        rot_min=rot_min, rot_max=rot_max, rot_step=rot_step,
        refine=refine,
    )
    s = iou_occ(Aocc_al, Aknown_al, Bocc, Bknown)
    return i, j, float(s)


def _log(msg: str):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


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

    # parallelism
    ap.add_argument("--workers", type=int, default=1,
                    help="Workers for pairwise alignment (0 = all CPUs, 1 = serial, N > 1 = N processes)")

    args = ap.parse_args()
    args.workers = os.cpu_count() if args.workers == 0 else args.workers

    root = Path(args.results)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    t0 = time.perf_counter()
    _log(f"Starting map repeatability analysis")
    _log(f"Results dir: {root.resolve()}")
    _log(f"Output dir: {out.resolve()}")
    _log(
        "Alignment params: "
        f"rot=[{args.rot_min}, {args.rot_max}] step={args.rot_step}, refine={not args.no_refine}"
    )
    _log(f"Workers: {args.workers} ({'serial' if args.workers == 1 else 'parallel'})")

    # Load all runs
    _log("Stage 1/5: scanning and loading record maps...")
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
            _log(f"  loaded: {rec.name} ({algo}/{mode}, run={run_no})")
        except Exception as e:
            print(f"[WARN] Skipping {rec.name}: {e}")

    if not all_runs:
        raise SystemExit("No valid maps found. Check --map-yaml and folder structure.")

    # Report counts
    from collections import Counter
    _log(f"Loaded {len(all_runs)} maps total")
    print("Parsed counts:", Counter((r.algo, r.mode) for r in all_runs), flush=True)

    algos = sorted(set(r.algo for r in all_runs))
    summary_rows = []

    def pairwise_scores(runs: List[RunMap]) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        Return pairwise IoU (upper triangle) and full matrix (optional).
        Supports parallel execution via args.workers.
        """
        n = len(runs)
        n_pairs = (n * (n - 1)) // 2
        _log(f"  pairwise scoring start: n={n}, pairs={n_pairs}")
        M = np.full((n, n), np.nan, dtype=float)
        for i in range(n):
            M[i, i] = 1.0

        # Build flat task list
        tasks = [
            (
                i, j,
                runs[i].gm.occ, runs[i].gm.known,
                runs[j].gm.occ, runs[j].gm.known,
                args.rot_min, args.rot_max,
                args.rot_step, not args.no_refine,
            )
            for i in range(n)
            for j in range(i + 1, n)
        ]

        progress_step = max(1, n_pairs // 20)  # ~5% updates
        done = 0

        if args.workers == 1:
            # --- serial ---
            for task in tasks:
                i, j, s = _score_pair_worker(task)
                M[i, j] = s
                M[j, i] = s
                done += 1
                if done % progress_step == 0 or done == n_pairs:
                    _log(f"    progress: {done}/{n_pairs} pairs")
        else:
            # --- parallel ---
            _log(f"  spawning ProcessPoolExecutor with {args.workers} workers")
            with ProcessPoolExecutor(max_workers=args.workers) as pool:
                futures = {pool.submit(_score_pair_worker, t): t for t in tasks}
                for fut in as_completed(futures):
                    i, j, s = fut.result()
                    M[i, j] = s
                    M[j, i] = s
                    done += 1
                    if done % progress_step == 0 or done == n_pairs:
                        _log(f"    progress: {done}/{n_pairs} pairs")

        iu = np.triu_indices(n, k=1)
        vals = M[iu]
        vals = vals[np.isfinite(vals)]
        _log("  pairwise scoring done")
        return vals, M

    _log(f"Stage 2/5: evaluating algorithms ({len(algos)} total)")
    for algo in algos:
        algo_t0 = time.perf_counter()
        _log(f"Algorithm '{algo}': preparing AMCL vs no_AMCL groups")
        runs_algo = [r for r in all_runs if r.algo == algo]
        amcl_runs = [r for r in runs_algo if r.mode == "amcl"]
        no_runs   = [r for r in runs_algo if r.mode == "no_amcl"]

        if len(amcl_runs) < args.min_runs or len(no_runs) < args.min_runs:
            print(f"[WARN] {algo}: insufficient runs (amcl={len(amcl_runs)}, no_amcl={len(no_runs)}); skipping")
            continue

        # Pairwise distributions
        _log(f"Algorithm '{algo}': scoring AMCL pairwise IoU")
        amcl_vals, amcl_M = pairwise_scores(amcl_runs)
        _log(f"Algorithm '{algo}': scoring no_AMCL pairwise IoU")
        no_vals, no_M     = pairwise_scores(no_runs)

        # Save matrices (for audit)
        pd.DataFrame(amcl_M, index=[r.folder for r in amcl_runs], columns=[r.folder for r in amcl_runs]).to_csv(
            out / f"{algo}_pairwise_IoU_amcl.csv"
        )
        pd.DataFrame(no_M, index=[r.folder for r in no_runs], columns=[r.folder for r in no_runs]).to_csv(
            out / f"{algo}_pairwise_IoU_no_amcl.csv"
        )
        _log(f"Algorithm '{algo}': saved pairwise matrices")

        # CI for mean IoU
        amcl_ci = bootstrap_ci_mean(amcl_vals)
        no_ci   = bootstrap_ci_mean(no_vals)

        # Tests + effect size (IoU higher is better)
        delta = cliffs_delta(amcl_vals, no_vals)  # positive means AMCL tends to be higher IoU
        welch_p = float("nan")
        mw_p = float("nan")
        if len(amcl_vals) >= 2 and len(no_vals) >= 2:
            welch_res = ttest_ind(amcl_vals, no_vals, equal_var=False)
            mw_res = mannwhitneyu(amcl_vals, no_vals, alternative="two-sided")
            welch_p = float(np.asarray(getattr(welch_res, "pvalue", welch_res[1]), dtype=float))
            mw_p = float(np.asarray(getattr(mw_res, "pvalue", mw_res[1]), dtype=float))

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
        _log(f"Algorithm '{algo}': saved boxplot")

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
            Image.fromarray(ov).save(out / f"{algo}_aligned_example_amcl.png")
            _log(f"Algorithm '{algo}': saved aligned example")

        _log(f"Algorithm '{algo}' done in {time.perf_counter() - algo_t0:.1f}s")

    if not summary_rows:
        raise SystemExit("No algorithms produced results (check --min-runs, --map-yaml).")

    _log("Stage 3/5: writing summary CSV")
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
    # Guard against tiny numeric inconsistencies or CI strings where mean can
    # lie slightly outside reported interval; matplotlib requires non-negative yerr.
    am_err = np.maximum(np.vstack([am - am_ci[:, 0], am_ci[:, 1] - am]), 0.0)
    nm_err = np.maximum(np.vstack([nm - nm_ci[:, 0], nm_ci[:, 1] - nm]), 0.0)

    _log("Stage 4/5: generating final mean+CI plots")
    plt.figure(figsize=(3.5, 2.2), dpi=300)
    off = 0.12
    am_valid = np.isfinite(am) & np.isfinite(am_err[0]) & np.isfinite(am_err[1])
    nm_valid = np.isfinite(nm) & np.isfinite(nm_err[0]) & np.isfinite(nm_err[1])

    if np.any(am_valid):
        plt.errorbar(x[am_valid] - off, am[am_valid], yerr=am_err[:, am_valid], fmt='o', capsize=3, label="AMCL")
    else:
        _log("No finite AMCL points for mean+CI plot")

    if np.any(nm_valid):
        plt.errorbar(x[nm_valid] + off, nm[nm_valid], yerr=nm_err[:, nm_valid], fmt='o', capsize=3, label="no AMCL")
    else:
        _log("No finite no_AMCL points for mean+CI plot")
    plt.xticks(x, algos2)
    plt.ylabel("Pairwise IoU (occ)")
    plt.xlabel("SLAM back-end")
    plt.grid(True, axis='y', linewidth=0.5, alpha=0.4)
    plt.legend(frameon=False, fontsize=8)
    plt.tight_layout()
    plt.savefig(out / "map_repeatability_mean_ci.pdf")
    plt.savefig(out / "map_repeatability_mean_ci.png", dpi=300)
    plt.close()
    _log("Stage 5/5: done")

    print("\nSaved outputs to:", out.resolve(), flush=True)
    print(df_sum.to_string(index=False), flush=True)
    _log(f"Total runtime: {time.perf_counter() - t0:.1f}s")


if __name__ == "__main__":
    main()
