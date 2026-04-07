#!/usr/bin/env python3
"""
evaluate_slam_paper_methods.py

A unified script to evaluate 2D SLAM repeatability according to the methodology:
  - Arc-length (distance-based) trajectory resampling.
  - Strict SE(2) rigid alignment.
  - Metrics: Pairwise ATE, Centroid ATE, RPE translational proxy, Run Completeness.
  - Statistics: Bootstrap CIs, Cliff's Delta, Welch's t-test, Mann-Whitney U test.
"""

from __future__ import annotations
import argparse
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, Dict, List

import numpy as np
import pandas as pd
from scipy.interpolate import interp1d

try:
    from scipy.stats import ttest_ind, mannwhitneyu
    SCIPY_OK = True
except Exception:
    SCIPY_OK = False

try:
    import yaml
    YAML_OK = True
except Exception:
    YAML_OK = False

# ==========================================
# 1. DATA LOADING & RESAMPLING
# ==========================================

def load_positions_from_json(path: Path, min_poses: int = 5) -> Tuple[np.ndarray, int]:
    """Loads JSON data and extracts 2D (x, y) coordinates."""
    obj = json.loads(path.read_text(encoding="utf-8"))
    data = obj.get("data", None)
    if not isinstance(data, list):
        raise ValueError(f"{path}: missing or non-list 'data'")

    def try_xy(v):
        if v is None: return None
        if isinstance(v, dict):
            for k in ("position", "translation"):
                if k in v:
                    got = try_xy(v[k])
                    if got is not None: return got
            if all(k in v for k in ("x", "y")):
                return float(v["x"]), float(v["y"])
            if all(k in v for k in ("X", "Y")):
                return float(v["X"]), float(v["Y"])
        if isinstance(v, (list, tuple)) and len(v) >= 2:
            return float(v[0]), float(v[1])
        return None

    pts = []
    for r in data:
        xy = try_xy(r)
        if xy is not None:
            pts.append(xy)

    raw_poses_count = len(pts)
    P = np.asarray(pts, dtype=float)
    if len(P) < min_poses:
        raise ValueError(f"too few valid poses in {path}")

    # drop consecutive identical poses
    if len(P) >= 2:
        d = np.linalg.norm(np.diff(P, axis=0), axis=1)
        keep = np.concatenate([[True], d > 0])
        P = P[keep]
        
    return P, raw_poses_count

def resample_trajectory_arc_length(trajectory: np.ndarray, num_samples: int = 1000) -> np.ndarray:
    """Arc-length parameterization for shared distance grid resampling."""
    if len(trajectory) < 2:
        return np.zeros((num_samples, 2))

    diffs = np.diff(trajectory, axis=0)
    distances = np.linalg.norm(diffs, axis=1)
    cumulative_dist = np.insert(np.cumsum(distances), 0, 0)
    
    total_dist = cumulative_dist[-1]
    if total_dist == 0:
        return np.zeros((num_samples, 2))
        
    progress = cumulative_dist / total_dist
    target_progress = np.linspace(0, 1, num_samples)
    
    progress, unique_indices = np.unique(progress, return_index=True)
    trajectory = trajectory[unique_indices]
    
    interp_x = interp1d(progress, trajectory[:, 0], kind='linear', bounds_error=False, fill_value="extrapolate")
    interp_y = interp1d(progress, trajectory[:, 1], kind='linear', bounds_error=False, fill_value="extrapolate")
    
    return np.column_stack((interp_x(target_progress), interp_y(target_progress)))

# ==========================================
# RAW PATH LENGTH ANALYSIS
# ==========================================

def path_length(P: np.ndarray) -> float:
    """Total path length computed from raw poses."""
    if P is None or len(P) < 2:
        return float("nan")
    return float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)))



# ==========================================
# 2. SE(2) ALIGNMENT & METRICS
# ==========================================


def align_se2(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Rigid alignment in SE(2) without scale, using SVD."""
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    
    AA = A - centroid_A
    BB = B - centroid_B
    
    H = AA.T @ BB
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    # Handle reflection
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
        
    t = centroid_B - R @ centroid_A
    return (R @ A.T).T + t

def ate_rmse(P: np.ndarray, Q: np.ndarray) -> float:
    d = P - Q
    return float(np.sqrt(np.mean(np.sum(d * d, axis=1))))

def rpe_trans_rmse(P: np.ndarray, Q: np.ndarray, delta: int = 10) -> float:
    n = min(len(P), len(Q))
    if n <= delta + 1:
        return float("nan")
    errs = []
    for i in range(n - delta):
        dp = P[i + delta] - P[i]
        dq = Q[i + delta] - Q[i]
        errs.append(np.linalg.norm(dp - dq))
    return float(np.sqrt(np.mean(np.square(errs))))

def pairwise_values(runs: list[np.ndarray]) -> np.ndarray:
    vals = []
    for i in range(len(runs)):
        for j in range(i + 1, len(runs)):
            Pi = align_se2(runs[i], runs[j])
            vals.append(ate_rmse(Pi, runs[j]))
    return np.asarray(vals, dtype=float)

def mean_trajectory(runs: list[np.ndarray]) -> np.ndarray:
    anchor = runs[0]
    aligned = [anchor]
    for k in range(1, len(runs)):
        aligned.append(align_se2(runs[k], anchor))
    A = np.stack(aligned, axis=0)
    return A.mean(axis=0)

# ==========================================
# 3. STATISTICAL ANALYSIS
# ==========================================

def bootstrap_ci_mean(x: np.ndarray, n_boot: int = 2000, alpha: float = 0.05, seed: int = 0) -> tuple[float, float]:
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan"), float("nan")
    rng = np.random.default_rng(seed)
    boots = np.array([rng.choice(x, size=len(x), replace=True).mean() for _ in range(n_boot)])
    return float(np.quantile(boots, alpha / 2)), float(np.quantile(boots, 1 - alpha / 2))

def cliffs_delta(a: np.ndarray, b: np.ndarray) -> float:
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    a, b = a[np.isfinite(a)], b[np.isfinite(b)]
    if a.size == 0 or b.size == 0: return float("nan")
    gt = sum(np.sum(x > b) for x in a)
    lt = sum(np.sum(x < b) for x in a)
    return float((gt - lt) / (len(a) * len(b)))

# ==========================================
# 4. RUN CONTAINER & FOLDER PARSING
# ==========================================

@dataclass
class Run:
    env: str
    measurement_no: str  # Replaced 'platform' with 'measurement_no'
    session: str
    algo: str
    mode: str
    run_id: str
    folder: str
    P: np.ndarray
    P_raw: np.ndarray
    raw_poses_count: int

def parse_results_folder_name(name: str) -> Optional[Tuple[str, str]]:
    """Parses folders like results_1A into (env='A', meas_no='1')."""
    match = re.match(r"^results_(\d+)([a-zA-Z]+)$", name)
    if not match: return None
    measurement_no = match.group(1)
    env = match.group(2).upper()
    return env, measurement_no

def parse_record_folder(name: str):
    if not name.startswith("record_"): return None
    parts = name[len("record_"):].split("_")
    if len(parts) < 3 or not parts[-1].isdigit(): return None
    if len(parts) >= 4 and parts[-3] == "no" and parts[-2] == "amcl":
        return "_".join(parts[:-3]), "no_amcl", parts[-1]
    elif parts[-2] == "amcl":
        return "_".join(parts[:-2]), "amcl", parts[-1]
    return None

# ==========================================
# 5. MAIN PIPELINE
# ==========================================

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", nargs="+", required=True, help="Results folders (e.g. results_AA)")
    ap.add_argument("--filename", required=True, help="JSON to evaluate (e.g., TF.json)")
    ap.add_argument("--out", default="out_csv", help="Output directory")
    ap.add_argument("--resample", type=int, default=1000)
    ap.add_argument("--rpe-delta", type=int, default=10)
    ap.add_argument("--c-min", type=float, default=0.8, help="Minimum completeness ratio for success")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    runs: list[Run] = []

    for results_dir in [Path(p) for p in args.results]:
        if not results_dir.exists(): continue
        parsed = parse_results_folder_name(results_dir.name)
        if not parsed: continue
        env, measurement_no = parsed
        session_id = "1" # Defaulting session since it's no longer in the folder name

        for rec in sorted(results_dir.glob("record_*")):
            if not rec.is_dir(): continue
            parsed_rec = parse_record_folder(rec.name)
            if not parsed_rec: continue
            algo, mode, run_no = parsed_rec

            jf = rec / args.filename
            if not jf.exists():
                matches = list(rec.rglob(args.filename))
                if not matches: continue
                jf = matches[0]

            try:
                P_raw, raw_count = load_positions_from_json(jf)
                P_resampled = resample_trajectory_arc_length(P_raw, args.resample)

                runs.append(Run(
                    env, measurement_no, session_id, algo, mode, run_no,
                    f"{results_dir.name}/{rec.name}",
                    P_resampled,
                    P_raw,
                    raw_count
                ))
            except Exception as e:
                print(f"[WARN] Skipping {rec.name}: {e}")

    if not runs:
        raise SystemExit("No runs loaded.")
    # ==========================================
    # RAW PATH LENGTH ANALYSIS
    # ==========================================

    per_run_len_rows = []

    for r in runs:
        L = path_length(r.P_raw)
        per_run_len_rows.append({
            "env": r.env,
            "measurement_no": r.measurement_no,
            "algorithm": r.algo,
            "mode": r.mode,
            "run_id": r.run_id,
            "raw_path_length_m": L,
            "raw_poses_count": r.raw_poses_count,
        })

    df_run_len = pd.DataFrame(per_run_len_rows)
    df_run_len.to_csv(out / "per_run_raw_path_length.csv", index=False)

    # ---- Per environment & measurement_no summary ----
    env_summary = (
        df_run_len.groupby(["env", "measurement_no"])["raw_path_length_m"]
        .agg(["count", "mean", "median", "std", "min", "max"])
        .reset_index()
        .rename(columns={
            "count": "n_runs",
            "mean": "mean_path_length_m",
            "median": "median_path_length_m",
            "std": "std_path_length_m",
            "min": "min_path_length_m",
            "max": "max_path_length_m",
        })
    )

    # Bootstrap 95% CI for mean (per env & measurement_no)
    ci_low, ci_high = [], []

    for _, row in env_summary.iterrows():
        vals = df_run_len[
            (df_run_len["env"] == row["env"]) &
            (df_run_len["measurement_no"] == row["measurement_no"])
            ]["raw_path_length_m"].to_numpy(dtype=float)

        lo, hi = bootstrap_ci_mean(vals, n_boot=2000, alpha=0.05, seed=0)
        ci_low.append(lo)
        ci_high.append(hi)

    env_summary["ci95_low_m"] = ci_low
    env_summary["ci95_high_m"] = ci_high

    env_summary.to_csv(out / "raw_path_length_by_env_and_measurement.csv", index=False)

    summary_rows, per_run_rows = [], []

    def key_fn(r: Run): return (r.env, r.measurement_no, r.session, r.algo)
    groups: Dict[tuple, list[Run]] = {}
    for r in runs: groups.setdefault(key_fn(r), []).append(r) 

    for (env, measurement_no, session, algo), gruns in groups.items():
        modes = {"amcl": [r for r in gruns if r.mode == "amcl"], "no_amcl": [r for r in gruns if r.mode == "no_amcl"]}
        mode_stats = {}

        # 1. Calculate Run Completeness Reference (N_ref)
        all_raw_counts = [r.raw_poses_count for r in gruns]
        n_ref = np.median(all_raw_counts) if all_raw_counts else 1

        for mode_name, mruns in modes.items():
            Ps = [r.P for r in mruns]
            if not Ps:
                mode_stats[mode_name] = None
                continue

            pv = pairwise_values(Ps) if len(Ps) >= 2 else np.asarray([], dtype=float)
            meanP = mean_trajectory(Ps)
            
            centroid, rpe, completeness = [], [], []
            for r in mruns:
                Pal = align_se2(r.P, meanP)
                centroid.append(ate_rmse(Pal, meanP))
                rpe.append(rpe_trans_rmse(Pal, meanP, delta=args.rpe_delta))
                
                c_k = r.raw_poses_count / n_ref
                completeness.append(c_k)
                per_run_rows.append({
                    "env": env, "measurement_no": measurement_no, "session": session, "algorithm": algo, "mode": mode_name,
                    "run_id": r.run_id, "centroid_ATE_m": centroid[-1], "RPE_trans_m": rpe[-1], "completeness_Ck": c_k
                })

            success_rate = sum(1 for c in completeness if c >= args.c_min) / len(completeness) if completeness else 0.0

            mode_stats[mode_name] = {
                "pairwise": pv, "pairwise_mean": float(np.nanmean(pv)) if pv.size else float("nan"),
                "pairwise_ci": bootstrap_ci_mean(pv), "centroid": np.asarray(centroid),
                "centroid_mean": float(np.nanmean(centroid)), "centroid_ci": bootstrap_ci_mean(centroid),
                "success_rate": success_rate, "mean_completeness": np.mean(completeness)
            }

        am, no = mode_stats.get("amcl"), mode_stats.get("no_amcl")
        if not am or not no: continue

        def test_and_effect(a_vals, b_vals):
            out = {"cliffs_delta": cliffs_delta(a_vals, b_vals), "welch_p": float("nan"), "mannwhitney_p": float("nan")}
            if SCIPY_OK and len(a_vals) >= 2 and len(b_vals) >= 2:
                out["welch_p"] = float(ttest_ind(a_vals, b_vals, equal_var=False).pvalue)
                out["mannwhitney_p"] = float(mannwhitneyu(a_vals, b_vals, alternative="two-sided").pvalue)
            return out

        pair_cmp = test_and_effect(am["pairwise"], no["pairwise"])
        cent_cmp = test_and_effect(am["centroid"], no["centroid"])

        summary_rows.append({
            "env": env, "measurement_no": measurement_no, "session": session, "algorithm": algo,
            "amcl_runs": len(modes["amcl"]), "no_amcl_runs": len(modes["no_amcl"]),
            "amcl_success_rate": am["success_rate"], "no_amcl_success_rate": no["success_rate"],
            "pairwise_ATE_amcl_mean": am["pairwise_mean"], "pairwise_ATE_no_amcl_mean": no["pairwise_mean"],
            "pairwise_ATE_cliffs_delta": pair_cmp["cliffs_delta"], "pairwise_ATE_welch_p": pair_cmp["welch_p"],
            "centroid_ATE_amcl_mean": am["centroid_mean"], "centroid_ATE_no_amcl_mean": no["centroid_mean"],
            "centroid_ATE_cliffs_delta": cent_cmp["cliffs_delta"], "centroid_ATE_mannwhitney_p": cent_cmp["mannwhitney_p"]
        })

    pd.DataFrame(summary_rows).to_csv(out / "summary_by_slice.csv", index=False)
    pd.DataFrame(per_run_rows).to_csv(out / "per_run_metrics.csv", index=False)
    print(f"Evaluation complete. Data saved to {out.resolve()}")

if __name__ == "__main__":
    main()
