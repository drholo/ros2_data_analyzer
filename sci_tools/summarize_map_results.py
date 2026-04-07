#!/usr/bin/env python3
"""
summarize_map_results.py

Collects per-test-folder map repeatability results produced by
calculate_map_repeatability.py and merges them into common CSV files,
analogous to what output/panther/ holds for trajectory evaluation.

Input folders are expected to match the glob pattern:
    output-map-test-{N}{L}/        e.g. output-map-test-1A, output-map-test-2C
where N is the measurement number and L is the environment letter.

Each such folder must contain:
    summary_by_algorithm.csv
    <algorithm>_pairwise_IoU_amcl.csv
    <algorithm>_pairwise_IoU_no_amcl.csv

Output (written to --out-dir, default: output-map-combined/):
    summary_by_algorithm.csv          – stacked summary_by_algorithm tables
                                        with env / measurement_no columns prepended
    pairwise_IoU_amcl.csv             – upper-triangle IoU values for every
                                        (env, measurement_no, algorithm) group
    pairwise_IoU_no_amcl.csv          – same for no-AMCL mode
    per_pair_IoU.csv                  – long-form: one row per pair
                                        (env, measurement_no, algorithm, mode,
                                         run_i, run_j, IoU)

Usage
-----
python summarize_map_results.py
    [--search-dir <dir>]       parent directory containing output-map-test-* folders
                               (default: current working directory / sci_tools)
    [--pattern <glob>]         glob relative to search-dir
                               (default: output-map-test-*)
    [--out-dir  <dir>]         where to write combined CSVs
                               (default: <search-dir>/output-map-combined)

Examples
--------
# From the sci_tools directory:
python summarize_map_results.py
python summarize_map_results.py --search-dir sci_tools --out-dir sci_tools/output-map-combined
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import pandas as pd


# ─────────────────────────────────────────────────────────────────────────────
# Folder-name parsing
# ─────────────────────────────────────────────────────────────────────────────

def parse_test_folder_name(name: str) -> tuple[str, str] | None:
    """
    Parse folder names like ``output-map-test-2A``.

    Returns (env, measurement_no) or None if the name doesn't match.
    E.g. "output-map-test-2A" → ("A", "2")
         "output-map-test-1B" → ("B", "1")
    """
    match = re.match(r".*-(\d+)([A-Za-z]+)$", name)
    if not match:
        return None
    measurement_no = match.group(1)
    env = match.group(2).upper()
    return env, measurement_no


# ─────────────────────────────────────────────────────────────────────────────
# Pairwise IoU matrix → long form
# ─────────────────────────────────────────────────────────────────────────────

def matrix_to_upper_triangle(
    df_mat: pd.DataFrame,
    env: str,
    measurement_no: str,
    algorithm: str,
    mode: str,
) -> pd.DataFrame:
    """
    Convert a square pairwise-IoU DataFrame (index = run labels, columns = run
    labels) to a long-form DataFrame with one row per pair (upper triangle only,
    excluding the diagonal).
    """
    rows = []
    labels = list(df_mat.index)
    for i in range(len(labels)):
        for j in range(i + 1, len(labels)):
            rows.append(
                {
                    "env": env,
                    "measurement_no": measurement_no,
                    "algorithm": algorithm,
                    "mode": mode,
                    "run_i": labels[i],
                    "run_j": labels[j],
                    "IoU": df_mat.iloc[i, j],
                }
            )
    return pd.DataFrame(rows)


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    # ── CLI ──────────────────────────────────────────────────────────────────
    ap = argparse.ArgumentParser(
        description="Merge per-test-folder map repeatability results into combined CSVs."
    )
    ap.add_argument(
        "--search-dir",
        default=None,
        help="Parent directory that contains the output-map-test-* folders. "
             "Defaults to the directory of this script.",
    )
    ap.add_argument(
        "--pattern",
        default="output-map-test-*",
        help="Glob pattern for test folders relative to --search-dir "
             "(default: output-map-test-*).",
    )
    ap.add_argument(
        "--out-dir",
        default=None,
        help="Output directory (default: <search-dir>/output-map-combined).",
    )
    args = ap.parse_args()

    script_dir = Path(__file__).resolve().parent
    search_dir = Path(args.search_dir).resolve() if args.search_dir else script_dir
    out_dir = Path(args.out_dir).resolve() if args.out_dir else search_dir / "output-map-combined"

    # ── Discover folders ─────────────────────────────────────────────────────
    test_folders = sorted(search_dir.glob(args.pattern))
    test_folders = [f for f in test_folders if f.is_dir()]

    if not test_folders:
        print(
            f"[ERROR] No folders matching '{args.pattern}' found under '{search_dir}'.",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"Found {len(test_folders)} test folder(s) under '{search_dir}':")
    for f in test_folders:
        print(f"  {f.name}")

    # ── Accumulate data ───────────────────────────────────────────────────────
    summary_frames: list[pd.DataFrame] = []
    pair_frames: list[pd.DataFrame] = []

    skipped = 0

    for folder in test_folders:
        parsed = parse_test_folder_name(folder.name)
        if parsed is None:
            print(f"[WARN] Cannot parse env/measurement from '{folder.name}', skipping.")
            skipped += 1
            continue
        env, measurement_no = parsed

        # ── summary_by_algorithm.csv ─────────────────────────────────────────
        summary_csv = folder / "summary_by_algorithm.csv"
        if summary_csv.exists():
            df_sum = pd.read_csv(summary_csv)
            df_sum.insert(0, "env", env)
            df_sum.insert(1, "measurement_no", measurement_no)
            summary_frames.append(df_sum)
        else:
            print(f"[WARN] '{summary_csv}' not found, skipping summary for {folder.name}.")
            skipped += 1

        # ── pairwise IoU matrices ─────────────────────────────────────────────
        for csv_path in sorted(folder.glob("*_pairwise_IoU_*.csv")):
            # Filename pattern: <algorithm>_pairwise_IoU_<mode>.csv
            stem = csv_path.stem  # e.g. cartographer_pairwise_IoU_amcl
            # Extract mode (last token after last underscore, could be "amcl" or "no_amcl")
            if "_pairwise_IoU_no_amcl" in stem:
                mode = "no_amcl"
                algo = stem.replace("_pairwise_IoU_no_amcl", "")
            elif "_pairwise_IoU_amcl" in stem:
                mode = "amcl"
                algo = stem.replace("_pairwise_IoU_amcl", "")
            else:
                print(f"[WARN] Unrecognised filename pattern '{csv_path.name}', skipping.")
                continue

            try:
                df_mat = pd.read_csv(csv_path, index_col=0)
            except Exception as exc:
                print(f"[WARN] Cannot read '{csv_path}': {exc}")
                continue

            df_pairs = matrix_to_upper_triangle(df_mat, env, measurement_no, algo, mode)
            pair_frames.append(df_pairs)

    # ── Write outputs ─────────────────────────────────────────────────────────
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1. Combined summary
    if summary_frames:
        df_summary_all = pd.concat(summary_frames, ignore_index=True)
        # Sort by env, measurement_no, algorithm for readability
        sort_cols = [c for c in ("env", "measurement_no", "algorithm") if c in df_summary_all.columns]
        df_summary_all.sort_values(sort_cols, inplace=True)
        df_summary_all.reset_index(drop=True, inplace=True)

        out_summary = out_dir / "summary_by_algorithm.csv"
        df_summary_all.to_csv(out_summary, index=False)
        print(f"\nWrote {len(df_summary_all)} rows → {out_summary}")
    else:
        print("[WARN] No summary data collected.")

    # 2. Long-form per-pair IoU (all modes combined)
    if pair_frames:
        df_pairs_all = pd.concat(pair_frames, ignore_index=True)
        sort_cols = [c for c in ("env", "measurement_no", "algorithm", "mode", "run_i", "run_j")
                     if c in df_pairs_all.columns]
        df_pairs_all.sort_values(sort_cols, inplace=True)
        df_pairs_all.reset_index(drop=True, inplace=True)

        out_pairs = out_dir / "per_pair_IoU.csv"
        df_pairs_all.to_csv(out_pairs, index=False)
        print(f"Wrote {len(df_pairs_all)} rows → {out_pairs}")

        # 3. Split by mode for convenience (mirrors original per-algo files)
        for mode in ("amcl", "no_amcl"):
            df_mode = df_pairs_all[df_pairs_all["mode"] == mode].copy()
            if df_mode.empty:
                continue
            out_mode = out_dir / f"pairwise_IoU_{mode}.csv"
            df_mode.to_csv(out_mode, index=False)
            print(f"Wrote {len(df_mode)} rows → {out_mode}")
    else:
        print("[WARN] No pairwise IoU data collected.")

    if skipped:
        print(f"\n[INFO] {skipped} item(s) were skipped due to warnings above.")

    print("\nDone.")


if __name__ == "__main__":
    main()
