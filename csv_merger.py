# csv_merger.py
#!/usr/bin/env python3
"""
Merge per-day detector/label CSVs (wide format) into per-month and optional all-months outputs.

Each input CSV:
  - index/col: 'sensor_id' + minute-bin columns (e.g., 0,5,10,...) and any extras
Per-month output:
  - rows: sensor_id
  - cols: "<YYYY-MM>_<day_XX>_<minute>" (plus any extra columns, prefixed the same way)
All-months output (optional):
  - outer-join of all per-month outputs on sensor_id
"""

import os
import re
from typing import List, Optional, Tuple

import pandas as pd

MONTH_DIR_RE = re.compile(r"^\d{4}-\d{2}$")   # YYYY-MM
DAY_DIR_RE   = re.compile(r"^day_(\d+)$", re.IGNORECASE)
MAX_MINUTE = 1439


# ----------------------------- helpers -----------------------------

def _find_month_dirs(base: str) -> List[str]:
    if not os.path.isdir(base):
        raise FileNotFoundError(f"Months base directory not found: {base}")
    months = [m for m in sorted(os.listdir(base))
              if MONTH_DIR_RE.match(m) and os.path.isdir(os.path.join(base, m))]
    if not months:
        raise RuntimeError(f"No YYYY-MM folders found under {base}")
    return months


def _find_day_dirs(month_path: str) -> List[str]:
    days: List[str] = []
    for name in sorted(os.listdir(month_path)):
        m = DAY_DIR_RE.match(name)
        if m and os.path.isdir(os.path.join(month_path, name)):
            days.append(name)
    return days


def _find_day_csvs(month_path: str, day_dir: str, must_contain: Optional[str]) -> List[str]:
    day_path = os.path.join(month_path, day_dir)
    picks: List[str] = []
    for root, _dirs, files in os.walk(day_path):
        for f in files:
            if not f.lower().endswith(".csv"):
                continue
            if must_contain and must_contain.lower() not in f.lower():
                continue
            picks.append(os.path.join(root, f))
    # prefer CSV directly under the day folder if multiple exist
    picks.sort(key=lambda p: (0 if os.path.dirname(p) == day_path else 1, p))
    return picks


def _is_time_col(c: str) -> bool:
    try:
        v = int(str(c))
        return 0 <= v <= 1439
    except (TypeError, ValueError):
        return False



def _merge_one_month(
    base: str,
    month_name: str,
    out_dir: str,
    file_name_must_contain: Optional[str],
) -> Tuple[Optional[pd.DataFrame], List[Tuple[str, str]]]:
    month_path = os.path.join(base, month_name)
    day_dirs = _find_day_dirs(month_path)
    if not day_dirs:
        print(f"⚠️  {month_name}: no day_## folders found; skipping.")
        return None, []

    frames: List[pd.DataFrame] = []
    picked_files: List[Tuple[str, str]] = []

    for day in day_dirs:
        csvs = _find_day_csvs(month_path, day, file_name_must_contain)
        if not csvs:
            print(f"⚠️  {month_name}/{day}: no CSVs matching filter '{file_name_must_contain}'. Skipping.")
            continue

        fp = csvs[0]
        picked_files.append((day, fp))

        # read as string to be robust; we'll coerce time cols to int ordering
        df = pd.read_csv(fp, dtype=str)
        if "sensor_id" not in df.columns:
            print(f"⚠️  {month_name}/{day}: file has no 'sensor_id' → {fp}. Skipping.")
            continue


        time_cols = [c for c in df.columns
             if c != "sensor_id" and _is_time_col(c)]
        other_cols = [c for c in df.columns if c not in time_cols + ["sensor_id"]]
        time_cols_sorted = sorted(time_cols, key=lambda x: int(x))
        ordered_cols = ["sensor_id"] + time_cols_sorted + other_cols

        df = df[ordered_cols].copy().set_index("sensor_id")

        # Prefix with month and day to avoid collisions across months/days
        df.columns = [f"{month_name}_{day}_{c}" for c in df.columns]


        frames.append(df)

    if not frames:
        print(f"⚠️  {month_name}: no usable day CSVs; skipping.")
        return None, picked_files

    combined = pd.concat(frames, axis=1, sort=False)
    os.makedirs(out_dir, exist_ok=True)
    out_csv = os.path.join(out_dir, f"merged_{month_name}_by_sensor.csv")
    combined.sort_index().to_csv(out_csv, index_label="sensor_id")

    print(f"✅ {month_name}: merged {len(frames)} day(s) → {out_csv}")
    for day, fp in picked_files:
        print(f"   - {day}: {fp}")

    return combined, picked_files


# ----------------------------- public API -----------------------------

def run_csv_merger(
    months_base_dir: str,
    out_dir: str,
    file_name_must_contain: Optional[str] = "labels",
    write_overall_combined: bool = True,
    overall_out_csv: Optional[str] = None,
) -> dict:
    """
    Merge per-day CSVs into per-month outputs and optionally an all-months CSV.

    Args:
        months_base_dir: folder with YYYY-MM subfolders.
        out_dir: destination for merged files.
        file_name_must_contain: substring to filter input CSVs (e.g., "labels" or "detector").
                                Use None to accept all CSVs.
        write_overall_combined: join all per-month outputs into a single CSV.
        overall_out_csv: explicit path for the overall CSV (defaults to <out_dir>/combined.csv)

    Returns:
        dict with keys: months, produced_months, overall_written(bool), overall_path(optional)
    """
    months = _find_month_dirs(months_base_dir)
    per_month_outputs: List[Tuple[str, pd.DataFrame]] = []

    for month_name in months:
        combined, _picked = _merge_one_month(
            base=months_base_dir,
            month_name=month_name,
            out_dir=out_dir,
            file_name_must_contain=file_name_must_contain,
        )
        if combined is not None:
            per_month_outputs.append((month_name, combined))

    result = {
        "months": months,
        "produced_months": [m for m, _ in per_month_outputs],
        "overall_written": False,
    }

    if write_overall_combined and per_month_outputs:
        overall = per_month_outputs[0][1].copy()
        for _, df in per_month_outputs[1:]:
            overall = overall.join(df, how="outer")

        if overall_out_csv is None:
            overall_out_csv = os.path.join(out_dir, "combined.csv")
        os.makedirs(out_dir, exist_ok=True)
        overall.sort_index().to_csv(overall_out_csv, index_label="sensor_id")
        print(f"\n🧩 ALL-MONTHS combined CSV → {overall_out_csv}")
        print(f"   Months merged: {len(per_month_outputs)} / {len(months)}")

        result["overall_written"] = True
        result["overall_path"] = overall_out_csv
    else:
        print("\n(Overall combined CSV disabled or no month outputs produced.)")

    return result


# ----------------------------- CLI -----------------------------

if __name__ == "__main__":
    # Defaults mirror your example paths; edit as needed or call via main.py
    MONTHS_BASE_DIR = r"test_closures/months"
    OUT_DIR = r"test_closures/merged"
    FILE_NAME_MUST_CONTAIN = "labels"    # e.g., "labels" or "detector"
    WRITE_OVERALL_COMBINED = True
    OVERALL_OUT_CSV = os.path.join(OUT_DIR, "clabels.csv")

    run_csv_merger(
        months_base_dir=MONTHS_BASE_DIR,
        out_dir=OUT_DIR,
        file_name_must_contain=FILE_NAME_MUST_CONTAIN,
        write_overall_combined=WRITE_OVERALL_COMBINED,
        overall_out_csv=OVERALL_OUT_CSV,
    )
