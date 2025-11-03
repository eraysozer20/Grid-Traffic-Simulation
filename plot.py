# plot_sensor_daily.py
import os
import re
import math
import glob
import pandas as pd
import matplotlib.pyplot as plt

# ==================== CONFIG ====================
MONTH_DIR  = r"C:\Users\erays\Desktop\traffic_simulation\new\closure_month_run\closure_month_outputs"
SENSOR_ID  = "det_A2B2_0"   # <-- change to the detector you want
PLOTS_DIR  = os.path.join(MONTH_DIR, "_plots", SENSOR_ID)
MAKE_OVERLAY = True         # also make one overlay plot (all days)
MAKE_TOTALS  = True         # also make bar chart of total counts per day
DPI = 140
# =================================================

def _to_numeric_columns(df: pd.DataFrame) -> pd.DataFrame:
    """Coerce pivot columns (minute bins) to numeric and sort."""
    out = df.copy()
    out.columns = pd.to_numeric(out.columns, errors="coerce")
    out = out.loc[:, ~out.columns.isna()]
    out = out.sort_index(axis=1)
    return out

def _find_detector_csv(day_dir: str) -> str | None:
    """Find detector CSV inside a day_XY folder."""
    # Expected name (detectors_dayXX.csv)
    day_num = re.search(r"day[_\- ]?(\d+)", os.path.basename(day_dir), re.I)
    if day_num:
        guess = os.path.join(day_dir, f"detectors_day{int(day_num.group(1)):02d}.csv")
        if os.path.exists(guess):
            return guess
    # Fallback: any detectors*.csv in folder
    candidates = sorted(glob.glob(os.path.join(day_dir, "detectors*.csv")))
    return candidates[0] if candidates else None

def _load_sensor_series(csv_path: str, sensor_id: str) -> pd.Series | None:
    """Load one sensor's row as a numeric Series indexed by minute bins."""
    df = pd.read_csv(csv_path, index_col=0)
    df = _to_numeric_columns(df)

    if sensor_id not in df.index:
        # try case-insensitive
        lower_map = {ix.lower(): ix for ix in df.index}
        if sensor_id.lower() in lower_map:
            true_id = lower_map[sensor_id.lower()]
        else:
            print(f"⚠️  Sensor '{sensor_id}' not found in {os.path.basename(csv_path)}; skipping.")
            return None
    else:
        true_id = sensor_id

    s = df.loc[true_id].copy()
    s = pd.to_numeric(s, errors="coerce").fillna(0)
    return s

def _infer_bin_minutes(index_like) -> int:
    """Infer bin size (in minutes) from numeric minute columns; default 5."""
    vals = sorted(list(index_like))
    if len(vals) < 2:
        return 5
    diffs = [b - a for a, b in zip(vals[:-1], vals[1:])]
    # use the smallest positive diff
    diffs = [d for d in diffs if d > 0]
    if not diffs:
        return 5
    # round to nearest integer minute
    return max(1, int(round(min(diffs))))

def _plot_one_day(series: pd.Series, day_label: str, bin_min: int, out_png: str):
    """Plot 24h profile for one day."""
    # Build full-day index based on inferred bin size
    full_index = list(range(0, 24*60, bin_min))
    s = series.reindex(full_index, fill_value=0)

    x_hours = [m/60.0 for m in s.index]
    plt.figure(figsize=(10, 4))
    plt.plot(x_hours, s.values, linewidth=1.5)
    plt.xlabel("Hour of day")
    plt.ylabel(f"Vehicles per {bin_min} min")
    plt.title(f"{SENSOR_ID} — {day_label}")
    plt.xlim(0, 24)
    plt.grid(True, linewidth=0.5)
    plt.tight_layout()
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    plt.savefig(out_png, dpi=DPI)
    plt.close()

def _plot_overlay(day_series_dict: dict[str, pd.Series], bin_min: int, out_png: str):
    """Overlay all days for the sensor (thin lines) + average thicker."""
    # Union of all minute bins across days
    all_bins = sorted(set().union(*[s.index for s in day_series_dict.values()]))
    # Ensure common index; reindex missing to 0
    aligned = {d: s.reindex(all_bins, fill_value=0) for d, s in day_series_dict.items()}
    # Average profile
    df = pd.DataFrame(aligned)
    avg = df.mean(axis=1)

    x_hours = [m/60.0 for m in all_bins]
    plt.figure(figsize=(11, 5))
    for d, s in aligned.items():
        plt.plot(x_hours, s.values, linewidth=0.8, alpha=0.5)
    plt.plot(x_hours, avg.values, linewidth=2.2)  # average
    plt.xlabel("Hour of day")
    plt.ylabel(f"Vehicles per {bin_min} min")
    plt.title(f"{SENSOR_ID} — All days overlay (thin) + average (thick)")
    plt.xlim(0, 24)
    plt.grid(True, linewidth=0.5)
    plt.tight_layout()
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    plt.savefig(out_png, dpi=DPI)
    plt.close()

def _plot_daily_totals(day_totals: pd.Series, out_png: str):
    plt.figure(figsize=(10, 4))
    day_totals.plot(kind="bar", width=0.85)
    plt.ylabel("Vehicles per day (total)")
    plt.title(f"{SENSOR_ID} — Daily totals")
    plt.xticks(rotation=45, ha="right")
    plt.tight_layout()
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    plt.savefig(out_png, dpi=DPI)
    plt.close()

def main():
    # find day folders like day_01, day_02, ...
    day_dirs = sorted([d for d in glob.glob(os.path.join(MONTH_DIR, "day_*")) if os.path.isdir(d)])
    if not day_dirs:
        print(f"❌ No day_* folders under {MONTH_DIR}")
        return

    day_series = {}     # day_label -> Series (minute bins)
    day_totals = {}     # day_label -> total

    for d in day_dirs:
        label = os.path.basename(d)
        csv_path = _find_detector_csv(d)
        if not csv_path:
            print(f"⚠️  No detector CSV found in {label}; skipping.")
            continue
        s = _load_sensor_series(csv_path, SENSOR_ID)
        if s is None:
            continue

        # infer bin size from this day's columns
        bin_min = _infer_bin_minutes(s.index)
        # plot one-day profile
        out_png = os.path.join(PLOTS_DIR, f"{label}.png")
        _plot_one_day(s, label, bin_min, out_png)

        day_series[label] = s
        day_totals[label] = float(s.sum())

        print(f"✅ Plotted {label} → {out_png}")

    if not day_series:
        print("❌ No days plotted (sensor missing everywhere?).")
        return

    # Optional overlay
    if MAKE_OVERLAY:
        # Use the most common bin size seen
        common_bin = min((_infer_bin_minutes(s.index) for s in day_series.values()), default=5)
        overlay_png = os.path.join(PLOTS_DIR, "_overlay_all_days.png")
        _plot_overlay(day_series, common_bin, overlay_png)
        print(f"✅ Overlay saved → {overlay_png}")

    # Optional daily totals
    if MAKE_TOTALS:
        s_totals = pd.Series(day_totals).sort_index()
        totals_png = os.path.join(PLOTS_DIR, "_daily_totals.png")
        _plot_daily_totals(s_totals, totals_png)
        print(f"✅ Daily totals saved → {totals_png}")

if __name__ == "__main__":
    main()
