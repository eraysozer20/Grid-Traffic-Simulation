# xml_csv_converter.py
import os
import re
import xml.etree.ElementTree as ET
from typing import List, Optional, Tuple

import pandas as pd

# --------- Patterns ----------
MONTH_DIR_RE = re.compile(r"^\d{4}-\d{2}$")  # YYYY-MM
DAY_DIR_RE   = re.compile(r"^day_\d{2}$")    # day_01
DAY_SEC = 24 * 60 * 60
MAX_MINUTE = 1439


# ========= Core conversion =========
def convert_e1_to_csv(det_xml_path: str, out_csv_path: str) -> Optional[pd.DataFrame]:
    import os
    import xml.etree.ElementTree as ET
    import pandas as pd

    DAY_SEC = 24 * 60 * 60
    MAX_MINUTE = 1439

    if not os.path.exists(det_xml_path):
        print(f"❌ detector file not found: {det_xml_path}")
        return None
    if os.path.getsize(det_xml_path) < 50:
        print(f"⚠️ detector file is very small ({os.path.getsize(det_xml_path)} bytes): {det_xml_path}")

    rows = []
    try:
        context = ET.iterparse(det_xml_path, events=("start", "end"))
        _, root = next(context)
        for event, elem in context:
            tag = elem.tag.split('}', 1)[-1]
            if event != "end" or tag != "interval":
                continue

            det_id = elem.get("id")
            if not det_id:
                elem.clear()
                continue

            # time bin
            try:
                begin = float(elem.get("begin", 0.0))
            except (TypeError, ValueError):
                begin = 0.0
            try:
                end = float(elem.get("end", begin))
            except (TypeError, ValueError):
                end = begin

            # drop intervals starting at/after 24h (drain ticks)
            if begin >= DAY_SEC:
                elem.clear()
                continue

            minute = int(begin // 60)
            if minute > MAX_MINUTE:
                minute = MAX_MINUTE

            bin_seconds = max(0.0, end - begin)

            # Prefer explicit counts
            count = None
            for key in ("nVehEntered", "entered", "nVehPassed", "nVeh"):
                v = elem.get(key)
                if v is not None:
                    try:
                        count = int(round(float(v)))
                        break
                    except (TypeError, ValueError):
                        pass

            # Fallback from flow (veh/h)
            if count is None:
                flow = elem.get("flow")
                if flow is not None:
                    try:
                        flow_val = float(flow)
                        count = int(round(flow_val * (bin_seconds / 3600.0)))
                    except (TypeError, ValueError):
                        count = 0
                else:
                    count = 0

            rows.append({"sensor_id": det_id, "minute": minute, "count": count})
            elem.clear()

        root.clear()

    except ET.ParseError as e:
        print(f"❌ detector parse error in {det_xml_path}: {e}")
        return None
    except StopIteration:
        print(f"❌ detector XML appears empty: {det_xml_path}")
        return None

    if not rows:
        print(f"⚠️ No detector intervals found in {det_xml_path}")
        return None

    df = pd.DataFrame(rows)
    piv = (df.pivot_table(index="sensor_id", columns="minute", values="count", aggfunc="sum")
             .fillna(0)
             .astype(int))

    # keep only minutes ≤ 1439
    piv = piv[[c for c in piv.columns if int(c) <= MAX_MINUTE]]

    os.makedirs(os.path.dirname(out_csv_path) or ".", exist_ok=True)
    piv.to_csv(out_csv_path)
    print(f"✅ Detector CSV → {out_csv_path}  shape={piv.shape[0]}×{piv.shape[1]}")
    return piv



# ========= Discovery helpers =========
def _find_month_day_paths(months_base_dir: str, root_fallback: str):
    """
    Yield (month_path, [(day_name, day_path), ...]).
    If months_base_dir is missing, fall back to scanning root_fallback recursively
    and treat any folder containing detector XML as a 'day'.
    """
    if os.path.isdir(months_base_dir):
        for entry in sorted(os.listdir(months_base_dir)):
            if not MONTH_DIR_RE.match(entry):
                continue
            month_path = os.path.join(months_base_dir, entry)
            if not os.path.isdir(month_path):
                continue
            days: List[Tuple[str, str]] = []
            for dname in sorted(os.listdir(month_path)):
                if DAY_DIR_RE.match(dname):
                    dpath = os.path.join(month_path, dname)
                    if os.path.isdir(dpath):
                        days.append((dname, dpath))
            yield month_path, days
    else:
        # Fallback: treat ROOT_FALLBACK as a single “month” and recurse for days
        all_days: List[Tuple[str, str]] = []
        for dirpath, _, filenames in os.walk(root_fallback):
            if any(fn.lower().endswith(".xml") and "detector" in fn.lower() for fn in filenames):
                all_days.append((os.path.basename(dirpath), dirpath))
        yield root_fallback, sorted(all_days)


def _collect_day_detector_xmls(day_path: str) -> List[str]:
    """Return all detector/e1 XML files in a day folder."""
    xs: List[str] = []
    for fn in os.listdir(day_path):
        if fn.lower().endswith(".xml"):
            low = fn.lower()
            if any(h in low for h in ("detector", "detectors", "e1")):
                xs.append(os.path.join(day_path, fn))
    return sorted(xs)


# ========= Rollups =========
def _day_sort_key(col_name) -> Tuple[int, int, str]:
    """
    Sort helper for columns like 'day_01', 'day_2', 'misc'.
    Returns a tuple so columns with digits come first, sorted numerically,
    then non-digit names sorted lexicographically.
    """
    s = str(col_name)
    m = re.search(r"\d+", s)
    if m:
        return (0, int(m.group()), s)
    return (1, 0, s)


def _build_month_rollups(
    month_path: str,
    per_day_frames: List[Tuple[str, Optional[pd.DataFrame]]],
    out_name_minutes: str = "monthly_minutes.csv",
    out_name_daily: str = "monthly_daily_totals.csv",
) -> None:
    """
    Build:
      - monthly_minutes.csv : sensor_id × (absolute minute across month)
      - monthly_daily_totals.csv : sensor_id × day_## totals
    """
    if not per_day_frames:
        return

    # Union of all sensor_ids across days
    all_sensors = set()
    for _, df in per_day_frames:
        if df is not None:
            all_sensors.update(df.index.tolist())
    if not all_sensors:
        return
    all_sensors = sorted(all_sensors)

    monthly_blocks: List[pd.DataFrame] = []
    day_totals_frames: List[pd.Series] = []

    for day_idx, (day_name, df) in enumerate(per_day_frames):
        if df is None:
            continue

        # Ensure consistent index (all sensors) and integer values
        day_df = df.reindex(all_sensors).fillna(0).astype(int)

        # Shift minute columns by day offset (1440 minutes per day)
        if len(day_df.columns) > 0:
            shifted_cols = {int(c): int(c) + day_idx * 1440 for c in day_df.columns}
            # Rename using the mapping; handle cases where columns might be str
            day_df.columns = [shifted_cols[int(c)] for c in day_df.columns]
            monthly_blocks.append(day_df)

        # Daily totals
        totals = day_df.sum(axis=1)
        totals.name = day_name
        day_totals_frames.append(totals)

    # Minutes rollup
    if monthly_blocks:
        monthly_minutes = (
            pd.concat(monthly_blocks, axis=1)
              .fillna(0)
              .astype(int)
        )
        # Order columns strictly ascending
        monthly_minutes = monthly_minutes.loc[all_sensors, sorted(map(int, monthly_minutes.columns))]
        out_minutes = os.path.join(month_path, out_name_minutes)
        monthly_minutes.to_csv(out_minutes)
        print(f"📦 Monthly (minutes) CSV → {out_minutes}  "
              f"shape={monthly_minutes.shape[0]}×{monthly_minutes.shape[1]}")

    # Daily totals rollup
    if day_totals_frames:
        monthly_daily = (
            pd.concat(day_totals_frames, axis=1)
              .fillna(0)
              .astype(int)
        )
        # Order columns by numeric day if present
        monthly_daily = monthly_daily.reindex(columns=sorted(monthly_daily.columns, key=_day_sort_key))
        out_daily = os.path.join(month_path, out_name_daily)
        monthly_daily.to_csv(out_daily)
        print(f"📦 Monthly (daily totals) CSV → {out_daily}  "
              f"shape={monthly_daily.shape[0]}×{monthly_daily.shape[1]}")


# ========= Public entrypoint =========
def run_xml_csv_converter(
    months_base_dir: str,
    root_fallback: str,
    skip_existing: bool = True,
    out_name_minutes: str = "monthly_minutes.csv",
    out_name_daily: str = "monthly_daily_totals.csv",
) -> dict:
    """
    Orchestrates month/day discovery, per-day conversions, and rollups.
    Returns a summary dict with counts and month info.
    """
    converted = 0
    skipped   = 0
    failed    = 0
    month_summaries = []

    for month_path, days in _find_month_day_paths(months_base_dir, root_fallback):
        print(f"\n===== Month: {month_path}  (days found: {len(days)}) =====")
        per_day_frames: List[Tuple[str, Optional[pd.DataFrame]]] = []

        for day_name, day_path in days:
            det_xmls = _collect_day_detector_xmls(day_path)
            if not det_xmls:
                print(f"  • {day_name}: no detector XMLs")
                per_day_frames.append((day_name, None))
                continue

            day_frames: List[pd.DataFrame] = []
            for det_xml in det_xmls:
                csv_path = os.path.splitext(det_xml)[0] + ".csv"

                if skip_existing and os.path.exists(csv_path) and os.path.getsize(csv_path) > 0:
                    print(f"⏭️  skip (exists): {csv_path}")
                    try:
                        df_existing = pd.read_csv(csv_path, index_col=0)
                        # Ensure int dtype (robust against prior versions)
                        df_existing = df_existing.fillna(0).astype(int)
                        day_frames.append(df_existing)
                        skipped += 1
                    except Exception:
                        ok_df = convert_e1_to_csv(det_xml, csv_path)
                        if ok_df is None:
                            failed += 1
                        else:
                            converted += 1
                            day_frames.append(ok_df)
                    continue

                try:
                    df_new = convert_e1_to_csv(det_xml, csv_path)
                    if df_new is None:
                        failed += 1
                    else:
                        converted += 1
                        day_frames.append(df_new)
                except Exception as e:
                    print(f"❌ exception converting {det_xml}: {e}")
                    failed += 1

            # Merge multiple detector files within the same day by summing matching cells
            if day_frames:
                base = day_frames[0].copy()
                for extra in day_frames[1:]:
                    base = base.add(extra, fill_value=0)
                base = base.fillna(0).astype(int)
                per_day_frames.append((day_name, base))
            else:
                per_day_frames.append((day_name, None))

        _build_month_rollups(
            month_path,
            per_day_frames,
            out_name_minutes=out_name_minutes,
            out_name_daily=out_name_daily,
        )
        month_summaries.append({
            "month_path": month_path,
            "days": [d for d, _ in per_day_frames],
        })

    summary = {
        "converted": converted,
        "skipped": skipped,
        "failed": failed,
        "months": month_summaries,
    }
    print("\n=== Summary ===")
    print(f"Converted: {converted}  Skipped: {skipped}  Failed: {failed}")
    return summary


# --- Compatibility wrapper for main.py ---
def run_xml_to_csv(
    months_base_dir: str,
    root_fallback: str | None = None,
    skip_existing: bool = True,
    out_name_minutes: str = "monthly_minutes.csv",
    out_name_daily: str = "monthly_daily_totals.csv",
) -> dict:
    """
    Thin wrapper so main.py can call run_xml_to_csv(...).
    Delegates to run_xml_csv_converter(...).
    """
    return run_xml_csv_converter(
        months_base_dir=months_base_dir,
        root_fallback=root_fallback or months_base_dir,
        skip_existing=skip_existing,
        out_name_minutes=out_name_minutes,
        out_name_daily=out_name_daily,
    )


# ========= Standalone CLI =========
if __name__ == "__main__":
    # Defaults keep your previous behavior; feel free to edit or pass via master script
    MONTHS_BASE_DIR = r"C:\Users\erays\Desktop\traffic_simulation\new\closure_on_test_month_run\months"
    ROOT_FALLBACK   = r"C:\Users\erays\Desktop\traffic_simulation\new\month_run"
    SKIP_EXISTING   = True

    run_xml_csv_converter(
        months_base_dir=MONTHS_BASE_DIR,
        root_fallback=ROOT_FALLBACK,
        skip_existing=SKIP_EXISTING,
    )
