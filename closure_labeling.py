# closure_labeling.py
import os
import re
import math
import pandas as pd
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Tuple, Pattern

# ============================== Helpers ==============================

def parse_sensors_from_detectors(det_xml_path: str) -> Tuple[List[str], Dict[str, List[str]]]:
    """
    Parse detectors.add.xml and return:
      - sensors: list of detector IDs (e1Detector@id or @name)
      - lane_to_sensors: mapping lane_id -> [detector_ids]
    """
    if not os.path.exists(det_xml_path):
        raise FileNotFoundError(f"detectors.add.xml not found at {det_xml_path}")

    tree = ET.parse(det_xml_path)
    root = tree.getroot()
    if root.tag != "additional":
        root = root.find("additional") or root

    sensors: List[str] = []
    lane_to_sensors: Dict[str, List[str]] = {}
    for e1 in root.findall(".//e1Detector"):
        did  = e1.get("id") or e1.get("name")
        lane = e1.get("lane")
        if not did or not lane:
            continue
        sensors.append(did)
        lane_to_sensors.setdefault(lane, []).append(did)

    if not sensors:
        print(f"⚠️ No <e1Detector> found in {det_xml_path}")

    sensors.sort()
    return sensors, lane_to_sensors


def _compute_minutes(day_min: int, bin_min: int) -> List[int]:
    """Return [0, bin_min, 2*bin_min, ..., day_min- bin_min]."""
    if bin_min <= 0:
        raise ValueError("bin_min must be > 0")
    return list(range(0, day_min, bin_min))


def _bins_from_interval(start_s: float, end_s: float, bin_min: int, minutes: List[int]) -> List[int]:
    """
    Map a [start_s, end_s) interval in seconds into minute-bin *labels*
    such as [0,5,10,...] for bin_min=5. Returns bin labels present in 'minutes'.
    """
    if end_s <= start_s:
        return []
    bin_sec = bin_min * 60.0
    b0 = max(0, int(start_s // bin_sec))                       # inclusive
    bE = min(len(minutes), int(math.ceil(end_s / float(bin_sec))))  # exclusive
    return [minutes[i] for i in range(b0, bE)]


def make_labels_for_day(
    closed_lanes_csv: str,
    sensors: List[str],
    lane_to_sensors: Dict[str, List[str]],
    out_csv: str,
    bin_min: int = 5,
    day_min: int = 24 * 60,
) -> bool:
    """
    Build a (sensor_id × minutes) matrix for one day and write to CSV.
    Minutes are 0..(day_min-1) in steps of bin_min.

    Labeling rules:
      - 0 = open (no active or imminent closure)
      - Positive value = minutes remaining until closure ends (ceil), if currently closed
      - Negative value = minutes until closure starts (ceil), only within the 2 hours before start
                         (e.g., -20 means 20 minutes before start)
      - Priority: positive (current closure) overrides any negative (upcoming) label.
                  For multiple upcoming closures, keep the one with the smallest absolute value
                  (i.e., the soonest start).
    """
    minutes = _compute_minutes(day_min=day_min, bin_min=bin_min)  # e.g., [0,5,10,...]
    df = pd.DataFrame(0, index=sensors, columns=minutes, dtype=int)
    df.index.name = "sensor_id"

    # Fast mapping from minute label -> (index, bin_start_seconds)
    minute_to_start_s: Dict[int, int] = {m: m * 60 for m in minutes}

    # If no closure file, write all-zero labels.
    if not os.path.exists(closed_lanes_csv) or os.path.getsize(closed_lanes_csv) == 0:
        os.makedirs(os.path.dirname(out_csv), exist_ok=True)
        df.to_csv(out_csv)
        print(f"✅ No closures found, wrote all-zero labels → {out_csv} shape={df.shape[0]}×{df.shape[1]}")
        return True

    try:
        rows = pd.read_csv(closed_lanes_csv)
    except Exception as e:
        print(f"❌ failed to read {closed_lanes_csv}: {e}")
        return False

    required = {"lane_id", "start_s", "end_s"}
    if not required.issubset(set(rows.columns)):
        print(f"❌ {os.path.basename(closed_lanes_csv)} missing columns {required}")
        return False

    TWO_HOURS_S = 120 * 60

    for _, r in rows.iterrows():
        try:
            lane = str(r["lane_id"])
            start_s = float(r["start_s"])
            end_s = float(r["end_s"])
        except Exception:
            continue

        # Expand wildcard like "EDGE_*"
        if lane.endswith("_*"):
            edge_base = lane[:-2]
            expanded_lanes = [lid for lid in lane_to_sensors.keys()
                              if lid == edge_base or lid.startswith(edge_base + "_")]
            if not expanded_lanes:
                continue
            target_lanes = expanded_lanes
        else:
            if lane not in lane_to_sensors:
                continue
            target_lanes = [lane]

        # Bins during closure: [start_s, end_s)
        bins_during = _bins_from_interval(start_s, end_s, bin_min=bin_min, minutes=minutes)

        # Bins before closure (pre-window): [max(0, start_s-2h), start_s)
        pre_start = max(0.0, start_s - TWO_HOURS_S)
        bins_before = _bins_from_interval(pre_start, start_s, bin_min=bin_min, minutes=minutes)

        for lane_id in target_lanes:
            sensor_ids = lane_to_sensors.get(lane_id, [])
            if not sensor_ids:
                continue

            # 1) Positive labels: minutes remaining until end (ceil)
            for m in bins_during:
                t_bin_start = minute_to_start_s[m]
                mins_left = int(math.ceil((end_s - t_bin_start) / 60.0))
                if mins_left <= 0:
                    continue
                for det_id in sensor_ids:
                    if det_id not in df.index:
                        continue
                    cur_val = int(df.at[det_id, m])
                    # Always prefer positive current closure; if both positive, keep the larger (later end)
                    if cur_val <= 0 or mins_left > cur_val:
                        df.at[det_id, m] = mins_left

            # 2) Negative labels: minutes until start (ceil), capped to 2 hours before
            for m in bins_before:
                t_bin_start = minute_to_start_s[m]
                mins_until = int(math.ceil((start_s - t_bin_start) / 60.0))
                if mins_until <= 0:
                    continue
                neg_val = -mins_until
                for det_id in sensor_ids:
                    if det_id not in df.index:
                        continue
                    cur_val = int(df.at[det_id, m])
                    # Do NOT overwrite positive (current closure has priority)
                    if cur_val > 0:
                        continue
                    # If existing is negative, keep the one closer in time (smaller absolute value)
                    if cur_val < 0:
                        if abs(neg_val) < abs(cur_val):
                            df.at[det_id, m] = neg_val
                    else:
                        # was zero → write negative
                        df.at[det_id, m] = neg_val

    os.makedirs(os.path.dirname(out_csv), exist_ok=True)
    df.to_csv(out_csv)
    print(f"✅ Closure countdown (+pre-window) labels → {out_csv} shape={df.shape[0]}×{df.shape[1]}")
    return True



# ============================== Orchestrator ==============================

def run_closure_labeling(
    month_out_dir: str,
    dets_file: str,
    raw_closure_pattern: Optional[Pattern] = None,
    out_suffix: str = ".labels.csv",
    bin_min: int = 5,
    day_min: int = 24 * 60,
    skip_existing: bool = True,
) -> dict:
    """
    - month_out_dir: folder containing day_XX subfolders (or deeper tree)
    - dets_file: path to detectors.add.xml
    - raw_closure_pattern: compiled regex for raw closure CSV filenames
        Default accepts either:
           closed_lanes_YYYYMMDD.csv  or  closed_lanes_dayXX.csv
    - out_suffix: suffix for labeled CSVs (default ".labels.csv")
    - bin_min: label bin size in minutes (default 5)
    - day_min: length of day in minutes (default 1440)
    - skip_existing: if True, do not overwrite existing label files

    Returns summary dict with converted/skipped/failed counts.
    """
    if raw_closure_pattern is None:
        raw_closure_pattern = re.compile(r"^closed_lanes_(?:\d{8}|day_\d{2})\.csv$", re.IGNORECASE)

    try:
        sensors, lane_to_sensors = parse_sensors_from_detectors(dets_file)
    except Exception as e:
        print(f"❌ cannot read detectors: {e}")
        return {"converted": 0, "skipped": 0, "failed": 1, "error": str(e)}

    converted = skipped = failed = 0
    processed_files: List[str] = []

    for dirpath, _, filenames in os.walk(month_out_dir):
        for fn in filenames:
            # Ignore already-derived artifacts
            low = fn.lower()
            if low.endswith(".wide.csv") or low.endswith(out_suffix.lower()):
                continue
            if not raw_closure_pattern.match(fn):
                continue

            in_csv = os.path.join(dirpath, fn)
            out_csv = os.path.splitext(in_csv)[0] + out_suffix

            if skip_existing and os.path.exists(out_csv) and os.path.getsize(out_csv) > 0:
                print(f"⏭️  skip (exists): {out_csv}")
                skipped += 1
                processed_files.append(in_csv)
                continue

            try:
                ok = make_labels_for_day(
                    closed_lanes_csv=in_csv,
                    sensors=sensors,
                    lane_to_sensors=lane_to_sensors,
                    out_csv=out_csv,
                    bin_min=bin_min,
                    day_min=day_min,
                )
                if ok:
                    converted += 1
                else:
                    failed += 1
                processed_files.append(in_csv)
            except Exception as e:
                print(f"❌ exception converting {in_csv}: {e}")
                failed += 1

    print("\n=== Summary ===")
    print(f"Converted: {converted}  Skipped: {skipped}  Failed: {failed}")

    return {
        "converted": converted,
        "skipped": skipped,
        "failed": failed,
        "processed_files": processed_files,
        "month_out_dir": month_out_dir,
        "dets_file": dets_file,
        "bin_min": bin_min,
        "day_min": day_min,
        "out_suffix": out_suffix,
        "pattern": raw_closure_pattern.pattern,
    }


# ============================== Standalone ==============================

if __name__ == "__main__":
    # Defaults matching your example; safe to edit or override via a master script
    MONTH_OUT_DIR = r"C:\Users\erays\Desktop\traffic_simulation\new\test_closures\months\2026-02"
    CONFIG_DIR    = os.path.join(os.path.dirname(MONTH_OUT_DIR), "config")
    DETS_FILE     = os.path.join(CONFIG_DIR, "detectors.add.xml")

    run_closure_labeling(
        month_out_dir=MONTH_OUT_DIR,
        dets_file=DETS_FILE,
        raw_closure_pattern=re.compile(r"^closed_lanes_(?:\d{8}|day_\d{2})\.csv$", re.IGNORECASE),
        out_suffix=".labels.csv",
        bin_min=5,
        day_min=24*60,
        skip_existing=True,
    )
