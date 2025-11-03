# create_DESPT_input.py
#!/usr/bin/env python3
"""
Build DESPT-style inputs from wide CSVs:

1) Wide time-series (detectors or labels)  → HDF5
   - Accepts columns like:
       a) 'day_01_0', 'day_01_5', ...
       b) 'YYYY-MM_day_01_0'  (from merged per-month outputs)
   - (b) timestamps are parsed from YYYY-MM + day_XX + minute
   - (a) timestamps are built from START_DATE

2) Distance matrix CSV                       → adjacency pickles (diag0 / diag1)

3) (NEW) Closure labels CSV                  → labels HDF5

4) (NEW) Closure effect matrix CSV (rect/square) → PKL
   - Pickled payload: {
       "row_ids", "col_ids",
       "row_id_to_index", "col_id_to_index",
       "matrix"  (np.float32)
     }
"""

import os
import re
import pickle
from dataclasses import dataclass
from datetime import timedelta
from typing import List, Tuple, Optional, Dict

import numpy as np
import pandas as pd

# ----------------------------- Patterns -----------------------------

DAY_ONLY_RE    = re.compile(r"^day_(\d+)_(\d+)$", re.IGNORECASE)
MONTH_AWARE_RE = re.compile(r"^(\d{4}-\d{2})_day_(\d{2})_(\d+)$", re.IGNORECASE)


# ============================== Config ==============================

@dataclass
class DesptConfig:
    # Inputs
    input_csv: str = r"test_closures\merged\clabels.csv"           # wide time-series CSV (detector OR labels)
    dist_csv: Optional[str] = r"combined_outputs\sensor_distance_matrix.csv"   # distance matrix CSV (optional)
    labels_csv: Optional[str] = None                                # (NEW) wide closure-labels CSV (optional)
    effect_matrix_csv: Optional[str] = None                         # (NEW) arbitrary effect matrix CSV (optional)

    # Outputs
    out_dir: str = r"combined_outputs\DESPT\SMALL_C"
    h5_key_timeseries: str = "sensor_readings"                      # HDF5 key for main timeseries
    h5_key_labels: str = "closure_labels"                           # (NEW) HDF5 key for labels
    adj_basename: str = "effect_matrix"                             # basename for adjacency pickles
    effect_matrix_pkl: str = "closure_effect.pkl"                   # (NEW) output PKL for effect matrix

    # Time interpretation
    start_date: str = "2025-01-01 00:00:00"                         # only used when headers are 'day_XX_min'
    step_min: int = 5

    # Adjacency
    threshold: float = 0.1
    write_adj_csv: bool = True


# ======================== Time-column parsing ========================

def parse_time_cols(cols: List[str],
                    start_date: Optional[pd.Timestamp]) -> pd.DataFrame:
    """
    From headers like:
      - 'day_01_0'              -> use start_date + (day-1, minute)
      - 'YYYY-MM_day_01_0'      -> compute from embedded year-month + day + minute
    Returns DataFrame with columns: col, timestamp
    """
    rows: List[Tuple[str, pd.Timestamp]] = []

    for c in cols:
        m2 = MONTH_AWARE_RE.match(c)
        if m2:
            ym, day_s, minute_s = m2.group(1), m2.group(2), m2.group(3)
            year, month = map(int, ym.split("-"))
            day    = int(day_s)
            minute = int(minute_s)
            ts = pd.Timestamp(year=year, month=month, day=day,
                              hour=0, minute=0, second=0) + timedelta(minutes=minute)
            rows.append((c, ts))
            continue

        m1 = DAY_ONLY_RE.match(c)
        if m1:
            if start_date is None:
                raise ValueError(f"Column '{c}' matches day_* format but START_DATE is not set.")
            day    = int(m1.group(1))
            minute = int(m1.group(2))
            ts = start_date + timedelta(days=day - 1, minutes=minute)
            rows.append((c, ts))
            continue

        # ignore non-time columns silently

    if not rows:
        raise ValueError("No time-like columns found. Expected 'day_XX_0' or 'YYYY-MM_day_XX_0' headers.")

    meta = pd.DataFrame(rows, columns=["col", "timestamp"]).sort_values("timestamp").reset_index(drop=True)
    return meta


# ======================== Adjacency helpers ========================

def get_adjacency_matrix(dist_mx_array: np.ndarray, normalized_k: float = 0.1) -> np.ndarray:
    """
    Thresholded RBF kernel on distances.
    If std == 0, fall back to unit scale.
    """
    distances = dist_mx_array[~np.isinf(dist_mx_array)].flatten()
    std = distances.std()
    if std == 0:
        adj = np.exp(-(dist_mx_array / 1.0) ** 2)
    else:
        adj = np.exp(-np.square(dist_mx_array / std))
    adj[adj < normalized_k] = 0
    return adj


# =========================== Converters ===========================

def make_hdf_from_wide(input_csv: str,
                       out_h5: str,
                       h5_key: str,
                       start_date_str: Optional[str]) -> Tuple[pd.DataFrame, List[str]]:
    """
    Load wide CSV and write HDF5 with timestamp index (rows) and sensors as columns.
    Works for both detector readings and label matrices as long as columns match the expected patterns.
    Returns (written_dataframe, sensor_ids)
    """
    if not os.path.exists(input_csv):
        raise FileNotFoundError(f"Input CSV not found: {input_csv}")

    df_wide = pd.read_csv(input_csv, sep=None, engine="python")
    if "sensor_id" not in df_wide.columns:
        raise ValueError("Expected a 'sensor_id' column in the wide CSV.")

    df_wide["sensor_id"] = df_wide["sensor_id"].astype(str)

    # Merge potential duplicate sensor rows
    if df_wide["sensor_id"].duplicated().any():
        df_wide = df_wide.groupby("sensor_id", as_index=False).sum(numeric_only=True)

    # Identify time-like columns
    time_cols = [c for c in df_wide.columns if c != "sensor_id" and
                 (DAY_ONLY_RE.match(str(c)) or MONTH_AWARE_RE.match(str(c)))]

    meta = parse_time_cols(
        cols=time_cols,
        start_date=(pd.Timestamp(start_date_str) if start_date_str else None),
    )
    ordered_time_cols = meta["col"].tolist()

    # Keep only sensor_id + time cols
    df_wide = df_wide[["sensor_id"] + ordered_time_cols].set_index("sensor_id")

    # Force numeric
    df_wide[ordered_time_cols] = df_wide[ordered_time_cols].apply(pd.to_numeric, errors="coerce")

    # PeMS-like layout: rows=time, cols=sensors
    data = df_wide[ordered_time_cols].T
    data.index = pd.DatetimeIndex(meta["timestamp"].values, name="timestamp")
    data = data.reindex(sorted(map(str, data.columns)), axis=1)

    os.makedirs(os.path.dirname(os.path.abspath(out_h5)) or ".", exist_ok=True)
    with pd.HDFStore(out_h5, mode="w") as store:
        # data_columns=True will index all columns for queries; safe for time-indexed tables
        store.put(h5_key, data, format="table", data_columns=True, complevel=9, complib="zlib")

    print(f"✅ HDF5 created: {out_h5}  (/{h5_key})  shape={data.shape}")
    print(f"   Time range: {data.index.min()} → {data.index.max()}")
    return data, list(map(str, data.columns))


def make_adj_from_dist(dist_csv: str,
                       out_dir: str,
                       basename: str,
                       threshold: float,
                       write_csv_inspection: bool = True) -> Tuple[str, str, List[str]]:
    """
    Build adjacency pickles (diag0 & diag1) from a distance matrix CSV.
    Returns (pkl_diag0_path, pkl_diag1_path, sensor_ids)
    """
    if not os.path.exists(dist_csv):
        raise FileNotFoundError(f"Distance CSV not found: {dist_csv}")

    os.makedirs(out_dir, exist_ok=True)
    df = pd.read_csv(dist_csv)

    first_col = df.columns[0]
    looks_like_id_col = (
        df[first_col].dtype == object
        or df[first_col].astype(str).str.contains(r"[A-Za-z_]").any()
    )

    if looks_like_id_col:
        sensor_ids = df[first_col].astype(str).tolist()
        df2 = df.set_index(first_col)
        # if header cols equal sensor_ids, align square
        if set(map(str, df2.columns)) == set(sensor_ids):
            df2 = df2.reindex(index=sensor_ids, columns=sensor_ids)
        dist = df2.to_numpy(dtype=float)
    else:
        col_labels = list(df.columns)
        if any(not re.match(r"^\d+$", str(c)) for c in col_labels):
            sensor_ids = [str(c) for c in col_labels]
        else:
            sensor_ids = [str(i) for i in range(len(df))]
        dist = df.to_numpy(dtype=float)

    dist = np.array(dist, dtype=float)
    np.fill_diagonal(dist, 0.0)
    dist[~np.isfinite(dist)] = np.inf
    dist[dist < 0] = np.inf

    adj_base = get_adjacency_matrix(dist, normalized_k=threshold).astype(np.float32)

    adj_diag0 = adj_base.copy()
    np.fill_diagonal(adj_diag0, 0.0)

    adj_diag1 = adj_base.copy()
    np.fill_diagonal(adj_diag1, 1.0)

    sensor_id_to_index = {sid: i for i, sid in enumerate(sensor_ids)}
    adj_list_diag0 = [sensor_ids, sensor_id_to_index, adj_diag0]
    adj_list_diag1 = [sensor_ids, sensor_id_to_index, adj_diag1]

    pkl0 = os.path.join(out_dir, f"{basename}_diag0.pkl")
    pkl1 = os.path.join(out_dir, f"{basename}_diag1.pkl")

    with open(pkl0, "wb") as f:
        pickle.dump(adj_list_diag0, f, protocol=4)
    with open(pkl1, "wb") as f:
        pickle.dump(adj_list_diag1, f, protocol=4)

    if write_csv_inspection:
        pd.DataFrame(adj_diag0, index=sensor_ids, columns=sensor_ids).to_csv(
            os.path.join(out_dir, f"{basename}_diag0.csv")
        )
        pd.DataFrame(adj_diag1, index=sensor_ids, columns=sensor_ids).to_csv(
            os.path.join(out_dir, f"{basename}_diag1.csv")
        )

    print("✅ wrote adjacency pickles:")
    print(" ", pkl0)
    print(" ", pkl1)
    print(f"   sensors: {len(sensor_ids)}, adj shape: {adj_diag0.shape}")
    return pkl0, pkl1, sensor_ids


# -------------------- (NEW) effect matrix → PKL --------------------

def _read_matrix_csv(matrix_csv: str) -> Tuple[np.ndarray, List[str], List[str]]:
    """
    Robust CSV loader for arbitrary rectangular matrices.
    If first column looks like IDs, treat as row labels; else use numeric index.
    Column labels always come from header.
    """
    df = pd.read_csv(matrix_csv)
    if df.empty:
        raise ValueError(f"Matrix CSV is empty: {matrix_csv}")

    # Column labels
    col_ids = [str(c) for c in df.columns[1:]] if df.columns.size > 1 else []

    # Row labels detection
    first_col = df.columns[0]
    first_is_id = (
        df[first_col].dtype == object
        or df[first_col].astype(str).str.contains(r"[A-Za-z_]").any()
    )
    if first_is_id and df.columns.size > 1:
        row_ids = df[first_col].astype(str).tolist()
        mat = df.iloc[:, 1:].to_numpy(dtype=float)
    else:
        row_ids = [str(i) for i in range(df.shape[0])]
        col_ids = [str(c) for c in df.columns]
        mat = df.to_numpy(dtype=float)

    mat = np.array(mat, dtype=float)
    mat[~np.isfinite(mat)] = 0.0
    return mat, row_ids, col_ids


def make_effect_matrix_pkl(effect_matrix_csv: str, out_pkl: str) -> str:
    """
    Convert any rectangular matrix CSV (e.g., closure effect matrix) to PKL.
    Payload schema:
      {
        "row_ids": [...],
        "col_ids": [...],
        "row_id_to_index": {id: i},
        "col_id_to_index": {id: j},
        "matrix": np.float32 array
      }
    """
    if not os.path.exists(effect_matrix_csv):
        raise FileNotFoundError(f"Effect matrix CSV not found: {effect_matrix_csv}")

    mat, row_ids, col_ids = _read_matrix_csv(effect_matrix_csv)
    payload = {
        "row_ids": row_ids,
        "col_ids": col_ids,
        "row_id_to_index": {rid: i for i, rid in enumerate(row_ids)},
        "col_id_to_index": {cid: j for j, cid in enumerate(col_ids)},
        "matrix": mat.astype(np.float32),
    }

    os.makedirs(os.path.dirname(os.path.abspath(out_pkl)) or ".", exist_ok=True)
    with open(out_pkl, "wb") as f:
        pickle.dump(payload, f, protocol=4)

    print(f"✅ Effect matrix PKL written: {out_pkl}  shape={mat.shape}")
    return out_pkl


# =========================== Orchestrator ===========================

def run_despt_prep(cfg: DesptConfig) -> Dict[str, str]:
    """
    Runs the requested conversions based on cfg.
    Returns a dict of produced artifact paths.
    """
    produced: Dict[str, str] = {}
    out_dir = os.path.abspath(cfg.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    # 1) Main wide CSV → HDF5 (timeseries)
    out_h5_timeseries = os.path.join(out_dir, "sensor_readings.h5")
    data, sensor_ids_from_hdf = make_hdf_from_wide(
        input_csv=cfg.input_csv,
        out_h5=out_h5_timeseries,
        h5_key=cfg.h5_key_timeseries,
        start_date_str=cfg.start_date
    )
    produced["h5_timeseries"] = out_h5_timeseries

    # 2) Distance CSV → adjacency pickles
    if cfg.dist_csv:
        p0, p1, sensor_ids_from_adj = make_adj_from_dist(
            dist_csv=cfg.dist_csv,
            out_dir=out_dir,
            basename=cfg.adj_basename,
            threshold=cfg.threshold,
            write_csv_inspection=cfg.write_adj_csv
        )
        produced["adj_diag0_pkl"] = p0
        produced["adj_diag1_pkl"] = p1

        # Optional sensor-set sanity
        s1, s2 = set(map(str, sensor_ids_from_hdf)), set(map(str, sensor_ids_from_adj))
        if s1 != s2:
            inter = len(s1 & s2)
            print(f"⚠️ Sensor id mismatch between HDF columns ({len(s1)}) and adjacency ({len(s2)}). "
                  f"Intersection={inter}. Consider reconciling names.")

    # 3) (NEW) Closure labels CSV → labels HDF5
    if cfg.labels_csv:
        out_h5_labels = os.path.join(out_dir, "closure_labels.h5")
        make_hdf_from_wide(
            input_csv=cfg.labels_csv,
            out_h5=out_h5_labels,
            h5_key=cfg.h5_key_labels,
            start_date_str=cfg.start_date
        )
        produced["h5_labels"] = out_h5_labels

    # 4) (NEW) Effect matrix CSV → PKL
    if cfg.effect_matrix_csv:
        out_pkl_effect = os.path.join(out_dir, cfg.effect_matrix_pkl)
        make_effect_matrix_pkl(cfg.effect_matrix_csv, out_pkl_effect)
        produced["effect_matrix_pkl"] = out_pkl_effect

    print("\n🎉 All done.")
    print("Artifacts:")
    for k, v in produced.items():
        print(f" - {k}: {v}")
    return produced


# ============================== CLI ==============================

if __name__ == "__main__":
    # Defaults mirror your original variables, with extras left None by default.
    cfg = DesptConfig(
        input_csv=r"test_closures\merged\clabels.csv",                 # wide time-series (detectors OR labels)
        dist_csv=r"combined_outputs\sensor_distance_matrix.csv",       # distance matrix (optional)
        labels_csv=None,                                               # e.g., r"test_closures\merged\clabels.csv"
        effect_matrix_csv=None,                                        # e.g., r"combined_outputs\closure_effect_matrix.csv"
        out_dir=r"combined_outputs\DESPT\SMALL_C",
        h5_key_timeseries="sensor_readings",
        h5_key_labels="closure_labels",
        adj_basename="adj_mx",
        effect_matrix_pkl="closure_effect.pkl",
        start_date="2025-01-01 00:00:00",
        step_min=5,
        threshold=0.1,
        write_adj_csv=True,
    )

    run_despt_prep(cfg)
