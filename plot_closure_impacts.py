# plot_closure_impacts.py
import os
import re
import glob
import math
import csv
import xml.etree.ElementTree as ET
import pandas as pd
import matplotlib.pyplot as plt

# ==================== CONFIG ====================
MONTH_DIR   = r"C:\Users\erays\Desktop\traffic_simulation\new\closure_month_run\closure_month_outputs"
# CONFIG_DIR is assumed to live next to MONTH_DIR (…\closure_month_run\config)
CONFIG_DIR  = os.path.join(os.path.dirname(MONTH_DIR), "config")
NET_FILE    = os.path.join(CONFIG_DIR, "MySUMOFile.net.xml")
DETS_FILE   = os.path.join(CONFIG_DIR, "detectors.add.xml")

PRE_MIN     = 30   # minutes to show before closure start
POST_MIN    = 30   # minutes to show after closure end
DPI         = 140
# =================================================

# ---------- helpers: file discovery ----------
def _find_detector_csv(day_dir: str) -> str | None:
    m = re.search(r"day[_\- ]?(\d+)", os.path.basename(day_dir), re.I)
    if m:
        guess = os.path.join(day_dir, f"detectors_day{int(m.group(1)):02d}.csv")
        if os.path.exists(guess):
            return guess
    cands = sorted(glob.glob(os.path.join(day_dir, "detectors*.csv")))
    return cands[0] if cands else None

def _find_closures_csv(day_dir: str) -> str | None:
    # simple file you log: closed_lanes_dayXX.csv
    m = re.search(r"day[_\- ]?(\d+)", os.path.basename(day_dir), re.I)
    if m:
        guess = os.path.join(day_dir, f"closed_lanes_day{int(m.group(1)):02d}.csv")
        if os.path.exists(guess):
            return guess
    # fallback: any file that looks like closed lanes
    for fn in os.listdir(day_dir):
        low = fn.lower()
        if low.endswith(".csv") and ("closed_lanes" in low or "closure_lanes" in low):
            return os.path.join(day_dir, fn)
    return None

# ---------- helpers: detectors + lanes ----------
def _lane_to_edge(lane_id: str) -> str:
    k = lane_id.rfind("_")
    return lane_id[:k] if k > 0 else lane_id

def parse_detectors_lane_map(det_xml: str) -> dict[str, list[str]]:
    """
    Return lane_id -> [detector_ids] from detectors.add.xml (e1Detector elements).
    """
    out: dict[str, list[str]] = {}
    try:
        tree = ET.parse(det_xml)
        root = tree.getroot()
        if root.tag != "additional":
            root = root.find("additional") or root
        for e1 in root.findall(".//e1Detector"):
            did  = e1.get("id") or e1.get("name")
            lane = e1.get("lane")
            if not did or not lane:
                continue
            out.setdefault(lane, []).append(did)
    except Exception as e:
        print(f"⚠️ failed to parse detectors file {det_xml}: {e}")
    return out

def parse_lane_centroids_from_net(net_file: str) -> dict[str, tuple[float, float]]:
    """
    Return lane_id -> (x,y) centroid from the SUMO .net.xml.
    Prefer lane @shape; else use edge (from,to) mid-point.
    """
    centroids: dict[str, tuple[float, float]] = {}
    try:
        tree = ET.parse(net_file)
        root = tree.getroot()
        # nodes
        nodes: dict[str, tuple[float, float]] = {}
        for jn in root.findall("junction"):
            nid, x, y = jn.get("id"), jn.get("x"), jn.get("y")
            if nid and x and y:
                nodes[nid] = (float(x), float(y))
        # edges + lanes
        for e in root.findall("edge"):
            if e.get("function") is not None:  # skip internal/special
                continue
            fr, to = e.get("from"), e.get("to")
            fr_xy = nodes.get(fr)
            to_xy = nodes.get(to)
            for ln in e.findall("lane"):
                lid = ln.get("id")
                shp = ln.get("shape")
                if not lid:
                    continue
                if shp:
                    pts = []
                    for pair in shp.split():
                        try:
                            xs, ys = pair.split(",")
                            pts.append((float(xs), float(ys)))
                        except Exception:
                            pass
                    if pts:
                        x = sum(p[0] for p in pts)/len(pts)
                        y = sum(p[1] for p in pts)/len(pts)
                        centroids[lid] = (x, y); continue
                # fallback: edge mid-point
                if fr_xy and to_xy:
                    x = 0.5*(fr_xy[0] + to_xy[0])
                    y = 0.5*(fr_xy[1] + to_xy[1])
                    centroids[lid] = (x, y)
    except Exception as e:
        print(f"⚠️ failed to parse net file {net_file}: {e}")
    return centroids

def _euclid(a: tuple[float,float], b: tuple[float,float]) -> float:
    return math.hypot(a[0]-b[0], a[1]-b[1])

def pick_neighbor_and_far(lane_id: str,
                          lane_xy: dict[str, tuple[float,float]],
                          lane_has_sensor: set[str]) -> tuple[str|None, str|None]:
    """
    Among lanes that have detectors, pick the nearest and farthest (excluding lane_id).
    """
    if lane_id not in lane_xy:
        return None, None
    origin = lane_xy[lane_id]
    cands = [(lid, _euclid(origin, lane_xy[lid]))
             for lid in lane_has_sensor if lid != lane_id and lid in lane_xy]
    if not cands:
        return None, None
    cands.sort(key=lambda t: t[1])
    neighbor = cands[0][0]
    far      = cands[-1][0]
    return neighbor, far

# ---------- helpers: sensor time series ----------
def _to_numeric_columns(df: pd.DataFrame) -> pd.DataFrame:
    out = df.copy()
    out.columns = pd.to_numeric(out.columns, errors="coerce")
    out = out.loc[:, ~out.columns.isna()]
    out = out.sort_index(axis=1)
    return out

def load_sensor_series(csv_path: str, sensor_id: str) -> pd.Series | None:
    if not os.path.exists(csv_path):
        return None
    df = pd.read_csv(csv_path, index_col=0)
    df = _to_numeric_columns(df)
    if sensor_id not in df.index:
        lower_map = {ix.lower(): ix for ix in df.index}
        tid = lower_map.get(sensor_id.lower())
        if not tid:
            return None
        sensor_id = tid
    s = df.loc[sensor_id].copy()
    return pd.to_numeric(s, errors="coerce").fillna(0)

def infer_bin_minutes(cols_like) -> int:
    vals = sorted(list(cols_like))
    if len(vals) < 2:
        return 5
    diffs = [b-a for a,b in zip(vals[:-1], vals[1:]) if b-a > 0]
    if not diffs:
        return 5
    return max(1, int(round(min(diffs))))

def _snap_down(m: int, step: int) -> int:
    # largest multiple of step <= m
    return (m // step) * step

def _snap_up(m: int, step: int) -> int:
    # smallest multiple of step >= m
    return ((m + step - 1) // step) * step

def build_window_bins(start_s: int, end_s: int, bin_min: int) -> list[int]:
    """
    Build a list of minute labels aligned to the detector bin grid.
    Example: start=473, PRE=30, bin=5  -> start grid = 440
             end=513,  POST=30, bin=5 -> end grid   = 545 (clamped to <1440)
    Returns [start, start+bin, ..., end] with each element a multiple of bin_min.
    """
    # raw window bounds in minutes
    start_m_raw = max(0, int(start_s // 60) - PRE_MIN)
    end_m_raw   = min(24*60, int(math.ceil(end_s / 60.0)) + POST_MIN)

    # snap to grid
    start_m = _snap_down(start_m_raw, bin_min)
    end_m   = _snap_up(end_m_raw, bin_min)

    # clamp to day; last valid bin start is 1440 - bin_min
    end_m = min(end_m, 24*60)
    if end_m == 24*60:
        end_m -= bin_min

    return list(range(start_m, end_m + 1, bin_min))


# ---------- plotting ----------
def plot_closure_triptych(day_label: str,
                          out_dir: str,
                          closure_row: dict,
                          lane_to_det: dict[str, list[str]],
                          lane_xy: dict[str, tuple[float,float]],
                          det_csv_path: str) -> bool:
    lane = str(closure_row["lane_id"])
    try:
        start_s = int(closure_row["start_s"])
        end_s   = int(closure_row["end_s"])
    except Exception:
        return False

    if lane not in lane_to_det or not lane_to_det[lane]:
        print(f"⚠️ {day_label}: no detector mapped to closed lane {lane}; skipping.")
        return False

    affected_det = lane_to_det[lane][0]

    # neighbor/far lane selection among lanes that have any detector
    lane_has_sensor = set(lane_to_det.keys())
    neighbor_lane, far_lane = pick_neighbor_and_far(lane, lane_xy, lane_has_sensor)

    # map neighbor/far lanes to detector IDs
    neighbor_det = lane_to_det.get(neighbor_lane, [None])[0] if neighbor_lane else None
    far_det      = lane_to_det.get(far_lane, [None])[0] if far_lane else None

    # load series
    s_aff = load_sensor_series(det_csv_path, affected_det)
    s_nei = load_sensor_series(det_csv_path, neighbor_det) if neighbor_det else None
    s_far = load_sensor_series(det_csv_path, far_det) if far_det else None
    if s_aff is None:
        print(f"⚠️ {day_label}: affected detector '{affected_det}' not in {os.path.basename(det_csv_path)}; skipping.")
        return False

    bin_min = infer_bin_minutes(s_aff.index)
    window_bins = build_window_bins(start_s, end_s, bin_min)

    # reindex to window
    s_aff = s_aff.reindex(window_bins, fill_value=0)
    if s_nei is not None:
        s_nei = s_nei.reindex(window_bins, fill_value=0)
    if s_far is not None:
        s_far = s_far.reindex(window_bins, fill_value=0)

    # x in hours for readability
    x_hours = [m/60.0 for m in window_bins]
    cl_start_h = start_s/3600.0
    cl_end_h   = end_s/3600.0

    # figure with 3 stacked panels
    fig, axes = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
    titles = [
        f"Affected: {affected_det}  (lane {lane})",
        f"Neighbor: {neighbor_det or '—'}  (lane {neighbor_lane or '—'})",
        f"Far: {far_det or '—'}  (lane {far_lane or '—'})",
    ]
    series_list = [s_aff, s_nei, s_far]

    for ax, s, ttl in zip(axes, series_list, titles):
        if s is not None:
            ax.plot(x_hours, s.values, linewidth=1.6)
        ax.axvspan(cl_start_h, cl_end_h, alpha=0.2)  # shade closure
        ax.set_ylabel(f"veh / {bin_min}min")
        ax.set_title(ttl, fontsize=10)
        ax.grid(True, linewidth=0.4)

    axes[-1].set_xlabel("Hour of day")
    fig.suptitle(f"{day_label} — Closure {lane} [{start_s//60:04d}–{math.ceil(end_s/60):04d}] min", fontsize=12)
    plt.tight_layout(rect=[0, 0.02, 1, 0.98])

    os.makedirs(out_dir, exist_ok=True)
    # sanitize lane for filename
    safe_lane = re.sub(r"[^\w\-]+", "_", lane)
    out_png = os.path.join(out_dir, f"{day_label}_{safe_lane}_{start_s:05d}_{end_s:05d}.png")
    plt.savefig(out_png, dpi=DPI)
    plt.close(fig)

    print(f"✅ {day_label}: plot → {out_png}")
    return True

# ---------- main ----------
def main():
    # sanity
    if not os.path.exists(MONTH_DIR):
        print(f"❌ MONTH_DIR not found: {MONTH_DIR}")
        return
    if not os.path.exists(NET_FILE):
        print(f"⚠️ net file not found: {NET_FILE} (neighbor/far may be unavailable)")
    if not os.path.exists(DETS_FILE):
        print(f"⚠️ detectors file not found: {DETS_FILE} (cannot map closed lane → detector)")
        return

    lane_to_det = parse_detectors_lane_map(DETS_FILE)
    lane_xy     = parse_lane_centroids_from_net(NET_FILE) if os.path.exists(NET_FILE) else {}

    day_dirs = sorted([d for d in glob.glob(os.path.join(MONTH_DIR, "day_*")) if os.path.isdir(d)])
    if not day_dirs:
        print(f"❌ No day_* folders under {MONTH_DIR}")
        return

    made = skipped = 0
    for day_dir in day_dirs:
        day_label = os.path.basename(day_dir)
        det_csv   = _find_detector_csv(day_dir)
        clos_csv  = _find_closures_csv(day_dir)
        if not det_csv:
            print(f"⚠️ {day_label}: no detectors CSV; skipping.")
            continue
        if not clos_csv:
            print(f"⚠️ {day_label}: no closures CSV; skipping.")
            continue

        # read simple closure rows
        try:
            rows = pd.read_csv(clos_csv)
        except Exception as e:
            print(f"❌ {day_label}: failed to read {clos_csv}: {e}")
            continue

        if not {"lane_id", "start_s", "end_s"}.issubset(rows.columns):
            print(f"⚠️ {day_label}: {os.path.basename(clos_csv)} missing required columns; skipping.")
            continue

        out_dir = os.path.join(MONTH_DIR, "_closure_plots", day_label)

        for _, r in rows.iterrows():
            ok = plot_closure_triptych(
                day_label=day_label,
                out_dir=out_dir,
                closure_row=r,
                lane_to_det=lane_to_det,
                lane_xy=lane_xy,
                det_csv_path=det_csv
            )
            made += 1 if ok else 0
            skipped += 0 if ok else 1

    print("\n=== Summary ===")
    print(f"Plots created: {made}  Skipped: {skipped}")

if __name__ == "__main__":
    main()
