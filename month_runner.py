# step5_month_runner.py  (PORTAL-ONLY spawns + multi-month runner)
# - Spawns on portal->boundary edges; destinations are boundary->portal edges
# - Takes an existing "new net" (with portal_* edges) as input
# - Runs for N months starting at (START_YEAR, START_MONTH), writing:
#       months/YYYY-MM/day_##/{detectors_*.xml, edgedata_*.xml, outputs_*.add.xml}
# - MESO only. Uses detector/edgeData writers (XML). No CSV conversion here.

import os
import sys
import json
import math
import random
import shutil
import calendar
import xml.etree.ElementTree as ET
from collections import defaultdict
from time import time, sleep
from typing import Dict, List, Tuple, Optional

# ============================== CONFIG ===================================== #

# Input network + detectors
NET_DIR = "my_grid_sim"
SRC_NET = os.path.join(NET_DIR, "MySUMOFile.net.xml")   # <- your NEW net (with portal_* edges)
SRC_DET = os.path.join(NET_DIR, "detectors.add.xml")    # must exist & already excludes portal edges

# Run outputs (now grouped by month under months/YYYY-MM/)
RUN_DIR = "month_run"
OUT_MONTHS_DIR = os.path.join(RUN_DIR, "months")
CONFIG_DIR = os.path.join(RUN_DIR, "config")
AUTOSCALE_JSON = os.path.join(CONFIG_DIR, "autoscale.json")
PORTAL_WEIGHTS_FILE = os.path.join(CONFIG_DIR, "portal_weights.json")
STAGE_PLAIN_FILES = True  # copy plain.* alongside (if available)

# SUMO engine (MESO ONLY)
USE_GUI = False                 # show GUI
GUI_DELAY_MS = 10               # GUI step delay (ms)
STEP_LENGTH = 1.0               # sim step (s)
BIN_SEC = 300                   # detector/edgeData frequency (s)

# Calendar span
START_YEAR = 2025
START_MONTH = 1                 # 1..12
NUM_MONTHS = 3                  # run this many consecutive months

# Demand mode: "daily_target" | "autoscale_stability"
DEMAND_MODE = "daily_target"

# Baseline spawn (will be modulated by diurnal + monthly factors)
BASE_SPAWN_RATE_TOTAL = 0.8     # base λ across all origins (veh/s) before scaling
TARGET_DAILY_VEH = 86400        # used only in "daily_target"
SCALE_SAFETY = 1.00             # extra margin for daily_target solve
MAX_VEH_PER_SEC_TOTAL = 2.0     # instantaneous cap

# Diurnal clamp (optional peak limiter)
DIURNAL_PEAK_CLAMP: Optional[float] = None  # e.g., 1.3 to squash extreme peaks

# ---------- Stability autoscaler knobs ----------
STABILITY_WARMUP_SEC = 180
STABILITY_MEASURE_SEC = 600
STABILITY_MINUTES_OF_DAY = [9*60, 17*60+30]  # probe 09:00 and 17:30
SCALE_SEARCH_LO = 0.05
SCALE_SEARCH_HI = 6.0
SCALE_SEARCH_ITERS = 10
SCALE_SAFETY_MARGIN = 0.95

# Stability acceptance thresholds
SLOPE_MAX = 0.25                # vehicles/sec (running vehicles slope)
ARRIVAL_EXCESS_MAX = 50         # allowed deficit (dep - arr) over window
MIN_MEAN_SPEED_KMH = 5.0

# Randomness control
SEED = 101
random.seed(SEED)

# I/O safety
EXTRA_DRAIN_STEPS = 10
WAIT_TIMEOUT_S = 60
WAIT_POLL_S = 0.25

# Portal spawn/dest weighting
WEIGHT_MODE = "uniform"         # "uniform" | "random" | "custom"
CUSTOM_WEIGHTS = {
    # "origin": {"portal_to_NODEID": weight, ...},
    # "sink":   {"NODEID_to_portal": weight, ...}
}

# ============================== SUMO BOOTSTRAP ============================= #

SUMO_HOME = os.environ.get("SUMO_HOME", "")
if not SUMO_HOME:
    raise EnvironmentError("SUMO_HOME is not set. Point it to your SUMO installation.")
sys.path.append(os.path.join(SUMO_HOME, "tools"))
import traci  # noqa: E402


# ============================== HELPERS ==================================== #

def abspath(p: str) -> str:
    return os.path.abspath(p)

def ensure_dirs() -> None:
    os.makedirs(RUN_DIR, exist_ok=True)
    os.makedirs(CONFIG_DIR, exist_ok=True)
    os.makedirs(OUT_MONTHS_DIR, exist_ok=True)

def wait_for_stable_file(path: str, timeout: float = WAIT_TIMEOUT_S, poll: float = WAIT_POLL_S) -> bool:
    """Wait until a file exists and stops growing (best-effort)."""
    t0 = time()
    last = -1
    while time() - t0 < timeout:
        if not os.path.exists(path):
            sleep(poll)
            continue
        sz = os.path.getsize(path)
        if sz > 0 and sz == last:
            return True
        last = sz
        sleep(poll)
    return False

def stage_network_and_detectors() -> Tuple[str, str]:
    """Copy network and detectors into CONFIG_DIR (and plain.* if present)."""
    ensure_dirs()
    if not os.path.exists(SRC_NET):
        raise FileNotFoundError(f"Cannot find network at {SRC_NET}")
    if not os.path.exists(SRC_DET):
        raise FileNotFoundError("detectors.add.xml not found in my_grid_sim/")

    staged_net = abspath(os.path.join(CONFIG_DIR, "MySUMOFile.net.xml"))
    shutil.copy2(SRC_NET, staged_net)

    staged_det_src = abspath(os.path.join(CONFIG_DIR, "detectors.add.xml"))
    shutil.copy2(SRC_DET, staged_det_src)

    if STAGE_PLAIN_FILES:
        for fname in ["plain.nod.xml", "plain.edg.xml", "plain.typ.xml", "plain.tll.xml"]:
            p = os.path.join(NET_DIR, fname)
            if os.path.exists(p):
                shutil.copy2(p, abspath(os.path.join(CONFIG_DIR, fname)))
    return staged_net, staged_det_src

# ---------------- Net parsing (PORTALS) ---------------- #

def read_net_edges(net_file: str) -> List[Tuple[str, str, str]]:
    """Return normal (non-internal) edges as (id, from, to)."""
    tree = ET.parse(net_file)
    root = tree.getroot()
    edges: List[Tuple[str, str, str]] = []
    for e in root.findall("edge"):
        if e.get("function") is not None:  # skip internal/special edges
            continue
        eid, fr, to = e.get("id"), e.get("from"), e.get("to")
        if eid and fr and to:
            edges.append((eid, fr, to))
    return edges

def collect_portal_edges(net_file: str) -> Tuple[List[str], List[str]]:
    """
    Returns:
      origin_edges: edges portal->boundary (ids start with 'portal_to_')
      sink_edges:   edges boundary->portal (ids end with '_to_portal')
    """
    tree = ET.parse(net_file)
    root = tree.getroot()
    origin, sink = [], []
    for e in root.findall("edge"):
        eid = e.get("id", "")
        if not eid or e.get("function") is not None:
            continue
        fr = e.get("from", "")
        to = e.get("to", "")
        if fr.startswith("portal_") and not to.startswith("portal_"):
            origin.append(eid)
        if to.startswith("portal_") and not fr.startswith("portal_"):
            sink.append(eid)
    if not origin or not sink:
        raise RuntimeError("Portal edges not found in net. Did you build the new portal net?")
    return origin, sink

# ---------------- Weighting helpers ---------------- #

def _normalize(ws: List[float]) -> List[float]:
    s = sum(ws)
    n = len(ws)
    if n == 0:
        return []
    return [w / s for w in ws] if s > 0 else [1.0 / n] * n

def load_or_make_portal_weights(path: str,
                                origin_edges: List[str],
                                sink_edges: List[str],
                                mode: str = WEIGHT_MODE) -> Tuple[List[float], List[float]]:
    """Create (or load) per-edge weights for origin and sink portals and return normalized lists."""
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    else:
        if mode == "custom":
            data = {"origin": CUSTOM_WEIGHTS.get("origin", {}), "sink": CUSTOM_WEIGHTS.get("sink", {})}
        elif mode == "random":
            data = {"origin": {e: random.expovariate(1.0) for e in origin_edges},
                    "sink":   {e: random.expovariate(1.0) for e in sink_edges}}
        else:  # uniform
            data = {"origin": {e: 1.0 for e in origin_edges},
                    "sink":   {e: 1.0 for e in sink_edges}}
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    ow = [float(data["origin"].get(e, 1.0)) for e in origin_edges]
    sw = [float(data["sink"].get(e, 1.0))   for e in sink_edges]
    return _normalize(ow), _normalize(sw)

# ---------------- Diurnal + Monthly demand ---------------- #

def diurnal_factor(mins_of_day: int, dow: int) -> float:
    """
    Smooth bimodal traffic factor for a given minute-of-day and day-of-week.
    Weekends (5=Sat, 6=Sun) have lower + later peaks.
    """
    h = mins_of_day / 60.0
    base = 0.35 if dow in (5, 6) else 0.40
    am_mu = 10.0 if dow in (5, 6) else 8.5
    pm_mu = 18.0 if dow in (5, 6) else 17.5
    am_amp = 0.6 if dow in (5, 6) else 1.0
    pm_amp = 0.9 if dow in (5, 6) else 1.3
    am = am_amp * math.exp(-0.5 * ((h - am_mu) / 1.1) ** 2)
    pm = pm_amp * math.exp(-0.5 * ((h - pm_mu) / 1.2) ** 2)
    f = base + am + pm
    if DIURNAL_PEAK_CLAMP is not None:
        f = min(f, DIURNAL_PEAK_CLAMP)
    return f

def diurnal_mean_over_day(dow: int) -> float:
    return sum(diurnal_factor(m, dow) for m in range(24 * 60)) / (24 * 60)

def monthly_factor(year: int, month: int) -> float:
    """
    Realistic seasonality (very generic):
      Jan 0.90, Feb 0.95, Mar 1.00, Apr 1.05, May 1.10, Jun 1.08,
      Jul 0.98, Aug 0.92, Sep 1.06, Oct 1.08, Nov 1.02, Dec 0.97
    Tweak as you like for your city/region.
    """
    factors = {
        1: 0.90,  2: 0.95,  3: 1.00,  4: 1.05,
        5: 1.10,  6: 1.08,  7: 0.98,  8: 0.92,
        9: 1.06, 10: 1.08, 11: 1.02, 12: 0.97,
    }
    return factors.get(month, 1.00)

def compute_scale_for_daily_target(target_daily: int, base_rate_total: float) -> float:
    """Choose scale so the day’s injected vehicles integrates to target_daily."""
    mean_factor = diurnal_mean_over_day(dow=2)  # Wednesday-ish
    secs = 24 * 3600
    return target_daily / (base_rate_total * mean_factor * secs)

# ============================== MESO SUMO ================================== #

def _which(exe: str) -> str:
    return shutil.which(exe) or os.path.join(SUMO_HOME, "bin", exe)

def _meso_args() -> List[str]:
    return [
        "--mesosim",
        "--meso-multi-queue",
        "--meso-lane-queue",
        "--meso-junction-control.limited",
        "--meso-overtaking",
        "--meso-recheck", "2",
    ]

def traci_start_basic(staged_net: str) -> None:
    bin_name = "sumo-gui" if USE_GUI else "sumo"
    exe = _which(bin_name)
    args = [
        exe,
        "-n", abspath(staged_net),
        "--no-step-log",
        "--duration-log.disable",
        "--step-length", str(STEP_LENGTH),
    ]
    if USE_GUI:
        args += ["--delay", str(GUI_DELAY_MS)]
    traci.start(args + _meso_args())

def traci_start_with_additional(staged_net: str, additional_files: str) -> None:
    bin_name = "sumo-gui" if USE_GUI else "sumo"
    exe = _which(bin_name)
    args = [
        exe,
        "-n", abspath(staged_net),
        "--no-step-log",
        "--duration-log.disable",
        "--step-length", str(STEP_LENGTH),
        "--additional-files", abspath(additional_files),
    ]
    if USE_GUI:
        args += ["--delay", str(GUI_DELAY_MS)]
    traci.start(args + _meso_args())

def traci_close() -> None:
    try:
        traci.close(False)
    except Exception:
        pass

# ---------------- Day writer (detectors + edgeData) ---------------- #

def build_day_add(staged_det_src: str,
                  out_add_path: str,
                  det_xml_abs: str,
                  edge_xml_abs: str,
                  freq_sec: int = BIN_SEC) -> int:
    """
    Clone detectors.add.xml, rewiring all <e1Detector file=... freq=...>,
    and append a single <edgeData> sink writing to edge_xml_abs.
    """
    tree = ET.parse(staged_det_src)
    root = tree.getroot()
    if root.tag != "additional":
        newroot = ET.Element("additional")
        newroot.append(root)
        root = newroot
    cnt = 0
    for e1 in root.findall(".//e1Detector"):
        e1.set("file", det_xml_abs)
        e1.set("freq", str(freq_sec))
        cnt += 1
    ET.SubElement(root, "edgeData", {"id": "ed1", "file": edge_xml_abs, "freq": str(freq_sec)})
    ET.ElementTree(root).write(out_add_path, encoding="utf-8", xml_declaration=True)
    return cnt

# ============================== AUTOSCALE (PORTALS) ======================== #

def run_window_metrics_portals(staged_net: str,
                               origin_edges: List[str],
                               sink_edges: List[str],
                               origin_probs: List[float],
                               sink_probs: List[float],
                               scale: float,
                               mins_of_day: int) -> dict:
    """
    Run a single probe window using portal-only spawn/dest with weights.
    Collect all metrics BEFORE closing TraCI to avoid 'Connection already closed'.
    """
    # Build per-origin lambdas (split lam_total by origin_probs)
    factor = diurnal_factor(mins_of_day, dow=2)  # treat as weekday probe
    lam_total = BASE_SPAWN_RATE_TOTAL * factor * scale
    lam_total = min(lam_total, MAX_VEH_PER_SEC_TOTAL)
    lam_per_origin = [lam_total * p for p in origin_probs]

    traci_start_basic(staged_net)

    # For speed metric:
    tree = ET.parse(staged_net)
    root = tree.getroot()
    edge_ids = [e.get("id") for e in root.findall("edge") if e.get("function") is None]

    vid = 0
    sum_s_times_n = 0.0
    sum_n = 0.0
    running_series: List[int] = []

    arrivals_0 = traci.simulation.getArrivedNumber()
    departed_0 = traci.simulation.getDepartedNumber()

    # Will fill these BEFORE closing TraCI
    arrivals_1 = arrivals_0
    departed_1 = departed_0

    total_steps = int((STABILITY_WARMUP_SEC + STABILITY_MEASURE_SEC) / STEP_LENGTH)
    try:
        for step in range(total_steps):
            # inject per origin
            for oe, lam in zip(origin_edges, lam_per_origin):
                nveh = poisson(lam * STEP_LENGTH)
                for _ in range(nveh):
                    dst = random.choices(sink_edges, weights=sink_probs, k=1)[0]
                    if dst == oe:
                        continue
                    try:
                        route = traci.simulation.findRoute(oe, dst, vType="DEFAULT_VEHTYPE")
                        if not route.edges:
                            continue
                        rid = f"r{vid}"
                        traci.route.add(rid, route.edges)
                        traci.vehicle.add(
                            f"veh{vid}", rid,
                            departLane="random", departPos="random", departSpeed="max"
                        )
                        vid += 1
                    except traci.TraCIException:
                        continue

            traci.simulationStep()

            t = (step + 1) * STEP_LENGTH
            if t > STABILITY_WARMUP_SEC:
                # speed metric
                for eid in edge_ids:
                    n = traci.edge.getLastStepVehicleNumber(eid)
                    if n <= 0:
                        continue
                    s = traci.edge.getLastStepMeanSpeed(eid)
                    if s is None or s != s:
                        continue
                    sum_s_times_n += s * n
                    sum_n += n
                # running vehicles
                running_series.append(traci.simulation.getMinExpectedNumber())

        # ⬅️ read counters BEFORE closing TraCI
        arrivals_1 = traci.simulation.getArrivedNumber()
        departed_1 = traci.simulation.getDepartedNumber()

    finally:
        traci_close()

    mean_speed_kmh = (sum_s_times_n / sum_n) * 3.6 if sum_n > 0 else 0.0
    arrivals = arrivals_1 - arrivals_0
    departures = departed_1 - departed_0

    # slope of running vehicles (least-squares)
    slope = 0.0
    if len(running_series) >= 2:
        n = len(running_series)
        x_mean = (n - 1) / 2.0
        y_mean = sum(running_series) / n
        num = sum((i - x_mean) * (running_series[i] - y_mean) for i in range(n))
        den = sum((i - x_mean) ** 2 for i in range(n)) or 1.0
        slope = (num / den) / STEP_LENGTH

    return {
        "mean_speed_kmh": mean_speed_kmh,
        "arrivals": arrivals,
        "departures": departures,
        "running_series": running_series,
        "slope_running": slope,
    }


def stability_violations(metrics: dict) -> List[str]:
    reasons: List[str] = []
    if metrics["slope_running"] > SLOPE_MAX:
        reasons.append(f"slope_running {metrics['slope_running']:.3f} > {SLOPE_MAX:.3f}")
    if (metrics["departures"] - metrics["arrivals"]) < -ARRIVAL_EXCESS_MAX:
        delta = metrics["departures"] - metrics["arrivals"]
        reasons.append(f"departures-arrivals = {delta} < -{ARRIVAL_EXCESS_MAX}")
    if metrics["mean_speed_kmh"] < MIN_MEAN_SPEED_KMH:
        reasons.append(f"mean_speed {metrics['mean_speed_kmh']:.2f} < {MIN_MEAN_SPEED_KMH:.2f} km/h")
    return reasons

def autoscale_stability_portals(staged_net: str,
                                origin_edges: List[str],
                                sink_edges: List[str],
                                origin_probs: List[float],
                                sink_probs: List[float]) -> float:
    """Binary search for the largest stable scale using portal-only spawns/dests."""
    lo, hi = SCALE_SEARCH_LO, SCALE_SEARCH_HI
    best_ok = SCALE_SEARCH_LO

    for it in range(SCALE_SEARCH_ITERS):
        mid = (lo + hi) / 2.0
        ok_all = True
        for mins in STABILITY_MINUTES_OF_DAY:
            m = run_window_metrics_portals(
                staged_net, origin_edges, sink_edges, origin_probs, sink_probs,
                scale=mid, mins_of_day=mins
            )
            reasons = stability_violations(m)
            if reasons:
                ok_all = False
                print(f"[stability it={it}] scale={mid:.4f} @ {mins//60:02d}:{mins%60:02d} → UNSTABLE: " +
                      "; ".join(reasons))
                break
            else:
                print(f"[stability it={it}] scale={mid:.4f} @ {mins//60:02d}:{mins%60:02d} → STABLE "
                      f"(mean_speed={m['mean_speed_kmh']:.2f} km/h, slope={m['slope_running']:.3f}, "
                      f"dep-arr={m['departures']-m['arrivals']})")
        if ok_all:
            best_ok = mid
            lo = mid
        else:
            hi = mid

    final_scale = best_ok * SCALE_SAFETY_MARGIN
    print(f"🔧 Stability autoscale selected {final_scale:.4f} "
          f"(best stable {best_ok:.4f}, margin {SCALE_SAFETY_MARGIN})")
    return final_scale

# ============================== MAIN (MULTI-MONTH) ========================= #

def build_month_dirs(year: int, month: int) -> str:
    ym = f"{year:04d}-{month:02d}"
    mdir = abspath(os.path.join(OUT_MONTHS_DIR, ym))
    os.makedirs(mdir, exist_ok=True)
    return mdir

def next_year_month(year: int, month: int) -> Tuple[int, int]:
    m = month + 1
    y = year + (1 if m == 13 else 0)
    m = 1 if m == 13 else m
    return y, m

def main() -> None:
    ensure_dirs()
    staged_net, staged_det_src = stage_network_and_detectors()

    # PORTAL origin/sink edges + weights
    origin_edges, sink_edges = collect_portal_edges(staged_net)
    origin_probs, sink_probs = load_or_make_portal_weights(PORTAL_WEIGHTS_FILE, origin_edges, sink_edges, WEIGHT_MODE)

    # ====== choose scale (base, month-independent) ======
    if DEMAND_MODE == "daily_target":
        scale = compute_scale_for_daily_target(TARGET_DAILY_VEH, BASE_SPAWN_RATE_TOTAL)
        scale *= SCALE_SAFETY
        print(f"🎯 Demand: daily_target={TARGET_DAILY_VEH:,} veh/day → base scale={scale:.4f}")
        with open(AUTOSCALE_JSON, "w", encoding="utf-8") as f:
            json.dump({
                "mode": "daily_target",
                "target_daily": TARGET_DAILY_VEH,
                "computed_scale": scale,
                "base_spawn_rate_total": BASE_SPAWN_RATE_TOTAL,
            }, f, indent=2)

    elif DEMAND_MODE == "autoscale_stability":
        print("🔧 Running autoscaler (stability mode; portals only)...")
        random.seed(SEED)
        scale = autoscale_stability_portals(staged_net, origin_edges, sink_edges, origin_probs, sink_probs)
        with open(AUTOSCALE_JSON, "w", encoding="utf-8") as f:
            json.dump({"mode": "autoscale_stability", "scale": scale}, f, indent=2)
        print(f"🎯 Chosen stability base scale = {scale:.4f}")

    else:
        raise ValueError(f"Unknown DEMAND_MODE: {DEMAND_MODE}")

    # ====== iterate months ======
    year, month = START_YEAR, START_MONTH
    for mi in range(NUM_MONTHS):
        month_dir = build_month_dirs(year, month)
        ndays = calendar.monthrange(year, month)[1]
        print(f"\n📆 Running {year:04d}-{month:02d} ({ndays} days)")

        # Month-specific multiplier (seasonality)
        m_mult = monthly_factor(year, month)

        # ====== run days in this month ======
        for day in range(1, ndays + 1):
            dow = calendar.weekday(year, month, day)  # 0=Mon..6=Sun
            day_dir = abspath(os.path.join(month_dir, f"day_{day:02d}"))
            os.makedirs(day_dir, exist_ok=True)

            det_xml = abspath(os.path.join(day_dir, f"detectors_{year:04d}{month:02d}{day:02d}.xml"))
            edge_xml = abspath(os.path.join(day_dir, f"edgedata_{year:04d}{month:02d}{day:02d}.xml"))
            add_file = abspath(os.path.join(day_dir, f"outputs_{year:04d}{month:02d}{day:02d}.add.xml"))

            cnt = build_day_add(staged_det_src, add_file, det_xml, edge_xml, BIN_SEC)
            print(f"[{year:04d}-{month:02d}-{day:02d}] Rewired {cnt} detectors → {os.path.basename(add_file)}")

            # Per-day deterministic randomness
            random.seed(SEED + (year * 10000 + month * 100 + day))

            # Start SUMO with per-day additional outputs (MESO ONLY)
            traci_start_with_additional(staged_net, additional_files=add_file)

            # Injection params
            secs_in_day = 24 * 3600
            # base lambda with scale and monthly multiplier
            lam_base = BASE_SPAWN_RATE_TOTAL * scale * m_mult
            # split across origins by origin_probs:
            lam_origins = [lam_base * p for p in origin_probs]

            vid = 0
            try:
                for t in range(secs_in_day):
                    mins = t // 60
                    factor = diurnal_factor(mins, dow)
                    lam_total = factor  # diurnal factor multiplier

                    # Hard instantaneous cap applied after composing per-origin rates
                    for oe, lam_o in zip(origin_edges, lam_origins):
                        lam = min(lam_o * lam_total, MAX_VEH_PER_SEC_TOTAL)
                        n = poisson(lam * STEP_LENGTH)
                        for _ in range(n):
                            dst_edge = random.choices(sink_edges, weights=sink_probs, k=1)[0]
                            if dst_edge == oe:
                                continue
                            try:
                                route = traci.simulation.findRoute(oe, dst_edge, vType="DEFAULT_VEHTYPE")
                                if not route.edges:
                                    continue
                                rid = f"r{vid}"
                                traci.route.add(rid, route.edges)
                                traci.vehicle.add(
                                    f"veh{vid}", rid,
                                    departLane="random", departPos="random", departSpeed="max"
                                )
                                vid += 1
                            except traci.TraCIException:
                                continue

                    traci.simulationStep()

                    # Progress log every 15 minutes
                    if (t + 1) % (15 * 60) == 0:
                        running = traci.simulation.getMinExpectedNumber()
                        print(f"  - {year:04d}-{month:02d}-{day:02d} | t={(t+1)//60:4d} min | running≈{running}")

                # Drain steps for clean writer flush
                for _ in range(EXTRA_DRAIN_STEPS):
                    traci.simulationStep()

            finally:
                traci_close()

            # Ensure outputs finished writing
            for p in (det_xml, edge_xml):
                ok = wait_for_stable_file(p)
                if not ok:
                    print(f"  ⚠️ Output not stable or missing: {p} "
                          f"(exists={os.path.exists(p)}, size={os.path.getsize(p) if os.path.exists(p) else 0})")

        # next month
        year, month = next_year_month(year, month)

    print("\n✅ Multi-month run complete. Month folders are in:", OUT_MONTHS_DIR)

# -------------------- math & small utils -------------------- #

def poisson(lam_per_sec: float) -> int:
    """Poisson(k; λ) sampling via product-of-uniforms (accurate for small λ)."""
    if lam_per_sec <= 0:
        return 0
    L = math.exp(-lam_per_sec)
    k, p = 0, 1.0
    while True:
        k += 1
        p *= random.random()
        if p <= L:
            return k - 1

# -------------------- entry -------------------- #

if __name__ == "__main__":
    main()
