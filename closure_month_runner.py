# closure_month_runner_portals.py
#!/usr/bin/env python3
# Month-by-month SUMO runner (MESO ONLY) + random lane closures + diagnostics
# Portal-only trips (portal_to_* → *_to_portal); multi-month folders:
#   <run_dir>/months/YYYY-MM/day_##/{detectors_*.xml, edgedata_*.xml, outputs_*.add.xml, logs/}
# No closures on portal edges. Supports date gates and rollover closures.

import os
import sys
import json
import math
import random
import shutil
import calendar
import xml.etree.ElementTree as ET
from collections import defaultdict
from dataclasses import dataclass, field, asdict
from time import time, sleep
from typing import Dict, List, Tuple, Optional

# -------- lazy TraCI bootstrap --------
TRACI = None  # set by _ensure_traci()
def _ensure_traci(sumo_home: str):
    global TRACI
    if TRACI:
        return TRACI
    if not sumo_home:
        raise EnvironmentError("SUMO_HOME is not set. Provide it via MonthRunnerConfig.sumo_home.")
    tools = os.path.join(sumo_home, "tools")
    if tools not in sys.path:
        sys.path.append(tools)
    import traci  # type: ignore
    TRACI = traci
    return TRACI


# ============================== CONFIG ============================== #

@dataclass
class MonthRunnerConfig:
    # SUMO
    sumo_home: str = os.environ.get("SUMO_HOME", "")
    use_gui: bool = False
    gui_delay_ms: int = 10
    step_length: float = 1.0
    bin_sec: int = 300  # detector/edgeData frequency

    # Network + detectors (source portal net created by clean_grid)
    net_dir: str = "my_grid_sim"
    run_dir: str = "closure_on_test_month_run"
    stage_plain_files: bool = True  # copy plain.* for reproducibility

    # === NEW: schedule mode ===
    run_mode: str = "months"  # "months" | "days"
    # months mode
    start_year: int = 2025
    start_month: int = 1
    num_months: int = 3
    # days mode
    start_day: int = 1
    days_to_run: int = 0  # if run_mode=="days", run this many consecutive days (>=1)

    # Per-day simulation length
    sim_seconds: int = 24 * 3600

    # Demand
    demand_mode: str = "daily_target"  # "daily_target" | "autoscale_stability"
    base_spawn_rate_total: float = 0.8
    target_daily_veh: int = 86400
    scale_safety: float = 1.00
    max_veh_per_sec_total: float = 2.0
    diurnal_peak_clamp: Optional[float] = None

    # Closures
    n_closures: int = 5  # legacy: per-day number of closures (will be replaced by new scheme)
    close_min_dur_s: int = 30 * 60
    close_max_dur_s: int = 8 * 60 * 60
    close_start_guard_s: int = 10 * 60
    exclude_portal_edges_from_closure: bool = True
    allow_cross_day: bool = True
    retarget_before_reroute: bool = True

    # New closure scheduling controls
    closure_pool_size: Optional[int] = None     # if None → all edges in base_closure_pool
    closure_day_period: int = 1                 # 1 = every day, 2 = every 2nd day, etc.
    closure_edge_prob: float = 1.0              # per-edge probability on closure days (0–1)
    late_closure_pool_size: Optional[int] = None  # subset of global pool that appears only later
    late_closure_start_frac: float = 0.7        # fraction of schedule after which "late" edges become eligible


    # Date gates
    closure_global_start: Optional[Tuple[int, int, int]] = (2025, 3, 5)   # None to disable
    deferred_closure_start: Optional[Tuple[int, int, int]] = (2024, 3, 1) # None to disable
    deferred_edge_ids: set = field(default_factory=set)
    deferred_edge_prefixes: List[str] = field(default_factory=list)

    # Portal spawn/dest weighting
    weight_mode: str = "uniform"   # "uniform" | "random" | "custom"
    custom_weights: Dict[str, Dict[str, float]] = field(default_factory=lambda: {"origin": {}, "sink": {}})

    # Diagnostics / safety
    seed: int = 104
    extra_drain_steps: int = 10
    wait_timeout_s: float = 60
    wait_poll_s: float = 0.25
    max_console_sample: int = 10

    # Visual overlay (GUI only)
    closed_lane_color: Tuple[int, int, int, int] = (255, 0, 0, 220)
    closed_lane_layer: int = 100

    # Exempt vType (vehicles on the source edge can keep moving)
    exempt_vtype_id: str = "closure_passenger_exempt"
    exempt_vclass: str = "emergency"

    # Derived paths (computed in __post_init__)
    out_months_dir: str = field(init=False)
    config_dir: str = field(init=False)
    autoscale_json: str = field(init=False)
    portal_weights_file: str = field(init=False)
    rollover_json: str = field(init=False)
    src_net: str = field(init=False)
    src_det: str = field(init=False)

    def __post_init__(self):
        self.out_months_dir = os.path.join(self.run_dir, "months")
        self.config_dir = os.path.join(self.run_dir, "config")
        self.autoscale_json = os.path.join(self.config_dir, "autoscale.json")
        self.portal_weights_file = os.path.join(self.config_dir, "portal_weights.json")
        self.rollover_json = os.path.join(self.out_months_dir, "_closure_rollover.json")
        self.src_net = os.path.join(self.net_dir, "MySUMOFile.net.xml")
        self.src_det = os.path.join(self.net_dir, "detectors.add.xml")

# ============================== HELPERS ============================== #

def abspath(p: str) -> str:
    return os.path.abspath(p)

def ensure_dirs(cfg: MonthRunnerConfig) -> None:
    os.makedirs(cfg.run_dir, exist_ok=True)
    os.makedirs(cfg.config_dir, exist_ok=True)
    os.makedirs(cfg.out_months_dir, exist_ok=True)

def wait_for_stable_file(path: str, timeout: float, poll: float) -> bool:
    t0 = time()
    last = -1
    while time() - t0 < timeout:
        if not os.path.exists(path):
            sleep(poll); continue
        sz = os.path.getsize(path)
        if sz > 0 and sz == last:
            return True
        last = sz
        sleep(poll)
    return False

def write_jsonl(path: str, records: List[dict]) -> None:
    if not records:
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        for r in records:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")

def stage_network_and_detectors(cfg: MonthRunnerConfig) -> Tuple[str, str]:
    ensure_dirs(cfg)
    if not os.path.exists(cfg.src_net):
        raise FileNotFoundError(f"Cannot find network at {cfg.src_net}")
    if not os.path.exists(cfg.src_det):
        raise FileNotFoundError("detectors.add.xml not found in my_grid_sim/")

    staged_net = abspath(os.path.join(cfg.config_dir, "MySUMOFile.net.xml"))
    shutil.copy2(cfg.src_net, staged_net)

    staged_det_src = abspath(os.path.join(cfg.config_dir, "detectors.add.xml"))
    shutil.copy2(cfg.src_det, staged_det_src)

    if cfg.stage_plain_files:
        for fname in ["plain.nod.xml", "plain.edg.xml", "plain.typ.xml", "plain.tll.xml"]:
            p = os.path.join(cfg.net_dir, fname)
            if os.path.exists(p):
                shutil.copy2(p, abspath(os.path.join(cfg.config_dir, fname)))
    return staged_net, staged_det_src


# -------- Net parsing (all edges + portals) -------- #

def read_net_nodes_edges(net_file: str) -> Tuple[Dict[str, Tuple[float, float]], List[Tuple[str, str, str]]]:
    tree = ET.parse(net_file)
    root = tree.getroot()

    nodes: Dict[str, Tuple[float, float]] = {}
    for jn in root.findall("junction"):
        nid, x, y = jn.get("id"), jn.get("x"), jn.get("y")
        if nid and x and y:
            nodes[nid] = (float(x), float(y))

    edges: List[Tuple[str, str, str]] = []
    for e in root.findall("edge"):
        if e.get("function") is not None:  # skip internal
            continue
        eid, fr, to = e.get("id"), e.get("from"), e.get("to")
        if eid and fr in nodes and to in nodes:
            edges.append((eid, fr, to))
    return nodes, edges

def collect_portal_edges(net_file: str) -> Tuple[List[str], List[str]]:
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
        raise RuntimeError("Portal edges not found in net. Did you build the portal net?")
    return origin, sink


# -------- Portal weights -------- #

def _normalize(ws: List[float]) -> List[float]:
    s = sum(ws)
    n = len(ws)
    if n == 0:
        return []
    return [w / s for w in ws] if s > 0 else [1.0 / n] * n

def load_or_make_portal_weights(path: str,
                                origin_edges: List[str],
                                sink_edges: List[str],
                                mode: str,
                                custom: Dict[str, Dict[str, float]]) -> Tuple[List[float], List[float]]:
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    else:
        if mode == "custom":
            data = {"origin": custom.get("origin", {}), "sink": custom.get("sink", {})}
        elif mode == "random":
            data = {"origin": {e: random.expovariate(1.0) for e in origin_edges},
                    "sink":   {e: random.expovariate(1.0) for e in sink_edges}}
        else:
            data = {"origin": {e: 1.0 for e in origin_edges},
                    "sink":   {e: 1.0 for e in sink_edges}}
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    ow = [float(data["origin"].get(e, 1.0)) for e in origin_edges]
    sw = [float(data["sink"].get(e, 1.0))   for e in sink_edges]
    return _normalize(ow), _normalize(sw)


# -------- Diurnal + monthly profile -------- #

def diurnal_factor(mins_of_day: int, dow: int, clamp: Optional[float]) -> float:
    h = mins_of_day / 60.0
    base = 0.35 if dow in (5, 6) else 0.40
    am_mu = 10.0 if dow in (5, 6) else 8.5
    pm_mu = 18.0 if dow in (5, 6) else 17.5
    am_amp = 0.6 if dow in (5, 6) else 1.0
    pm_amp = 0.9 if dow in (5, 6) else 1.3
    am = am_amp * math.exp(-0.5 * ((h - am_mu) / 1.1) ** 2)
    pm = pm_amp * math.exp(-0.5 * ((h - pm_mu) / 1.2) ** 2)
    f = base + am + pm
    return min(f, clamp) if clamp is not None else f

def diurnal_mean_over_day(dow: int, clamp: Optional[float]) -> float:
    return sum(diurnal_factor(m, dow, clamp) for m in range(24 * 60)) / (24 * 60)

def monthly_factor(month: int) -> float:
    factors = {
        1: 0.90,  2: 0.95,  3: 1.00,  4: 1.05,
        5: 1.10,  6: 1.08,  7: 0.98,  8: 0.92,
        9: 1.06, 10: 1.08, 11: 1.02, 12: 0.97,
    }
    return factors.get(month, 1.00)

def compute_scale_for_daily_target(target_daily: int, base_rate_total: float, clamp: Optional[float]) -> float:
    mean_factor = diurnal_mean_over_day(dow=2, clamp=clamp)
    secs = 24 * 3600
    return target_daily / (base_rate_total * mean_factor * secs)


# -------- MESO process helpers -------- #

def _which(exe: str, sumo_home: str) -> str:
    return shutil.which(exe) or os.path.join(sumo_home, "bin", exe)

def _meso_args() -> List[str]:
    return [
        "--mesosim",
        "--meso-multi-queue",
        "--meso-lane-queue",
        "--meso-junction-control.limited",
        "--meso-overtaking",
        "--meso-recheck", "2",
        "--ignore-route-errors",
    ]

def traci_start(cfg: MonthRunnerConfig, staged_net: str, additional_files: Optional[str] = None) -> None:
    traci = _ensure_traci(cfg.sumo_home)
    bin_name = "sumo-gui" if cfg.use_gui else "sumo"
    exe = _which(bin_name, cfg.sumo_home)
    args = [
        exe, "-n", abspath(staged_net),
        "--no-step-log", "--duration-log.disable",
        "--step-length", str(cfg.step_length),
    ]
    if cfg.use_gui:
        args += ["--delay", str(cfg.gui_delay_ms)]
    if additional_files:
        args += ["--additional-files", abspath(additional_files)]
    traci.start(args + _meso_args())
    _ensure_exempt_type(cfg)

def traci_close() -> None:
    try:
        if TRACI:
            TRACI.close(False)
    except Exception:
        pass

def _ensure_exempt_type(cfg: MonthRunnerConfig):
    traci = _ensure_traci(cfg.sumo_home)
    try:
        ids = set(traci.vehicletype.getIDList())
        base = "DEFAULT_VEHTYPE" if "DEFAULT_VEHTYPE" in ids else (next(iter(ids)) if ids else None)
        if base is None:
            return
        if cfg.exempt_vtype_id not in ids:
            traci.vehicletype.copy(base, cfg.exempt_vtype_id)
        try:
            traci.vehicletype.setVehicleClass(cfg.exempt_vtype_id, cfg.exempt_vclass)
        except traci.TraCIException:
            traci.vehicletype.setParameter(cfg.exempt_vtype_id, "vClass", cfg.exempt_vclass)
    except Exception:
        pass


# -------- Visualization (GUI only) -------- #

def _lane_buffer_polygon(cfg: MonthRunnerConfig, lane_id: str, width_scale: float = 0.92) -> List[Tuple[float, float]]:
    traci = _ensure_traci(cfg.sumo_home)
    try:
        pts = traci.lane.getShape(lane_id)
        if len(pts) < 2:
            return pts
        try:
            w = traci.lane.getWidth(lane_id)
            if not w or w != w:
                w = 3.2
        except traci.TraCIException:
            w = 3.2
        half = 0.5 * w * width_scale

        left: List[Tuple[float, float]] = []
        right: List[Tuple[float, float]] = []

        for i in range(len(pts) - 1):
            x1, y1 = pts[i]
            x2, y2 = pts[i + 1]
            dx, dy = (x2 - x1), (y2 - y1)
            L = math.hypot(dx, dy) or 1e-6
            nx, ny = -dy / L, dx / L
            left.append((x1 + nx * half, y1 + ny * half))
            right.append((x1 - nx * half, y1 - ny * half))
            if i == len(pts) - 2:
                left.append((x2 + nx * half, y2 + ny * half))
                right.append((x2 - nx * half, y2 - ny * half))
        return left + right[::-1]
    except Exception:
        return []

def _highlight_closed_edge(cfg: MonthRunnerConfig, edge_id: str, on: bool) -> None:
    if not cfg.use_gui:
        return
    traci = _ensure_traci(cfg.sumo_home)
    try:
        nlanes = traci.edge.getLaneNumber(edge_id)
        for i in range(nlanes):
            lane_id = f"{edge_id}_{i}"
            poly_id = f"closurebuf::{lane_id}"
            if on:
                poly = _lane_buffer_polygon(cfg, lane_id)
                if not poly:
                    continue
                if poly_id in traci.polygon.getIDList():
                    traci.polygon.setShape(poly_id, poly)
                    traci.polygon.setColor(poly_id, cfg.closed_lane_color)
                    traci.polygon.setFilled(poly_id, True)
                    try:
                        traci.polygon.setLayer(poly_id, cfg.closed_lane_layer)
                    except Exception:
                        pass
                else:
                    traci.polygon.add(poly_id, poly, cfg.closed_lane_color, fill=True, layer=cfg.closed_lane_layer)
            else:
                if poly_id in traci.polygon.getIDList():
                    traci.polygon.remove(poly_id)
    except Exception:
        pass


# -------- Route/vehicle safe access -------- #

def _remaining_route(route: List[str], idx: int) -> List[str]:
    if not route:
        return []
    if idx is None or idx < 0 or idx >= len(route):
        return route
    return route[idx:]

def _veh_exists(cfg: MonthRunnerConfig, vid: str) -> bool:
    traci = _ensure_traci(cfg.sumo_home)
    try:
        return vid in traci.vehicle.getIDList()
    except Exception:
        return False

def _safe_get_route(cfg: MonthRunnerConfig, vid: str) -> List[str]:
    traci = _ensure_traci(cfg.sumo_home)
    try:
        return list(traci.vehicle.getRoute(vid))
    except Exception:
        return []

def _safe_road(cfg: MonthRunnerConfig, vid: str) -> str:
    traci = _ensure_traci(cfg.sumo_home)
    try:
        return traci.vehicle.getRoadID(vid)
    except Exception:
        return "?"

def _safe_index(cfg: MonthRunnerConfig, vid: str) -> int:
    traci = _ensure_traci(cfg.sumo_home)
    try:
        return traci.vehicle.getRouteIndex(vid)
    except Exception:
        return -1


# -------- Per-day <additional> (detectors + edgeData) -------- #

def build_day_add(cfg: MonthRunnerConfig,
                  staged_det_src: str,
                  out_add_path: str,
                  det_xml_abs: str,
                  edge_xml_abs: str) -> int:
    tree = ET.parse(staged_det_src)
    root = tree.getroot()
    if root.tag != "additional":
        newroot = ET.Element("additional")
        newroot.append(root)
        root = newroot
    cnt = 0
    for e1 in root.findall(".//e1Detector"):
        e1.set("file", det_xml_abs)
        e1.set("freq", str(cfg.bin_sec))
        cnt += 1
    ET.SubElement(root, "edgeData", {"id": "ed1", "file": edge_xml_abs, "freq": str(cfg.bin_sec)})
    ET.ElementTree(root).write(out_add_path, encoding="utf-8", xml_declaration=True)
    return cnt


# ============================== CLOSURES + LOGGING ========================= #

def _edge_lane_ids(cfg: MonthRunnerConfig, edge_id: str) -> List[str]:
    traci = _ensure_traci(cfg.sumo_home)
    n = traci.edge.getLaneNumber(edge_id)
    return [f"{edge_id}_{i}" for i in range(n)]

def set_edge_closed(cfg: MonthRunnerConfig, edge_id: str, closed: bool) -> int:
    traci = _ensure_traci(cfg.sumo_home)
    lids = _edge_lane_ids(cfg, edge_id)
    for lid in lids:
        try:
            if closed:
                traci.lane.setDisallowed(lid, ["passenger"])
            else:
                traci.lane.setAllowed(lid, ["passenger"])
        except Exception:
            pass
    _highlight_closed_edge(cfg, edge_id, on=closed)
    return len(lids)

def append_closure_records_for_edge(csv_path: str, edge_id: str, start_s: int, end_s: int) -> None:
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    new_file = not os.path.exists(csv_path)
    with open(csv_path, "a", encoding="utf-8") as f:
        if new_file:
            f.write("lane_id,start_s,end_s\n")
        # write a record per lane
        # we can’t know nlanes without TraCI here safely (we already log at close time elsewhere)
        f.write(f"{edge_id}_*,{start_s},{end_s}\n")


# ============================== DATE HELPERS =============================== #

def _date_tuple(y: int, m: int, d: int) -> Tuple[int, int, int]:
    return (y, m, d)

def _date_ge(a: Tuple[int,int,int], b: Tuple[int,int,int]) -> bool:
    return a >= b

def _is_deferred_edge(cfg: MonthRunnerConfig, eid: str) -> bool:
    if eid in cfg.deferred_edge_ids:
        return True
    for pref in cfg.deferred_edge_prefixes:
        if eid.startswith(pref):
            return True
    return False


def next_year_month(year: int, month: int) -> Tuple[int, int]:
    m = month + 1
    y = year + (1 if m == 13 else 0)
    m = 1 if m == 13 else m
    return y, m

def _iter_dates_months(start_year: int, start_month: int, num_months: int):
    """Yield (y,m,d) for every day across num_months starting at y/m (d=1..end)."""
    y, m = start_year, start_month
    for _ in range(num_months):
        ndays = calendar.monthrange(y, m)[1]
        for d in range(1, ndays + 1):
            yield (y, m, d)
        y, m = next_year_month(y, m)

def _iter_dates_days(start_year: int, start_month: int, start_day: int, days_to_run: int):
    """Yield exactly N consecutive days (y,m,d), crossing months if needed."""
    if days_to_run <= 0:
        days_to_run = 1
    y, m, d = start_year, start_month, start_day
    for _ in range(days_to_run):
        yield (y, m, d)
        ndays = calendar.monthrange(y, m)[1]
        if d < ndays:
            d += 1
        else:
            y, m = next_year_month(y, m)
            d = 1


# ============================== MAIN (MULTI-MONTH) ========================= #

def build_month_dirs(cfg: MonthRunnerConfig, year: int, month: int) -> str:
    ym = f"{year:04d}-{month:02d}"
    mdir = abspath(os.path.join(cfg.out_months_dir, ym))
    os.makedirs(mdir, exist_ok=True)
    return mdir

def next_year_month(year: int, month: int) -> Tuple[int, int]:
    m = month + 1
    y = year + (1 if m == 13 else 0)
    m = 1 if m == 13 else m
    return y, m

def build_day_add_paths(month_dir: str, y: int, m: int, d: int) -> Tuple[str, str, str]:
    det_xml = abspath(os.path.join(month_dir, f"day_{d:02d}", f"detectors_{y:04d}{m:02d}{d:02d}.xml"))
    edge_xml = abspath(os.path.join(month_dir, f"day_{d:02d}", f"edgedata_{y:04d}{m:02d}{d:02d}.xml"))
    add_file = abspath(os.path.join(month_dir, f"day_{d:02d}", f"outputs_{y:04d}{m:02d}{d:02d}.add.xml"))
    os.makedirs(os.path.dirname(det_xml), exist_ok=True)
    return det_xml, edge_xml, add_file

def poisson(lam_per_sec: float) -> int:
    if lam_per_sec <= 0:
        return 0
    L = math.exp(-lam_per_sec)
    k, p = 0, 1.0
    while True:
        k += 1
        p *= random.random()
        if p <= L:
            return k - 1

def _retarget_to_portal_sink(cfg: MonthRunnerConfig, sink_edges: List[str], sink_probs: List[float], closed_now: set) -> Optional[str]:
    if not sink_edges:
        return None
    for _ in range(8):
        cand = random.choices(sink_edges, weights=sink_probs, k=1)[0]
        if cand not in closed_now:
            return cand
    return None


def run_closure_month_runner(cfg: MonthRunnerConfig) -> dict:
    """Run portal-only simulations with random closures over a day schedule.
       Mode 'months' = original behavior; mode 'days' = run exactly N days."""
    traci = _ensure_traci(cfg.sumo_home)
    random.seed(cfg.seed)

    staged_net, staged_det_src = stage_network_and_detectors(cfg)

    # Parse network + portal edges
    nodes, edges = read_net_nodes_edges(staged_net)
    origin_edges, sink_edges = collect_portal_edges(staged_net)
    origin_probs, sink_probs = load_or_make_portal_weights(
        cfg.portal_weights_file, origin_edges, sink_edges, cfg.weight_mode, cfg.custom_weights)

    # Base closure pool (exclude portals + light "corner" filter)
    portal_like = set([*origin_edges, *sink_edges])

    xs = [x for x, _ in nodes.values()]; ys = [y for _, y in nodes.values()]
    xmin, xmax = (min(xs), max(xs)) if xs else (0.0, 0.0)
    ymin, ymax = (min(ys), max(ys)) if ys else (0.0, 0.0)
    eps = 1e-3 * max(xmax - xmin, ymax - ymin) if nodes else 1e-3

    nbrs: Dict[str, set] = defaultdict(set)
    for _eid, fr, to in edges:
        nbrs[fr].add(to); nbrs[to].add(fr)

    corner_nodes = set()
    for nid, (x, y) in nodes.items():
        on_x = (abs(x - xmin) < eps) or (abs(x - xmax) < eps)
        on_y = (abs(y - ymin) < eps) or (abs(y - ymax) < eps)
        if (on_x and on_y) or (len(nbrs[nid]) <= 2):
            corner_nodes.add(nid)

    def edge_is_corner(fr, to):
        return (fr in corner_nodes) or (to in corner_nodes)

    base_closure_pool = []
    for (eid, fr, to) in edges:
        if cfg.exclude_portal_edges_from_closure and (eid in portal_like or fr.startswith("portal_") or to.startswith("portal_")):
            continue
        if edge_is_corner(fr, to):
            continue
        base_closure_pool.append((eid, fr, to))
    print(f"🛡️  Base closure pool sized: {len(base_closure_pool)} (excluded portals & corners).")

    # ---- New: global closure pool + early/late subsets (fixed for the whole run) ----
    all_edge_ids = [eid for (eid, _fr, _to) in base_closure_pool]

    # Global closure set: edges that can EVER be closed
    if not all_edge_ids:
        global_closure_ids: List[str] = []
    elif cfg.closure_pool_size is None or cfg.closure_pool_size <= 0 or cfg.closure_pool_size >= len(all_edge_ids):
        # Use full pool
        global_closure_ids = list(all_edge_ids)
    else:
        # Sample a fixed subset for the entire run
        global_closure_ids = random.sample(all_edge_ids, cfg.closure_pool_size)

    # Late subset: chosen once from the global set, never changes
    late_closure_ids = set()
    if cfg.late_closure_pool_size and cfg.late_closure_pool_size > 0:
        if cfg.late_closure_pool_size >= len(global_closure_ids):
            # All edges become "late" edges (no early edges at all)
            late_closure_ids = set(global_closure_ids)
        else:
            late_closure_ids = set(random.sample(global_closure_ids, cfg.late_closure_pool_size))

    early_closure_ids = [eid for eid in global_closure_ids if eid not in late_closure_ids]

    print(
        f"🧩 Global closure ids: {len(global_closure_ids)} "
        f"(early={len(early_closure_ids)}, late={len(late_closure_ids)})"
    )


    # Pretty-print which exact edges were chosen (once per run)
    # Build a small lookup from edge id -> (from, to)
    edge_meta = {eid: (fr, to) for (eid, fr, to) in base_closure_pool}

    def _fmt_edge(eid: str) -> str:
        fr, to = edge_meta.get(eid, ("?", "?"))
        return f"{eid} ({fr}->{to})"

    if global_closure_ids:
        print("   • Global closure edges:")
        for eid in sorted(global_closure_ids):
            tag = []
            if eid in early_closure_ids:
                tag.append("early")
            if eid in late_closure_ids:
                tag.append("late")
            tag_str = f" [{' & '.join(tag)}]" if tag else ""
            print(f"      - {_fmt_edge(eid)}{tag_str}")
    else:
        print("   • Global closure edges: NONE (no base_closure_pool).")


    # Demand scaling (month-independent base)
    if cfg.demand_mode == "daily_target":
        scale = compute_scale_for_daily_target(cfg.target_daily_veh, cfg.base_spawn_rate_total, cfg.diurnal_peak_clamp)
        scale *= cfg.scale_safety
        print(f"🎯 Demand: daily_target={cfg.target_daily_veh:,} veh/day → base scale={scale:.4f}")
        os.makedirs(cfg.config_dir, exist_ok=True)
        with open(cfg.autoscale_json, "w", encoding="utf-8") as f:
            json.dump({
                "mode": "daily_target",
                "target_daily": cfg.target_daily_veh,
                "computed_scale": scale,
                "base_spawn_rate_total": cfg.base_spawn_rate_total,
            }, f, indent=2)
    elif cfg.demand_mode == "autoscale_stability":
        scale = 1.0
        os.makedirs(cfg.config_dir, exist_ok=True)
        with open(cfg.autoscale_json, "w", encoding="utf-8") as f:
            json.dump({"mode": "autoscale_stability", "scale": scale}, f, indent=2)
    else:
        raise ValueError(f"Unknown DEMAND_MODE: {cfg.demand_mode}")

    # ---------------- schedule ----------------
    if cfg.run_mode.lower() == "days":
        dates = list(_iter_dates_days(cfg.start_year, cfg.start_month, cfg.start_day, cfg.days_to_run))
    else:
        dates = list(_iter_dates_months(cfg.start_year, cfg.start_month, cfg.num_months))

    # New: when does the late subset become eligible?
    if dates:
        total_days = len(dates)
        late_start_idx = int(total_days * cfg.late_closure_start_frac)
        late_start_idx = max(0, min(late_start_idx, total_days))  # clamp for safety
        print(f"⏱️  Late-closure subset activates at schedule index {late_start_idx} "
              f"out of {total_days} day(s).")
    else:
        late_start_idx = 0

    months_run: List[str] = []
    days_run: List[str] = []

    prev_ym = None
    month_dir = None

    for idx, (year, month, day) in enumerate(dates):
        # New month folder on month change
        ym = (year, month)
        if ym != prev_ym:
            month_dir = build_month_dirs(cfg, year, month)
            months_run.append(f"{year:04d}-{month:02d}")
            print(f"\n📆 Running {year:04d}-{month:02d} (scheduled days in this month will run)")
            prev_ym = ym

        # Per-day setup
        m_mult = monthly_factor(month)
        day_label = f"{year:04d}-{month:02d}-{day:02d}"
        dow = calendar.weekday(year, month, day)  # 0=Mon..6=Sun

        det_xml, edge_xml, add_file = build_day_add_paths(month_dir, year, month, day)
        day_dir = os.path.dirname(det_xml)
        log_dir = os.path.join(day_dir, "logs")
        os.makedirs(log_dir, exist_ok=True)

        cnt = build_day_add(cfg, staged_det_src, add_file, det_xml, edge_xml)
        rel_add = os.path.relpath(add_file, month_dir)
        print(f"[{day_label}] Rewired {cnt} detectors → {rel_add}")

        # Deterministic per-day randomness
        random.seed(cfg.seed + (year * 10000 + month * 100 + day))

        # Build today's closure pool with:
        #  - date gates (closure_global_start / deferred_closure_start)
        #  - early/late subset depending on schedule index
        #  - day-period gating (closure_day_period)
        today_tuple = (year, month, day)

        # Date-based permission for closures at all (used for rollovers, too)
        closures_allowed_by_date = (cfg.closure_global_start is None) or _date_ge(today_tuple, cfg.closure_global_start)

        # Is this a "closure day" for NEW closures?
        # (we still allow rollovers on non-closure days if date allows)
        if cfg.closure_day_period <= 1:
            closure_day_ok = closures_allowed_by_date
        else:
            closure_day_ok = closures_allowed_by_date and (idx % cfg.closure_day_period == 0)

        # Choose which IDs are eligible TODAY ignoring defers:
        #  - before late_start_idx → only early_closure_ids
        #  - after late_start_idx  → early + late
        if idx < late_start_idx:
            base_ids_for_today = set(early_closure_ids)
            phase_label = "early-only"
        else:
            base_ids_for_today = set(early_closure_ids) | late_closure_ids
            phase_label = "early+late"

        # Now intersect with base_closure_pool and apply deferred filter
        todays_pool = []
        if base_ids_for_today:
            for (eid, fr, to) in base_closure_pool:
                if eid not in base_ids_for_today:
                    continue
                # Deferred edges are excluded until deferred_closure_start
                if cfg.deferred_closure_start is not None and not _date_ge(today_tuple, cfg.deferred_closure_start):
                    if _is_deferred_edge(cfg, eid):
                        continue
                todays_pool.append((eid, fr, to))

        print(
            f"[{day_label}] Pool phase={phase_label}, "
            f"closure_day_ok={closure_day_ok}, candidates_today={len(todays_pool)}"
        )


        closures_csv = os.path.join(day_dir, f"closed_lanes_{year:04d}{month:02d}{day:02d}.csv")
        with open(closures_csv, "w", encoding="utf-8", newline="") as f:
            f.write("lane_id,start_s,end_s\n")

        closure_events: List[dict] = []

        # --- New closure scheduling (per-edge probability, no cap) ---
        if closure_day_ok and todays_pool and cfg.closure_edge_prob > 0.0:
            # For each candidate edge today, flip a coin with prob=closure_edge_prob
            # to decide whether it gets a closure.
            scheduled_count = 0
            random.shuffle(todays_pool)  # just to avoid any positional bias

            for (eid, _fr, _to) in todays_pool:
                if random.random() >= cfg.closure_edge_prob:
                    continue

                dur = random.randint(cfg.close_min_dur_s, cfg.close_max_dur_s)
                if cfg.allow_cross_day:
                    start = random.randint(cfg.close_start_guard_s, cfg.sim_seconds - cfg.close_start_guard_s)
                else:
                    latest_start = max(cfg.close_start_guard_s, cfg.sim_seconds - dur - cfg.close_start_guard_s)
                    start = cfg.close_start_guard_s if latest_start <= cfg.close_start_guard_s else random.randint(
                        cfg.close_start_guard_s, latest_start
                    )
                end = start + dur
                closure_events.append({"edge": eid, "start": start, "end": end, "active": False, "_handled": False})
                scheduled_count += 1

            if scheduled_count > 0:
                print(f"🗓️  Closure schedule ({scheduled_count} closure(s)) (edge, start→end, duration min):")
                for c in sorted(closure_events, key=lambda z: z["start"]):
                    sM, sS = divmod(c["start"], 60); eM, eS = divmod(c["end"], 60)
                    print(
                        f"  • {c['edge']}: {sM:02d}:{sS:02d} → {eM:02d}:{eS:02d}  "
                        f"(~{(c['end'] - c['start']) // 60}m)"
                    )
            else:
                print(
                    f"[{day_label}] 🎲 Closure day but no edges selected "
                    f"(candidates={len(todays_pool)}, edge_prob={cfg.closure_edge_prob:.2f})."
                )
        else:
            # Reasons why we might not schedule new closures:
            if not closure_day_ok:
                print(f"[{day_label}] 💤 Not a closure day (closure_day_period={cfg.closure_day_period}).")
            elif not todays_pool:
                print(f"[{day_label}] 💤 No candidate edges available for closures today.")
            elif cfg.closure_edge_prob <= 0.0:
                print(f"[{day_label}] 💤 closure_edge_prob={cfg.closure_edge_prob:.2f} → no closures by design.")


        # Add rollovers (t=0) if enabled and closures allowed
        rollovers = []
        if cfg.allow_cross_day and closures_allowed_by_date and os.path.exists(cfg.rollover_json):
            try:
                with open(cfg.rollover_json, "r", encoding="utf-8") as f:
                    rollovers = json.load(f)
            except Exception:
                rollovers = []
        if rollovers:
            print(f"[{day_label}] Resuming {len(rollovers)} rollover closure(s) at t=0")
            for r in rollovers:
                eid = r["edge"]
                if (eid in portal_like) or (cfg.deferred_closure_start is not None and not _date_ge(today_tuple, cfg.deferred_closure_start) and _is_deferred_edge(cfg, eid)):
                    continue
                closure_events.append({"edge": eid, "start": 0, "end": r["remaining"], "active": False, "_handled": False})
            # clear after consumption
            try:
                with open(cfg.rollover_json, "w", encoding="utf-8") as f:
                    json.dump([], f)
            except Exception:
                pass

        # Start SUMO with this day's additional
        traci_start(cfg, staged_net, additional_files=add_file)

        lam_base = cfg.base_spawn_rate_total * scale * m_mult
        lam_per_origin = [lam_base * p for p in origin_probs]
        vid = 0

        try:
            for t in range(cfg.sim_seconds):
                # Activate/deactivate closures
                for c in closure_events:
                    if (not c["active"]) and (not c["_handled"]) and t == c["start"]:
                        append_closure_records_for_edge(closures_csv, c["edge"], c["start"], c["end"])

                        # Exempt vehicles currently on the source edge
                        on_edge = [v for v in traci.vehicle.getIDList() if _safe_road(cfg, v) == c["edge"]]
                        if on_edge:
                            for v in on_edge:
                                try:
                                    orig = traci.vehicle.getTypeID(v)
                                    traci.vehicle.setType(v, cfg.exempt_vtype_id)
                                except Exception:
                                    pass

                        lane_count = set_edge_closed(cfg, c["edge"], True)
                        c["active"] = True
                        c["_handled"] = True

                        # Optional immediate reroute for others
                        if cfg.retarget_before_reroute:
                            closed_now = {c["edge"]}
                            rerouted = 0
                            for v2 in list(traci.vehicle.getIDList()):
                                if _safe_road(cfg, v2) == c["edge"]:
                                    continue
                                r2 = _safe_get_route(cfg, v2)
                                i2 = _safe_index(cfg, v2)
                                if not r2:
                                    continue
                                if any(e in closed_now for e in _remaining_route(r2, i2)):
                                    try:
                                        traci.vehicle.rerouteTraveltime(v2)
                                        rerouted += 1
                                    except Exception:
                                        pass
                            print(f"    [post] filtered_rerouted={rerouted}")

                        # Safeguard retargeting (if someone’s destination equals closed edge)
                        closed_now = {c["edge"]}
                        new_sink = _retarget_to_portal_sink(cfg, sink_edges, sink_probs, closed_now)
                        if new_sink:
                            changed = 0
                            for v in list(traci.vehicle.getIDList()):
                                r = _safe_get_route(cfg, v)
                                if r and r[-1] == c["edge"]:
                                    try:
                                        traci.vehicle.changeTarget(v, new_sink)
                                        traci.vehicle.rerouteTraveltime(v)
                                        changed += 1
                                    except Exception:
                                        pass
                            if changed:
                                print(f"    [post] retargeted {changed} vehicles to portal sink {new_sink}")

                        m, s = divmod(t, 60)
                        print(f"[{day_label} CLOSE  @ {m:02d}:{s:02d}] edge={c['edge']} lanes={lane_count}")

                    elif c["active"] and t == c["end"]:
                        lane_count = set_edge_closed(cfg, c["edge"], False)
                        c["active"] = False
                        m, s = divmod(t, 60)
                        print(f"[{day_label} OPEN   @ {m:02d}:{s:02d}] edge={c['edge']} lanes={lane_count}")

                # Injection (PORTAL ORIGINS → PORTAL SINKS)
                mins = t // 60
                factor = diurnal_factor(mins, dow, cfg.diurnal_peak_clamp)
                closed_now = {ce["edge"] for ce in closure_events if ce["active"] and ce["start"] <= t < ce["end"]}

                for oe, lam_o in zip(origin_edges, lam_per_origin):
                    lam = min(lam_o * factor, cfg.max_veh_per_sec_total)
                    if oe in closed_now:
                        continue
                    n = poisson(lam * cfg.step_length)
                    for _ in range(n):
                        dst = None
                        for _tries in range(5):
                            cand = random.choices(sink_edges, weights=sink_probs, k=1)[0]
                            if cand != oe and cand not in closed_now:
                                dst = cand; break
                        if not dst:
                            continue
                        try:
                            route = traci.simulation.findRoute(oe, dst, vType="DEFAULT_VEHTYPE")
                            if not getattr(route, "edges", []):
                                continue
                            rid = f"r{vid}"
                            traci.route.add(rid, route.edges)
                            traci.vehicle.add(f"veh{vid}", rid, departLane="random", departPos="random", departSpeed="max")
                            vid += 1
                        except Exception:
                            continue

                traci.simulationStep()

                # Progress log every 15 minutes
                if (t + 1) % (15 * 60) == 0:
                    running = traci.simulation.getMinExpectedNumber()
                    act = [ce["edge"] for ce in closure_events if ce["active"] and ce["start"] <= t < ce["end"]]
                    print(f"[{day_label}] t={(t+1)//60:4d} min | running≈{running} | active_closures={len(act)} {act}")

            for _ in range(cfg.extra_drain_steps):
                traci.simulationStep()

        finally:
            traci_close()

        # Ensure detector/edge outputs finished writing
        for p in (det_xml, edge_xml):
            ok = wait_for_stable_file(p, cfg.wait_timeout_s, cfg.wait_poll_s)
            if not ok:
                size = os.path.getsize(p) if os.path.exists(p) else 0
                print(f"[{day_label}] ⚠️ Output not stable or missing: {p} (exists={os.path.exists(p)}, size={size})")

        # Rollover handling: only keep for tomorrow if there IS a tomorrow in the schedule
        has_tomorrow = (idx < len(dates) - 1)
        if cfg.allow_cross_day:
            next_rollovers = []
            if has_tomorrow:
                for c in closure_events:
                    if c["_handled"] and c["end"] > cfg.sim_seconds:
                        next_rollovers.append({"edge": c["edge"], "remaining": int(c["end"] - cfg.sim_seconds)})
            try:
                with open(cfg.rollover_json, "w", encoding="utf-8") as f:
                    json.dump(next_rollovers, f)
            except Exception:
                pass

        print(f"\n✅ {day_label} run complete. Outputs in: {day_dir}")
        days_run.append(day_label)

    print("\n✅ Schedule complete. Month folders are in:", cfg.out_months_dir)
    return {
        "months_dir": cfg.out_months_dir,
        "months_run": sorted(set(months_run)),
        "days_run": days_run,
        "run_mode": cfg.run_mode,
        "config": asdict(cfg),
    }



# -------------------- CLI -------------------- #

if __name__ == "__main__":
    # Test configuration for new closure features
    cfg = MonthRunnerConfig(
        # Run a short sequence of days so logs are easy to inspect
        run_mode="days",
        start_year=2025,
        start_month=1,
        start_day=1,
        days_to_run=10,        # 10 consecutive days
        sim_seconds=6 * 3600,  # 6 hours per day instead of 24h, faster test

        # Make sure closures are allowed from the very first day
        closure_global_start=None,
        deferred_closure_start=None,

        # New closure controls:
        closure_pool_size=6,        # pick 5 edges once; only those can ever be closed
        closure_day_period=2,       # every 2nd day is a closure day (idx 0,2,4,...)
        closure_edge_prob=0.5,      # each candidate edge has 50% chance to close on closure days

        late_closure_pool_size=6,   # 2 of the 5 edges are "late" edges
        late_closure_start_frac=0.7,# late edges only appear in the last ~30% of days

        # Optional: keep GUI off and use a fixed seed for reproducibility
        use_gui=False,
        seed=104,
        run_dir="data/test",
        net_dir="data/my_grid_sim",
    )

    run_closure_month_runner(cfg)

