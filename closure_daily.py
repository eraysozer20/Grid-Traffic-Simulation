#!/usr/bin/env python3
"""
Portal-closure day runner (MESO-only) — with robust closure tactics:

What’s new vs your base:
  • Pre-close snapshot: find vehicles whose remaining route hits the soon-to-close edge
  • Watchlist: track likely warners (dest == closed edge) & vehicles on source edge at close
  • On-edge exemptions: temporarily set a vType (vClass='emergency') so they can roll off
  • HARD close (disallow 'passenger') + (optional overlay when GUI)
  • RETARGET-BEFORE-REROUTE: changeTarget for dest==closed vehicles, then selective reroute
  • Selective reroute: only cars whose remaining route still intersects any closed edge
  • Maintenance: revert exempt type + conditional reroute after they leave the source edge
  • Teleport logging (lightweight)

Outputs (same as before):
- detectors XML: <OUT_DIR>/detectors.xml
- edgeData XML : <OUT_DIR>/edgedata.xml
- net file     : <SOURCE_DIR>/MySUMOFile.net.xml
"""

import os
import sys
import math
import random
import shutil
import json
import xml.etree.ElementTree as ET
from time import time, sleep
from typing import List, Tuple, Optional, Dict, Iterable
from collections import defaultdict

# ============================== CONFIG (EDIT ME) ==============================

# Grid + portals
GRID_SIZE    = 6
EDGE_LENGTH  = 300.0        # meters
SPUR_LEN     = EDGE_LENGTH * 0.5  # portal stub length outside boundary

# Demand (veh/s across all portal origins, scaled by diurnal)
BASE_SPAWN_RATE_TOTAL = 0.8
STEP_LENGTH           = 1.0   # seconds
DIURNAL_PEAK_CLAMP: Optional[float] = None  # e.g. 1.3 or None to disable

# Schedule
BASELINE_SECS = 60 * 60        # 1h
CLOSE_SECS    = 60 * 60        # 1h per edge closure
BREAK_SECS    = 5 * 60         # 5 min open between closures

# Engine & outputs
USE_GUI      = False
GUI_DELAY_MS = 10
BIN_SEC      = 300             # e1/edgeData interval

# Project layout (change if desired)
SOURCE_DIR = "my_grid_sim"                 # net + plain.* live here
RUN_DIR    = "day_portal_closures"         # run workspace
OUT_DIR    = os.path.join(RUN_DIR, "outputs")
CONF_DIR   = os.path.join(RUN_DIR, "config")
LOG_DIR    = os.path.join(RUN_DIR, "logs")

# Randomness
SEED = 101
random.seed(SEED)

# File stabilization wait (after sim ends)
STABILIZE_WAIT_S = 30
STABILIZE_POLL_S = 0.25

# Tactics toggles
RETARGET_BEFORE_REROUTE = True
MAX_CONSOLE_SAMPLE = 10

# Exempt vType (so on-edge cars can roll off during hard close)
EXEMPT_VTYPE_ID = "closure_passenger_exempt"
EXEMPT_VCLASS   = "emergency"   # IMPORTANT: keep this class allowed when disallowing 'passenger'

# Watchlist TTL (seconds)
WATCH_TTL = 15 * 60

# --- Exempt logging state ---
EXEMPT_META: Dict[str, dict] = {}  # vid -> {"t_set": int, "edge": str, "orig_type": str | None}


# =========================== SUMO environment ===========================

SUMO_HOME = os.environ.get("SUMO_HOME", "")
if not SUMO_HOME:
    raise EnvironmentError("SUMO_HOME is not set. Point it to your SUMO installation.")
sys.path.append(os.path.join(SUMO_HOME, "tools"))
import traci  # noqa: E402

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
        "--ignore-route-errors",
    ]

def traci_start(net_file: str, additional_files: str):
    os.makedirs(LOG_DIR, exist_ok=True)
    bin_name = "sumo-gui" if USE_GUI else "sumo"
    exe = _which(bin_name)
    args = [
        exe,
        "-n", absnorm(net_file),
        "--no-step-log", "--duration-log.disable",
        "--step-length", str(STEP_LENGTH),
        "--additional-files", absnorm(additional_files),
    ]
    if USE_GUI:
        args += ["--delay", str(GUI_DELAY_MS)]
    traci.start(args + _meso_args())
    _ensure_exempt_type()

def traci_close():
    try:
        traci.close(False)
    except Exception:
        pass

# ======================== Utils ========================

def absnorm(p: str) -> str:
    return os.path.normpath(os.path.abspath(p))

def ensure_dirs():
    os.makedirs(SOURCE_DIR, exist_ok=True)
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(CONF_DIR, exist_ok=True)
    os.makedirs(LOG_DIR, exist_ok=True)

def run_cmd(args: List[str]) -> None:
    import subprocess
    print("▶", " ".join(args))
    r = subprocess.run(args, capture_output=True, text=True)
    if r.returncode != 0:
        if r.stdout: print(r.stdout)
        if r.stderr: print(r.stderr)
        raise RuntimeError(f"Command failed: {' '.join(args)}")

def write_jsonl(path: str, records: Iterable[dict]) -> None:
    if not records:
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        for r in records:
            f.write(json.dumps(r) + "\n")

# ======================== Grid + Portal augmentation ====================

def _read_plain_nodes(plain_nod_path):
    tree = ET.parse(plain_nod_path)
    root = tree.getroot()
    pos = {}
    for n in root.findall("node"):
        nid = n.get("id"); x = n.get("x"); y = n.get("y")
        if nid and x and y:
            pos[nid] = (float(x), float(y))
    return tree, root, pos

def _add_boundary_portals(plain_dir: str, spur_len: float):
    nod_path = os.path.join(plain_dir, "plain.nod.xml")
    edg_path = os.path.join(plain_dir, "plain.edg.xml")

    nod_tree, nod_root, pos = _read_plain_nodes(nod_path)
    if not pos:
        raise RuntimeError("No nodes in plain.nod.xml (portal augmentation)")

    xs = [p[0] for p in pos.values()]
    ys = [p[1] for p in pos.values()]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    eps = max(1e-6, 0.001 * max(max_x - min_x, max_y - min_y))

    def side_of(nid):
        x, y = pos[nid]
        if abs(x - min_x) <= eps: return "L"
        if abs(x - max_x) <= eps: return "R"
        if abs(y - min_y) <= eps: return "B"
        if abs(y - max_y) <= eps: return "T"
        return None

    existing_node_ids = {n.get("id") for n in nod_root.findall("node")}
    edg_tree = ET.parse(edg_path)
    edg_root = edg_tree.getroot()
    existing_edge_ids = {e.get("id") for e in edg_root.findall("edge")}

    new_nodes = 0
    new_edges = 0
    for nid in pos:
        side = side_of(nid)
        if side is None:
            continue
        x, y = pos[nid]
        if side == "L":   px, py = x - spur_len, y
        elif side == "R": px, py = x + spur_len, y
        elif side == "B": px, py = x, y - spur_len
        else:             px, py = x, y + spur_len

        portal_id = f"portal_{nid}"

        if portal_id not in existing_node_ids:
            nod_root.append(ET.Element("node", id=portal_id, x=str(px), y=str(py)))
            existing_node_ids.add(portal_id)
            new_nodes += 1

        in_id  = f"portal_to_{nid}"
        out_id = f"{nid}_to_portal"

        if in_id not in existing_edge_ids:
            edg_root.append(ET.Element("edge", id=in_id, **{"from": portal_id, "to": nid, "numLanes": "1", "speed": "13.9"}))
            existing_edge_ids.add(in_id)
            new_edges += 1
        if out_id not in existing_edge_ids:
            edg_root.append(ET.Element("edge", id=out_id, **{"from": nid, "to": portal_id, "numLanes": "1", "speed": "13.9"}))
            existing_edge_ids.add(out_id)
            new_edges += 1

    nod_tree.write(nod_path, encoding="utf-8", xml_declaration=True)
    edg_tree.write(edg_path, encoding="utf-8", xml_declaration=True)
    print(f"🟢 Added {new_nodes} portal nodes & {new_edges} portal edges")

def _rebuild_net_from_plain(plain_dir: str, out_net_path: str):
    type_file = "plain.typ.xml" if os.path.exists(os.path.join(plain_dir, "plain.typ.xml")) else "plain.con.xml"
    args = [
        "netconvert",
        "--node-files", os.path.join(plain_dir, "plain.nod.xml"),
        "--edge-files", os.path.join(plain_dir, "plain.edg.xml"),
        "--type-files", os.path.join(plain_dir, type_file),
        "--output-file", out_net_path
    ]
    if os.path.exists(os.path.join(plain_dir, "plain.tll.xml")):
        args += ["--tls.guess", "true"]
    run_cmd(args)
    print(f"✅ Rebuilt net with portals → {out_net_path}")

def generate_grid_with_portals(out_dir: str) -> str:
    os.makedirs(out_dir, exist_ok=True)
    net_path = os.path.join(out_dir, "MySUMOFile.net.xml")
    run_cmd([
        "netgenerate", "--grid",
        f"--grid.number={GRID_SIZE}",
        f"--grid.length={EDGE_LENGTH}",
        f"--plain-output-prefix={os.path.join(out_dir, 'plain')}",
        f"--output-file={net_path}"
    ])
    print("✅ Base grid generated.")
    _add_boundary_portals(out_dir, SPUR_LEN)
    _rebuild_net_from_plain(out_dir, net_path)
    return net_path

# ========================= Net parsing & portal edges ======================

def collect_portal_edges_and_nodes(net_file: str) -> Tuple[List[str], List[str], List[str], Dict[str, Tuple[str, str]], Dict[str, List[str]]]:
    """
    Returns:
      origin_edges: portal->boundary
      sink_edges:   boundary->portal
      closeable_edges: other non-internal edges
      edge_to_nodes: edge -> (from_node, to_node)
      inbound_map:  node -> list of inbound edge IDs
    """
    tree = ET.parse(net_file)
    root = tree.getroot()
    origin, sink, closeable = [], [], []
    edge_to_nodes: Dict[str, Tuple[str, str]] = {}
    inbound_map: Dict[str, List[str]] = defaultdict(list)

    for e in root.findall("edge"):
        if e.get("function") is not None:
            continue
        eid = e.get("id", "")
        fr = e.get("from", "")
        to = e.get("to", "")
        if eid.startswith(":"):
            continue
        edge_to_nodes[eid] = (fr, to)
        inbound_map[to].append(eid)

        if fr.startswith("portal_") and not to.startswith("portal_"):
            origin.append(eid)
        elif to.startswith("portal_") and not fr.startswith("portal_"):
            sink.append(eid)
        else:
            closeable.append(eid)

    if not origin or not sink:
        raise RuntimeError("Portal edges not found — did augmentation run?")
    return origin, sink, closeable, edge_to_nodes, inbound_map

def write_detectors_add(net_file: str, add_path: str, sensor_out_xml: str, freq: int = BIN_SEC):
    """Build <additional> with e1Detector for each non-portal lane."""
    tree = ET.parse(net_file)
    root = tree.getroot()
    add = ET.Element("additional")

    for e in root.findall("edge"):
        eid = e.get("id", "")
        if eid.startswith(":"):
            continue
        if eid.startswith("portal_to_") or eid.endswith("_to_portal"):
            continue  # skip portal spurs
        for lane in e.findall("lane"):
            ET.SubElement(add, "e1Detector", {
                "id": f"det_{lane.get('id')}",
                "lane": lane.get("id"),
                "pos": str(float(lane.get("length", "0")) / 2.0),
                "freq": str(freq),
                "file": absnorm(sensor_out_xml),
            })

    ET.ElementTree(add).write(add_path, encoding="utf-8", xml_declaration=True)
    print(f"✅ Detectors written (portal edges excluded): {add_path}")

# ================================ Demand =================================

def diurnal_factor(mins_of_day: int) -> float:
    """Simple weekday-ish profile; repeats each 24h if sim runs longer."""
    h = mins_of_day / 60.0
    base = 0.40
    am = math.exp(-0.5 * ((h - 8.5) / 1.1) ** 2)
    pm = 1.3 * math.exp(-0.5 * ((h - 17.5) / 1.2) ** 2)
    f = base + am + pm
    if DIURNAL_PEAK_CLAMP is not None:
        f = min(f, DIURNAL_PEAK_CLAMP)
    return f

def poisson(lam_per_sec: float) -> int:
    L = math.exp(-lam_per_sec); k = 0; p = 1.0
    while True:
        k += 1; p *= random.random()
        if p <= L:
            return k - 1

# ================================ Closures core ================================

def edge_lane_ids(edge_id: str) -> List[str]:
    n = traci.edge.getLaneNumber(edge_id)
    return [f"{edge_id}_{i}" for i in range(n)]

def set_edge_closed(edge_id: str, closed: bool) -> int:
    lids = edge_lane_ids(edge_id)
    for lid in lids:
        if closed:
            # HARD close to 'passenger' (exempt type uses 'emergency' class)
            traci.lane.setDisallowed(lid, ["passenger"])
        else:
            traci.lane.setAllowed(lid, ["passenger"])
    return len(lids)

def build_run_plan(closeable_edges: List[str]) -> List[Tuple[Optional[str], int, int]]:
    """
    Returns (edge_id|None, start_t, end_t):
      - First 1h with edge_id=None (baseline open)
      - Then for each edge: 5min None (open break), then 1h with that edge closed
    """
    plan = []
    t = 0
    plan.append((None, t, t + BASELINE_SECS)); t += BASELINE_SECS
    for eid in closeable_edges:
        plan.append((None, t, t + BREAK_SECS)); t += BREAK_SECS
        plan.append((eid,  t, t + CLOSE_SECS));  t += CLOSE_SECS
    return plan

# ====== TACTIC HELPERS (from month-runner, adapted) ======

def _ensure_exempt_type():
    """Create/prepare an exempt vType with allowed vClass (e.g., 'emergency')."""
    try:
        ids = set(traci.vehicletype.getIDList())
        base = "DEFAULT_VEHTYPE" if "DEFAULT_VEHTYPE" in ids else (next(iter(ids)) if ids else None)
        if base is None:
            return
        if EXEMPT_VTYPE_ID not in ids:
            traci.vehicletype.copy(base, EXEMPT_VTYPE_ID)
        try:
            traci.vehicletype.setVehicleClass(EXEMPT_VTYPE_ID, EXEMPT_VCLASS)
        except traci.TraCIException:
            traci.vehicletype.setParameter(EXEMPT_VTYPE_ID, "vClass", EXEMPT_VCLASS)
    except traci.TraCIException:
        pass

# put near other utils
def logp(*args, **kwargs):
    print(*args, **kwargs, flush=True)



def _veh_exists(vid: str) -> bool:
    try:
        return vid in traci.vehicle.getIDList()
    except traci.TraCIException:
        return False

def _safe_get_route(vid: str) -> List[str]:
    try:
        return list(traci.vehicle.getRoute(vid))
    except traci.TraCIException:
        return []

def _safe_road(vid: str) -> str:
    try:
        return traci.vehicle.getRoadID(vid)
    except traci.TraCIException:
        return "?"

def _safe_index(vid: str) -> int:
    try:
        return traci.vehicle.getRouteIndex(vid)
    except traci.TraCIException:
        return -1

def _remaining_route(route: List[str], idx: int) -> List[str]:
    if not route:
        return []
    if idx is None or idx < 0 or idx >= len(route):
        return route
    return route[idx:]  # current edge onward

def _remaining_ahead(route: List[str], idx: int) -> List[str]:
    if not route:
        return []
    if idx is None or idx < 0:
        return route
    return route[idx+1:]

def _edge_ahead_in_remaining(route: List[str], idx: int, target_edge: str) -> bool:
    rem = _remaining_route(route, idx)
    return target_edge in rem

def _remaining_intersects_closed(route: List[str], idx: int, closed: set) -> bool:
    rem = _remaining_route(route, idx)
    return any(e in closed for e in rem)

def _fmt_route_line(vehicle: str, route: List[str], idx: int, cur: str, dest_equals_closed: bool=False) -> str:
    dest = route[-1] if route else None
    flag = " (dest==edge)" if dest_equals_closed else ""
    return f"- {vehicle} idx={idx} cur={cur} dest={dest} | route={route}{flag}"

# Watchlist structures
WATCHLIST: Dict[str, dict] = {}     # vid -> meta {edge, t_added, reason}
SKIP_REROUTE_ON_EDGE: Dict[str, Dict[str, dict]] = defaultdict(dict)  # edge -> {vid -> meta}

def watch_add(vids: List[str], reason: str, t: int, edge: str) -> None:
    if not vids: return
    recs = []
    for v in vids:
        WATCHLIST[v] = {"edge": edge, "t_added": t, "reason": reason}
        recs.append({"t_sec": t, "event": "watch_add", "vehicle": v, "edge": edge, "reason": reason})
    write_jsonl(os.path.join(LOG_DIR, "watchlist.jsonl"), recs)

def watch_step(t: int) -> None:
    if not WATCHLIST:
        return
    try:
        now_ids = set(traci.vehicle.getIDList())
        arrived = set(traci.simulation.getArrivedIDList())
        tp_end = set(traci.simulation.getEndingTeleportIDList())
    except traci.TraCIException:
        return

    done = []
    recs = []
    for v, meta in list(WATCHLIST.items()):
        if t - meta["t_added"] > WATCH_TTL:
            recs.append({"t_sec": t, "event": "watch_expire", "vehicle": v, **meta})
            done.append(v); continue
        if v in now_ids:
            continue
        outcome = "arrived" if v in arrived else ("teleported" if v in tp_end else "disappeared")
        recs.append({"t_sec": t, "event": "watch_done", "vehicle": v, "outcome": outcome, **meta})
        done.append(v)
    for v in done:
        WATCHLIST.pop(v, None)
    write_jsonl(os.path.join(LOG_DIR, "watchlist.jsonl"), recs)

def log_teleports(t: int, closed_now: set) -> None:
    try:
        starts = traci.simulation.getStartingTeleportIDList()
        ends = traci.simulation.getEndingTeleportIDList()
    except traci.TraCIException:
        return
    if not starts and not ends:
        return
    recs = []
    for kind, lst in (("start", starts), ("end", ends)):
        for vid in lst:
            r = _safe_get_route(vid)
            recs.append({
                "t_sec": t, "kind": kind, "vehicle": vid,
                "current_edge": _safe_road(vid), "route": r,
                "dest": (r[-1] if r else None),
                "dest_is_closed_now": ((r[-1] in closed_now) if r else None),
                "closed_edges_now": list(closed_now),
            })
    write_jsonl(os.path.join(LOG_DIR, "teleports.jsonl"), recs)

def log_pre_and_collect(closed_edge: str, t: int) -> List[str]:
    try:
        vids = traci.vehicle.getIDList()
    except traci.TraCIException:
        vids = []
    touched, recs = [], []
    for vid in vids:
        r = _safe_get_route(vid)
        idx = _safe_index(vid)
        cur = _safe_road(vid)
        if r and _edge_ahead_in_remaining(r, idx, closed_edge):
            touched.append(vid)
            recs.append({
                "t_sec": t, "phase": "pre", "vehicle": vid, "route": r,
                "route_index": idx, "current_edge": cur,
                "dest_equals_closed": (r[-1] == closed_edge if r else False)
            })
    write_jsonl(os.path.join(LOG_DIR, "closure_pre.jsonl"), recs)
    if touched:
        print(f"    [pre] vehicles touching {closed_edge}: {len(touched)}")
        for rec in recs[:MAX_CONSOLE_SAMPLE]:
            print("      " + _fmt_route_line(
                rec["vehicle"], rec["route"], rec["route_index"],
                rec["current_edge"], rec["dest_equals_closed"]
            ))
        if len(touched) > MAX_CONSOLE_SAMPLE:
            print(f"      ... {len(touched) - MAX_CONSOLE_SAMPLE} more")
    else:
        print(f"    [pre] no vehicles with {closed_edge} ahead in their remaining route.")
    return touched

def build_edge_to_neighbors(closed_edge: str,
                            edge_to_nodes: Dict[str, Tuple[str, str]],
                            inbound_map: Dict[str, List[str]],
                            closed_now: set) -> List[str]:
    """Inbound edges of the same destination node (preferred alt targets)."""
    if closed_edge not in edge_to_nodes:
        return []
    _fr, to_node = edge_to_nodes[closed_edge]
    cands = [e for e in inbound_map.get(to_node, []) if e != closed_edge and e not in closed_now]
    return cands

def retarget_dest_if_needed(vids: List[str],
                            closed_edge: str,
                            t: int,
                            edge_to_nodes: Dict[str, Tuple[str, str]],
                            inbound_map: Dict[str, List[str]],
                            also_closed: set) -> int:
    # Skip cars currently on the source edge; they are handled via exemption flow
    vids = [v for v in vids if _veh_exists(v) and _safe_road(v) != closed_edge]
    recs, changed = [], 0
    neighbors = build_edge_to_neighbors(closed_edge, edge_to_nodes, inbound_map, also_closed)
    all_inbound = [e for es in inbound_map.values() for e in es]
    fallback = [e for e in all_inbound if e != closed_edge and e not in also_closed]

    for vid in vids:
        if not _veh_exists(vid):
            recs.append({"t_sec": t, "vehicle": vid, "action": "skip_no_longer_exists"}); continue
        r = _safe_get_route(vid)
        if not r or r[-1] != closed_edge:
            continue
        new_dest = random.choice(neighbors) if neighbors else (random.choice(fallback) if fallback else None)
        if not new_dest:
            recs.append({"t_sec": t, "vehicle": vid, "old_dest": closed_edge, "action": "no_alt_dest"}); continue
        try:
            traci.vehicle.changeTarget(vid, new_dest)
            traci.vehicle.rerouteTraveltime(vid)
            recs.append({"t_sec": t, "vehicle": vid, "old_dest": closed_edge, "new_dest": new_dest, "action": "changeTarget+reroute"})
            changed += 1
        except traci.TraCIException as ex:
            recs.append({"t_sec": t, "vehicle": vid, "old_dest": closed_edge, "new_dest": new_dest, "action": "error", "exception": str(ex)})
    write_jsonl(os.path.join(LOG_DIR, "retargets.jsonl"), recs)
    return changed

def post_partition_from_pre(pre_vids: List[str], closed_edge: str, t: int):
    snaps = {}
    for vid in pre_vids:
        snaps[vid] = {
            "route": _safe_get_route(vid),
            "route_index": _safe_index(vid),
            "current_edge": _safe_road(vid)
        }
    still_contains, cleared = [], []
    for vid, s in snaps.items():
        r = s["route"]; i = s["route_index"]
        ahead = _remaining_ahead(r, i)
        if r and (closed_edge in ahead):
            still_contains.append(vid)
        else:
            cleared.append(vid)

    print(f"    [post] {closed_edge}: still_contains={len(still_contains)} | cleared={len(cleared)}")
    return still_contains, cleared

def skip_reroute_maintenance(t: int, closed_now: set) -> None:
    """After exemptions: revert type + conditional reroute when car leaves the source edge."""
    if not SKIP_REROUTE_ON_EDGE:
        return
    recs = []
    for edge, vids_meta in list(SKIP_REROUTE_ON_EDGE.items()):
        done = []
        for v, meta in list(vids_meta.items()):
            if not _veh_exists(v):
                recs.append({"t_sec": t, "event": "skip_drop_missing", "vehicle": v, "edge": edge, **{k: meta.get(k) for k in ("orig_type","exempted")}})
                done.append(v); continue
            if _safe_road(v) == edge:
                continue  # still on the source edge → keep exemption

            # revert type if exempted
            if meta.get("exempted") and meta.get("orig_type"):
                try:
                    traci.vehicle.setType(v, meta["orig_type"])
                    recs.append({"t_sec": t, "event": "revert_type", "vehicle": v, "edge": edge, "orig_type": meta["orig_type"]})
                except traci.TraCIException as ex:
                    recs.append({"t_sec": t, "event": "revert_type_error", "vehicle": v, "edge": edge, "exception": str(ex)})

            # conditional reroute if their remaining path still hits any closed edge
            need_reroute = meta.get("needs_reroute", False)
            if not need_reroute:
                r = _safe_get_route(v); i = _safe_index(v)
                if r:
                    need_reroute = _remaining_intersects_closed(r, i, closed_now)
            if need_reroute and _veh_exists(v):
                try:
                    traci.vehicle.rerouteTraveltime(v)
                    recs.append({"t_sec": t, "event": "left_source_rerouted", "vehicle": v, "edge": edge})
                except traci.TraCIException as ex:
                    recs.append({"t_sec": t, "event": "left_source_rerouted_error", "vehicle": v, "edge": edge, "exception": str(ex)})
            else:
                recs.append({"t_sec": t, "event": "left_source_no_reroute", "vehicle": v, "edge": edge})

            done.append(v)

        for v in done:
            vids_meta.pop(v, None)
        if not vids_meta:
            SKIP_REROUTE_ON_EDGE.pop(edge, None)

    write_jsonl(os.path.join(LOG_DIR, "skip_reroute.jsonl"), recs)

# ================================ MAIN ==================================

def main():
    ensure_dirs()

    # Generate net with portals
    net_file = generate_grid_with_portals(SOURCE_DIR)

    # Discover portal origins/sinks and closeable edges + topology helpers
    origin_edges, sink_edges, closeable_edges, edge_to_nodes, inbound_map = collect_portal_edges_and_nodes(net_file)
    print(f"Origins (portal->boundary): {len(origin_edges)} | Sinks (boundary->portal): {len(sink_edges)}")
    print(f"Closeable edges (non-portal): {len(closeable_edges)}")

    # Detectors (additional) and EdgeData outputs (ABSOLUTE PATHS)
    det_add     = absnorm(os.path.join(CONF_DIR, "detectors.add.xml"))
    sensor_xml  = absnorm(os.path.join(OUT_DIR,  "detectors.xml"))
    edged_xml   = absnorm(os.path.join(OUT_DIR,  "edgedata.xml"))
    write_detectors_add(net_file, det_add, sensor_xml, BIN_SEC)

    # Compose a single <additional> = detectors + edgeData (file=ABS)
    root = ET.Element("additional")
    det_tree = ET.parse(det_add)
    for e1 in det_tree.getroot().iter("e1Detector"):
        root.append(e1)
    ET.SubElement(root, "edgeData", {"id": "ed1", "file": edged_xml, "freq": str(BIN_SEC)})
    add_path = absnorm(os.path.join(CONF_DIR, "run_outputs.add.xml"))
    ET.ElementTree(root).write(add_path, encoding="utf-8", xml_declaration=True)

    # Start SUMO
    traci_start(net_file, add_path)

    # Build plan; may exceed 24h (OK). Diurnal repeats each 24h.
    plan = build_run_plan(closeable_edges)
    T_END = plan[-1][2]
    print(f"Total simulated time: {T_END/3600.0:.2f} h "
          f"(1h baseline + {len(closeable_edges)} × (5min break + 1h closure))")

    vid = 0

    try:
        for t in range(T_END):
            # Determine which edge is currently closed (at most one in our plan)
            active_edge = None
            for eid, ts, te in plan:
                if eid and ts <= t < te:
                    active_edge = eid
                    break

            # Toggle closures at plan boundaries (open all; close only the active)
            if t == 0 or any(t == x for _eid, s, e in plan for x in (s, e)):
                # open all closeable edges
                for e in closeable_edges:
                    try: set_edge_closed(e, False)
                    except Exception: pass

                if active_edge:
                    # ===== PRE-CLOSE TACTICS =====
                    touched = log_pre_and_collect(active_edge, t)

                    # Likely warnings: dest == closed edge → watch
                    warn_candidates = []
                    for v in touched:
                        r = _safe_get_route(v)
                        if r and r[-1] == active_edge:
                            warn_candidates.append(v)
                    watch_add(warn_candidates, reason="dest_equals_closed_at_pre", t=t, edge=active_edge)

                    # Vehicles currently ON the edge? → EXEMPT + skip reroute until they leave
                    on_edge = [v for v in traci.vehicle.getIDList() if _safe_road(v) == active_edge]
                    if on_edge:
                        closed_now = {active_edge}
                        for v in on_edge:
                            r = _safe_get_route(v) or []
                            # if they also have other closed edges ahead (rare here), mark for reroute after leaving
                            needs = any(e in closed_now for e in r if e != active_edge)
                            orig = None
                            try:
                                orig = traci.vehicle.getTypeID(v)
                                traci.vehicle.setType(v, EXEMPT_VTYPE_ID)  # allow them through the hard close
                                EXEMPT_META[v] = {"t_set": t, "edge": active_edge, "orig_type": orig}
                                logp(f"[exempt_set] t={t:>6} veh={v} edge={active_edge} orig_type={orig}")

                            except traci.TraCIException:
                                pass
                            SKIP_REROUTE_ON_EDGE[active_edge][v] = {
                                "needs_reroute": needs, "t_added": t,
                                "orig_type": orig, "exempted": True
                            }
                        watch_add(on_edge, reason="on_source_edge_at_close_exempted", t=t, edge=active_edge)

                    # HARD CLOSE
                    lanes = set_edge_closed(active_edge, True)
                    m,s = divmod(t,60)
                    print(f"[{m:02d}:{s:02d}] CLOSE {active_edge} ({lanes} lanes)")

                    # ===== POST-CLOSE: RETARGET then FILTERED REROUTE =====
                    if RETARGET_BEFORE_REROUTE:
                        closed_now = {active_edge}
                        changed = retarget_dest_if_needed(
                            touched, active_edge, t, edge_to_nodes, inbound_map,
                            also_closed=closed_now
                        )
                        # Filtered reroute: only those whose remaining route intersects a closed edge
                        skip_set = set(SKIP_REROUTE_ON_EDGE.get(active_edge, {}).keys())
                        rerouted = 0
                        for v2 in list(traci.vehicle.getIDList()):
                            if v2 in skip_set:
                                continue
                            r2 = _safe_get_route(v2); i2 = _safe_index(v2)
                            if not r2: continue
                            if not _remaining_intersects_closed(r2, i2, closed_now):
                                continue
                            try:
                                traci.vehicle.rerouteTraveltime(v2)
                                rerouted += 1
                            except traci.TraCIException:
                                pass
                        print(f"    [post] retargeted={changed} | filtered_rerouted={rerouted}")
                    else:
                        # (Alternative order) global reroute except on-edge, then retarget
                        skip_set = set(SKIP_REROUTE_ON_EDGE.get(active_edge, {}).keys())
                        for v2 in list(traci.vehicle.getIDList()):
                            if v2 in skip_set: continue
                            try: traci.vehicle.rerouteTraveltime(v2)
                            except traci.TraCIException: pass
                        changed = retarget_dest_if_needed(
                            touched, active_edge, t, edge_to_nodes, inbound_map,
                            also_closed={active_edge}
                        )

                    # Partition PRE vehicles into still_contains / cleared
                    still_contains, cleared = post_partition_from_pre(touched, active_edge, t)
                else:
                    m,s = divmod(t,60)
                    print(f"[{m:02d}:{s:02d}] OPEN (no closure)")

            # Injection from portal origins to portal sinks
            mins = (t // 60) % (24 * 60)  # diurnal minute-of-day
            factor = diurnal_factor(mins)
            lam_total = BASE_SPAWN_RATE_TOTAL * factor
            lam_per_origin = lam_total / max(1, len(origin_edges))
            closed_now = {active_edge} if active_edge else set()

            for o in origin_edges:
                if o in closed_now:
                    continue
                n = poisson(lam_per_origin * STEP_LENGTH)
                for _ in range(n):
                    # pick a sink not equal to origin and not closed
                    dst = None
                    for _try in range(6):
                        cand = random.choice(sink_edges)
                        if cand != o and cand not in closed_now:
                            dst = cand; break
                    if not dst:
                        continue
                    try:
                        r = traci.simulation.findRoute(o, dst, vType="DEFAULT_VEHTYPE")
                        if not r.edges:
                            write_jsonl(os.path.join(LOG_DIR, "route_find_failures.jsonl"), [{
                                "t_sec": t, "origin": o, "dest": dst,
                                "closed_now": list(closed_now), "reason": "empty_route_edges"
                            }])
                            continue
                        rid = f"r{vid}"
                        traci.route.add(rid, r.edges)
                        traci.vehicle.add(f"veh{vid}", rid,
                                          departLane="random", departPos="random", departSpeed="max")
                        vid += 1
                    except traci.TraCIException as ex:
                        write_jsonl(os.path.join(LOG_DIR, "route_find_failures.jsonl"), [{
                            "t_sec": t, "origin": o, "dest": dst,
                            "closed_now": list(closed_now), "exception": str(ex)
                        }])
                        continue

            # Advance simulation
            traci.simulationStep()

            # Teleport diagnostics
            log_teleports(t, closed_now)

            # Watchlist maintenance
            watch_step(t)

            # After cars leave the source edge: revert type + conditional reroute
            skip_reroute_maintenance(t, closed_now)

            # Progress every 15 minutes
            if (t + 1) % (15 * 60) == 0:
                running = traci.simulation.getMinExpectedNumber()
                m, s = divmod(t + 1, 60)
                print(f"[t={m:4d}m] running≈{running} active={active_edge or 'none'}")

        # Drain a few steps for flush
        for _ in range(5):
            traci.simulationStep()

    finally:
        traci_close()

    # Wait for outputs to stabilize
    for p in (sensor_xml, edged_xml):
        t0 = time(); last = -1
        while time() - t0 < STABILIZE_WAIT_S:
            if os.path.exists(p):
                sz = os.path.getsize(p)
                if sz > 0 and sz == last:
                    break
                last = sz
            sleep(STABILIZE_POLL_S)

    print("\n✅ Done.")
    print(" Detectors  →", sensor_xml)
    print(" EdgeData   →", edged_xml)
    print(" Net        →", os.path.join(SOURCE_DIR, "MySUMOFile.net.xml"))

if __name__ == "__main__":
    main()
